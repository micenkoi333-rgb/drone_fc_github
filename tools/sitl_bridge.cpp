// tools/sitl_bridge.cpp — фізика цього симулятора для ArduPilot SITL (бекенд JSON).
// Справжня прошивка ArduPilot керує нашим конвертопланом: так перевіряється перенесення на залізо до першого польоту.
// Збирання (Windows): cl /std:c++17 /O2 /EHsc /utf-8 tools\sitl_bridge.cpp ws2_32.lib
//          (Linux):   g++ -std=c++17 -O2 -o sitl_bridge tools/sitl_bridge.cpp
// Запуск:  sitl_bridge [порт 9002] [файл каналів tools/sitl_map.ini] [вітер м/с]
//          sim_vehicle.py -v ArduPlane --model JSON:127.0.0.1 --add-param-file=tools/sitl_tiltrotor.param
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#include "../sim/console_utf8.h"
#include "../sim/sitl_json.h"

using namespace sim;

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::atoi(argv[1]) : 9002;
    SitlChannelMap map;
    const std::string map_file = argc > 2 ? argv[2] : "tools/sitl_map.ini";
    if (map.load(map_file)) std::printf("канали — з %s\n", map_file.c_str());
    const double wind = argc > 3 ? std::atof(argv[3]) : 0.0;
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    const int sock = int(socket(AF_INET, SOCK_DGRAM, 0));
    sockaddr_in me{}; me.sin_family = AF_INET; me.sin_port = htons(uint16_t(port)); me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, reinterpret_cast<sockaddr*>(&me), sizeof me) != 0) { std::printf("порт %d зайнятий\n", port); return 1; }
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    auto fresh = [&] {   // лише фізика: автопілот — ArduPilot
        auto r = std::make_unique<QpFlightRig>(a, tb, 1);
        if (wind > 0) { WindConfig w; w.mean20ft = Vec3d{wind, 0, 0}; w.turbulence = 1.0; r->setWind(w, 7); }
        return r;
    };
    auto rp = fresh();
    map.max_defl = float(a.num("controls.max_deflection_deg") * 3.14159265 / 180);
    std::printf("чекаю ArduPilot на UDP %d…\n", port);
    uint32_t last_frame = 0;
    double t = 0;
    Vec3d v_prev = rp->sim.velWorld();
    long frames = 0;
    for (;;) {
        uint8_t buf[256];
        sockaddr_in from{}; socklen_t fl = sizeof from;
        const int n = int(recvfrom(sock, reinterpret_cast<char*>(buf), sizeof buf, 0, reinterpret_cast<sockaddr*>(&from), &fl));
        SitlServoPacket p;
        if (n <= 0 || !parseSitlServo(buf, size_t(n), p)) continue;
        if (p.frame_count < last_frame) {   // ArduPilot перезапущено — апарат знову на землі
            rp = fresh();
            t = 0; v_prev = rp->sim.velWorld();
            std::printf("перезапуск SITL\n");
        } else if (p.frame_count == last_frame && frames > 0) {
            continue;   // повтор того самого кадру
        }
        last_frame = p.frame_count;
        QpFlightRig& rig = *rp;
        const double dt = p.frame_rate > 0 ? 1.0 / p.frame_rate : 0.001;
        const QpControls u = map.controls(p);
        const int steps = std::max(1, int(dt * 1000 + 0.5));
        for (int k = 0; k < steps; ++k) rig.sim.step(0.001, u);
        t += steps * 0.001;
        const Vec3d vw = rig.sim.velWorld();
        const Vec3d f = rig.sim.state().q.rotateInv((vw - v_prev) / (steps * 0.001) + Vec3d{0, 0, QuadPlaneSim::kG});
        v_prev = vw;
        const std::string js = sitlStateJson(rig.sim, t, f);
        sendto(sock, js.c_str(), int(js.size()), 0, reinterpret_cast<sockaddr*>(&from), fl);
        if (++frames % 4000 == 0)
            std::printf("t %.0f с: висота %.1f м, повітряна %.1f м/с, гондоли %.0f°%s\n", t, rig.sim.state().pos.z, rig.sim.airspeed(), rig.sim.tilt(0) * 57.3,
                        rig.sim.crashed() ? ", АВАРІЯ" : "");
    }
}
