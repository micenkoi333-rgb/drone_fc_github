#pragma once
// Міст до ArduPilot SITL (бекенд JSON, libraries/SITL/SIM_JSON.cpp): ArduPilot сам керує апаратом цього симулятора.
// ArduPilot → симулятор: UDP-пакет з ШІМ виходів (magic 18458 — 16 каналів, 29569 — 32), у lockstep — один пакет = один кадр фізики.
// Симулятор → ArduPilot: рядок JSON з IMU (тіло FRD), позицією й швидкістю (NED), кватерніоном, повітряною швидкістю, батареєю.
// Тут — розбір, перетворення координат і відображення каналів; сокети — у tools/sitl_bridge.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include "../fc/frames.h"
#include "qp_flight.h"

namespace sim {

struct SitlServoPacket {
    uint16_t frame_rate = 0;
    uint32_t frame_count = 0;
    uint16_t pwm[32] = {};
    int channels = 0;
};

inline bool parseSitlServo(const uint8_t* buf, size_t n, SitlServoPacket& p) {
    if (n < 8) return false;
    uint16_t magic; std::memcpy(&magic, buf, 2);
    const int ch = magic == 18458 ? 16 : (magic == 29569 ? 32 : 0);
    if (!ch || n < size_t(8 + 2 * ch)) return false;
    std::memcpy(&p.frame_rate, buf + 2, 2);
    std::memcpy(&p.frame_count, buf + 4, 4);
    std::memcpy(p.pwm, buf + 8, size_t(2 * ch));
    p.channels = ch;
    return true;
}

// Канали (з 0) — як у параметрах ArduPilot SERVOn_FUNCTION для конвертоплана з 4 гондолами й V-хвостом:
// мотори 1–4 (33–36, порядок quad X: FR, RL, FL, RR — той самий, що fc::kQuadX), елерон, ліве/праве перо V-хвоста, нахил кожної гондоли.
struct SitlChannelMap {
    int motor[4] = {0, 1, 2, 3};
    int aileron = 4;
    int vtail_left = 5, vtail_right = 6;   // ArduPilot мікшує V-хвіст сам (79/80): висота = (L+R)/2, напрям = (L−R)/2
    int tilt[4] = {8, 9, 10, 11};
    float tilt_pwm0 = 1000, tilt_pwm90 = 2000;   // ШІМ для 0° (вгору) і 90° (уперед)
    float max_defl = 0.43f;                       // рад при ±500 мкс

    bool load(const std::string& path) {
        FILE* f = std::fopen(path.c_str(), "rb");
        if (!f) return false;
        char line[256];
        while (std::fgets(line, sizeof line, f)) {
            char key[64]; float v;
            if (std::sscanf(line, " %63[A-Za-z0-9_] = %f", key, &v) != 2) continue;
            const std::string k(key);
            for (int i = 0; i < 4; ++i) {
                if (k == "motor" + std::to_string(i + 1)) motor[i] = int(v) - 1;
                if (k == "tilt" + std::to_string(i + 1)) tilt[i] = int(v) - 1;
            }
            if (k == "aileron") aileron = int(v) - 1;
            if (k == "vtail_left") vtail_left = int(v) - 1;
            if (k == "vtail_right") vtail_right = int(v) - 1;
            if (k == "tilt_pwm0") tilt_pwm0 = v;
            if (k == "tilt_pwm90") tilt_pwm90 = v;
        }
        std::fclose(f);
        return true;
    }

    QpControls controls(const SitlServoPacket& p) const {
        auto pwm = [&](int c) { return (c >= 0 && c < p.channels && p.pwm[c] > 0) ? float(p.pwm[c]) : 0.0f; };
        auto surf = [&](int c) { const float w = pwm(c); return w > 0 ? std::fmax(-1.0f, std::fmin(1.0f, (w - 1500.0f) / 500.0f)) * max_defl : 0.0f; };
        QpControls u;
        for (int i = 0; i < 4; ++i) { const float w = pwm(motor[i]); u.duty_vtol[i] = w > 0 ? std::fmax(0.0f, std::fmin(1.0f, (w - 1000.0f) / 1000.0f)) : 0.0; }
        u.aileron = surf(aileron);
        const float l = surf(vtail_left), r = surf(vtail_right);
        u.elevator = 0.5 * (l + r); u.rudder = 0.5 * (l - r);
        for (int i = 0; i < 4; ++i) {
            const float w = pwm(tilt[i]);
            u.tilt[i] = w > 0 ? (w - tilt_pwm0) / (tilt_pwm90 - tilt_pwm0) * 1.5707963 : 0.0;
        }
        return u;
    }
};

// Стан для ArduPilot. Акселерометр — питома сила (у спокої z = −9.81 у FRD), гіро — тіло FRD.
inline std::string sitlStateJson(const QuadPlaneSim& s, double timestamp, const Vec3d& accel_flu) {
    const QpState& st = s.state();
    const fc::Quat q_enu(float(st.q.w), float(st.q.x), float(st.q.y), float(st.q.z));
    const fc::Quat q = fc::qEnuFluToNedFrd(q_enu);
    const fc::Vec3 g = fc::fluToFrd(fc::Vec3{float(st.omega.x), float(st.omega.y), float(st.omega.z)});
    const fc::Vec3 a = fc::fluToFrd(fc::Vec3{float(accel_flu.x), float(accel_flu.y), float(accel_flu.z)});
    const Vec3d vw = s.velWorld();
    const fc::Vec3 pn = fc::enuToNed(fc::Vec3{float(st.pos.x), float(st.pos.y), float(st.pos.z)});
    const fc::Vec3 vn = fc::enuToNed(fc::Vec3{float(vw.x), float(vw.y), float(vw.z)});
    char b[900];
    std::snprintf(b, sizeof b,
                  "\n{\"timestamp\":%.6f,\"imu\":{\"gyro\":[%.6f,%.6f,%.6f],\"accel_body\":[%.5f,%.5f,%.5f]},"
                  "\"position\":[%.4f,%.4f,%.4f],\"quaternion\":[%.7f,%.7f,%.7f,%.7f],\"velocity\":[%.4f,%.4f,%.4f],"
                  "\"airspeed\":%.3f,\"battery\":{\"voltage\":%.3f,\"current\":%.3f}}\n",
                  timestamp, g.x, g.y, g.z, a.x, a.y, a.z, pn.x, pn.y, pn.z, q.w, q.x, q.y, q.z, vn.x, vn.y, vn.z,
                  std::fmax(0.0, s.airspeedVec(Vec3d{}).x), s.busVoltage(), s.busCurrent());
    return b;
}

}  // namespace sim
