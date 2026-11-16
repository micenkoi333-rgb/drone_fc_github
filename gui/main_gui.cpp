// main_gui.cpp — інтерфейс конвертоплана: GLFW + OpenGL + Dear ImGui + ImPlot. 3D-вид, PFD, карта, стан, енергія, графіки, керування.
// Модель і контролер — ті самі, що в консольних тестах (sim/qp_flight.h). Фізика 1 кГц, контролер 500 Гц, кадри — за частотою монітора.
// Запуск: drone_gui.exe [--frames N] [--speed X] [--shot файл.bmp] (знімок останнього кадру й вихід — для перевірки);
// для перевірки «Мозку ШІ»: [--size W H] [--brain 0|1|2] [--train трюк спроб] [--show трюк|-1] [--sim с] [--ai 0|1|2] [--compare].
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>   // ShellExecuteA: відкрити HTML-звіт / папку
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <ctime>
#include <mutex>
#include <thread>
#include <chrono>
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "../sim/qp_flight.h"
#include "../ai/mission_planner.h"
#include "../sim/qp_report_html.h"
#include "../sim/qp_tricks.h"
#include "sound.h"

using namespace sim;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

struct App {
    std::string ac_path = activeAircraftPath();   // який апарат літає (aircraft/active.txt)
    AircraftFile file = AircraftFile::load(ac_path);
    bool forget_learned = false;
    AeroTables tables = AeroTables::load("aircraft/aero_tables.json");
    std::unique_ptr<QpFlightRig> rig;
    std::deque<Vec3d> trail;
    // параметри сцени/місії
    float leg_km = 1.0f, wind = 5.0f, wind_dir = 20.0f, time_scale = 1.0f, turbulence = 1.5f;
    int camera = 0;             // 0 за апаратом, 1 орбітальна, 2 з землі, 3 згори
    bool paused = true, show_trail = true, noise_on = true, manual = false;
    bool thermals = false, wvane = true, fwd_thr = true, dyn_stall = true, rotor_fx = true;   // уточнення моделі й режими висіння у вітрі
    bool use_ekf = true;           // сирі датчики + EKF замість «правда + шум»
    bool shot_request = false;     // зробити знімок після рендера
    int shot_n = 0;
    // відтворення журналу CSV
    std::vector<FlightRow> replay;
    bool replay_on = false, replay_play = true;
    float replay_t = 0, replay_speed = 1;
    // редактор параметрів
    std::string edit_section = "vtol_motor", edit_msg;
    bool gamepad = false;
    bool relayout = true;
    int manual_mode = 0;   // fc::QpManual::Mode
    bool show_help = true;           // довідка відкрита при запуску
    bool show_report = false, report_done = false;   // звіт після польоту
    std::string report_text, report_file, report_html;
    double rep_x0 = 0, rep_x1 = 1;   // спільний відрізок часу на графіках звіту
    size_t rep_rows = 0;
    std::map<std::string, std::vector<double>> rep;   // ряди для графіків звіту (з rig->log)
    int terrain = 0;                 // 0 поле, 1 передмістя, 2 місто — показник профілю вітру
    float key_ramp = 2.5f;           // швидкість відхилення «ручки» клавішею, 1/с
    float alt_takeoff = 30, alt_cruise = 60, alt_hover = 40;   // висоти місії, м
    float hover_s = 20;
    struct Waypoint { float x, y, alt; };
    std::vector<Waypoint> route;    // свій маршрут: точки (світ, м; висота над домом); порожньо — прямо до цілі
    bool route_on = false;
    float payload_kg = 0;            // вантаж (автопілот про нього не знає — оцінює масу сам)              // скільки висіти над точкою, с
    // Трюки (вкладка «Трюки»): навчання ШІ у фоновому потоці, виконання в польоті
    int trick_sel = 0, trick_episodes = 100, train_total = 0;
    std::thread train_thread;
    std::atomic<bool> train_running{false}, train_cancel{false}, train_reload{false};
    std::atomic<int> train_ep{0};
    std::mutex train_mx;                                  // охороняє tr_* і train_msg
    std::vector<double> tr_x, tr_score, tr_best;
    std::string train_msg;
    float trick_u[fc::kTrickCount][fc::kTrickParams] = {};
    double trick_score[fc::kTrickCount] = {};
    bool trick_have[fc::kTrickCount] = {}, tricks_loaded = false;
    AirShow show;                       // виконання трюку або програми авіашоу в польоті
    bool show_was_running = false;
    bool train_continue = true;   // навчати далі з уже навченого (інакше — з нуля)
    bool train_robust = true;     // кожна спроба — ще й у вітрі з вантажем
    float real_speed = 1.0f;   // фактична швидкість моделі відносно реального часу
    std::string trick_result;
    std::unique_ptr<QpFlightRig> trick_base;   // той самий крейсерський старт, що й у навчанні (показ спроби в 3D)
    // «Мозок ШІ»
    bool show_brain = false, brain_full = true, show_forces = true;
    int brain_tab = 0, brain_tab_req = -1;
    struct BrainTrain {
        bool have = false;
        fc::Trick trick = fc::Trick::Loop;
        std::vector<TrickTrainer::Episode> eps;
        std::vector<TrickTrainer::Generation> gens;
        std::vector<std::string> thoughts;
        float mu[fc::kTrickParams] = {}, sd[fc::kTrickParams] = {};
        int best = -1;
    } br;                               // охороняє train_mx
    int view_ep = -1;
    bool replay_req = false; fc::Trick replay_trick = fc::Trick::Loop; float replay_u[fc::kTrickParams] = {};
    struct RateHist { std::deque<float> t, wc[3], w[3]; } rh;
    std::deque<Vec3d> trick_trail;
    struct AiHist { std::deque<float> t, pz, tz, px, tx, used, mz, trust; } aih;
    std::thread cmp_thread;
    std::atomic<bool> cmp_running{false};
    std::mutex cmp_mx;
    std::vector<float> cmp_t, cmp_off, cmp_on;
    double cmp_rms[2] = {0, 0};
    bool cmp_done = false;
    std::string cmp_msg;
    void loadTricks() { for (int k = 0; k < fc::kTrickCount; ++k) trick_have[k] = TrickTrainer::load(aiFile(file, "ai/tricks_learned.ini", true), fc::Trick(k), trick_u[k], &trick_score[k]); }
    ~App() { train_cancel = true; mc_cancel = true; if (train_thread.joinable()) train_thread.join(); if (cmp_thread.joinable()) cmp_thread.join(); if (mc_thread.joinable()) mc_thread.join(); }
    bool pilot_on = false;   // ШІ-пілот, навчений з нуля (ai/policy_hover.txt)
    bool pilot2_on = false;  // v2 (ai/policy_hover2.txt)
    int ai_mode = 0;   // ШІ «модель сил»: 0 — вимкнено, 1 — тінь (рахує, не впливає), 2 — радить контролеру
    bool energy_ai = true;   // ШІ-енергоменеджер: швидкість і висота крейсеру за вітром
    int rear_mode = -1, cruise_rear = -1;   // задні гвинти: 1 штовхальні / 0 тягнучі; у польоті: 0 передні, 1 усі, 2 задні на допомогу (−1 — як у файлі)
    bool sound_on = true; float volume = 0.7f;
    ImFont* big_font = nullptr;   // розкласти панелі заново (кнопка в «Камера й журнал»)

    // поза, яку показує 3D-вид: симуляція або рядок журналу
    QpState viewState() const {
        if (!replay_on || replay.empty()) return rig->sim.state();
        size_t i = 0; while (i + 1 < replay.size() && replay[i + 1].t <= replay_t) ++i;
        const FlightRow& r = replay[i];
        QpState st; st.pos = Vec3d{r.x, r.y, r.z}; st.q = Quatd::fromEuler(r.roll, r.pitch_down, r.yaw);
        return st;
    }
    bool loadReplay(const std::string& path) {
        std::ifstream f(path);
        if (!f) return false;
        std::string line; std::getline(f, line);
        std::vector<FlightRow> rows;
        while (std::getline(f, line)) {
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream ls(line);
            FlightRow r{};
            if (ls >> r.t >> r.x >> r.y >> r.z >> r.vx >> r.vy >> r.vz >> r.roll >> r.pitch_down >> r.yaw >> r.airspeed >> r.volts >> r.amps >> r.wv >> r.phase >> r.step) rows.push_back(r);
        }
        if (rows.size() < 2) return false;
        replay = std::move(rows); replay_t = replay.front().t; replay_on = true;
        return true;
    }
    float cam_az = 0.6f, cam_el = 0.35f, cam_dist = 6.0f;
    double prop_angle = 0;
    size_t plot_cache_rows = 0;
    std::vector<float> pt, pz, pvz, pva, pvg, proll, ppitch, ppower, pamps, psoc;
    std::string status = "Натисніть «Старт»";
    Vec3d home_pos;

    std::vector<fc::QpStep> buildMission() const {
        using S = fc::QpStep;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, alt_takeoff};
        S hv; hv.type = S::HoverAt; hv.p = {leg_km * 1000.0f, 0, alt_hover}; hv.hold_s = hover_s;   // висіння над точкою (огляд, камера)
        S ld; ld.type = S::Land; ld.p = {0, 0, 0};
        if (!(route_on && !route.empty())) return {tk, hv, ld};
        std::vector<S> m{tk};   // свій маршрут: літаком через точки, висіння над останньою, додому
        for (size_t i = 0; i + 1 < route.size(); ++i) { S f; f.type = S::FlyTo; f.p = {route[i].x, route[i].y, route[i].alt}; m.push_back(f); }
        hv.p = {route.back().x, route.back().y, alt_hover};
        m.push_back(hv); m.push_back(ld);
        return m;
    }

    // усе, що «Скинути» налаштовує в моделі з панелі «Місія й середовище» (також для перевірки місії на всіх ядрах)
    void setupRig(QpFlightRig& r, float wind_mps, float wind_deg, unsigned wind_seed, bool sensors) const {
        r.setMission(buildMission());
        WindConfig w; w.mean20ft = Vec3d{wind_mps * std::cos(wind_deg * kDeg), wind_mps * std::sin(wind_deg * kDeg), 0}; w.turbulence = turbulence;
        const double shear[3] = {0.14, 0.22, 0.30}; w.shear_exp = shear[std::clamp(terrain, 0, 2)];   // вітер росте з висотою: (h/6 м)^α
        if (thermals) {   // три термічні потоки вздовж маршруту (w* 2.5 м/с, шар 1000 м)
            for (float f : {0.3f, 0.55f, 0.8f}) { Thermal th; th.x = leg_km * 1000.0f * f; th.y = (f - 0.5f) * 200.0f; th.w_star = 2.5; th.zi = 1000; w.thermals.push_back(th); }
        }
        r.setWind(w, wind_seed);
        r.ctl.cfg.weathervane = wvane;
        r.ctl.cfg.fwd_pitch_lim = fwd_thr ? 3 * float(kDeg) : -1.0f;
        r.sim.aeroModel().setDynamicStall(dyn_stall);
        r.sim.setRotorEffects(rotor_fx, rotor_fx, rotor_fx);
        r.ai_mode = ai_mode;
        r.pilot_on = pilot_on && r.pilot.load(aiFile(file, "ai/policy_hover.txt"));
        r.pilot2_on = pilot2_on && r.pilot2.load(aiFile(file, "ai/policy_hover2.txt"));
        r.energy.on = energy_ai;
        r.ai_trace_on = true;
        r.sim.setPayload(payload_kg);
        r.ctl.cfg.cruise_alt = alt_cruise;   // висота польоту літаком
        r.noise = sensors ? SensorNoise::realistic() : SensorNoise::ideal();   // реальні датчики: сирі IMU/GPS/баро/компас → EKF, Піто, ESC, енкодери, затримки
        if (sensors && !use_ekf) { r.noise.use_ekf = false; }
    }
    // Перевірка місії в різних вітрах — одночасно на всіх ядрах
    struct McCase { float wind = 0, dir = 0; int state = 0; bool landed = false, crashed = false; float err = 0, wh = 0, time = 0, soc = 0; };
    std::vector<McCase> mc;
    std::thread mc_thread;
    std::atomic<bool> mc_running{false}, mc_cancel{false};
    std::mutex mc_mx;
    double mc_secs = 0; int mc_threads = 0; bool mc_open = false;

    void reset() {
        if (rig) show.stop(*rig);
        ai::EnergyManager learned; if (rig && !forget_learned) learned = rig->energy;   // виміряна крива потужності переживає «Скинути»
        ai::AnomalyMonitor healthy; if (rig && !forget_learned) healthy = rig->anomaly;   // і «здоровий» профіль апарата (але не при зміні апарата)
        forget_learned = false;
        if (rear_mode < 0) rear_mode = file.num("tilt.rear_pusher") > 0.5 ? 1 : 0;
        if (cruise_rear < 0) cruise_rear = int(file.num("tilt.cruise_rear") + 0.5);
        file.values["tilt.rear_pusher"] = {std::to_string(rear_mode)};
        file.values["tilt.cruise_rear"] = {std::to_string(cruise_rear)};
        rig = std::make_unique<QpFlightRig>(file, tables, 1u);
        rig->energy.importTable(learned);
        rig->anomaly.importProfile(healthy);
        setupRig(*rig, wind, wind_dir, 3, noise_on);
        trail.clear(); trick_trail.clear(); plot_cache_rows = 0; prop_angle = 0; aih = AiHist{}; rh = RateHist{};
        pt.clear(); pz.clear(); pvz.clear(); pva.clear(); pvg.clear(); proll.clear(); ppitch.clear(); ppower.clear(); pamps.clear(); psoc.clear();
        status = "Готово до польоту"; paused = true; show_report = false; report_done = false; report_file.clear(); report_html.clear(); rep_rows = 0;
    }
};

// ---------- GL допоміжні ----------
void perspective(double fovy, double aspect, double zn, double zf) {
    const double h = std::tan(fovy * kDeg / 2) * zn, w = h * aspect;
    glFrustum(-w, w, -h, h, zn, zf);
}
void lookAt(const Vec3d& e, const Vec3d& c, const Vec3d& up) {
    const Vec3d f = (c - e).normalized(), s = f.cross(up).normalized(), u = s.cross(f);
    const double m[16] = {s.x, u.x, -f.x, 0, s.y, u.y, -f.y, 0, s.z, u.z, -f.z, 0, 0, 0, 0, 1};
    glMultMatrixd(m);
    glTranslated(-e.x, -e.y, -e.z);
}
void bodyMatrix(const Quatd& q, const Vec3d& p) {
    const Vec3d x = q.rotate({1, 0, 0}), y = q.rotate({0, 1, 0}), z = q.rotate({0, 0, 1});
    const double m[16] = {x.x, x.y, x.z, 0, y.x, y.y, y.z, 0, z.x, z.y, z.z, 0, p.x, p.y, p.z, 1};
    glMultMatrixd(m);
}
void quad(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d, const float* fill, const float* line) {
    glColor3fv(fill);
    glBegin(GL_QUADS); glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z); glVertex3d(d.x, d.y, d.z); glEnd();
    glColor3fv(line);
    glBegin(GL_LINE_LOOP); glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z); glVertex3d(d.x, d.y, d.z); glEnd();
}
void tri(const Vec3d& a, const Vec3d& b, const Vec3d& c, const float* fill, const float* line) {
    glColor3fv(fill);
    glBegin(GL_TRIANGLES); glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z); glEnd();
    glColor3fv(line);
    glBegin(GL_LINE_LOOP); glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z); glEnd();
}
void line(const Vec3d& a, const Vec3d& b, const float* col, float w = 1.0f) {
    glLineWidth(w); glColor3fv(col);
    glBegin(GL_LINES); glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glEnd();
}
void disc(const Vec3d& c, const Vec3d& axis_x, const Vec3d& axis_y, double r, const float* col) {
    glColor4f(col[0], col[1], col[2], 0.35f);
    glBegin(GL_TRIANGLE_FAN); glVertex3d(c.x, c.y, c.z);
    for (int i = 0; i <= 24; ++i) { const double a = i * 2 * kPi / 24; const Vec3d p = c + axis_x * (r * std::cos(a)) + axis_y * (r * std::sin(a)); glVertex3d(p.x, p.y, p.z); }
    glEnd();
}

const float kWing[3] = {0.40f, 0.42f, 0.46f}, kBody[3] = {0.30f, 0.31f, 0.34f}, kTurret[3] = {0.12f, 0.13f, 0.15f}, kBlade[3] = {0.10f, 0.10f, 0.11f};
const float kBoom[3] = {0.22f, 0.23f, 0.25f}, kNac[3] = {0.55f, 0.56f, 0.60f};
// Примітиви з простим затіненням: яскравість грані — за кутом між її нормаллю (у світі) і напрямком світла
Quatd g_draw_q{1, 0, 0, 0};
void sface(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d, const float* col) {
    Vec3d n = (b - a).cross(d - a);
    if (n.norm() < 1e-12) n = (c - b).cross(a - b);
    const double nn = n.norm();
    const Vec3d wn = nn > 1e-12 ? g_draw_q.rotate(n * (1.0 / nn)) : Vec3d{0, 0, 1};
    const double l = 0.50 + 0.50 * std::fabs(wn.x * 0.35 + wn.y * 0.25 + wn.z * 0.90);
    glColor3f(float(col[0] * l), float(col[1] * l), float(col[2] * l));
    glBegin(GL_QUADS); glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z); glVertex3d(d.x, d.y, d.z); glEnd();
}
// тіло з перерізів-еліпсів у площині YZ (центри — уздовж довільної кривої), 12 граней
void loft(const std::vector<Vec3d>& cen, const std::vector<double>& ry, const std::vector<double>& rz, const float* col) {
    const int N = 12;
    for (size_t k = 0; k + 1 < cen.size(); ++k)
        for (int i = 0; i < N; ++i) {
            const double a0 = i * 2 * kPi / N, a1 = (i + 1) * 2 * kPi / N;
            auto pt = [&](size_t q, double a) { return cen[q] + Vec3d{0, ry[q] * std::cos(a), rz[q] * std::sin(a)}; };
            sface(pt(k, a0), pt(k, a1), pt(k + 1, a1), pt(k + 1, a0), col);
        }
}
// циліндр між двома точками
void cyl(const Vec3d& p0, const Vec3d& p1, double r, const float* col) {
    const Vec3d ax = p1 - p0;
    if (ax.norm() < 1e-9) return;
    const Vec3d u0 = std::fabs(ax.normalized().z) < 0.9 ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
    const Vec3d u = ax.cross(u0).normalized(), v = ax.cross(u).normalized();
    const int N = 8;
    for (int i = 0; i < N; ++i) {
        const double a0 = i * 2 * kPi / N, a1 = (i + 1) * 2 * kPi / N;
        const Vec3d d0 = u * (r * std::cos(a0)) + v * (r * std::sin(a0)), d1 = u * (r * std::cos(a1)) + v * (r * std::sin(a1));
        sface(p0 + d0, p0 + d1, p1 + d1, p1 + d0, col);
    }
}
void ball(const Vec3d& c, double r, const float* col) {
    std::vector<Vec3d> cen; std::vector<double> rr;
    for (int k = 0; k <= 6; ++k) { const double t = -kPi / 2 + k * kPi / 6; cen.push_back(c + Vec3d{r * std::sin(t), 0, 0}); rr.push_back(std::max(1e-4, r * std::cos(t))); }
    loft(cen, rr, rr, col);
}
// ділянка несучої поверхні з товщиною: від кореневої до кінцевої передньої кромки, хорди c0/c1 (уздовж −X), частка хорди x0…x1,
// up — нормаль поверхні, t — відносна товщина; d — відхилення (рад) навколо передньої межі ділянки (для рулів)
void panel(const Vec3d& le0, const Vec3d& le1, double c0, double c1, const Vec3d& up, double t, double x0, double x1, double d, const float* col) {
    static const double st[6] = {0.0, 0.06, 0.30, 0.60, 0.85, 1.0};
    auto th = [](double x) { return x < 0.30 ? std::sqrt(std::max(0.0, x) / 0.30) : (1.0 - x) / 0.70; };
    std::vector<double> xs{x0};
    for (double x : st) if (x > x0 + 1e-6 && x < x1 - 1e-6) xs.push_back(x);
    xs.push_back(x1);
    auto pt = [&](const Vec3d& le, double c, double x, double sgn) {
        const double s = (x - x0) * c, h = 0.5 * t * c * th(x) * sgn;
        return le + Vec3d{-x0 * c, 0, 0} + Vec3d{-s * std::cos(d), 0, 0} - up * (s * std::sin(d)) + up * h;
    };
    for (size_t k = 0; k + 1 < xs.size(); ++k)
        for (double sg : {1.0, -1.0})
            sface(pt(le0, c0, xs[k], sg), pt(le1, c1, xs[k], sg), pt(le1, c1, xs[k + 1], sg), pt(le0, c0, xs[k + 1], sg), col);
    for (size_t k = 0; k + 1 < xs.size(); ++k)   // торець на кінці
        sface(pt(le1, c1, xs[k], 1), pt(le1, c1, xs[k + 1], 1), pt(le1, c1, xs[k + 1], -1), pt(le1, c1, xs[k], -1), col);
}



// Модель конвертоплана (координати з tiltrotor.ini, відн. ЦМ): крило, фюзеляж, хвостова балка, V-хвіст, 4 гондоли, що повертаються
void drawAircraft(App& A) {
    const QuadPlaneSim& s = A.rig->sim;
    const AircraftFile& f = A.file;
    const Vec3d cg = Vec3d{s.derived().cg_x, s.derived().cg_y, s.derived().cg_z};
    auto P = [&](double x, double y, double z) { return Vec3d{x, y, z} - cg; };
    const double b = f.num("geometry.wing_span_m"), c = f.num("geometry.wing_chord_m"), le = s.derived().wing_le_x, wz = f.num("geometry.wing_z_m");
    glPushMatrix();
    const QpState vs = A.viewState();
    bodyMatrix(vs.q, vs.pos);
    g_draw_q = vs.q;
    const auto opt = [&](const char* k, double dflt) { return f.has(k) ? f.num(k) : dflt; };
    const double ct = opt("geometry.wing_tip_chord_m", c), lam = ct / c;
    const double cm = 2.0 / 3.0 * c * (1 + lam + lam * lam) / (1 + lam), xq = le - 0.25 * cm;
    auto wle = [&](double eta) { return xq + 0.25 * (c + (ct - c) * eta); };   // передня кромка на частці напіврозмаху
    auto wch = [&](double eta) { return c + (ct - c) * eta; };
    // крило: три ділянки по розмаху, середня — з елероном
    {
        const double in_f = opt("controls.aileron_inner_frac", 0.45), out_f = opt("controls.aileron_outer_frac", 0.95), af = opt("controls.aileron_chord_frac", 0.25);
        for (double sy : {1.0, -1.0}) {
            const double d = (sy > 0 ? s.servoAileron() : -s.servoAileron());
            const float col[3] = {0.9f, 0.5f + float(std::clamp(d * 2, -0.4, 0.4)), 0.2f};
            auto pt = [&](double eta) { return P(wle(eta), sy * eta * b / 2, wz); };
            panel(pt(0), pt(in_f), wch(0), wch(in_f), Vec3d{0, 0, 1}, 0.10, 0, 1, 0, kWing);
            panel(pt(in_f), pt(out_f), wch(in_f), wch(out_f), Vec3d{0, 0, 1}, 0.10, 0, 1 - af, 0, kWing);
            panel(pt(in_f), pt(out_f), wch(in_f), wch(out_f), Vec3d{0, 0, 1}, 0.10, 1 - af, 1, d, col);
            panel(pt(out_f), pt(1), wch(out_f), wch(1), Vec3d{0, 0, 1}, 0.10, 0, 1, 0, kWing);
        }
    }
    // корпус-гондола (тіло обертання), обтічник до хвостової балки, балка, купол камери
    const double nose = f.num("geometry.nose_x_m"), tail = f.num("geometry.tail_x_m");
    {
        const double R = opt("visual.fuselage_d_m", 0.11) / 2, xe = opt("visual.fuselage_end_x_m", -0.12), rb = f.num("geometry.boom_od_m") / 2;
        const double zc = opt("visual.fuselage_z_m", 0.0), L = nose - xe, xt = std::max(xe - 0.19, tail + 0.02);
        const double st[9][2] = {{0, 0.10}, {0.03, 0.42}, {0.08, 0.68}, {0.16, 0.90}, {0.28, 1.0}, {0.62, 1.0}, {0.85, 0.92}, {1.0, 0.78}, {-1, 0}};
        std::vector<Vec3d> cen; std::vector<double> ry, rz;
        for (int k = 0; st[k][0] >= 0; ++k) { cen.push_back(P(nose - st[k][0] * L, 0, zc)); ry.push_back(R * st[k][1]); rz.push_back(R * st[k][1]); }
        cen.push_back(P(xt, 0, 0.02 * 0 + opt("visual.boom_z_m", 0.01))); ry.push_back(rb); rz.push_back(rb);
        cen.push_back(P(tail, 0, opt("visual.boom_z_m", 0.01))); ry.push_back(rb); rz.push_back(rb);
        loft(cen, ry, rz, kBody);
        if (f.has("visual.turret_x_m")) ball(P(f.num("visual.turret_x_m"), 0, zc - R * 0.9), opt("visual.turret_d_m", 0.048) / 2, kTurret);
    }
    // кілі / V-хвіст: панелі під кутом; якщо є рулі — задня частина відхиляється (висота — однаково, напрям — різницево)
    {
        const double L = f.num("tail.vtail_panel_m"), tc = f.num("tail.vtail_chord_m"), tle = f.num("tail.vtail_le_x_m"), z0 = f.num("tail.vtail_root_z_m");
        const double g = f.num("tail.vtail_dihedral_deg") * kDeg, ef = std::max(opt("controls.elevator_chord_frac", 0.35), opt("controls.rudder_chord_frac", 0.35));
        const double tr = opt("visual.fin_root_chord_m", tc * 1.2), tt = opt("visual.fin_tip_chord_m", tc * 0.8);
        for (double sy : {1.0, -1.0}) {
            const Vec3d r0 = P(tle + 0.25 * (tr - tc), 0, z0), tip = r0 + Vec3d{0.25 * (tt - tr) , sy * L * std::cos(g), L * std::sin(g)};
            const Vec3d nrm{0, -sy * std::sin(g), std::cos(g)};
            const double d = s.servoElevator() * 1.0 + sy * s.servoRudder() * 1.0;
            const float rc[3] = {0.9f, 0.5f + float(std::clamp(d * 2, -0.4, 0.4)), 0.2f};
            panel(r0, tip, tr, tt, nrm, 0.09, 0, 1 - ef, 0, kWing);
            if (ef > 0.01) panel(r0, tip, tr, tt, nrm, 0.09, 1 - ef, 1, d, rc);
        }
    }
    // заднє крило / стабілізатор з рулем висоти
    const bool rwing = f.has("rear_wing.span_m");
    const double b2 = rwing ? f.num("rear_wing.span_m") : 0, c2 = rwing ? f.num("rear_wing.chord_m") : 0;
    const double le2 = rwing ? f.num("rear_wing.le_x_m") : 0, z2 = rwing ? f.num("rear_wing.z_m") : 0;
    if (rwing) {
        const double ef = f.num("rear_wing.elevator_chord_frac"), d = s.servoElevator();
        const float col[3] = {0.9f, 0.5f + float(std::clamp(d * 2, -0.4, 0.4)), 0.2f};
        for (double sy : {1.0, -1.0}) {
            panel(P(le2, 0, z2), P(le2, sy * b2 / 2, z2), c2, c2, Vec3d{0, 0, 1}, 0.09, 0, 1 - ef, 0, kWing);
            panel(P(le2, 0, z2), P(le2, sy * b2 / 2, z2), c2, c2, Vec3d{0, 0, 1}, 0.09, 1 - ef, 1, d, col);
        }
        if (std::fabs(z2 - opt("visual.boom_z_m", 0.01)) > 0.015) cyl(P(le2 - 0.4 * c2, 0, z2), P(le2 - 0.4 * c2, 0, opt("visual.boom_z_m", 0.01)), 0.006, kBoom);
    }
    // пілони: трубка від свого крила до осі повороту, з обтічною гондолою біля крила
    for (int i = 0; i < 4; ++i) {
        const Vec3d pv = s.pivot(i) + s.cgOffset();
        const bool rear = rwing && pv.x < 0;
        const double eta = std::min(1.0, std::fabs(pv.y) / (b / 2));
        const double xa = rear ? le2 - 0.5 * c2 : wle(eta) - 0.55 * wch(eta), za = (rear ? z2 : wz) - cg.z - 0.012;
        const Vec3d att{xa - cg.x, pv.y, za}, piv{pv.x, pv.y, pv.z};
        cyl(att, piv, opt("visual.pylon_d_m", 0.012) / 2, kBoom);
        const double dir = piv.x > att.x ? 1.0 : -1.0, fl = std::min(0.24, std::fabs(piv.x - att.x) * 0.8), fr = opt("visual.nacelle_d_m", 0.042) / 2;
        const Vec3d u = (piv - att).normalized();
        std::vector<Vec3d> cen{att - u * 0.03, att + u * (0.15 * fl), att + u * (0.6 * fl), att + u * fl};
        (void)dir;
        loft(cen, {fr * 0.3, fr, fr * 0.85, fr * 0.3}, {fr * 0.3, fr, fr * 0.85, fr * 0.3}, kBody);
    }
    if (f.has("ground.gear_points")) {
        const std::vector<double> gp = f.nums("ground.gear_points");
        for (size_t k = 0; k + 2 < gp.size(); k += 3) {
            const double top = std::min(gp[k + 2] + opt("geometry.gear_height_m", 0.2), (rwing && gp[k] < le2 ? z2 : wz));
            cyl(P(gp[k], gp[k + 1], top), P(gp[k], gp[k + 1], gp[k + 2]), 0.005, kBoom);
        }
    }
    // мотори й гвинти: мотор — циліндр уздовж осі тяги на фактичному куті; диск гвинта — з боку тяги (тягнучий) або позаду мотора (штовхальний)
    for (int i = 0; i < 4; ++i) {
        const Vec3d pv = s.pivot(i), ax = s.rotorAxis(i), disk = s.rotorPos(i);
        const double rp = s.vtolProp(i).diameter() / 2;
        const Vec3d side{0, 1, 0}, perp = ax.cross(side).normalized();   // друга вісь диска (у площині XZ)
        cyl(pv - (disk - pv).normalized() * 0.018, disk - (disk - pv).normalized() * 0.006, 0.014, kNac);
        cyl(disk - (disk - pv).normalized() * 0.006, disk + (disk - pv).normalized() * 0.012, 0.006, kBoom);
        const double rps = s.vtolRps(i), k = std::clamp(rps / 120.0, 0.0, 1.0);
        const float col[3] = {float(0.3 + 0.7 * k), float(0.8 - 0.5 * k), 0.3f};
        const double a = A.prop_angle * (i % 2 ? 1 : -1);
        if (rps > 5) disc(disk, side, perp, rp, col);
        const Vec3d u = side * std::cos(a) + perp * std::sin(a);
        if (rps <= 20) {   // лопаті видно, поки гвинт стоїть або ледве крутиться
            const Vec3d w = side * -std::sin(a) + perp * std::cos(a);
            for (double sg : {1.0, -1.0}) sface(disk + w * 0.008, disk + u * (sg * rp) + w * 0.004, disk + u * (sg * rp) - w * 0.004, disk - w * 0.008, kBlade);
        } else line(disk - u * rp, disk + u * rp, kBoom, 2);
        line(disk, disk + ax * (0.02 + 0.15 * k), col, 3);   // стрілка тяги
    }
    if (A.show_forces && !A.replay_on) {   // «Мозок ШІ»: тяга кожного гвинта, сила повітря на планер і передбачення ШІ (масштаб 4 см/Н)
        glDisable(GL_DEPTH_TEST);
        const double kN = 0.04;
        const float tc[3] = {0.35f, 1.0f, 0.45f}, ac[3] = {0.45f, 0.8f, 1.0f}, ic[3] = {1.0f, 0.35f, 0.95f};
        for (int i = 0; i < 4; ++i) {
            const Vec3d a0 = s.rotorPos(i), a1 = a0 + s.rotorAxis(i) * (s.vtolThrust(i) * kN);
            line(a0, a1, tc, 4);
        }
        const Vec3d F = s.aeroNow().force;
        line(Vec3d{0, 0, 0}, F * kN, ac, 5);
        const QpFlightRig& r = *A.rig;
        if (r.ai_mode > 0 && r.ai_trace_valid) line(Vec3d{0, 0, 0.01}, Vec3d{r.ai_trace.out[0], r.ai_trace.out[1], r.ai_trace.out[2]} * kN + Vec3d{0, 0, 0.01}, ic, 3);
        glEnable(GL_DEPTH_TEST);
    }
    glPopMatrix();
}

void drawScene(App& A, int w, int h) {
    glViewport(0, 0, w, h);
    glClearColor(0.45f, 0.62f, 0.85f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (A.show_brain && !A.brain_full) {   // «Мозок ШІ» праворуч — 3D-вид у вільній частині ліворуч від нього
        const float W = float(w), H = float(h), left = std::min(430.0f, W * 0.28f), right = std::min(400.0f, W * 0.24f), bottom = std::min(230.0f, H * 0.26f);
        const float bw = std::max(560.0f, (W - left - right) * 0.6f), x1 = W - right - bw;
        if (x1 - left > 120) { glViewport(int(left), int(bottom), int(x1 - left), int(H - bottom)); w = int(x1 - left); h = int(H - bottom); }
    }
    glEnable(GL_DEPTH_TEST); glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glDisable(GL_CULL_FACE);
    const QpState st = A.viewState();
    const Vec3d p = st.pos, yawDir{std::cos(st.q.euler().z), std::sin(st.q.euler().z), 0};
    Vec3d eye, target = p;
    switch (A.camera) {
        case 0: eye = p - yawDir * A.cam_dist * 1.2 + Vec3d{0, 0, A.cam_dist * 0.35}; break;
        case 1: eye = p + Vec3d{std::cos(A.cam_az) * std::cos(A.cam_el), std::sin(A.cam_az) * std::cos(A.cam_el), std::sin(A.cam_el)} * A.cam_dist; break;
        case 2: eye = Vec3d{-15, -10, 2}; break;
        case 3: eye = p + Vec3d{0, 0.001, std::max(30.0, double(A.cam_dist) * 8)}; break;
        default: eye = p + st.q.rotate(Vec3d{0.55, 0, 0.06}); target = p + st.q.rotate(Vec3d{30, 0, -3}); break;   // з борту: камера на носі
    }
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); perspective(50, double(w) / std::max(h, 1), 0.2, 8000);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity(); lookAt(eye, target, A.camera == 3 ? Vec3d{1, 0, 0} : (A.camera == 4 ? st.q.rotate(Vec3d{0, 0, 1}) : Vec3d{0, 0, 1}));
    // земля: заливка й сітка 10 м навколо апарата
    const float ground[3] = {0.28f, 0.45f, 0.26f}, gridc[3] = {0.33f, 0.52f, 0.30f};
    const double R = 1200, gx = std::floor(p.x / 10) * 10, gy = std::floor(p.y / 10) * 10;   // земля й сітка 10 м на 1.2 км навколо апарата
    quad({gx - R, gy - R, -0.02}, {gx + R, gy - R, -0.02}, {gx + R, gy + R, -0.02}, {gx - R, gy + R, -0.02}, ground, ground);
    glLineWidth(1);
    glColor3fv(gridc);
    glBegin(GL_LINES);
    for (double d = -R; d <= R; d += 10) { glVertex3d(gx + d, gy - R, 0); glVertex3d(gx + d, gy + R, 0); glVertex3d(gx - R, gy + d, 0); glVertex3d(gx + R, gy + d, 0); }
    glEnd();
    // позначки дому й цілі
    const float red[3] = {0.9f, 0.2f, 0.2f}, white[3] = {1, 1, 1};
    line({0, 0, 0}, {0, 0, 3}, white, 2);   // позначка дому (3 м)
    if (A.route_on && !A.route.empty()) {
        const float blue[3] = {0.4f, 0.6f, 1.0f};
        for (size_t i = 0; i < A.route.size(); ++i) line({A.route[i].x, A.route[i].y, 0}, {A.route[i].x, A.route[i].y, 40}, i + 1 < A.route.size() ? blue : red, 2);
    } else line({A.leg_km * 1000, 0, 0}, {A.leg_km * 1000, 0, 40}, red, 2);
    // слід
    if (A.replay_on && A.replay.size() > 1) {   // траєкторія журналу
        const float rc[3] = {0.4f, 0.9f, 1.0f};
        glLineWidth(2); glColor3fv(rc);
        glBegin(GL_LINE_STRIP); for (const FlightRow& r : A.replay) if (r.t <= A.replay_t) glVertex3d(r.x, r.y, r.z); glEnd();
    }
    if (A.show_trail && A.trail.size() > 1 && !A.replay_on) {
        const float tc[3] = {1.0f, 0.85f, 0.2f};
        glLineWidth(2); glColor3fv(tc);
        glBegin(GL_LINE_STRIP); for (const Vec3d& q : A.trail) glVertex3d(q.x, q.y, q.z); glEnd();
        glColor4f(0, 0, 0, 0.25f);   // тінь сліду
        glBegin(GL_LINE_STRIP); for (const Vec3d& q : A.trail) glVertex3d(q.x, q.y, 0.0); glEnd();
    }
    if (A.trick_trail.size() > 1 && !A.replay_on) {   // траєкторія трюків
        glLineWidth(3); glColor4f(1.0f, 0.35f, 0.9f, 0.9f);
        glBegin(GL_LINE_STRIP); for (const Vec3d& q : A.trick_trail) glVertex3d(q.x, q.y, q.z); glEnd();
    }
    // тінь апарата
    glColor4f(0, 0, 0, 0.35f);
    glBegin(GL_TRIANGLE_FAN); glVertex3d(p.x, p.y, 0.01);
    for (int i = 0; i <= 16; ++i) glVertex3d(p.x + 0.9 * std::cos(i * kPi / 8), p.y + 0.9 * std::sin(i * kPi / 8), 0.01);
    glEnd();
    // апарат — масштабується візуально на великих відстанях камери (щоб було видно на «згори»)
    const double vis = A.camera == 3 ? std::max(1.0, A.cam_dist / 4.0) : 1.0;
    glPushMatrix(); glTranslated(p.x, p.y, p.z); glScaled(vis, vis, vis); glTranslated(-p.x, -p.y, -p.z);
    drawAircraft(A);
    glPopMatrix();
}

// ---------- ImGui панелі ----------
// Кнопки, що завжди видно (над вкладками)
void topButtons(App& A) {
    const float bw = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3;
    if (ImGui::Button(A.paused ? "Старт" : "Пауза", ImVec2(bw, 0))) A.paused = !A.paused;
    ImGui::SameLine();
    if (ImGui::Button("Скинути", ImVec2(bw, 0))) A.reset();
    ImGui::SameLine();
    if (ImGui::Button("Записати CSV", ImVec2(bw, 0))) { A.status = A.rig->writeCsv("flight_log.csv") ? "Журнал записано: flight_log.csv" : "Не вдалось записати flight_log.csv"; }
    const float hb = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2;
    if (ImGui::Button(A.show_help ? "Сховати довідку (H)" : "Довідка (H)", ImVec2(hb, 0))) A.show_help = !A.show_help;
    ImGui::SameLine();
    if (ImGui::Button("Звіт про політ", ImVec2(hb, 0))) { A.report_text = A.rig->reportText(); A.rep_rows = 0; A.show_report = !A.show_report; }
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.32f, 0.75f, 1)); ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.65f, 0.42f, 0.85f, 1));
    if (ImGui::Button(A.show_brain ? "Сховати «Мозок ШІ» (B)" : "Мозок ШІ (B): що він думає і робить", ImVec2(-1, 0))) A.show_brain = !A.show_brain;
    ImGui::PopStyleColor(2);
    ImGui::SetNextItemWidth(-120);
    ImGui::SliderFloat("Швидкість часу", &A.time_scale, 0.25f, 16.0f, "x%.2f");
    if (!A.paused && A.real_speed < A.time_scale * 0.9f)
        ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Реально x%.1f — одне ядро процесора не встигає більше (вікно лишається плавним)", A.real_speed);
    ImGui::Checkbox("Звук", &A.sound_on); ImGui::SameLine(); ImGui::SetNextItemWidth(-60); ImGui::SliderFloat("##vol", &A.volume, 0.0f, 1.0f, "гучність %.1f");
    ImGui::TextWrapped("%s", A.status.c_str());
}

void startMissionCheck(App& A) {
    if (A.mc_running) return;
    if (A.mc_thread.joinable()) A.mc_thread.join();
    {
        std::lock_guard<std::mutex> l(A.mc_mx);
        A.mc.clear();
        App::McCase c0; A.mc.push_back(c0);
        for (float w : {3.0f, 6.0f, 9.0f, 12.0f}) for (float d : {180.0f, 0.0f, 90.0f, 270.0f}) { App::McCase c; c.wind = w; c.dir = d; A.mc.push_back(c); }
    }
    A.mc_running = true; A.mc_cancel = false;
    A.mc_threads = int(std::max(1u, std::thread::hardware_concurrency()));
    // копія апарата й моделі на момент запуску (тут, у потоці вікна): польоти не залежать від того, що користувач змінить на панелі
    struct Snap { AircraftFile file; AeroTables tables; std::vector<std::unique_ptr<QpFlightRig>> rigs; };
    auto snp = std::make_shared<Snap>(Snap{A.file, A.tables, {}});
    for (size_t i = 0; i < A.mc.size(); ++i) {
        snp->rigs.push_back(std::make_unique<QpFlightRig>(snp->file, snp->tables, 100u + unsigned(i)));
        A.setupRig(*snp->rigs.back(), A.mc[i].wind, A.mc[i].dir, 11u + unsigned(i), true);
        snp->rigs.back()->ai_trace_on = false;
    }
    A.mc_thread = std::thread([&A, snp]() {
        const auto t0 = std::chrono::steady_clock::now();
        Snap& sn = *snp;
        const size_t n = sn.rigs.size();
        std::atomic<size_t> next{0};
        std::vector<std::thread> pool;
        for (int k = 0; k < A.mc_threads; ++k) pool.emplace_back([&]() {
            for (size_t i; (i = next++) < n;) {
                { std::lock_guard<std::mutex> l(A.mc_mx); A.mc[i].state = 1; }
                QpFlightRig& r = *sn.rigs[i];
                r.run(3600, [&](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed || g.sim.crashed() || A.mc_cancel.load(); });
                const auto& p = r.sim.state().pos;
                std::lock_guard<std::mutex> l(A.mc_mx);
                App::McCase& c = A.mc[i];
                c.landed = r.ctl.phase() == fc::QpPhase::Landed; c.crashed = r.sim.crashed();
                c.err = float(std::hypot(p.x, p.y)); c.wh = float(r.sim.battery().usedWh()); c.time = float(r.sim.time()); c.soc = float(r.sim.battery().soc());
                c.state = A.mc_cancel ? 0 : 2;
                sn.rigs[i].reset();
            }
        });
        for (std::thread& th : pool) th.join();
        A.mc_secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        A.mc_running = false;
    });
}

void missionCheckUi(App& A) {
    if (A.mc_open) { ImGui::SetNextItemOpen(true); A.mc_open = false; }
    if (!ImGui::TreeNode("Перевірка місії в різних вітрах (на всіх ядрах)")) return;
    ImGui::TextDisabled("17 польотів цієї місії з реальними датчиками: штиль і вітер 3–12 м/с з чотирьох боків.");
    if (!A.mc_running) {
        if (ImGui::Button("Перевірити місію", ImVec2(-1, 0))) startMissionCheck(A);
    } else if (ImGui::Button("Зупинити перевірку", ImVec2(-1, 0))) A.mc_cancel = true;
    if (!A.mc_running && A.mc_thread.joinable()) A.mc_thread.join();
    std::lock_guard<std::mutex> l(A.mc_mx);
    if (!A.mc.empty()) {
        int done = 0, ok = 0; float wh_max = 0, err_max = 0;
        for (const App::McCase& c : A.mc) if (c.state == 2) { ++done; const bool good = c.landed && !c.crashed && c.err < 5; ok += good; if (good) { wh_max = std::max(wh_max, c.wh); err_max = std::max(err_max, c.err); } }
        if (A.mc_running) ImGui::Text("Летять: готово %d з %d (потоків: %d)", done, int(A.mc.size()), A.mc_threads);
        else if (done > 0) ImGui::TextColored(ok == done ? ImVec4(0.5f, 0.9f, 0.5f, 1) : ImVec4(1, 0.6f, 0.3f, 1), "Успішно %d з %d за %.0f с; найбільше %.0f Вт·год, посадка до %.1f м від дому", ok, done, A.mc_secs, wh_max, err_max);
        if (ImGui::BeginTable("mc", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("вітер"); ImGui::TableSetupColumn("проти"); ImGui::TableSetupColumn("у спину"); ImGui::TableSetupColumn("справа"); ImGui::TableSetupColumn("зліва");
            ImGui::TableHeadersRow();
            auto cell = [](const App::McCase& c) {
                if (c.state == 0) { ImGui::TextDisabled("—"); return; }
                if (c.state == 1) { ImGui::TextDisabled("летить…"); return; }
                if (c.crashed) { ImGui::TextColored(ImVec4(1, 0.35f, 0.3f, 1), "АВАРІЯ %.0f с", c.time); return; }
                if (!c.landed) { ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "не сів"); return; }
                const bool home = c.err < 5;
                ImGui::TextColored(home ? ImVec4(0.6f, 0.95f, 0.6f, 1) : ImVec4(1, 0.8f, 0.3f, 1), home ? "%.0f" : "за %.0f м", home ? c.wh : c.err);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("час %.0f с, витрачено %.1f Вт·год, заряд у кінці %.0f%%, посадка %.1f м від дому", c.time, c.wh, c.soc * 100, c.err);
            };
            ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::Text("штиль"); ImGui::TableNextColumn(); cell(A.mc[0]);
            for (size_t r = 0; r + 4 < A.mc.size(); r += 4) {
                ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::Text("%.0f м/с", A.mc[1 + r].wind);
                for (size_t k = 0; k < 4; ++k) { ImGui::TableNextColumn(); cell(A.mc[1 + r + k]); }
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Числа — витрачено Вт·год за місію; подробиці — під курсором.");
    }
    ImGui::TreePop();
}

void tabFlight(App& A) {
    ImGui::PushItemWidth(-150);   // місце для підписів праворуч
    if (ImGui::CollapsingHeader("Місія й середовище", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextDisabled("Діють після «Скинути»");
        ImGui::SliderFloat("До цілі, км", &A.leg_km, 0.3f, 5.0f, "%.1f");
        ImGui::Checkbox("Свій маршрут (карта: ліва кнопка — точка, права — прибрати останню)", &A.route_on);
        if (A.route_on) {
            if (A.route.empty()) ImGui::TextDisabled("Точок ще немає — апарат летить прямо до цілі.");
            int del = -1;
            for (size_t i = 0; i < A.route.size(); ++i) {
                ImGui::PushID(int(i));
                ImGui::Text("%d: %.0f, %.0f м", int(i) + 1, A.route[i].x, A.route[i].y); ImGui::SameLine();
                if (i + 1 < A.route.size()) { ImGui::SetNextItemWidth(110); ImGui::SliderFloat("висота, м", &A.route[i].alt, 30.0f, 200.0f, "%.0f"); ImGui::SameLine(); }
                else { ImGui::TextDisabled("(висіння)"); ImGui::SameLine(); }
                if (ImGui::SmallButton("x")) del = int(i);
                ImGui::PopID();
            }
            if (del >= 0) A.route.erase(A.route.begin() + del);
            if (!A.route.empty() && ImGui::Button("Очистити маршрут")) A.route.clear();
            ImGui::TextDisabled("Маршрут діє після «Скинути».");
        }
        {   // енергія маршруту до зльоту
            const double al[3] = {0.14, 0.22, 0.30}, k = std::pow(std::max(A.alt_cruise, 10.0f) / 6.1, al[std::clamp(A.terrain, 0, 2)]);
            const fc::Vec3 wv{float(A.wind * k * std::cos(A.wind_dir * kDeg)), float(A.wind * k * std::sin(A.wind_dir * kDeg)), 0};
            ai::MissionPlannerCfg pc; pc.batt_wh = A.rig->health.batt_wh;
            {   // апріорні потужності — з моделі цього апарата (до першого польоту з енергоменеджером)
                const auto& tv = A.rig->trim_v; const auto& tw = A.rig->trim_w;
                if (tv.size() >= 2) for (int i = 0; i < 10; ++i) {
                    const float v = 13.0f + i; size_t k = 1;
                    while (k + 1 < tv.size() && tv[k] < v) ++k;
                    const float f = std::clamp((v - tv[k - 1]) / std::max(tv[k] - tv[k - 1], 1e-3f), 0.0f, 1.5f);
                    pc.p_table[i] = tw[k - 1] + f * (tw[k] - tw[k - 1]);
                }
                if (A.rig->hover_w > 1) pc.fm_eta = ai::hoverPower(A.rig->ctl.cfg, float(A.rig->sim.mass()), 1.0f) / A.rig->hover_w;
            }
            fc::QpFlightConfig cfg = A.rig->ctl.cfg; cfg.cruise_alt = A.alt_cruise;
            const ai::MissionEnergy me = ai::planMissionEnergy(A.buildMission(), fc::Vec3{0, 0, 0}, wv, cfg, float(A.rig->sim.mass()), pc, &A.rig->energy);
            const ImVec4 col = me.fits ? (me.frac > 0.6f ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(0.5f, 0.9f, 0.5f, 1)) : ImVec4(1, 0.35f, 0.3f, 1);
            ImGui::TextColored(col, "Прогноз: %.0f Вт·год (%.0f%% батареї), ≈ %.0f хв%s", me.wh, me.frac * 100, me.minutes, me.fits ? "" : " — НЕ ВИСТАЧИТЬ із запасом 20%");
            if (A.route_on && A.route.size() > 1 && me.wh_reversed < 0.95f * me.wh) {
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1, 1), "Навпаки вийде %.0f Вт·год (−%.0f%%)", me.wh_reversed, 100 * (1 - me.wh_reversed / me.wh));
                ImGui::SameLine(); if (ImGui::SmallButton("Пройти навпаки")) std::reverse(A.route.begin(), A.route.end());
            }
            ImGui::TextDisabled(A.rig->energy.knownBins() >= 3 ? "Крейсерська потужність — виміряна в польоті." : "Крейсерська потужність — з моделі, точність ±25%%; після польоту з енергоменеджером — виміряна.");
        }
        ImGui::SliderFloat("Вітер на 6 м, м/с", &A.wind, 0.0f, 12.0f, "%.1f");
        ImGui::SliderFloat("Напрям вітру, °", &A.wind_dir, 0.0f, 360.0f, "%.0f");
        ImGui::SliderFloat("Турбулентність", &A.turbulence, 0.0f, 2.0f, "%.1f");
        const char* terr[] = {"поле, вода (α 0.14)", "передмістя, ліс (α 0.22)", "місто (α 0.30)"};
        ImGui::Combo("Місцевість", &A.terrain, terr, 3);
        {   const double al[3] = {0.14, 0.22, 0.30}, a = al[std::clamp(A.terrain, 0, 2)];
            ImGui::TextDisabled("Вітер за висотою: 6 м — %.1f, 30 м — %.1f, 60 м — %.1f, 100 м — %.1f м/с", A.wind, A.wind * std::pow(30 / 6.1, a), A.wind * std::pow(60 / 6.1, a), A.wind * std::pow(100 / 6.1, a)); }
        if (ImGui::TreeNodeEx("Висоти", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Зліт, м", &A.alt_takeoff, 10.0f, 240.0f, "%.0f");
            ImGui::SliderFloat("Політ літаком, м", &A.alt_cruise, 30.0f, 300.0f, "%.0f");
            ImGui::SliderFloat("Висіння над ціллю, м", &A.alt_hover, 15.0f, 240.0f, "%.0f");
            ImGui::TreePop();
        }
        ImGui::SliderFloat("Висіти над ціллю, с", &A.hover_s, 0.0f, 120.0f, "%.0f");
        ImGui::SliderFloat("Вантаж, кг", &A.payload_kg, 0.0f, 1.5f, "%.2f");
        ImGui::SameLine(); ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Автопілот про вантаж не знає: сам оцінює масу на зльоті й у висінні.\nОцінку видно в «Стан» і в «Мозку ШІ».");
        {   // схема гвинтів — порівняння варіантів
            const char* rm[] = {"тягнучі (гвинт над мотором)", "штовхальні (гвинт під мотором)"};
            ImGui::Combo("Задні гвинти", &A.rear_mode, rm, 2);
            const char* cr[] = {"лише передні (задні стоять)", "усі 4", "передні, задні — на допомогу"};
            ImGui::Combo("У польоті тягнуть", &A.cruise_rear, cr, 3);
        }
        ImGui::Checkbox("Термічні потоки на маршруті", &A.thermals);
        missionCheckUi(A);
        if (A.leg_km > 3.0f) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "Довга місія: збільште швидкість часу.");
    }
    if (ImGui::CollapsingHeader("Модель і автопілот")) {
        ImGui::TextDisabled("Діють після «Скинути»");
        ImGui::Checkbox("Реальні датчики (шум, затримки, Піто, ESC)", &A.noise_on);
        ImGui::Checkbox("EKF на сирих датчиках (2 IMU, GPS, баро, компас)", &A.use_ekf);
        ImGui::Checkbox("Висіння носом проти вітру (Q_WVANE)", &A.wvane);
        ImGui::Checkbox("Тяга нахилом гондол у висінні (Q_FWD_THR_USE)", &A.fwd_thr);
        ImGui::Checkbox("ШІ-енергоменеджер: швидкість і висота за вітром", &A.energy_ai);
        ImGui::Checkbox("Динамічний зрив", &A.dyn_stall);
        ImGui::Checkbox("Ефект землі, вихрове кільце, слід гвинтів", &A.rotor_fx);
        {   // ШІ-співпроцесор: модель сил (діє одразу, без «Скинути»)
            const char* am[] = {"вимкнено", "тінь — рахує, не впливає", "радить контролеру"};
            if (ImGui::Combo("ШІ: модель сил", &A.ai_mode, am, 3)) A.rig->ai_mode = A.ai_mode;
            if (ImGui::Checkbox("ШІ-пілот, навчений з нуля (утримання точки)", &A.pilot_on)) {
                A.rig->pilot_on = A.pilot_on && A.rig->pilot.load(aiFile(A.file, "ai/policy_hover.txt"));
                if (A.pilot_on && !A.rig->pilot_on) A.pilot_on = false;
            }
            if (A.rig->pilot_on) ImGui::TextDisabled("покоління %d, на відкладених умовах %+.0f%% до звичайного; порада %.1f м/с²", A.rig->pilot.generation, 100 * A.rig->pilot.score, A.rig->pilot.acc.norm());
            else ImGui::TextDisabled("ваги — train_pilot.bat (ai/policy_hover.txt)");
            if (ImGui::Checkbox("ШІ-пілот v2 (+ повітряна швидкість, оберти моторів, кут носа)", &A.pilot2_on)) {
                A.rig->pilot2_on = A.pilot2_on && A.rig->pilot2.load(aiFile(A.file, "ai/policy_hover2.txt"));
                if (A.pilot2_on && !A.rig->pilot2_on) A.pilot2_on = false;
                if (A.pilot2_on) { A.pilot_on = false; A.rig->pilot_on = false; }
            }
            if (A.rig->pilot2_on) ImGui::TextDisabled("v2: покоління %d, %+.0f%%; порада %.1f м/с², ніс %+.1f°", A.rig->pilot2.generation, 100 * A.rig->pilot2.score, A.rig->pilot2.acc.norm(), A.rig->pilot2.pitch_up * 57.3f);
            if (!A.rig->advisor.net.loaded()) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "Ваг немає: ai/weights_force.h (tools/ai_train.py)");
            else if (A.ai_mode > 0 && A.rig->ai_stats.n > 0)
                ImGui::TextDisabled("Передбачення сили: похибка %.1f Н при силі %.1f Н (висіння)", A.rig->aiRmsError(), A.rig->aiRmsTruth());
        }
    }
    if (ImGui::CollapsingHeader("Ручне керування", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Checkbox("Увімкнути ручне керування", &A.manual) && A.manual) A.paused = false;
        const char* modes[] = {"F1 Асистований (QLOITER)", "F2 QSTABILIZE — коптер, кути", "F3 QACRO — коптер, кутові швидкості", "F4 FBWA — літак, кути", "F5 MANUAL — літак, рулі напряму"};
        ImGui::Combo("Режим", &A.manual_mode, modes, 5);
        const fc::QpManual& m = A.rig->ctl.manual;
        if (A.manual_mode == 0) {
            ImGui::TextDisabled("W/S вперед-назад, A/D вбік, Q/E поворот, R/F вгору-вниз, Shift швидше");
            ImGui::TextDisabled("Автопілот тримає точку й висоту, коли клавіші відпущено");
        } else {
            ImGui::TextDisabled("W/S тангаж (W — ніс униз), A/D крен, Q/E курс");
            ImGui::TextDisabled(A.manual_mode >= 3 ? "R/F — газ (гондоли вперед, тримається), X — газ 0; V — назад у коптер (F2)"
                                                   : "R/F — тяга вгору (тримається), T/G — гондоли вперед/назад (G нижче нуля — гальмування), X — усе 0; V — літак (F4)");
            ImGui::ProgressBar(m.throttle, ImVec2(-1, 0), A.manual_mode >= 3 ? "газ уперед" : "тяга вгору");
            if (A.manual_mode < 3) {
                char pb[64]; std::snprintf(pb, sizeof pb, m.pusher >= 0 ? "гондоли вперед %.0f%%" : "гондоли НАЗАД %.0f%% (гальмування)", 100 * std::fabs(m.pusher));
                ImGui::ProgressBar(std::fabs(m.pusher), ImVec2(-1, 0), pb);
            }
            if (A.manual_mode == 2) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "QACRO не вирівнює апарат — горизонт тримаєте ви");
            if (A.manual_mode == 4) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "MANUAL: без стабілізації; гондоли вперед, тягнуть як літак");
            if (A.manual_mode >= 3) ImGui::TextDisabled("Перехід руками: QSTABILIZE + T (нахил гондол уперед) до ≈ 11 м/с, потім V (або F4/F5)");
        }
        if (A.manual_mode > 0 && ImGui::TreeNode("Чутливість (як на пульті)")) {
            fc::QpManual& mm = A.rig->ctl.manual;
            ImGui::SliderFloat("Експо (м'якість біля центру)", &mm.expo, 0.0f, 0.8f, "%.2f");
            ImGui::SliderFloat("Макс. нахил коптера, °", &mm.max_angle_deg, 10.0f, 40.0f, "%.0f");
            ImGui::SliderFloat("QACRO: макс. кутова швидкість, °/с", &mm.acro_rate_dps, 60.0f, 360.0f, "%.0f");
            ImGui::SliderFloat("FBWA: макс. крен, °", &mm.fbwa_bank_deg, 15.0f, 60.0f, "%.0f");
            ImGui::SliderFloat("FBWA: макс. тангаж, °", &mm.fbwa_pitch_deg, 10.0f, 30.0f, "%.0f");
            ImGui::SliderFloat("Клавіші: швидкість ручки, 1/с", &A.key_ramp, 0.5f, 8.0f, "%.1f");
            ImGui::TextDisabled("Менше — плавніше. Повне відхилення ручки за %.1f с", 1.0f / A.key_ramp);
            ImGui::TreePop();
        }
        ImGui::TextDisabled("Вимкнути ручне в повітрі — автопілот поверне додому й посадить");
        ImGui::TextDisabled(A.gamepad ? "Геймпад (режим 2): ліва — газ/курс, права — крен/тангаж, курок — нахил гондол уперед, Y — режим" : "Геймпад не знайдено");
    }
    if (ImGui::CollapsingHeader("Камера й журнал", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char* cams[] = {"За апаратом", "Орбітальна (миша)", "З землі", "Згори", "З борту (ніс)"};
        ImGui::Combo("Камера", &A.camera, cams, 5);
        ImGui::SliderFloat("Відстань камери", &A.cam_dist, 2.0f, 80.0f, "%.0f м");
        ImGui::Checkbox("Слід траєкторії", &A.show_trail);
        if (ImGui::Button("Знімок екрана")) A.shot_request = true;
        ImGui::SameLine();
        if (ImGui::Button("Розкласти панелі")) A.relayout = true;
        ImGui::SameLine();
        if (ImGui::Button(A.replay_on ? "Закрити відтворення" : "Відтворити журнал")) {
            if (A.replay_on) A.replay_on = false;
            else A.status = A.loadReplay("flight_log.csv") ? "Відтворення журналу flight_log.csv" : "Не вдалось прочитати flight_log.csv (спершу «Записати CSV»)";
        }
        if (A.replay_on) {
            ImGui::SliderFloat("Час журналу, с", &A.replay_t, A.replay.front().t, A.replay.back().t, "%.1f");
            ImGui::Checkbox("Грати", &A.replay_play); ImGui::SameLine(); ImGui::SliderFloat("##rp", &A.replay_speed, 0.25f, 16.0f, "x%.2f");
        }
    }
    ImGui::PopItemWidth();
}

void panelState(App& A) {
    const QuadPlaneSim& s = A.rig->sim;
    ImGui::Begin("Стан");
    ImGui::Text("Режим: %s  (крок місії %zu/3)", A.rig->ctl.statusText(), A.rig->ctl.stepIndex() + 1);
    ImGui::Text("Вага коптерних моторів: %.0f%%   Q_ASSIST: %s", 100 * A.rig->ctl.vtolWeight(), A.rig->ctl.assisting() ? "ТАК" : "ні");
    {   // аварійні стани контролера (відмови, обмеження струму)
        const std::string al = A.rig->ctl.alertText();
        if (!al.empty()) ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1), "УВАГА: %s", al.c_str());
        if (A.rig->noise.use_ekf && A.rig->nav.ready()) {
            const fc::Eskf& L = A.rig->nav.primary();
            ImGui::Text("EKF: основний IMU №%d (перемикань %d), узгодженість %.2f, σ позиції %.1f м, VIBE %.0f м/с²%s", A.rig->nav.primaryIndex() + 1, A.rig->nav.laneSwitches(),
                        L.testRatio(), L.posSigma(), A.rig->sensors.vibrationPeak(), A.rig->nav.imuBad(0) || A.rig->nav.imuBad(1) ? "  — IMU відбраковано" : "");
        }
        if (A.rig->ctl.liftMotorFailed() >= 0) ImGui::Text("Обертання по рисканню: %.0f°/с (%.1f об/с)", s.state().omega.z / kDeg, s.state().omega.z / (2 * 3.14159265358979));
    }
    ImGui::Separator();
    ImGui::Text("Маса: %.3f кг (вантаж %.2f)   Задні гвинти: %s", s.mass(), s.payload(), s.rearPusher() ? "штовхальні" : "тягнучі");
    ImGui::Text("Автопілот оцінює масу: %.2f кг (%+.0f%%)", A.rig->ctl.massEstimate(), 100.0 * (A.rig->ctl.massEstimate() / s.mass() - 1.0));
    {
        fc::QpAutotune& tn = A.rig->ctl.tune;
        if (tn.state == fc::QpAutotune::Run) ImGui::Text("Самоналаштування: коло %d, %.0f%% (крен і тангаж смикаються навмисно)", tn.rounds, tn.progress() * 100);
        else {
            if (ImGui::SmallButton("Самоналаштування регуляторів (у висінні)")) A.rig->ctl.startAutotune();
            if (tn.state == fc::QpAutotune::Done) { ImGui::SameLine(); ImGui::Text("готово: ефективність крену ×%.2f, тангажу ×%.2f (кіл %d)", tn.total[0], tn.total[1], tn.rounds); }
        }
    }
    {
        fc::QpNavigator& nv = A.rig->nav;
        ImGui::Checkbox("Автопошук власної частоти рами (режектор)", &nv.struct_detect);
        if (nv.struct_notch_hz[0] + nv.struct_notch_hz[1] > 1) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "рама тремтить: крен %.1f Гц, тангаж %.1f Гц — режектор увімкнено", nv.struct_notch_hz[0], nv.struct_notch_hz[1]);
        }
    }
    ImGui::Text("Армування: %s", A.rig->ctl.armed() ? "армовано" : (A.rig->ctl.preflightFailures() ? fc::preflightText(A.rig->ctl.preflightFailures()).c_str() : "очікує"));
    if (s.tiltEnabled()) {
        ImGui::Text("Приводи гондол, момент Н·м / °C:");
        for (int i = 0; i < 4; ++i) { ImGui::SameLine(); ImGui::Text("%.2f/%.0f", s.tiltServo(i).load(), s.tiltServo(i).temperature()); }
    }
    ImGui::Text("Рулі, момент Н·м: %.3f %.3f %.3f", s.surfaceServo(0).load(), s.surfaceServo(1).load(), s.surfaceServo(2).load());
    {
        const ai::HealthReport& h = A.rig->health.r;
        ImGui::Text("Діагностика: гвинти %.3f %.3f %.3f %.3f%s%s", h.motor_ratio[0], h.motor_ratio[1], h.motor_ratio[2], h.motor_ratio[3],
                    h.prop_damaged >= 0 ? "  ПОШКОДЖЕНО гвинт " : "", h.prop_damaged >= 0 ? std::to_string(h.prop_damaged + 1).c_str() : "");
        if (h.servo_slow >= 0) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "Привід гондоли %d повільний: %.0f°/с", h.servo_slow + 1, h.tilt_rate[h.servo_slow] * 57.3f);
        if (h.wind_ok) ImGui::Text("Вітер (оцінка борту): %.1f м/с з %.0f°", std::hypot(h.wind[0], h.wind[1]), std::fmod(std::atan2(-h.wind[1], -h.wind[0]) * 57.3f + 360.0f, 360.0f));
        if (h.cg_ok) ImGui::Text("Центр мас: %+.0f мм від розрахункового", h.cg_x_mm); else ImGui::TextDisabled("Центр мас: оцінюється у висінні в тиху погоду");
        const ai::AnomalyMonitor& an = A.rig->anomaly;
        if (!an.base.ready) ImGui::TextDisabled("Передбачення поломок: вивчає «здоровий» апарат (перші 90 с польоту)");
        else {
            ImGui::Text("Проти норми: вібрації ×%.2f, опір батареї ×%.2f (%.0f мОм)", an.vibRatio(), an.rRatio(), an.r_batt * 1000);
            if (an.vib_bad) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "Вібрації зросли ×%.1f — перевір гвинти й кріплення моторів%s", an.vibRatio(), an.vib_motor >= 0 ? (std::string(", схоже мотор ") + std::to_string(an.vib_motor + 1)).c_str() : "");
            if (an.batt_bad) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "Опір батареї ×%.1f від норми — старіння, комірка або роз'єм", an.rRatio());
            if (an.servo_bad >= 0) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "Привід гондоли %d: розбіжність %.1f° (норма %.1f°) — люфт або знос", an.servo_bad + 1, an.servo_err[size_t(an.servo_bad)] * 57.3f, an.base.servo_err[size_t(an.servo_bad)] * 57.3f);
        }
        ImGui::Text("Енергія: %.0f Вт·год (≈ %.0f хв), додому треба %.0f — %s", h.wh_left, h.minutes_left, h.wh_home, h.can_return ? "вистачає" : "НЕ ВИСТАЧАЄ");
        const ai::EnergyManager& em = A.rig->energy;
        if (em.on) {
            if (em.v_best > 0) ImGui::Text("ШІ-енергоменеджер: %s %.0f м/с, висота %.0f м, %.2f Вт·год/км", em.exploring ? "вивчає швидкість" : "найвигідніше", em.v_best, em.alt_best, em.wh_per_km);
            ImGui::Text("Вивчена потужність, Вт:");
            for (int i = 0; i < ai::EnergyManager::kBins; ++i) { float t = 0; const float p = em.binPower(i, &t); if (t > 5) { ImGui::SameLine(); ImGui::Text("%.0f м/с %.0f", em.binSpeed(i), p); } }
        }
    }
    ImGui::Text("Гондоли, °: FR %.0f  RL %.0f  FL %.0f  RR %.0f   (0 — вгору, 90 — уперед)", s.tilt(0) / kDeg, s.tilt(1) / kDeg, s.tilt(2) / kDeg, s.tilt(3) / kDeg);
    ImGui::Separator();
    const double soc = s.battery().soc(), P = s.busPower();
    ImGui::Text("Батарея: %.1f В  %.1f А  %.0f Вт  SoC %.0f%%  витрачено %.1f Вт·год  T %.0f °C", s.busVoltage(), s.busCurrent(), P, 100 * soc, s.battery().usedWh(), s.battery().temperature());
    ImGui::ProgressBar(float(soc), ImVec2(-1, 0));
    const double tleft = s.busCurrent() > 0.5 ? soc * s.battery().capacityAh() * 3600 / s.busCurrent() : 0;
    ImGui::Text("Залишок при поточному струмі: %.0f хв", tleft / 60);
    ImGui::Separator();
    ImGui::Text("Мотори, об/хв: FR %.0f  RL %.0f  FL %.0f  RR %.0f", 60 * s.vtolRps(0), 60 * s.vtolRps(1), 60 * s.vtolRps(2), 60 * s.vtolRps(3));
    ImGui::Text("Тяга гвинтів, Н: %.1f %.1f %.1f %.1f", s.vtolThrust(0), s.vtolThrust(1), s.vtolThrust(2), s.vtolThrust(3));
    ImGui::Text("Рулі: елерон %+.1f°  V-хвіст: висота %+.1f°  напрям %+.1f°", s.servoAileron() / kDeg, s.servoElevator() / kDeg, s.servoRudder() / kDeg);
    ImGui::Separator();
    const Vec3d w = s.windWorld();
    ImGui::Text("Вітер біля апарата (%.0f м): %.1f м/с (%+.1f, %+.1f, %+.1f)   ρ %.3f кг/м³", s.state().pos.z, w.norm(), w.x, w.y, w.z, s.density());
    ImGui::End();
}

// Відмови: вмикаються під час польоту, скидаються кнопкою «Скинути»
void tabFaults(App& A) {
    QpFaults& f = A.rig->sim.faults;
    SensorFaults& sf = A.rig->sfaults;
    const char* mn[4] = {"Мотор FR (передній правий)", "Мотор RL (задній лівий)", "Мотор FL (передній лівий)", "Мотор RR (задній правий)"};
    for (int i = 0; i < 4; ++i) {
        bool dead = f.vtol_scale[i] < 0.01, part = !dead && f.vtol_scale[i] < 0.99;
        if (ImGui::Checkbox((std::string(mn[i]) + ": вимкнено").c_str(), &dead)) f.vtol_scale[i] = dead ? 0.0 : 1.0;
        ImGui::SameLine();
        if (ImGui::Checkbox((std::string("60%##") + std::to_string(i)).c_str(), &part)) f.vtol_scale[i] = part ? 0.6 : 1.0;
    }
    for (int i = 0; i < 4; ++i) ImGui::Checkbox((std::string("Привід гондоли ") + mn[i] + ": заклинило").c_str(), &f.tilt_stuck[i]);
    for (int i = 0; i < 4; ++i) {
        bool dmg = f.prop_scale[i] < 0.99;
        if (ImGui::Checkbox((std::string("Гвинт ") + mn[i] + ": відламано кінчик (−15% тяги)").c_str(), &dmg)) f.prop_scale[i] = dmg ? 0.85 : 1.0;
    }
    for (int i = 0; i < 4; ++i) {
        bool slow = A.rig->sim.tiltServo(i).spec().rate < 0.9 * A.rig->ctl.cfg.tilt_rate;
        if (ImGui::Checkbox((std::string("Привід гондоли ") + mn[i] + ": зношений (×0.4 швидкості)").c_str(), &slow)) A.rig->sim.tiltServo(i).spec().rate = A.rig->ctl.cfg.tilt_rate * (slow ? 0.4 : 1.0);
    }
    for (int i = 0; i < 4; ++i) {
        bool vb = A.rig->sensors.motor_vib[size_t(i)] > 1.5;
        if (ImGui::Checkbox((std::string("Гвинт ") + mn[i] + ": розбалансований (вібрації ×3)").c_str(), &vb)) A.rig->sensors.motor_vib[size_t(i)] = vb ? 3.0 : 1.0;
        bool bl = A.rig->sim.tiltServo(i).spec().backlash > 0.01;
        if (ImGui::Checkbox((std::string("Привід гондоли ") + mn[i] + ": люфт ×6").c_str(), &bl)) A.rig->sim.tiltServo(i).spec().backlash = (bl ? 6.0 : 1.0) * 0.3 * 3.14159265 / 180;
    }
    {
        bool soft = A.rig->sim.flexScale() > 1.5;
        if (ImGui::Checkbox("Рама м'яка (пружні коливання ×8)", &soft)) A.rig->sim.setFlexScale(soft ? 8.0 : 1.0);
    }
    ImGui::Separator();
    ImGui::Checkbox("Елерон заклинило", &f.servo_stuck[0]);
    ImGui::Checkbox("Кермо висоти заклинило", &f.servo_stuck[1]);
    ImGui::Checkbox("Кермо напрямку заклинило", &f.servo_stuck[2]);
    ImGui::Separator();
    ImGui::Checkbox("GPS втрачено", &sf.gps_lost);
    ImGui::Checkbox("Труба Піто мертва (0 м/с)", &sf.pitot_dead);
    ImGui::Checkbox("Зник зв'язок з пультом", &sf.rc_lost);
    bool weak = f.battery_r_scale > 1.5;
    if (ImGui::Checkbox("Слабка батарея (опір ×2, ємність ×0.8)", &weak)) { f.battery_r_scale = weak ? 2.0 : 1.0; f.battery_cap_scale = weak ? 0.8 : 1.0; }
    ImGui::Separator();
    if (A.rig->noise.use_ekf) {
        const char* imf[] = {"справний", "стрибок біасу гіро", "стрибок біасу акселерометра", "вихід завис", "мертвий"};
        int f0 = int(A.rig->sensors.imu_fault[0]);
        if (ImGui::Combo("IMU №1", &f0, imf, 5)) A.rig->sensors.imu_fault[0] = ImuFault(f0);
        float vib = float(A.rig->sensors.vib_scale);
        if (ImGui::SliderFloat("Вібрації ×", &vib, 0.0f, 10.0f, "%.1f")) A.rig->sensors.vib_scale = vib;
    } else ImGui::TextDisabled("Відмови IMU й вібрації — з увімкненим EKF (панель «Керування»).");
    ImGui::TextDisabled("GPS: апарат переходить у коптер і сідає вертикально.");
    ImGui::TextDisabled("Мотор: у висінні — посадка з обертанням (вітер до ≈ 4 м/с), у польоті — посадка літаком.");
    ImGui::TextDisabled("Привід гондоли: виявляється датчиком положення — перехід переривається, посадка коптером або літаком.");
}

// Редактор параметрів апарата: числові параметри обраного розділу tiltrotor.ini (окрім [mass] і таблиць).
// «Застосувати» — новий політ із цими параметрами (перевірка deriveAircraft); «Зберегти» — у файл з резервною копією, коментарі зберігаються.
bool saveIni(const AircraftFile& f, const std::string& path) {
    std::ifstream in(path);
    if (!in) return false;
    std::vector<std::string> lines;
    for (std::string l; std::getline(in, l);) lines.push_back(l);
    in.close();
    { std::ofstream bak(path + ".bak"); for (const std::string& l : lines) bak << l << "\n"; }
    std::string section;
    for (std::string& l : lines) {
        const size_t h = l.find('#');
        const std::string code = l.substr(0, h);
        std::istringstream ls(code);
        std::string key, eq, val;
        if (!(ls >> key)) continue;
        if (key.front() == '[') { section = key.substr(1, key.size() - 2); continue; }
        if (section == "mass" || !(ls >> eq >> val) || eq != "=") continue;
        std::string rest; ls >> rest;
        if (!rest.empty()) continue;   // таблиці (кілька чисел) не редагуються
        auto it = f.values.find(section + "." + key);
        if (it == f.values.end() || it->second.size() != 1 || it->second[0] == val) continue;
        const size_t pos = code.find(val, code.find('=') + 1);
        if (pos == std::string::npos) continue;
        l = l.substr(0, pos) + it->second[0] + l.substr(pos + val.size());   // лише значення; ключ і коментар без змін
    }
    std::ofstream out(path);
    if (!out) return false;
    for (const std::string& l : lines) out << l << "\n";
    return true;
}

// Трюки (sim/qp_tricks.h): ШІ вчиться у фоновому потоці на цьому комп'ютері; «Мозок ШІ» показує, що він пробує і що вирішує
std::string generationThought(fc::Trick t, const TrickTrainer::Generation& g) {
    const fc::TrickParamSpec* sp = fc::trickSpec(t);
    char b[1200];
    int n = std::snprintf(b, sizeof b, "Покоління %d (спроби %d–%d): виконано %d з %d, аварій %d, середня оцінка %.0f, три найкращі: %.0f, %.0f, %.0f.",
                          g.index, (g.index - 1) * g.size + 1, g.index * g.size, g.completed, g.size, g.crashed, g.mean_score, g.elite_scores[0], g.elite_scores[1], g.elite_scores[2]);
    int most = 0, sure = 0, unsure = 0;
    float best_shift = -1, min_sd = 2, max_sd = -1;
    for (int i = 0; i < fc::kTrickParams; ++i) {
        const float d = std::fabs(g.mu_after[i] - g.mu_before[i]);
        if (d > best_shift) { best_shift = d; most = i; }
        if (g.sd_after[i] < min_sd) { min_sd = g.sd_after[i]; sure = i; }
        if (g.sd_after[i] > max_sd) { max_sd = g.sd_after[i]; unsure = i; }
    }
    auto val = [&](int i, float u) { return sp[i].lo + std::clamp(u, 0.0f, 1.0f) * (sp[i].hi - sp[i].lo); };
    auto spread = [&](int i, float s) { return s * (sp[i].hi - sp[i].lo); };
    if (best_shift > 0.02f)
        n += std::snprintf(b + n, sizeof b - size_t(n), "\n  → Найбільше змінюю «%s»: %.2f → %.2f (у кращих спроб було в середньому %.2f).",
                           sp[most].name, val(most, g.mu_before[most]), val(most, g.mu_after[most]), val(most, g.elite_mean[most]));
    else n += std::snprintf(b + n, sizeof b - size_t(n), "\n  → Кращі спроби близькі до того, що я вже думав, — майже нічого не міняю.");
    n += std::snprintf(b + n, sizeof b - size_t(n), "\n  → Найвпевненіший щодо «%s»: %.2f ± %.2f.", sp[sure].name, val(sure, g.mu_after[sure]), spread(sure, g.sd_after[sure]));
    if (max_sd > 0.12f) n += std::snprintf(b + n, sizeof b - size_t(n), "\n  → Ще шукаю «%s»: пробуватиму %.2f ± %.2f.", sp[unsure].name, val(unsure, g.mu_after[unsure]), spread(unsure, g.sd_after[unsure]));
    else n += std::snprintf(b + n, sizeof b - size_t(n), "\n  → Усі параметри вже звузились — доводжу до точності.");
    return b;
}

void startTrickTraining(App& A, fc::Trick t) {
    if (A.train_thread.joinable()) A.train_thread.join();
    {
        std::lock_guard<std::mutex> l(A.train_mx);
        A.tr_x.clear(); A.tr_score.clear(); A.tr_best.clear(); A.train_msg = "Готую крейсерський політ…";
        A.br = App::BrainTrain{}; A.br.have = true; A.br.trick = t;
    }
    const bool cont = A.train_continue && A.trick_have[int(t)];
    std::array<float, fc::kTrickParams> start_u{};
    for (int i = 0; i < fc::kTrickParams; ++i) start_u[size_t(i)] = A.trick_u[int(t)][i];
    {
        std::lock_guard<std::mutex> l(A.train_mx);
        for (int i = 0; i < fc::kTrickParams; ++i) { A.br.mu[i] = cont ? start_u[size_t(i)] : 0.5f; A.br.sd[i] = cont ? 0.12f : 0.3f; }
        if (cont) {
            char b[300];
            std::snprintf(b, sizeof b, "Продовжую з того, що вже вмію (оцінка минулого навчання %.0f). Спершу перелітаю навчене (спроба 0) — це планка, нижче якої не опущусь. Далі шукаю поруч із вузьким розкидом.", A.trick_score[int(t)]);
            A.br.thoughts.push_back(b);
        } else A.br.thoughts.push_back("Починаю з нуля: нічого не знаю про цей трюк, тож усі параметри — посередині діапазону з великим розкидом. Перші 10 спроб — майже навмання.");
    }
    A.view_ep = -1;
    A.train_cancel = false; A.train_running = true; A.train_ep = 0; A.train_total = A.trick_episodes;
    const AircraftFile f = A.file;
    const AeroTables tb = A.tables;
    const int n = A.trick_episodes;
    const unsigned seed = unsigned(std::time(nullptr));
    const int robust = A.train_robust ? 3 : 0;
    A.train_thread = std::thread([&A, f, tb, n, t, seed, cont, start_u, robust]() {
        TrickTrainer tr(f, tb);
        tr.robust = robust;
        tr.threads = std::max(1, int(std::thread::hardware_concurrency()) - 1);   // одне ядро лишається вікну
        tr.record_traj = true;
        tr.cancel = [&A] { return A.train_cancel.load(); };
        tr.on_episode = [&A](const TrickTrainer::Episode& e, const TrickTrainer::Episode& best) {
            std::lock_guard<std::mutex> l(A.train_mx);
            A.tr_x.push_back(e.index); A.tr_score.push_back(e.r.score); A.tr_best.push_back(best.r.score);
            A.br.eps.push_back(e);
            if (best.index == e.index) A.br.best = int(A.br.eps.size()) - 1;
            char b[400];
            std::snprintf(b, sizeof b, "Спроба %d: %s\nНайкраща — спроба %d, оцінка %.0f", e.index, e.r.text().c_str(), best.index, best.r.score);
            A.train_msg = b;
            A.train_ep = e.index;
        };
        tr.on_generation = [&A, t](const TrickTrainer::Generation& g) {
            std::lock_guard<std::mutex> l(A.train_mx);
            A.br.gens.push_back(g);
            for (int i = 0; i < fc::kTrickParams; ++i) { A.br.mu[i] = g.mu_after[i]; A.br.sd[i] = g.sd_after[i]; }
            A.br.thoughts.push_back(generationThought(t, g));
        };
        const TrickTrainer::Episode best = tr.train(t, n, seed, cont ? start_u.data() : nullptr);
        if (best.index == 0 && cont) {
            std::lock_guard<std::mutex> l(A.train_mx);
            char b[300];
            std::snprintf(b, sizeof b, "\nНавчання завершено: жодна нова спроба не обігнала вже навчене (оцінка %.0f) — лишаю його.", best.r.score);
            A.train_msg += b;
            A.br.thoughts.push_back("Нічого кращого за те, що вже вмів, не знайшов — лишаю навчене. Можна спробувати більше спроб або навчання з нуля.");
        } else if (best.index > 0 && TrickTrainer::save(aiFile(A.file, "ai/tricks_learned.ini"), t, best.u, best.r.score)) {
            int ok = 0; for (const auto& e : tr.history) ok += e.r.completed;
            std::lock_guard<std::mutex> l(A.train_mx);
            char b[300];
            std::snprintf(b, sizeof b, "\nНавчання завершено: %zu спроб, вдалих %d. Найкраща (спроба %d) збережена в ai/tricks_learned.ini.", tr.history.size(), ok, best.index);
            A.train_msg += b;
            A.br.thoughts.push_back(std::string("Готово. Запам'ятав найкращу спробу №") + std::to_string(best.index) + " — її параметри і використовуватиму в польоті.");
        }
        A.train_reload = true;
        A.train_running = false;
    });
}

const float* trickParamsFor(App& A, fc::Trick t) { return A.trick_have[int(t)] ? A.trick_u[int(t)] : defaultTrickParams(t); }

void startShow(App& A, std::vector<fc::Trick> prog) {
    for (int k = 0; k < fc::kTrickCount; ++k) { const float* u = trickParamsFor(A, fc::Trick(k)); for (int i = 0; i < fc::kTrickParams; ++i) A.show.u[k][i] = u[i]; }
    A.show.begin(*A.rig, std::move(prog));
    A.show_was_running = true;
    A.trick_result.clear();
    A.rh = App::RateHist{};
}

// Повторити спробу навчання в 3D: той самий крейсерський старт, що в навчанні, і ті самі параметри — апарат пролетить її точно так само
void replayEpisode(App& A, fc::Trick t, const float u[fc::kTrickParams]) {
    if (A.train_running) return;
    if (!A.trick_base) { TrickTrainer tr(A.file, A.tables); A.trick_base = tr.cruiseRig(); }
    A.show.stop(*A.rig);
    A.rig = std::make_unique<QpFlightRig>(*A.trick_base);
    A.rig->ai_trace_on = true;
    A.trail.clear(); A.plot_cache_rows = 0;
    A.pt.clear(); A.pz.clear(); A.pvz.clear(); A.pva.clear(); A.pvg.clear(); A.proll.clear(); A.ppitch.clear(); A.ppower.clear(); A.pamps.clear(); A.psoc.clear();
    A.report_done = true;
    for (int k = 0; k < fc::kTrickCount; ++k) for (int i = 0; i < fc::kTrickParams; ++i) A.show.u[k][i] = u[i];
    A.show.begin(*A.rig, {t});
    A.show_was_running = true; A.trick_result.clear(); A.rh = App::RateHist{};
    A.paused = false;
    A.status = std::string("Показ спроби: ") + fc::trickName(t);
}

void tabTricks(App& A) {
    if (!A.tricks_loaded || A.train_reload.exchange(false)) { A.loadTricks(); A.tricks_loaded = true; }
    if (!A.train_running && A.train_thread.joinable()) A.train_thread.join();
    const char* names[fc::kTrickCount];
    for (int i = 0; i < fc::kTrickCount; ++i) names[i] = fc::trickName(fc::Trick(i));
    if (A.train_running) ImGui::BeginDisabled();
    ImGui::Combo("Трюк", &A.trick_sel, names, fc::kTrickCount);
    if (A.train_running) ImGui::EndDisabled();
    const fc::Trick t = fc::Trick(A.trick_sel);
    const int k = A.trick_sel;
    ImGui::TextWrapped("%s", fc::trickAbout(t));
    if (ImGui::Button(A.show_brain ? "Сховати «Мозок ШІ» (B)" : "Відкрити «Мозок ШІ» (B) — що він думає", ImVec2(-1, 0))) A.show_brain = !A.show_brain;

    ImGui::SeparatorText("Навчання");
    ImGui::TextWrapped("ШІ пробує трюк у симуляторі, оцінює кожну спробу й кожні 10 спроб бере 3 найкращі за основу для наступних. "
                       "Рахує цей комп'ютер: 10 спроб покоління — одночасно на всіх ядрах, крім одного (воно лишається вікну). Що він пробує й чому — у «Мозку ШІ».");
    ImGui::TextDisabled("Ядер процесора: %u", std::thread::hardware_concurrency());
    if (!A.train_running) {
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("спроб", &A.trick_episodes, 10, 50);
        A.trick_episodes = std::clamp(A.trick_episodes, 10, 2000);
        ImGui::SameLine();
        if (ImGui::Button(A.train_continue && A.trick_have[k] ? "Довчити" : "Навчити")) { startTrickTraining(A, t); A.show_brain = true; A.brain_tab_req = 0; }
        if (A.trick_have[k]) {
            ImGui::Checkbox("Продовжити з навченого (не з нуля)", &A.train_continue);
            ImGui::Checkbox("Перевіряти кожну спробу ще й у вітрі з вантажем", &A.train_robust);
            ImGui::TextDisabled(A.train_continue ? "Шукає поруч із тим, що вже вміє; гіршим за навчене не стане — навчене перелітається першим і лишається, якщо нове не краще."
                                                 : "З нуля: усе навчене забувається, пошук знову з усього діапазону. Збережеться найкраще з нового навчання.");
        }
    } else {
        char b[64]; std::snprintf(b, sizeof b, "%d / %d", A.train_ep.load(), A.train_total);
        ImGui::ProgressBar(float(A.train_ep.load()) / float(std::max(A.train_total, 1)), ImVec2(-90, 0), b);
        ImGui::SameLine();
        if (ImGui::Button("Зупинити")) A.train_cancel = true;
    }
    {
        std::lock_guard<std::mutex> l(A.train_mx);
        if (!A.train_msg.empty()) ImGui::TextWrapped("%s", A.train_msg.c_str());
    }

    ImGui::SeparatorText("Що вивчено");
    if (!A.trick_have[k]) ImGui::TextDisabled("Ще не навчено — у польоті будуть запасні параметри.");
    else ImGui::Text("Оцінка останнього навчання: %.0f (100 — ідеально)", A.trick_score[k]);
    if (ImGui::BeginTable("tp", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
        const float* u = trickParamsFor(A, t);
        for (int i = 0; i < fc::kTrickParams; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(fc::trickSpec(t)[i].name);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", fc::trickValue(t, u, i));
        }
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Виконати в польоті");
    const bool cruise = A.rig->ctl.phase() == fc::QpPhase::Cruise && A.rig->ctl.vtolWeight() == 0 && !A.paused;
    const bool high = A.rig->sim.state().pos.z > 45;
    if (A.show.running()) {
        if (A.show.inTrick()) ImGui::Text("Виконується: %s — %s, %.1f с", fc::trickName(A.show.pilot.trick()), A.show.pilot.phaseText(), A.show.pilot.time());
        else ImGui::Text("Авіашоу: автопілот вирівнює політ перед наступним трюком (%.0f с)…", A.show.waitTime());
        if (ImGui::Button("Зупинити", ImVec2(-1, 0))) A.show.stop(*A.rig);
    } else {
        const bool can = cruise && high;
        if (!can) ImGui::BeginDisabled();
        if (ImGui::Button("Виконати трюк", ImVec2(-1, 0))) { startShow(A, {t}); A.show_brain = true; A.brain_tab_req = 1; }
        if (ImGui::Button("АВІАШОУ АСА: усі 8 трюків поспіль", ImVec2(-1, 34))) {
            startShow(A, airShowProgram());
            A.show_brain = true; A.brain_tab_req = 1;
        }
        if (!can) ImGui::EndDisabled();
        if (!cruise) ImGui::TextDisabled("Натисніть «Старт» і дочекайтесь польоту літаком (фаза «літак»).");
        else if (!high) ImGui::TextDisabled("Замало висоти: потрібно щонайменше 45 м.");
        int fallback = 0; for (int i = 0; i < fc::kTrickCount; ++i) fallback += !A.trick_have[i];
        if (fallback) ImGui::TextDisabled("Не навчено трюків: %d — для них запасні параметри.", fallback);
    }
    if (!A.trick_result.empty()) ImGui::TextWrapped("Результат: %s", A.trick_result.c_str());
}

#include "brain_view.h"

void tabParams(App& A) {
    {   // вибір апарата: усі aircraft/*.ini; вибір запам'ятовується в aircraft/active.txt (його читають і інструменти)
        std::vector<std::string> files;
        std::error_code ec;
        for (const auto& de : std::filesystem::directory_iterator("aircraft", ec)) if (de.path().extension() == ".ini") files.push_back("aircraft/" + de.path().filename().string());
        std::sort(files.begin(), files.end());
        if (ImGui::BeginCombo("Апарат", A.ac_path.c_str())) {
            for (const std::string& fp : files) if (ImGui::Selectable(fp.c_str(), fp == A.ac_path) && fp != A.ac_path) {
                try {
                    AircraftFile nf = AircraftFile::load(fp);
                    A.ac_path = fp; A.file = nf; A.rear_mode = -1; A.cruise_rear = -1; A.forget_learned = true; A.tricks_loaded = false;
                    { std::ofstream o("aircraft/active.txt"); o << fp.substr(fp.find('/') + 1) << "\n"; }
                    A.reset(); A.edit_msg = "Апарат: " + fp + " (ШІ навчений на іншому апараті — поради ШІ поки краще не вмикати)";
                } catch (const std::exception& e) { A.edit_msg = std::string("Не вдалось відкрити: ") + e.what(); }
            }
            ImGui::EndCombo();
        }
    }
    std::vector<std::string> sections;
    for (const auto& kv : A.file.values) {
        const std::string sec = kv.first.substr(0, kv.first.find('.'));
        if (sections.empty() || sections.back() != sec) sections.push_back(sec);
    }
    if (ImGui::BeginCombo("Розділ", A.edit_section.c_str())) {
        for (const std::string& sec : sections) if (ImGui::Selectable(sec.c_str(), sec == A.edit_section)) A.edit_section = sec;
        ImGui::EndCombo();
    }
    ImGui::BeginChild("params", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() * 2 - 30), true);
    for (auto& kv : A.file.values) {
        if (kv.first.rfind(A.edit_section + ".", 0) != 0 || kv.second.size() != 1) continue;
        char* end = nullptr;
        double v = std::strtod(kv.second[0].c_str(), &end);
        if (end == kv.second[0].c_str() || *end) { ImGui::TextDisabled("%s = %s", kv.first.c_str() + A.edit_section.size() + 1, kv.second[0].c_str()); continue; }
        if (ImGui::InputDouble(kv.first.c_str() + A.edit_section.size() + 1, &v, 0, 0, "%.6g")) {
            char buf[64]; std::snprintf(buf, sizeof buf, "%.6g", v); kv.second[0] = buf;
        }
    }
    ImGui::EndChild();
    if (ImGui::Button("Застосувати (новий політ)")) {
        const AircraftDerived d = deriveAircraft(A.file);
        A.edit_msg = d.problems.empty() ? "Застосовано" : "Застосовано, але порушено: " + d.problems.front();
        try { A.reset(); } catch (const std::exception& e) { A.edit_msg = std::string("Помилка: ") + e.what() + " — параметри відновлено з файлу"; A.file = AircraftFile::load(A.ac_path); A.reset(); }
    }
    ImGui::SameLine();
    if (ImGui::Button(("Зберегти в " + A.ac_path.substr(A.ac_path.find_last_of("/\\") + 1)).c_str())) A.edit_msg = saveIni(A.file, A.ac_path) ? "Збережено (резервна копія: .bak)" : "Не вдалось записати файл";
    ImGui::SameLine();
    if (ImGui::Button("Відновити з файлу")) { A.file = AircraftFile::load(A.ac_path); A.reset(); A.edit_msg = "Прочитано з файлу"; }
    ImGui::TextWrapped("%s", A.edit_msg.c_str());
}

// Ліва панель: кнопки + вкладки «Політ / Відмови / Параметри»
void panelLeft(App& A) {
    ImGui::Begin("Керування");
    topButtons(A);
    if (ImGui::BeginTabBar("tabs")) {
        if (ImGui::BeginTabItem("Політ")) { ImGui::BeginChild("tf"); tabFlight(A); ImGui::EndChild(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Відмови")) { ImGui::BeginChild("tv"); tabFaults(A); ImGui::EndChild(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Трюки")) { ImGui::BeginChild("tt"); tabTricks(A); ImGui::EndChild(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Параметри")) { tabParams(A); ImGui::EndTabItem(); }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

// ---------- Довідка ----------
void helpLine(const char* k, const char* d) { ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextColored(ImVec4(1, 0.85f, 0.4f, 1), "%s", k); ImGui::TableNextColumn(); ImGui::TextWrapped("%s", d); }
void step(int n, const char* t) { ImGui::TextColored(ImVec4(0.5f, 0.85f, 1, 1), "%d.", n); ImGui::SameLine(); ImGui::TextWrapped("%s", t); }

void panelHelp(App& A, float W, float H) {
    if (!A.show_help) return;
    ImGui::SetNextWindowSize(ImVec2(std::min(760.0f, W * 0.55f), std::min(620.0f, H * 0.75f)), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImVec2(W * 0.5f, H * 0.45f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Довідка: як керувати", &A.show_help);
    if (ImGui::BeginTabBar("help")) {
        if (ImGui::BeginTabItem("1. Перший політ")) {
            ImGui::TextWrapped("Почніть з автоматичної місії — апарат усе зробить сам, а ви дивитесь, як він літає.");
            ImGui::Separator();
            step(1, "Натисніть «Старт» (ліва панель угорі). Апарат злетить вертикально на 30 м.");
            step(2, "Далі сам: гондоли нахиляються вперед → перехід у літак → політ до цілі (червона позначка на карті) → гондоли назад, висіння над ціллю → повернення → посадка.");
            step(3, "Щоб не чекати — повзунок «Швидкість часу» (x2…x8).");
            step(4, "Камера: вкладка «Політ» → «Камера й журнал». Мишею тягніть по 3D-виду — камера обертається; коліщатко — ближче/далі. «З борту» — вигляд з носа.");
            step(5, "«Скинути» — почати знову. Вітер, відстань до цілі, EKF тощо змінюються у вкладці «Політ» і діють після «Скинути».");
            ImGui::Separator();
            ImGui::TextWrapped("Поверх 3D-виду завжди є підказка: що зараз відбувається і що натискати.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("2. Ручний політ")) {
            ImGui::TextColored(ImVec4(0.6f, 1, 0.6f, 1), "Найпростіше — F1 «Асистований» (як дрон DJI)");
            step(1, "Вкладка «Політ» → «Ручне керування» → поставте галочку «Увімкнути». Режим F1.");
            step(2, "Тримайте R — апарат піднімається. Відпустіть на 10–20 м — він зависне сам.");
            step(3, "W/S — вперед/назад, A/D — вліво/вправо, Q/E — поворот. Відпустили клавіші — стоїть на місці.");
            step(4, "Тримайте F — знижується й сідає.");
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 0.85f, 0.4f, 1), "Як справжній коптер — F2 QSTABILIZE");
            step(1, "Тепер R/F — це газ, і він ЛИШАЄТЬСЯ там, де ви відпустили (як ручка газу на пульті). Висіння ≈ 28%.");
            step(2, "Зліт: тримайте R до ≈ 35%, коли піднявся — F до ≈ 28%. Висоту тримаєте ви: стежте за «Варіометр» на PFD (має бути ≈ 0).");
            step(3, "W/A/S/D нахиляють апарат (він сам вирівнюється, коли відпустити), Q/E — поворот. Вітер зносить — компенсуйте нахилом.");
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 0.85f, 0.4f, 1), "Перехід у літак руками");
            step(1, "У F2 наберіть ≥ 30 м і тримайте рівно.");
            step(2, "Тримайте T — гондоли нахиляються вперед, гвинти тягнуть уперед. Підказка зверху показує швидкість: чекайте 11 м/с.");
            step(3, "Натисніть F4 (FBWA) — гондоли стають на 90°, тепер це літак. Газ (R/F) — тяга вперед.");
            step(4, "У літаку: S — ніс угору (набір), W — ніс униз, A/D — крен і поворот, Q/E — кермо напряму. Не опускайтесь нижче 10 м/с — звалювання!");
            step(5, "Назад у коптер: скиньте газ (F), сповільніться до < 10 м/с і натисніть F2 — або просто зніміть галочку «Увімкнути»: автопілот сам поверне додому й посадить.");
            ImGui::Separator();
            ImGui::TextWrapped("F3 QACRO і F5 MANUAL — для досвідчених: апарат не вирівнюється сам, горизонт тримаєте ви.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("3. Клавіші")) {
            if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
                ImGui::TableSetupColumn("Клавіша", ImGuiTableColumnFlags_WidthFixed, 150);
                helpLine("H", "показати / сховати цю довідку");
                helpLine("F1 … F5", "режим ручного керування: асистований, QSTABILIZE, QACRO, FBWA, MANUAL");
                helpLine("V", "перемикач коптер ↔ літак: QSTABILIZE (F2) ↔ FBWA (F4)");
                helpLine("W / S", "F1: вперед / назад.  Інші: тангаж — W ніс униз, S ніс угору");
                helpLine("A / D", "F1: вліво / вправо.  Інші: крен ліворуч / праворуч");
                helpLine("Q / E", "поворот (курс) ліворуч / праворуч");
                helpLine("R / F", "F1: вгору / вниз.  Інші: газ більше / менше (тримається)");
                helpLine("T / G", "F2, F3: гондоли вперед (розгін і перехід у літак) / назад — нижче нуля гальмування");
                helpLine("X", "газ у нуль (аварійно)");
                helpLine("Shift", "F1: швидше");
                helpLine("Миша", "тягнути — обертати камеру, коліщатко — відстань");
                helpLine("Геймпад", "ліва ручка — газ і курс, права — крен і тангаж, правий курок — нахил гондол уперед, Y — наступний режим");
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("4. Що на екрані")) {
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 1, 1), "PFD (праворуч угорі) — головний прилад:");
            ImGui::TextWrapped("синє — небо, коричневе — земля; лінія між ними — горизонт, жовта позначка — ваш літак. Нахил горизонту — крен, зсув угору/вниз — тангаж (шкала через 10°). "
                               "Під ним: V возд. — швидкість відносно повітря (для крила; літак падає нижче ≈ 10 м/с), V земн. — відносно землі, курс, висота, варіометр (швидкість підйому, + вгору).");
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 1, 1), "Карта (праворуч унизу):");
            ImGui::TextWrapped("вигляд згори: білий — дім, червоний — ціль, лінія — маршрут і ваша траєкторія.");
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 1, 1), "Стан (ліворуч унизу):");
            ImGui::TextWrapped("режим польоту, червоний рядок «УВАГА» (відмови, низька напруга), маса й кути гондол, батарея (напруга, струм, заряд), оберти й тяга кожного мотора, рулі, вітер.");
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 1, 1), "Графіки (знизу):");
            ImGui::TextWrapped("висота й Vz, швидкості, кути, потужність і струм за весь політ.");
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 1, 1), "Звук:");
            ImGui::TextWrapped("гул чотирьох гвинтів (висота тону — оберти), шум вітру, писк при тривозі. Вимикається галочкою «Звук».");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("5. Відмови")) {
            ImGui::TextWrapped("Вкладка «Відмови» ламає апарат прямо в польоті — так перевіряють, що він переживе на практиці.");
            step(1, "Запустіть місію («Старт»), дочекайтесь висіння або польоту літаком.");
            step(2, "Поставте галочку, наприклад «Мотор FR: вимкнено». Апарат виявить відмову за обертами й почне аварійну посадку (з обертанням у висінні або літаком у крейсері).");
            step(3, "Що відбувається — у червоному рядку «УВАГА» на панелі «Стан» і в підказці зверху; звучить сигнал.");
            ImGui::TextWrapped("Також: GPS, труба Піто, рулі, слабка батарея; з увімкненим EKF — відмови IMU й вібрації. Знімаються кнопкою «Скинути».");
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

// ---------- Підказка поверх 3D-виду: що відбувається і що натискати ----------
void coach(App& A, float x0, float x1) {
    const QuadPlaneSim& s = A.rig->sim;
    const fc::QpManual& m = A.rig->ctl.manual;
    const float z = float(s.state().pos.z), v = float(s.airspeed()), vz = float(s.velWorld().z);
    std::string title, hint; ImVec4 col(1, 1, 1, 1);
    char b[256];
    if (s.crashed()) { title = "АВАРІЯ"; hint = "Натисніть «Скинути», щоб почати знову"; col = ImVec4(1, 0.3f, 0.3f, 1); }
    else if (!A.manual) {
        if (A.paused) { title = "Автоматична місія"; hint = "Натисніть «Старт» — апарат сам злетить, долетить до цілі, повисить над нею й повернеться.  H — довідка"; }
        else { std::snprintf(b, sizeof b, "Автопілот: %s", A.rig->ctl.statusText()); title = b; hint = "Хочете самі? «Політ» → «Ручне керування» → «Увімкнути».  H — довідка"; }
    } else {
        static const char* names[] = {"F1 Асистований", "F2 QSTABILIZE (коптер)", "F3 QACRO (коптер, без вирівнювання)", "F4 FBWA (літак)", "F5 MANUAL (літак, рулі напряму)"};
        title = names[A.manual_mode];
        if (A.manual_mode == 0) {
            if (z < 0.6f) hint = "Тримайте R — зліт";
            else { std::snprintf(b, sizeof b, "Висота %.0f м.  W/S/A/D — рух, Q/E — поворот, R/F — вгору/вниз. Відпустіть — зависне", z); hint = b; }
        } else if (A.manual_mode <= 2) {
            if (z < 0.6f && m.throttle < 0.3f) { std::snprintf(b, sizeof b, "Газ %.0f%%. Тримайте R до ≈ 35%%, щоб злетіти (висіння ≈ 28%%)", 100 * m.throttle); hint = b; }
            else {
                std::snprintf(b, sizeof b, "Газ %.0f%%  висота %.0f м  Vz %+.1f м/с.  R/F — газ, W/S/A/D — нахил, Q/E — поворот", 100 * m.throttle, z, vz); hint = b;
                if (m.pusher > 0.02f) {
                    std::snprintf(b, sizeof b, "\nТяга вперед %.0f%%  швидкість %.1f з 11 м/с — %s", 100 * m.pusher, v, v >= 11 ? "МОЖНА V (F4) — літак!" : "ще рано для літака");
                    hint += b; if (v >= 11) col = ImVec4(0.5f, 1, 0.5f, 1);
                } else if (z > 25) hint += "\nДля переходу в літак тримайте T (гондоли вперед)";
            }
        } else {
            std::snprintf(b, sizeof b, "Швидкість %.1f м/с  висота %.0f м  газ %.0f%%.  S — ніс угору, W — униз, A/D — віраж, R/F — газ", v, z, 100 * m.throttle); hint = b;
            if (v < 9.5f && z > 1) { hint = std::string("ЗАМАЛА ШВИДКІСТЬ — ЗВАЛЮВАННЯ!  R — газ, W — ніс униз, або F2 — коптер\n") + b; col = ImVec4(1, 0.4f, 0.3f, 1); }
        }
    }
    const std::string al = A.rig->ctl.alertText();
    ImGui::SetNextWindowPos(ImVec2(0.5f * (x0 + x1), 8), ImGuiCond_Always, ImVec2(0.5f, 0));
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::SetNextWindowSizeConstraints(ImVec2(200, 0), ImVec2(x1 - x0 - 20, 400));
    ImGui::Begin("##coach", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    if (A.big_font) ImGui::PushFont(A.big_font, 26.0f);
    ImGui::TextColored(col, "%s", title.c_str());
    if (A.big_font) ImGui::PopFont();
    ImGui::PushTextWrapPos(x1 - x0 - 40);
    ImGui::TextColored(col.x < 0.9f || col.y < 0.9f ? col : ImVec4(0.92f, 0.92f, 0.92f, 1), "%s", hint.c_str());
    if (!al.empty()) ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "УВАГА: %s", al.c_str());
    ImGui::PopTextWrapPos();
    ImGui::End();
}

// ---------- Звіт про політ: підсумок, графіки всіх величин, карта; після посадки/аварії відкривається сам і зберігається у файли ----------
void reportSeries(App& A) {
    const auto& lg = A.rig->log;
    if (lg.size() == A.rep_rows) return;
    auto& R = A.rep;
    R.clear();
    auto add = [&](const char* k, double v) { R[k].push_back(v); };
    double wh = 0;
    for (size_t i = 0; i < lg.size(); ++i) {
        const FlightRow& r = lg[i];
        if (i > 0) wh += r.power * (r.t - lg[i - 1].t) / 3600.0;
        add("t", r.t); add("z", r.z); add("vz", r.vz); add("va", r.airspeed); add("vg", std::hypot(r.vx, r.vy)); add("wind", std::hypot(r.wx, r.wy)); add("wz", r.wz);
        add("home", std::hypot(r.x, r.y)); add("roll", r.roll / kDeg); add("pitch", -r.pitch_down / kDeg); add("yaw", std::fmod(r.yaw / kDeg + 720.0, 360.0));
        add("p", r.omx / kDeg); add("q", r.omy / kDeg); add("r", r.omz / kDeg);
        add("volts", r.volts); add("amps", r.amps); add("power", r.power); add("soc", r.soc * 100); add("wh", wh); add("temp", r.batt_temp);
        add("rps1", r.rps[0]); add("rps2", r.rps[1]); add("rps3", r.rps[2]); add("rps4", r.rps[3]);
        add("d1", r.duty[0] * 100); add("d2", r.duty[1] * 100); add("d3", r.duty[2] * 100); add("d4", r.duty[3] * 100);
        add("t1", r.tilt[0]); add("t2", r.tilt[1]); add("t3", r.tilt[2]); add("t4", r.tilt[3]);
        add("ail", r.aileron); add("ele", r.elevator); add("rud", r.rudder); add("wv", r.wv * 100); add("lim", r.cur_lim * 100);
        add("x", r.x); add("y", r.y); add("ph", r.phase);
    }
    A.rep_rows = lg.size();
    if (!lg.empty()) { A.rep_x0 = lg.front().t; A.rep_x1 = lg.back().t; }
}

const char* kPhaseUa[] = {"на землі", "зліт", "висіння", "перехід у літак", "літак", "перехід у коптер", "посадка", "сів", "посадка літаком", "ручне"};
const ImVec4 kPhaseCol[] = {{0.55f, 0.55f, 0.55f, 1}, {0.91f, 0.64f, 0.24f, 1}, {0.82f, 0.42f, 0.84f, 1}, {0.31f, 0.70f, 0.85f, 1}, {0.23f, 0.48f, 0.88f, 1},
                            {0.13f, 0.72f, 0.64f, 1}, {0.88f, 0.35f, 0.35f, 1}, {0.55f, 0.55f, 0.55f, 1}, {0.71f, 0.40f, 0.17f, 1}, {0.61f, 0.61f, 0.18f, 1}};

void reportPlot(App& A, const char* title, std::initializer_list<std::pair<const char*, const char*>> ser, float h) {
    auto& R = A.rep;
    const std::vector<double>& t = R["t"];
    if (!ImPlot::BeginPlot(title, ImVec2(-1, h), ImPlotFlags_NoMenus)) return;
    ImPlot::SetupAxes(nullptr, nullptr, 0, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisLinks(ImAxis_X1, &A.rep_x0, &A.rep_x1);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    for (const auto& [key, name] : ser) {
        const std::vector<double>& y = R[key];
        if (y.size() == t.size() && !t.empty()) ImPlot::PlotLine(name, t.data(), y.data(), int(t.size()));
    }
    ImPlot::EndPlot();
}

void panelReport(App& A, float W, float H) {
    if (!A.show_report) return;
    ImGui::SetNextWindowSize(ImVec2(std::min(1200.0f, W * 0.8f), std::min(820.0f, H * 0.88f)), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImVec2(W * 0.5f, H * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Звіт про політ", &A.show_report);
    if (!A.report_file.empty()) {
        ImGui::TextColored(ImVec4(0.6f, 1, 0.6f, 1), "Збережено: %s, %s (графіки й карта), журнал .csv", A.report_file.c_str(), A.report_html.empty() ? "—" : A.report_html.c_str());
    }
    if (ImGui::Button("Оновити")) { A.report_text = A.rig->reportText(); A.rep_rows = 0; }
    ImGui::SameLine();
    if (ImGui::Button("Копіювати текст")) ImGui::SetClipboardText(A.report_text.c_str());
    ImGui::SameLine();
    if (ImGui::Button("Зберегти й відкрити HTML-звіт")) {   // повний звіт у браузері: усі графіки, карта, кнопки «скачати CSV»
        if (A.report_html.empty()) {
            char base[64]; int n = 1;
            for (; n < 1000; ++n) { std::snprintf(base, sizeof base, "flight_%03d.html", n); if (GetFileAttributesA(base) == INVALID_FILE_ATTRIBUTES) break; }
            char ttl[32]; std::snprintf(ttl, sizeof ttl, "flight_%03d", n);
            if (writeHtmlReport(*A.rig, base, ttl)) A.report_html = base;
        } else {
            writeHtmlReport(*A.rig, A.report_html, A.report_html.substr(0, A.report_html.size() - 5));
        }
        if (!A.report_html.empty()) ShellExecuteA(nullptr, "open", A.report_html.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    }
    ImGui::SameLine();
    if (ImGui::Button("Відкрити папку з файлами")) ShellExecuteA(nullptr, "open", ".", nullptr, nullptr, SW_SHOWNORMAL);
    ImGui::Separator();
    reportSeries(A);
    if (ImGui::BeginTabBar("rep_tabs")) {
        if (ImGui::BeginTabItem("Підсумок")) {
            ImGui::BeginChild("rep");
            ImGui::TextUnformatted(A.report_text.c_str());
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Графіки")) {
            ImGui::TextDisabled("Колесо / перетягування — масштаб і зсув часу (спільні для всіх графіків); подвійний клік — увесь політ");
            ImGui::BeginChild("rep_plots");
            const float ph = 210;
            if (ImGui::BeginTable("rp", 2)) {
                auto cell = [&](const char* tt, std::initializer_list<std::pair<const char*, const char*>> ser) { ImGui::TableNextColumn(); reportPlot(A, tt, ser, ph); };
                cell("Висота, м", {{"z", "висота"}});
                cell("Вертикальна швидкість, м/с", {{"vz", "Vz"}});
                cell("Швидкість, м/с", {{"va", "повітряна"}, {"vg", "земна"}, {"wind", "вітер"}});
                cell("Відстань від дому, м", {{"home", "відстань"}});
                cell("Крен і тангаж, °", {{"roll", "крен"}, {"pitch", "тангаж"}});
                cell("Курс, °", {{"yaw", "курс"}});
                cell("Кутові швидкості, °/с", {{"p", "p"}, {"q", "q"}, {"r", "r"}});
                cell("Вітер: горизонтальний / вертикальний, м/с", {{"wind", "горизонт."}, {"wz", "вертик."}});
                cell("Напруга, В", {{"volts", "В"}});
                cell("Струм, А", {{"amps", "А"}});
                cell("Потужність, Вт", {{"power", "Вт"}});
                cell("Заряд, % / витрачено, Вт·год", {{"soc", "заряд %"}, {"wh", "Вт·год"}});
                cell("Температура батареї, °C", {{"temp", "°C"}});
                cell("Оберти моторів, об/с", {{"rps1", "М1"}, {"rps2", "М2"}, {"rps3", "М3"}, {"rps4", "М4"}});
                cell("Газ моторів, %", {{"d1", "М1"}, {"d2", "М2"}, {"d3", "М3"}, {"d4", "М4"}});
                cell("Нахил гондол, ° (0 — вгору, 90 — уперед)", {{"t1", "FR"}, {"t2", "RL"}, {"t3", "FL"}, {"t4", "RR"}});
                cell("Керма (−1…1)", {{"ail", "елерони"}, {"ele", "руль висоти"}, {"rud", "руль напрямку"}});
                cell("Вага коптера / дозволений струм, %", {{"wv", "коптер"}, {"lim", "струм"}});
                ImGui::EndTable();
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Карта")) {
            if (ImPlot::BeginPlot("##map", ImVec2(-1, -1), ImPlotFlags_Equal | ImPlotFlags_NoMenus)) {
                ImPlot::SetupAxes("схід, м", "північ, м");
                auto& R = A.rep;
                const std::vector<double>&X = R["x"], &Y = R["y"], &P = R["ph"];
                size_t s0 = 0;
                for (size_t i = 1; i <= X.size(); ++i) {   // носій: відрізки кольору режиму
                    if (i == X.size() || P[i] != P[s0]) {
                        const int ph = std::clamp(int(P[s0]), 0, 9);
                        ImPlotSpec sp; sp.LineColor = kPhaseCol[ph]; sp.LineWeight = 2;
                        const size_t e = std::min(i + 1, X.size());
                        ImPlot::PlotLine(kPhaseUa[ph], X.data() + s0, Y.data() + s0, int(e - s0), sp);
                        s0 = i;
                    }
                }
                const double hx = 0, hy = 0;
                ImPlotSpec hm; hm.Marker = ImPlotMarker_Up; hm.MarkerSize = 7; hm.MarkerFillColor = ImVec4(1, 1, 1, 1);
                ImPlot::PlotScatter("дім", &hx, &hy, 1, hm);
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

void panelPfd(App& A) {
    const QuadPlaneSim& s = A.rig->sim;
    ImGui::Begin("PFD", nullptr, ImGuiWindowFlags_NoScrollbar);
    const Vec3d eu = s.eulerRad(), v = s.velWorld();
    const double roll = eu.x, pitch_up = -eu.y, yaw = eu.z;
    ImVec2 p0 = ImGui::GetCursorScreenPos(), avail = ImGui::GetContentRegionAvail();
    const float sz = std::max(150.0f, std::min(avail.x, avail.y - 60.0f));
    const ImVec2 c(p0.x + sz / 2, p0.y + sz / 2);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float R = sz / 2 - 4;
    ImGui::PushClipRect(ImVec2(c.x - R, c.y - R), ImVec2(c.x + R, c.y + R), true);
    // горизонт: обертання навколо центру на кут крену, зсув за тангажем (3 пікс/°)
    const float cr = float(std::cos(-roll)), sr = float(std::sin(-roll)), off = float(pitch_up / kDeg) * 3.0f;
    auto rot = [&](float x, float y) { return ImVec2(c.x + x * cr - (y + off) * sr, c.y + x * sr + (y + off) * cr); };
    const float big = R * 3;
    dl->AddQuadFilled(rot(-big, -big), rot(big, -big), rot(big, 0), rot(-big, 0), IM_COL32(60, 120, 200, 255));
    dl->AddQuadFilled(rot(-big, 0), rot(big, 0), rot(big, big), rot(-big, big), IM_COL32(120, 80, 40, 255));
    dl->AddLine(rot(-big, 0), rot(big, 0), IM_COL32_WHITE, 2);
    for (int d = -30; d <= 30; d += 10) if (d) {
        const float y = -d * 3.0f, hw = d % 20 == 0 ? 28.0f : 16.0f;
        dl->AddLine(rot(-hw, y), rot(hw, y), IM_COL32_WHITE, 1.5f);
        char b[8]; std::snprintf(b, 8, "%d", std::abs(d));
        dl->AddText(rot(hw + 4, y - 6), IM_COL32_WHITE, b);
    }
    ImGui::PopClipRect();
    dl->AddCircle(c, R, IM_COL32(220, 220, 220, 255), 48, 3);
    dl->AddLine(ImVec2(c.x - 40, c.y), ImVec2(c.x - 12, c.y), IM_COL32(255, 220, 0, 255), 3); dl->AddLine(ImVec2(c.x + 12, c.y), ImVec2(c.x + 40, c.y), IM_COL32(255, 220, 0, 255), 3);
    dl->AddCircleFilled(c, 3, IM_COL32(255, 220, 0, 255));
    dl->AddTriangleFilled(ImVec2(c.x, c.y - R + 2), ImVec2(c.x - 6, c.y - R + 14), ImVec2(c.x + 6, c.y - R + 14), IM_COL32(255, 220, 0, 255));
    ImGui::Dummy(ImVec2(sz, sz));
    const double gs = std::hypot(v.x, v.y);
    const int hdg = int(std::fmod(450.0 - yaw / kDeg, 360.0));
    ImGui::Text("V возд. %5.1f м/с   V земн. %5.1f м/с   Курс %03d°", s.airspeed(), gs, hdg);
    ImGui::Text("Висота %6.1f м   Варіометр %+5.1f м/с   Крен %+5.1f°  Тангаж %+5.1f°", s.state().pos.z, v.z, roll / kDeg, pitch_up / kDeg);
    ImGui::End();
}

void panelMap(App& A) {
    ImGui::Begin("Карта");
    ImVec2 p0 = ImGui::GetCursorScreenPos(), avail = ImGui::GetContentRegionAvail();
    const float W = std::max(150.0f, avail.x), H = std::max(150.0f, avail.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + W, p0.y + H), IM_COL32(24, 34, 24, 255));
    const bool own = A.route_on && !A.route.empty();
    double x0 = 0, x1 = 0, y0 = 0, y1 = 0;   // межі: дім, ціль або точки маршруту, слід
    auto grow = [&](double x, double y) { x0 = std::min(x0, x); x1 = std::max(x1, x); y0 = std::min(y0, y); y1 = std::max(y1, y); };
    if (own) { for (const auto& w : A.route) grow(w.x, w.y); } else grow(A.leg_km * 1000.0, 0);
    if (A.route_on) { grow(0, 300); grow(0, -300); }   // поле для розстановки точок
    for (const Vec3d& q : A.trail) grow(q.x, q.y);
    const double scale = std::min(W / std::max(150.0, (x1 - x0) * 1.25), H / std::max(150.0, (y1 - y0) * 1.25));
    const double cxw = 0.5 * (x0 + x1), cyw = 0.5 * (y0 + y1);
    auto M = [&](double x, double y) { return ImVec2(float(p0.x + W / 2 + (x - cxw) * scale), float(p0.y + H / 2 - (y - cyw) * scale)); };
    ImGui::InvisibleButton("##mapclick", ImVec2(W, H), ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);   // клік не тягне вікно
    if (A.route_on) {   // розстановка точок мишею
        const ImVec2 mp = ImGui::GetMousePos();
        if (ImGui::IsItemClicked(0)) A.route.push_back({float(cxw + (mp.x - p0.x - W / 2) / scale), float(cyw - (mp.y - p0.y - H / 2) / scale), A.alt_cruise});
        if (ImGui::IsItemClicked(1) && !A.route.empty()) A.route.pop_back();
    }
    if (own) {
        ImVec2 a = M(0, 0);
        for (size_t i = 0; i < A.route.size(); ++i) {
            const ImVec2 b = M(A.route[i].x, A.route[i].y);
            dl->AddLine(a, b, IM_COL32(90, 110, 160, 255), 1.5f);
            dl->AddCircleFilled(b, 4, i + 1 < A.route.size() ? IM_COL32(120, 170, 255, 255) : IM_COL32(220, 80, 80, 255));
            char t[8]; std::snprintf(t, sizeof t, "%d", int(i) + 1);
            dl->AddText(ImVec2(b.x + 6, b.y - 14), IM_COL32_WHITE, t);
            a = b;
        }
        dl->AddLine(a, M(0, 0), IM_COL32(70, 90, 120, 160), 1.0f);
    } else {
        dl->AddLine(M(0, 0), M(A.leg_km * 1000, 0), IM_COL32(90, 110, 160, 255), 1.5f);
        dl->AddCircleFilled(M(A.leg_km * 1000, 0), 5, IM_COL32(220, 80, 80, 255)); dl->AddText(ImVec2(M(A.leg_km * 1000, 0).x - 40, M(A.leg_km * 1000, 0).y + 6), IM_COL32_WHITE, "ціль");
    }
    dl->AddCircleFilled(M(0, 0), 5, IM_COL32(60, 200, 60, 255)); dl->AddText(ImVec2(M(0, 0).x + 6, M(0, 0).y + 4), IM_COL32_WHITE, "дім");
    ImVec2 prev;
    bool first = true;
    for (const Vec3d& q : A.trail) { const ImVec2 pt = M(q.x, q.y); if (!first) dl->AddLine(prev, pt, IM_COL32(255, 220, 60, 200), 1.5f); prev = pt; first = false; }
    const QpState& st = A.rig->sim.state();
    const double yaw = A.rig->sim.eulerRad().z;
    const ImVec2 ap = M(st.pos.x, st.pos.y);
    dl->AddTriangleFilled(ImVec2(ap.x + 9 * float(std::cos(yaw)), ap.y - 9 * float(std::sin(yaw))), ImVec2(ap.x + 6 * float(std::cos(yaw + 2.5)), ap.y - 6 * float(std::sin(yaw + 2.5))),
                          ImVec2(ap.x + 6 * float(std::cos(yaw - 2.5)), ap.y - 6 * float(std::sin(yaw - 2.5))), IM_COL32(255, 255, 255, 255));
    ImGui::End();
}

// Власний графік на ImDrawList (ImPlot 1.0 з динамічним атласом ImGui 1.93 малює лінії пунктиром): осі, сітка, кілька кривих, легенда
struct Series { const std::vector<float>* y; const char* name; ImU32 color; };
void miniPlot(const char* title, const std::vector<float>& x, const std::vector<Series>& ser, float height = 130) {
    ImGui::TextUnformatted(title);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const float W = std::max(120.0f, ImGui::GetContentRegionAvail().x - 4), H = std::max(60.0f, height), L = 46, B = 16;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + W, p0.y + H), IM_COL32(20, 20, 24, 255));
    const size_t n = x.size();
    if (n > 1) {
        float x0 = x.front(), x1 = x.back(), y0 = 1e30f, y1 = -1e30f;
        for (const Series& s : ser) for (size_t i = 0; i < n; ++i) { y0 = std::min(y0, (*s.y)[i]); y1 = std::max(y1, (*s.y)[i]); }
        if (y1 - y0 < 1e-3f) { y0 -= 1; y1 += 1; }
        const float pad = (y1 - y0) * 0.08f; y0 -= pad; y1 += pad;
        auto X = [&](float v) { return p0.x + L + (v - x0) / std::max(x1 - x0, 1e-3f) * (W - L - 4); };
        auto Y = [&](float v) { return p0.y + 4 + (1 - (v - y0) / (y1 - y0)) * (H - B - 8); };
        for (int g = 0; g <= 4; ++g) {   // горизонтальна сітка з підписами
            const float v = y0 + (y1 - y0) * g / 4, yy = Y(v);
            dl->AddLine(ImVec2(p0.x + L, yy), ImVec2(p0.x + W - 4, yy), IM_COL32(60, 60, 68, 255));
            char b[16]; std::snprintf(b, 16, "%.4g", v);
            dl->AddText(ImVec2(p0.x + 3, yy - 8), IM_COL32(170, 170, 180, 255), b);
        }
        for (int g = 0; g <= 4; ++g) {
            const float v = x0 + (x1 - x0) * g / 4;
            char b[16]; std::snprintf(b, 16, "%.0f c", v);
            dl->AddText(ImVec2(X(v) - 10, p0.y + H - 14), IM_COL32(170, 170, 180, 255), b);
        }
        const size_t step = std::max<size_t>(1, n / 1500);   // не більше ~1500 точок на криву
        for (const Series& s : ser) {
            std::vector<ImVec2> pts;
            for (size_t i = 0; i < n; i += step) pts.push_back(ImVec2(X(x[i]), Y((*s.y)[i])));
            if (pts.size() > 1) dl->AddPolyline(pts.data(), int(pts.size()), s.color, ImDrawFlags_None, 2.0f);
        }
    }
    float lx = p0.x + L + 8;
    for (const Series& s : ser) {   // легенда
        dl->AddRectFilled(ImVec2(lx, p0.y + 8), ImVec2(lx + 10, p0.y + 18), s.color);
        dl->AddText(ImVec2(lx + 14, p0.y + 4), IM_COL32(220, 220, 230, 255), s.name);
        lx += 24 + ImGui::CalcTextSize(s.name).x;
    }
    ImGui::Dummy(ImVec2(W, H + 4));
}

void panelPlots(App& A) {
    const auto& lg = A.rig->log;
    if (lg.size() != A.plot_cache_rows) {
        for (size_t i = A.plot_cache_rows; i < lg.size(); ++i) {
            const FlightRow& r = lg[i];
            A.pt.push_back(r.t); A.pz.push_back(r.z); A.pvz.push_back(r.vz); A.pva.push_back(r.airspeed); A.pvg.push_back(std::hypot(r.vx, r.vy));
            A.proll.push_back(r.roll / float(kDeg)); A.ppitch.push_back(-r.pitch_down / float(kDeg)); A.ppower.push_back(r.volts * r.amps); A.pamps.push_back(r.amps);
        }
        A.plot_cache_rows = lg.size();
    }
    ImGui::Begin("Графіки");
    const ImU32 c1 = IM_COL32(90, 150, 255, 255), c2 = IM_COL32(255, 150, 70, 255);
    const float h = std::max(80.0f, ImGui::GetContentRegionAvail().y - ImGui::GetTextLineHeightWithSpacing() - 8);
    const int cols = ImGui::GetContentRegionAvail().x > 900 ? 4 : 2;   // широке вікно — 4 графіки в ряд
    if (ImGui::BeginTable("plots", cols)) {
        ImGui::TableNextColumn(); miniPlot("Висота, м / Vz, м/с", A.pt, {{&A.pz, "Висота", c1}, {&A.pvz, "Vz", c2}}, cols == 4 ? h : h / 2 - 20);
        ImGui::TableNextColumn(); miniPlot("Швидкість, м/с", A.pt, {{&A.pva, "Повітряна", c1}, {&A.pvg, "Земна", c2}}, cols == 4 ? h : h / 2 - 20);
        ImGui::TableNextColumn(); miniPlot("Кути, °", A.pt, {{&A.proll, "Крен", c1}, {&A.ppitch, "Тангаж", c2}}, cols == 4 ? h : h / 2 - 20);
        ImGui::TableNextColumn(); miniPlot("Потужність, Вт / струм, А", A.pt, {{&A.ppower, "Вт", c1}, {&A.pamps, "А", c2}}, cols == 4 ? h : h / 2 - 20);
        ImGui::EndTable();
    }
    ImGui::End();
}
bool saveBmp(const char* path, int w, int h) {
    std::vector<unsigned char> px(size_t(w) * h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);   // без вирівнювання рядків: розмір буфера w·h·3
    glReadPixels(0, 0, w, h, GL_BGR_EXT, GL_UNSIGNED_BYTE, px.data());
    FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const unsigned rowpad = (4 - (w * 3) % 4) % 4, img = (w * 3 + rowpad) * h;
    unsigned char hdr[54] = {'B', 'M'};
    auto put = [&](int off, unsigned v) { for (int i = 0; i < 4; ++i) hdr[off + i] = (v >> (8 * i)) & 255; };
    put(2, 54 + img); put(10, 54); put(14, 40); put(18, unsigned(w)); put(22, unsigned(h)); hdr[26] = 1; hdr[28] = 24; put(34, img);
    std::fwrite(hdr, 1, 54, f);
    const unsigned char pad[3] = {0, 0, 0};
    for (int y = 0; y < h; ++y) { std::fwrite(&px[size_t(y) * w * 3], 1, size_t(w) * 3, f); std::fwrite(pad, 1, rowpad, f); }
    std::fclose(f);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    int max_frames = 0;
    float cam_arg[4] = {-1, 0, 0, 0};   // --cam режим азимут кут_місця відстань (для знімків)
    const char* shot = nullptr;
    float speed_arg = 0, sim_arg = 0;
    int win_w = 1500, win_h = 900;
    bool compare_arg = false, mcheck_arg = false;
    int brain_arg = -1, train_arg = -1, train_n = 30, show_arg = -9, ai_arg = -1;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--brain") && i + 1 < argc) brain_arg = std::atoi(argv[++i]);
        if (!std::strcmp(argv[i], "--train") && i + 2 < argc) { train_arg = std::atoi(argv[++i]); train_n = std::atoi(argv[++i]); }
        if (!std::strcmp(argv[i], "--show") && i + 1 < argc) show_arg = std::atoi(argv[++i]);
        if (!std::strcmp(argv[i], "--sim") && i + 1 < argc) sim_arg = float(std::atof(argv[++i]));
        if (!std::strcmp(argv[i], "--ai") && i + 1 < argc) ai_arg = std::atoi(argv[++i]);
        if (!std::strcmp(argv[i], "--compare")) compare_arg = true;
        if (!std::strcmp(argv[i], "--mcheck")) mcheck_arg = true;
        if (!std::strcmp(argv[i], "--size") && i + 2 < argc) { win_w = std::atoi(argv[++i]); win_h = std::atoi(argv[++i]); }
        if (!std::strcmp(argv[i], "--cam") && i + 4 < argc) { cam_arg[0] = float(std::atof(argv[++i])); cam_arg[1] = float(std::atof(argv[++i])); cam_arg[2] = float(std::atof(argv[++i])); cam_arg[3] = float(std::atof(argv[++i])); }
        if (!std::strcmp(argv[i], "--frames") && i + 1 < argc) max_frames = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--shot") && i + 1 < argc) shot = argv[++i];
        else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc) speed_arg = float(std::atof(argv[++i]));
    }
    if (!glfwInit()) { std::fprintf(stderr, "GLFW: не вдалось ініціалізувати\n"); return 1; }
    GLFWwindow* win = glfwCreateWindow(win_w, win_h, "Конвертоплан — симулятор", nullptr, nullptr);
    if (!win) { std::fprintf(stderr, "GLFW: не вдалось створити вікно\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext(); ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;   // розкладка будується з розміру вікна (стара drone_gui.ini більше не потрібна)
    ImFont* big_font = nullptr;
    if (GetFileAttributesA("C:\\Windows\\Fonts\\segoeui.ttf") != INVALID_FILE_ATTRIBUTES) {
        io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 17.0f);
        const char* bf = GetFileAttributesA("C:\\Windows\\Fonts\\segoeuib.ttf") != INVALID_FILE_ATTRIBUTES ? "C:\\Windows\\Fonts\\segoeuib.ttf" : "C:\\Windows\\Fonts\\segoeui.ttf";
        big_font = io.Fonts->AddFontFromFileTTF(bf, 26.0f);   // напівжирний для підказки над 3D-видом
    }
    else std::fprintf(stderr, "Шрифт Segoe UI не знайдено — кирилиця може не відображатись\n");
    ImGui::StyleColorsDark();
    ImGui::GetStyle().AntiAliasedLinesUseTex = false;   // ImPlot 1.0 + динамічний атлас ImGui 1.93: лінії через текстуру малюються пунктиром
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    std::unique_ptr<App> app;
    try { app = std::make_unique<App>(); app->reset(); }
    catch (const std::exception& e) { std::fprintf(stderr, "Помилка завантаження моделі: %s\n(запускайте з кореня проєкту, де є aircraft/)\n", e.what()); return 2; }
    app->big_font = big_font;
    EngineSound sound;
    sound.open();   // без звукової карти — просто тиша
    if (max_frames > 0) { app->paused = false; app->show_help = false; }
    if (speed_arg > 0) app->time_scale = speed_arg;
    if (cam_arg[0] >= 0) { app->camera = int(cam_arg[0]); app->cam_az = cam_arg[1]; app->cam_el = cam_arg[2]; app->cam_dist = cam_arg[3]; }
    if (brain_arg >= 0) { app->show_brain = true; app->brain_tab_req = brain_arg; }
    if (ai_arg >= 0) { app->ai_mode = ai_arg; app->rig->ai_mode = ai_arg; }
    if (train_arg >= 0) { app->trick_episodes = train_n; startTrickTraining(*app, fc::Trick(train_arg)); }
    if (show_arg > -9) {
        if (show_arg >= 0) replayEpisode(*app, fc::Trick(show_arg), defaultTrickParams(fc::Trick(show_arg)));
        else { replayEpisode(*app, fc::Trick::Loop, defaultTrickParams(fc::Trick::Loop)); startShow(*app, airShowProgram()); }
        app->rig->ai_mode = app->ai_mode;
    }
    if (sim_arg > 0) {
        App& A = *app;
        for (long i = 0; i < long(sim_arg * 1000); ++i) {
            A.rig->step(); A.show.after(*A.rig);
            if (A.show.inTrick() && i % 20 == 0) brainSample(A);
            if (i % 50 == 0) { aiSample(A); if (A.trail.empty() || (A.rig->sim.state().pos - A.trail.back()).norm() > 1.5) A.trail.push_back(A.rig->sim.state().pos); }
            if (A.show_was_running && !A.show.running()) { A.show_was_running = false; A.trick_result = A.show.lastText; }
            if (A.rig->sim.crashed()) break;
        }
    }
    if (max_frames > 0 && train_arg >= 0) while (app->train_running) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (compare_arg) { startCompare(*app); while (app->cmp_running) std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    if (mcheck_arg) { app->mc_open = true; startMissionCheck(*app); while (app->mc_running) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (const App::McCase& c : app->mc) std::printf("вітер %4.0f м/с, напрям %3.0f°: %s, %.0f с, %.1f Вт·год, посадка %.1f м\n", c.wind, c.dir, c.crashed ? "АВАРІЯ" : (c.landed ? "сів" : "не сів"), c.time, c.wh, c.err);
        std::printf("перевірка місії: %.0f с на %d потоках\n", app->mc_secs, app->mc_threads); }
    double last = glfwGetTime(), drag_x = 0, drag_y = 0;
    int frame = 0;
    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        const double now = glfwGetTime(), dtf = std::min(now - last, 0.1);
        last = now;
        App& A = *app;
        {   // B — «Мозок ШІ»
            static bool prev_b = false;
            const bool bk = glfwGetKey(win, GLFW_KEY_B) == GLFW_PRESS && !io.WantCaptureKeyboard;
            if (bk && !prev_b) A.show_brain = !A.show_brain;
            prev_b = bk;
        }
        {   // H — довідка
            static bool prev_h = false;
            const bool h = glfwGetKey(win, GLFW_KEY_H) == GLFW_PRESS && !io.WantCaptureKeyboard;
            if (h && !prev_h) A.show_help = !A.show_help;
            prev_h = h;
        }
        {   // ручне керування (див. вкладку «Політ» → «Ручне керування»)
            fc::QpManual& m = A.rig->ctl.manual;
            m.on = A.manual;
            auto key = [&](int k) { return glfwGetKey(win, k) == GLFW_PRESS; };
            static bool prev_f[5] = {false, false, false, false, false}, prev_y = false, prev_v = false;
            const bool kb = A.manual && !io.WantCaptureKeyboard;
            if (kb) for (int i = 0; i < 5; ++i) { const bool d = key(GLFW_KEY_F1 + i); if (d && !prev_f[i]) A.manual_mode = i; prev_f[i] = d; }
            {   // V — перемикач «коптер ↔ літак»: QSTABILIZE (F2) ↔ FBWA (F4); з інших режимів — у протилежний за типом
                const bool v = kb && key(GLFW_KEY_V);
                if (v && !prev_v) A.manual_mode = A.manual_mode >= 3 ? 1 : 3;
                prev_v = v;
            }
            GLFWgamepadstate gp;
            A.gamepad = glfwJoystickIsGamepad(GLFW_JOYSTICK_1) && glfwGetGamepadState(GLFW_JOYSTICK_1, &gp);
            if (A.manual && A.gamepad) { const bool y = gp.buttons[GLFW_GAMEPAD_BUTTON_Y] == GLFW_PRESS; if (y && !prev_y) A.manual_mode = (A.manual_mode + 1) % 5; prev_y = y; }
            // зміна режиму: газ одразу на розумний рівень (висіння або крейсер), щоб апарат не впав
            static int prev_mode = -1;
            if (A.manual_mode != prev_mode) {
                const bool airborne = A.rig->sim.state().pos.z > 1.0;
                if (A.manual_mode == 1 || A.manual_mode == 2) m.throttle = airborne ? 0.28f : 0.0f;
                if (A.manual_mode >= 3) m.throttle = 0.65f;
                if (A.manual_mode < 3) m.pusher = 0;
                prev_mode = A.manual_mode;
            }
            m.mode = fc::QpManual::Mode(A.manual_mode);
            const float dt_in = float(dtf);
            auto ease = [&](float& v, float target) { const float r = (std::fabs(target) < std::fabs(v) ? 2.0f : 1.0f) * A.key_ramp * dt_in; v += std::clamp(target - v, -r, r); };   // клавіша — плавне відхилення ручки (повернення вдвічі швидше)
            if (!m.direct()) {   // асистований (як QLOITER): W/S вперед-назад, A/D вбік, Q/E поворот, R/F вгору-вниз, Shift — швидше
                if (kb) {
                    m.fwd = float(key(GLFW_KEY_W)) - float(key(GLFW_KEY_S));
                    m.left = float(key(GLFW_KEY_A)) - float(key(GLFW_KEY_D));
                    m.yaw_rate = float(key(GLFW_KEY_Q)) - float(key(GLFW_KEY_E));
                    m.climb = float(key(GLFW_KEY_R)) - float(key(GLFW_KEY_F));
                    m.speed_max = key(GLFW_KEY_LEFT_SHIFT) ? 14.0f : 7.0f;
                } else { m.fwd = m.left = m.yaw_rate = m.climb = 0; }
            } else if (kb) {   // прямі режими: W/S тангаж (W — ніс униз), A/D крен, Q/E курс, R/F газ (тримається), T/G нахил гондол уперед у коптері, X — газ 0
                ease(m.pitch, float(key(GLFW_KEY_W)) - float(key(GLFW_KEY_S)));
                ease(m.roll, float(key(GLFW_KEY_D)) - float(key(GLFW_KEY_A)));
                ease(m.yaw, float(key(GLFW_KEY_E)) - float(key(GLFW_KEY_Q)));
                m.throttle = std::clamp(m.throttle + (float(key(GLFW_KEY_R)) - float(key(GLFW_KEY_F))) * 0.5f * dt_in, 0.0f, 1.0f);
                m.pusher = std::clamp(m.pusher + (float(key(GLFW_KEY_T)) - float(key(GLFW_KEY_G))) * 0.5f * dt_in, -1.0f, 1.0f);   // < 0 — гондоли назад (гальмування)
                if (key(GLFW_KEY_X)) { m.throttle = 0; m.pusher = 0; }
            } else { m.roll = m.pitch = m.yaw = 0; }
            if (A.manual && A.gamepad) {   // геймпад, «режим 2» як у RC-пульта
                auto dz = [](float v) { return std::fabs(v) < 0.08f ? 0.0f : (v - std::copysign(0.08f, v)) / 0.92f; };
                const float lx = dz(gp.axes[GLFW_GAMEPAD_AXIS_LEFT_X]), ly = dz(gp.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]);
                const float rx = dz(gp.axes[GLFW_GAMEPAD_AXIS_RIGHT_X]), ry = dz(gp.axes[GLFW_GAMEPAD_AXIS_RIGHT_Y]);
                if (!m.direct()) {   // асистований: ліва — рух, права — курс/висота
                    if (lx != 0 || ly != 0 || rx != 0 || ry != 0) { m.fwd = -ly; m.left = -lx; m.yaw_rate = -rx; m.climb = -ry; }
                    if (gp.buttons[GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER]) m.speed_max = 14.0f;
                } else {   // ліва Y — газ (додати/зменшити, ручка пружинить), ліва X — курс, права — крен/тангаж, правий курок — нахил гондол уперед у коптері
                    m.throttle = std::clamp(m.throttle - ly * 0.6f * dt_in, 0.0f, 1.0f);
                    if (lx != 0) m.yaw = lx;
                    if (rx != 0) m.roll = rx;
                    if (ry != 0) m.pitch = -ry;
                    if (m.mode == fc::QpManual::QStabilize || m.mode == fc::QpManual::QAcro)   // правий курок — гондоли вперед, лівий — назад (гальмування)
                        m.pusher = std::clamp((gp.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] - gp.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]) * 0.5f, -1.0f, 1.0f);
                }
            }
        }
        if (A.replay_on && A.replay_play && !A.replay.empty()) A.replay_t = std::min(A.replay_t + float(dtf) * A.replay_speed, A.replay.back().t);
        const bool done = (A.rig->ctl.phase() == fc::QpPhase::Landed && !A.manual) || A.rig->sim.crashed() || A.rig->sim.diverged();   // з ручним — можна злетіти знову
        if (!A.paused && !done) {
            // не більше ≈ 12 мс розрахунку на кадр: вікно лишається плавним, а якщо комп'ютер не встигає за обраною швидкістю часу,
            // модель просто йде повільніше (реальну швидкість видно під повзунком)
            const int steps = std::clamp(int(dtf * 1000.0 * A.time_scale + 0.5), 1, 4000);
            const double t_wall0 = glfwGetTime(), t_sim0 = A.rig->sim.time();
            for (int i = 0; i < steps; ++i) {
                if ((i & 15) == 15 && glfwGetTime() - t_wall0 > 0.012) break;
                A.rig->step();
                A.show.after(*A.rig);
                if (A.show.inTrick() && i % 20 == 0) brainSample(A);
                if (A.rig->sim.crashed() || A.rig->sim.diverged()) break;
            }
            {
                const double adv = A.rig->sim.time() - t_sim0, want = dtf * A.time_scale;
                A.real_speed = 0.9f * A.real_speed + 0.1f * float(want > 1e-6 ? A.time_scale * std::min(1.0, adv / want) : A.time_scale);
            }
            if (A.show_was_running && !A.show.running()) { A.show_was_running = false; A.trick_result = A.show.lastText; }
            else if (A.show.running() && !A.show.lastText.empty()) A.trick_result = A.show.lastText;
            aiSample(A);
            A.prop_angle += dtf * A.time_scale * 25;
            if (A.trail.empty() || (A.rig->sim.state().pos - A.trail.back()).norm() > 1.5) { A.trail.push_back(A.rig->sim.state().pos); if (A.trail.size() > 4000) A.trail.pop_front(); }
        }
        if (done) A.status = A.rig->sim.crashed() ? "АВАРІЯ" : (A.rig->sim.diverged() ? "Модель розійшлась" : "Посадка виконана — натисніть «Скинути» для повтору");
        if (done && !A.report_done && A.rig->report.t > 1.0) {   // звіт після кожного польоту: вікно + файли flight_NNN.txt / .csv
            A.report_done = true;
            A.report_text = A.rig->reportText();
            char base[64]; int n = 1;
            for (; n < 1000; ++n) { std::snprintf(base, sizeof base, "flight_%03d.txt", n); if (GetFileAttributesA(base) == INVALID_FILE_ATTRIBUTES) break; }
            if (FILE* f = std::fopen(base, "wb")) { std::fwrite(A.report_text.data(), 1, A.report_text.size(), f); std::fclose(f); A.report_file = base; }
            std::snprintf(base, sizeof base, "flight_%03d.csv", n);
            A.rig->writeCsv(base);
            std::snprintf(base, sizeof base, "flight_%03d.html", n);   // повний звіт: графіки всіх величин, карта, кнопки «скачати CSV»
            char ttl[32]; std::snprintf(ttl, sizeof ttl, "flight_%03d", n);
            if (writeHtmlReport(*A.rig, base, ttl)) A.report_html = base;
            A.rep_rows = 0;
            A.show_report = true;
        }
        else if (!A.paused) A.status = "Політ триває";

        // мишa: орбітальна камера
        if (!io.WantCaptureMouse) {
            double mx, my; glfwGetCursorPos(win, &mx, &my);
            if (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) { A.cam_az -= float(mx - drag_x) * 0.01f; A.cam_el = std::clamp(A.cam_el + float(my - drag_y) * 0.01f, 0.02f, 1.5f); if (A.camera == 0 || A.camera == 2) A.camera = 1; }
            A.cam_dist = std::clamp(A.cam_dist * (1.0f - float(io.MouseWheel) * 0.1f), 2.0f, 80.0f);
            drag_x = mx; drag_y = my;
        } else { double mx, my; glfwGetCursorPos(win, &mx, &my); drag_x = mx; drag_y = my; }

        int fbw, fbh; glfwGetFramebufferSize(win, &fbw, &fbh);
        drawScene(A, fbw, fbh);
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        {   // розкладка: ліворуч керування + стан, праворуч PFD + карта, знизу графіки, центр — 3D-вид; при зміні розміру вікна — заново
            static int lw = 0, lh = 0;
            const bool re = fbw != lw || fbh != lh || A.relayout;
            lw = fbw; lh = fbh; A.relayout = false;
            const ImGuiCond c = re ? ImGuiCond_Always : ImGuiCond_None;
            const float W = float(fbw), H = float(fbh), left = std::min(430.0f, W * 0.28f), right = std::min(400.0f, W * 0.24f), bottom = std::min(230.0f, H * 0.26f);
            const float top_l = H * 0.60f, pfd_h = std::min(right + 70, H * 0.55f);
            if (re) { ImGui::SetNextWindowPos(ImVec2(0, 0), c); ImGui::SetNextWindowSize(ImVec2(left, top_l), c); }
            panelLeft(A);
            if (re) { ImGui::SetNextWindowPos(ImVec2(0, top_l), c); ImGui::SetNextWindowSize(ImVec2(left, H - top_l), c); }
            panelState(A);
            if (re) { ImGui::SetNextWindowPos(ImVec2(W - right, 0), c); ImGui::SetNextWindowSize(ImVec2(right, pfd_h), c); }
            panelPfd(A);
            if (re) { ImGui::SetNextWindowPos(ImVec2(W - right, pfd_h), c); ImGui::SetNextWindowSize(ImVec2(right, H - pfd_h), c); }
            panelMap(A);
            if (re) { ImGui::SetNextWindowPos(ImVec2(left, H - bottom), c); ImGui::SetNextWindowSize(ImVec2(W - left - right, bottom), c); }
            panelPlots(A);
            coach(A, left, A.show_brain && !A.brain_full ? W - right - std::max(560.0f, (W - left - right) * 0.6f) : W - right);
            panelBrain(A, W, H, left, right, bottom);
            panelHelp(A, W, H);
            panelReport(A, W, H);
        }
        {   // звук із поточного стану моделі
            SoundParams sp;
            const QuadPlaneSim& s = A.rig->sim;
            for (int i = 0; i < 4; ++i) { sp.rps[i] = float(s.vtolRps(i)); sp.thrust += float(s.vtolThrust(i)); }
            sp.rps[4] = 0;
            sp.airspeed = float(s.airspeed());
            sp.distance = A.camera == 4 ? 0.5f : (A.camera == 2 ? float((s.state().pos - Vec3d{-15, -10, 2}).norm()) : (A.camera == 3 ? std::max(30.0f, A.cam_dist * 8) : A.cam_dist));
            sp.alert = !A.rig->ctl.alertText().empty() && !s.crashed();
            sp.crash = s.crashed();
            sp.active = A.sound_on && !A.paused && !A.replay_on;
            sp.volume = A.volume;
            sound.pump(sp);
        }
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (shot && max_frames > 0 && frame == max_frames - 1) { glFinish(); saveBmp(shot, fbw, fbh); }
        if (A.shot_request) {   // кнопка «Знімок екрана»
            A.shot_request = false; glFinish();
            char name[64]; std::snprintf(name, sizeof name, "shot_%03d.bmp", ++A.shot_n);
            A.status = saveBmp(name, fbw, fbh) ? std::string("Знімок збережено: ") + name : "Не вдалось зберегти знімок";
        }
        glfwSwapBuffers(win);
        ++frame;
        if (max_frames > 0 && frame >= max_frames) break;
    }
    sound.close();
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImPlot::DestroyContext(); ImGui::DestroyContext();
    glfwDestroyWindow(win); glfwTerminate();
    return 0;
}
