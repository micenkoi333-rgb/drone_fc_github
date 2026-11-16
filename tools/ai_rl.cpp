// Навчання ШІ-пілота з нуля (еволюційні стратегії, як OpenAI-ES): мережа ai/flight_policy.h радить контролеру прискорення
// в утриманні точки. Ніяких прикладів «як правильно» — лише оцінка польоту: наскільки далеко від точки, скільки енергії, чи не впав.
// Кожне покоління — нові випадкові умови (вітер 0–11 м/с з будь-якого боку, пориви, невідомий вантаж до 1 кг),
// однакові для всіх кандидатів; оцінка — наскільки кандидат кращий за звичайний контролер у тих самих умовах.
//
//   ai_rl train [поколінь=300] [cont]   навчання (cont — продовжити з ai/policy_hover.txt), усі ядра
//   ai_rl eval                          порівняння зі звичайним контролером на 24 відкладених умовах
//   ai_rl train2 / eval2                v2: + повітряна швидкість, оберти кожного мотора, керує ще й кутом носа (ai/policy_hover2.txt)
//
// Вихід: ai/policy_hover.txt (найкраща на відкладених умовах), data/rl_log.csv (по поколіннях).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <random>
#include <thread>
#include <vector>
#include "../sim/qp_flight.h"

using namespace sim;

struct Cond { double wind, dir, turb, payload; unsigned seed; };
static std::string g_warm;   // мережа іншого апарата як початкове наближення (якщо для цього ще немає)
static float g_ffxy = -1, g_ffz = -1, g_kp = 0, g_ki = 0, g_tau = 0.3f, g_gust = 0, g_wing = 0, g_hold = 0;   // ai_rl ctl: перевизначити частку сили наперед

static Cond randomCond(std::mt19937& g, bool strong_mix = true) {
    auto u = [&g](double a, double b) { return a + (b - a) * prng::uni01(g); };
    Cond c;
    c.wind = (!strong_mix || prng::uni01(g) < 0.5) ? u(0, 11) : u(7, 12); c.dir = u(0, 6.2831853); c.turb = u(0.5, 1.5);   // половина — сильний вітер: там контролерові найважче
    c.payload = prng::uni01(g) < 0.4 ? u(0.2, 1.0) : 0.0;
    c.seed = unsigned(g() % 100000u) + 1u;
    return c;
}

struct Result { double cost = 0, pos_rms = 0, wh = 0, h_rms = 0, z_rms = 0, wander = 0; bool crash = false; };

class Env {
public:
    Env(const AircraftFile& a, const AeroTables& t) : a_(a), t_(t) {}
    // зліт і 4 с висіння звичайним контролером — спільний початок для всіх кандидатів у цих умовах
    QpFlightRig start(const Cond& c) const {
        QpFlightRig r(a_, t_, c.seed);
        r.noise = SensorNoise::realistic();
        WindConfig w; w.mean20ft = Vec3d{c.wind * std::cos(c.dir), c.wind * std::sin(c.dir), 0}; w.turbulence = c.turb;
        r.setWind(w, c.seed + 11);
        if (c.payload > 0) r.sim.setPayload(c.payload);
        if (g_ffxy >= 0) { r.ctl.cfg.dist_ff_xy = g_ffxy; r.ctl.cfg.dist_ff_z = g_ffz; r.ctl.cfg.wind_kp_boost = g_kp; r.ctl.cfg.wind_ki_boost = g_ki; r.ctl.cfg.dist_ff_tau = g_tau; r.ctl.cfg.gust_ff = g_gust; r.ctl.cfg.wind_wing_up = g_wing * 0.0174533f; r.ctl.cfg.hold_gps_tau = g_hold; }
        using S = fc::QpStep;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {0, 0, 30}; hv.hold_s = 1e6f;
        r.setMission({tk, hv});
        double t_hold = -1;
        r.run(80, [&](const QpFlightRig& g) {
            if (g.ctl.phase() == fc::QpPhase::VtolHold && t_hold < 0) t_hold = g.sim.time();
            return t_hold > 0 && g.sim.time() > t_hold + 4;
        });
        return r;
    }
    template <class Pol>
    Result episode(const QpFlightRig& base, const Pol* pol, double T = 30) const {
        QpFlightRig r = base;
        if (pol) {
            if constexpr (Pol::kOut > 3) { r.pilot2 = *pol; r.pilot2.reset(); r.pilot2_on = true; }
            else { r.pilot = *pol; r.pilot.reset(); r.pilot_on = true; }
        }
        Result o;
        const double wh0 = r.sim.battery().usedWh(), t0 = r.sim.time();
        const fc::Vec3 tgt = r.ctl.holdPoint();
        double e2 = 0, v2 = 0, h2 = 0, z2 = 0, sx = 0, sy = 0, sxx = 0, syy = 0; long n = 0;
        r.run(T, [&](const QpFlightRig& g) {
            const auto& s = g.sim.state();
            const double dx = s.pos.x - tgt.x, dy = s.pos.y - tgt.y, dz = s.pos.z - tgt.z;
            e2 += dx * dx + dy * dy + 2 * dz * dz; h2 += dx * dx + dy * dy; z2 += dz * dz; sx += dx; sy += dy; sxx += dx * dx; syy += dy * dy;
            const Vec3d v = g.sim.velWorld(); v2 += v.x * v.x + v.y * v.y + v.z * v.z;
            ++n;
            const Vec3d eu = g.sim.eulerRad();
            if (g.sim.crashed() || std::fabs(eu.x) > 1.0 || std::fabs(eu.y) > 1.0) { o.crash = true; return true; }
            return false;
        });
        const double used = r.sim.battery().usedWh() - wh0, dur = std::fmax(r.sim.time() - t0, 1.0);
        o.pos_rms = std::sqrt(e2 / std::max(n, 1L));
        o.wh = used; o.h_rms = std::sqrt(h2 / std::max(n, 1L)); o.z_rms = std::sqrt(z2 / std::max(n, 1L));
        { const double nn = double(std::max(n, 1L)); o.wander = std::sqrt(std::fmax(0.0, sxx / nn - (sx / nn) * (sx / nn) + syy / nn - (sy / nn) * (sy / nn))); }
        const double watts = used * 3600 / dur;
        o.cost = e2 / std::max(n, 1L) + 0.1 * v2 / std::max(n, 1L) + 0.0003 * watts + (o.crash ? 50.0 : 0.0);
        return o;
    }
private:
    const AircraftFile& a_;
    const AeroTables& t_;
};

template <class F>
static void parallelFor(int n, int threads, F f) {
    std::atomic<int> next{0};
    std::vector<std::thread> th;
    for (int k = 0; k < threads; ++k) th.emplace_back([&] { for (int i; (i = next++) < n;) f(i); });
    for (auto& t : th) t.join();
}

template <class Pol>
static int runMain(int argc, char** argv, const std::string mode, const std::string path, const std::string logpath) {
    const Pol* none = nullptr;
    const AircraftFile a = AircraftFile::load(activeAircraftPath());
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    const Env env(a, tb);
    const int threads = int(std::max(1u, std::thread::hardware_concurrency()));

    // відкладені умови: за ними обирається найкраща мережа (на них не вчиться)
    std::mt19937 gh(777);   // відкладені — як і раніше 0…11 м/с рівномірно (порівнянність між запусками)
    std::vector<Cond> held;
    for (int i = 0; i < 24; ++i) held.push_back(randomCond(gh, false));
    std::vector<QpFlightRig*> held_base(held.size(), nullptr);
    std::vector<Result> held_ref(held.size());
    auto prepareHeld = [&] {
        parallelFor(int(held.size()), threads, [&](int i) {
            held_base[size_t(i)] = new QpFlightRig(env.start(held[size_t(i)]));
            held_ref[size_t(i)] = env.episode(*held_base[size_t(i)], none);
        });
    };
    auto evalHeld = [&](const Pol& p, double* pos_b, double* pos_p, double* wh_b, double* wh_p, int* crashes) {
        std::vector<Result> rs(held.size());
        parallelFor(int(held.size()), threads, [&](int i) { rs[size_t(i)] = env.episode(*held_base[size_t(i)], &p); });
        double f = 0; *pos_b = *pos_p = *wh_b = *wh_p = 0; *crashes = 0;
        for (size_t i = 0; i < held.size(); ++i) {
            f += (held_ref[i].cost - rs[i].cost) / (held_ref[i].cost + 0.1);
            *pos_b += held_ref[i].pos_rms; *pos_p += rs[i].pos_rms; *wh_b += held_ref[i].wh; *wh_p += rs[i].wh; *crashes += rs[i].crash;
        }
        const double n = double(held.size());
        *pos_b /= n; *pos_p /= n; *wh_b /= n; *wh_p /= n;
        return f / n;
    };

    if (mode == "ctl") {   // ai_rl ctl xy z — контролер із часткою сили наперед проти без неї (без мережі)
        const float xy = argc > 2 ? float(std::atof(argv[2])) : 0.5f, z = argc > 3 ? float(std::atof(argv[3])) : 0.5f;
        std::vector<Result> r0(held.size()), r1(held.size());
        const float wing_def = 6; g_ffxy = 0; g_ffz = 0; g_gust = 0; g_wing = getenv("WING0") ? float(std::atof(getenv("WING0"))) : wing_def; g_hold = 0;
        if (getenv("STRONG")) { std::vector<Cond> h2; std::vector<QpFlightRig*> b2; for (size_t i = 0; i < held.size(); ++i) if (held[i].wind > 7) { h2.push_back(held[i]); b2.push_back(nullptr); } held = h2; r0.resize(held.size()); r1.resize(held.size()); }
        parallelFor(int(held.size()), threads, [&](int i) { QpFlightRig b = env.start(held[size_t(i)]); r0[size_t(i)] = env.episode(b, none); });
        g_ffxy = xy; g_ffz = z; g_kp = argc > 4 ? float(std::atof(argv[4])) : 0; g_ki = argc > 5 ? float(std::atof(argv[5])) : 0; g_tau = argc > 6 ? float(std::atof(argv[6])) : 0.3f;
        g_gust = getenv("GUST") ? float(std::atof(getenv("GUST"))) : 0; g_wing = getenv("WING") ? float(std::atof(getenv("WING"))) : wing_def; g_hold = getenv("HOLD") ? float(std::atof(getenv("HOLD"))) : 0;
        parallelFor(int(held.size()), threads, [&](int i) { QpFlightRig b = env.start(held[size_t(i)]); r1[size_t(i)] = env.episode(b, none); });
        double p0 = 0, p1 = 0, w0 = 0, w1 = 0, f = 0, q0 = 0, q1 = 0; int c1 = 0;
        for (size_t i = 0; i < held.size(); ++i) {
            p0 += r0[i].pos_rms; p1 += r1[i].pos_rms; q0 += r0[i].wander; q1 += r1[i].wander; w0 += r0[i].wh; w1 += r1[i].wh; c1 += r1[i].crash; f += (r0[i].cost - r1[i].cost) / (r0[i].cost + 0.1);
            std::printf("  вітер %4.1f, пориви ×%.1f, вантаж %.1f: %.2f → %.2f м (блукання %.2f → %.2f), %.2f → %.2f Вт·год%s\n", held[i].wind, held[i].turb, held[i].payload, r0[i].pos_rms, r1[i].pos_rms, r0[i].wander, r1[i].wander, r0[i].wh, r1[i].wh, r1[i].crash ? " АВАРІЯ" : "");
        }
        const double n = double(held.size());
        std::printf("блукання %.3f → %.3f м | утримання τ %.0f с | порив %.2f крило %.0f° | сила наперед xy %.2f z %.2f, kp+%.1f ki+%.1f: положення %.3f → %.3f м, енергія %.3f → %.3f Вт·год, аварій %d, оцінка %+.1f%%\n", q0 / n, q1 / n, g_hold, g_gust, g_wing, xy, z, g_kp, g_ki, p0 / n, p1 / n, w0 / n, w1 / n, c1, 100 * f / n);
        return 0;
    }
    if (mode == "eval") {
        Pol p;
        if (!p.load(path)) { std::printf("Немає %s — спершу навчання (ai_rl train)\n", path.c_str()); return 1; }
        prepareHeld();
        double pb, pp, wb, wp; int cr;
        const double f = evalHeld(p, &pb, &pp, &wb, &wp, &cr);
        std::printf("Покоління %d. Відкладені умови (24): похибка положення RMS %.2f → %.2f м, енергія за 30 с %.2f → %.2f Вт·год, аварій %d, оцінка %+.1f%%\n",
                    p.generation, pb, pp, wb, wp, cr, 100 * f);
        for (size_t i = 0; i < held.size(); ++i) {
            const Result r = env.episode(*held_base[i], &p);
            std::printf("  вітер %4.1f м/с, пориви ×%.1f, вантаж %.1f кг: %.2f → %.2f м (гориз. %.2f → %.2f, верт. %.2f → %.2f), %.2f → %.2f Вт·год%s\n", held[i].wind, held[i].turb, held[i].payload,
                        held_ref[i].pos_rms, r.pos_rms, held_ref[i].h_rms, r.h_rms, held_ref[i].z_rms, r.z_rms, held_ref[i].wh, r.wh, r.crash ? "  АВАРІЯ" : "");
        }
        return 0;
    }

    const int gens = argc > 2 ? std::atoi(argv[2]) : 300;
    const bool cont = argc > 3 && std::string(argv[3]) == "cont";
    const int pairs = getenv("RL_PAIRS") ? std::atoi(getenv("RL_PAIRS")) : 32, K = getenv("RL_K") ? std::atoi(getenv("RL_K")) : 4;
    const double sigma = 0.03, lr = 0.01, wd = 0.005;
    Pol mean;
    if (cont && mean.load(path + ".cur")) std::printf("Продовжую з поточної точки, покоління %d\n", mean.generation);
    else if (cont && mean.load(path)) std::printf("Продовжую з покоління %d (оцінка %+.1f%%)\n", mean.generation, 100 * mean.score);
    else if (cont && !g_warm.empty() && mean.load(g_warm)) { mean.generation = 0; mean.score = 0; std::printf("Початок — з мережі іншого апарата (%s), далі вчиться на цьому\n", g_warm.c_str()); }
    else { mean.initRandom(1); mean.generation = 0; mean.score = 0; }
    std::printf("ШІ-пілот з нуля: %d параметрів, %d потоків, %d пар кандидатів × %d умов на покоління\n", Pol::kParams, threads, pairs, K);
    std::printf("Готую 24 відкладені умови...\n");
    std::fflush(stdout);
    prepareHeld();
    double best = -1e9;
    { double pb, pp, wb, wp; int cr; best = evalHeld(mean, &pb, &pp, &wb, &wp, &cr); std::printf("Старт: оцінка %+.1f%% (0 — як звичайний контролер)\n", 100 * best); }
    { Pol b; if (cont && b.load(path) && b.score > best) best = b.score; }
    { Pol b; if (!b.load(path)) { mean.score = float(best); mean.save(path); } }   // рекорд збереженої найкращої не губимо після перерви
    std::filesystem::create_directories("data");
    std::ofstream log(logpath, cont ? std::ios::app : std::ios::trunc);
    if (!cont) log << "gen,train_fit,held_fit,pos_base,pos_pilot,wh_base,wh_pilot,crashes,seconds\n";
    const size_t P = mean.w.size();
    std::vector<double> m1(P, 0), m2(P, 0);
    std::mt19937 g(unsigned(12345 + mean.generation * 7919));
    for (int gen = 0; gen < gens; ++gen) {
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<Cond> conds;
        for (int k = 0; k < K; ++k) conds.push_back(randomCond(g));
        std::vector<QpFlightRig*> base(static_cast<size_t>(K), nullptr);
        std::vector<Result> ref(static_cast<size_t>(K));
        parallelFor(K, threads, [&](int k) { base[size_t(k)] = new QpFlightRig(env.start(conds[size_t(k)])); ref[size_t(k)] = env.episode(*base[size_t(k)], none, 20); });
        std::vector<std::vector<float>> eps(static_cast<size_t>(pairs), std::vector<float>(P));
        for (auto& e : eps) for (float& v : e) v = prng::normal<float>(0, 1)(g);
        std::vector<double> fit(size_t(2 * pairs), 0);
        parallelFor(2 * pairs * K, threads, [&](int job) {
            const int c = job / K, k = job % K, pi = c / 2;
            const float sgn = c % 2 ? -1.0f : 1.0f;
            Pol cand = mean;
            for (size_t j = 0; j < P; ++j) cand.w[j] += sgn * float(sigma) * eps[size_t(pi)][j];
            const Result r = env.episode(*base[size_t(k)], &cand, 20);
            static std::mutex mu;
            std::lock_guard<std::mutex> lk(mu);
            fit[size_t(c)] += (ref[size_t(k)].cost - r.cost) / (ref[size_t(k)].cost + 0.1) / K;
        });
        for (auto* b : base) delete b;
        // ранги замість сирих оцінок: стійко до одного вдалого/невдалого польоту
        std::vector<int> idx(fit.size());
        for (size_t i = 0; i < idx.size(); ++i) idx[i] = int(i);
        std::sort(idx.begin(), idx.end(), [&](int x, int y) { return fit[size_t(x)] < fit[size_t(y)]; });
        std::vector<double> rk(fit.size());
        for (size_t r = 0; r < idx.size(); ++r) rk[size_t(idx[r])] = double(r) / double(idx.size() - 1) - 0.5;
        std::vector<double> grad(P, 0);
        for (int pi = 0; pi < pairs; ++pi) {
            const double d = rk[size_t(2 * pi)] - rk[size_t(2 * pi + 1)];
            for (size_t j = 0; j < P; ++j) grad[j] += d * eps[size_t(pi)][j];
        }
        const int step = mean.generation + 1;
        for (size_t j = 0; j < P; ++j) {   // Adam, сходження вгору
            const double gj = grad[j] / (2 * pairs * sigma) - wd * mean.w[j];
            m1[j] = 0.9 * m1[j] + 0.1 * gj; m2[j] = 0.999 * m2[j] + 0.001 * gj * gj;
            const double mh = m1[j] / (1 - std::pow(0.9, step)), vh = m2[j] / (1 - std::pow(0.999, step));
            mean.w[j] += float(lr * mh / (std::sqrt(vh) + 1e-8));
        }
        mean.generation = step;
        mean.save(path + ".cur");   // поточна точка (для продовження після перерви), найкраща — окремо
        double tr = 0; for (double f : fit) tr += f; tr /= double(fit.size());
        double hf = 0, pb = 0, pp = 0, wb = 0, wp = 0; int cr = 0;
        const bool check = gen % 10 == 9 || gen == gens - 1;
        if (check) {
            hf = evalHeld(mean, &pb, &pp, &wb, &wp, &cr);
            if (hf > best && cr == 0) { best = hf; mean.score = float(hf); mean.save(path); }
        }
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (check) std::printf("покоління %4d: навчальні %+5.1f%%, відкладені %+5.1f%% (положення %.2f → %.2f м, енергія %.2f → %.2f Вт·год, аварій %d)%s, %.0f с\n",
                               step, 100 * tr, 100 * hf, pb, pp, wb, wp, cr, hf >= best && cr == 0 ? " ★ збережено" : "", sec);
        else std::printf("покоління %4d: навчальні %+5.1f%%, %.0f с\n", step, 100 * tr, sec);
        std::fflush(stdout);
        log << step << "," << tr << "," << (check ? hf : NAN) << "," << pb << "," << pp << "," << wb << "," << wp << "," << cr << "," << sec << "\n";
        log.flush();
    }
    std::printf("Готово. Найкраща на відкладених умовах: %+.1f%% — %s\n", 100 * best, path.c_str());
    return 0;
}

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "train";
    const AircraftFile af = AircraftFile::load(activeAircraftPath());
    std::printf("Апарат: %s\n", activeAircraftPath().c_str());
    if (!mode.empty() && mode.back() == '2') {   // train2 / eval2 / ctl2: v2 — ще й повітряна швидкість, оберти моторів і кут носа
        mode.pop_back();
        if (!aiVariant(af).empty()) g_warm = "ai/policy_hover2.txt";
        return runMain<ai::FlightPolicy2>(argc, argv, mode, aiFile(af, "ai/policy_hover2.txt"), aiFile(af, "data/rl_log2.csv"));
    }
    return runMain<ai::FlightPolicy>(argc, argv, mode, aiFile(af, "ai/policy_hover.txt"), aiFile(af, "data/rl_log.csv"));
}
