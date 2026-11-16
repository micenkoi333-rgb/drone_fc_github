// qp_tricks.h — навчання пілотажних трюків (fc/qp_tricks.h) у симуляторі: спроба трюку → оцінка → кращі спроби → нові.
// Метод перехресної ентропії (CEM, навчання з підкріпленням «чорною скринькою»): покоління по 10 спроб, параметри наступних
// спроб беруться навколо 3 найкращих; 100 спроб = 10 поколінь. Оцінка — з правди моделі (кут атаки, перевантаження, втрата висоти).
// Навчені параметри — ai/tricks_learned.ini (рядок на трюк: 7 чисел u ∈ [0, 1] і оцінка), їх читає GUI.
#pragma once
#include <algorithm>
#include <cstdio>
#include <atomic>
#include <functional>
#include <thread>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>
#include "../fc/qp_tricks.h"
#include "qp_flight.h"
#include "portable_rng.h"

namespace sim {

struct TrickResult {
    double score = -100;
    bool completed = false, crashed = false, stalled = false;
    double alt_loss = 0, min_airspeed = 1e9, max_g = 0, max_alpha_deg = 0, loop_deg = 0, time_s = 0, max_yaw_rate = 0, max_roll_dev_deg = 0;
    double roll_deg = 0, heading_deg = 0, alt_dev = 0, stop_dist = 0, drift = 0, spun_deg = 0, nose_max_deg = 0, flip_deg = 0, flip_loss = 0, tumble_rot = 0;
    std::string extra;
    std::string text() const {
        char b[420];
        std::snprintf(b, sizeof b, "оцінка %.0f: %s; втрата висоти %.1f м, мін. швидкість %.1f м/с, перевантаження до %.1f g, кут атаки до %.0f°%s%s",
                      score, crashed ? "АВАРІЯ" : (completed ? "виконано" : "не виконано"), alt_loss, min_airspeed, max_g, max_alpha_deg,
                      loop_deg > 0 ? (", петля " + std::to_string(int(loop_deg)) + "°").c_str() : "", extra.c_str());
        return b;
    }
};

// Спостерігач трюку: міряє все, що потрібно для оцінки (за правдою моделі), і рахує оцінку
class TrickJudge {
public:
    void begin(const QpFlightRig& r, fc::Trick t) {
        trick_ = t; res_ = TrickResult{};
        z0_ = r.sim.state().pos.z; zmin_ = z0_; zmax_dev_ = 0; yaw0_ = r.sim.eulerRad().z; v_prev_ = r.sim.velWorld(); first_ = true; roll_int_ = 0; stall_t_ = 0; stall_z_ = z0_; t_ = 0;
    }
    void sample(const QpFlightRig& r, const fc::TrickPilot& p, double dt) {
        t_ += dt;
        const QpState& s = r.sim.state();
        const Vec3d v = r.sim.velWorld();
        if (!first_) {   // перевантаження: |a − g| / g
            const Vec3d a = (v - v_prev_) * (1.0 / dt) + Vec3d{0, 0, QuadPlaneSim::kG};
            res_.max_g = std::fmax(res_.max_g, a.norm() / QuadPlaneSim::kG);
        }
        first_ = false; v_prev_ = v;
        res_.min_airspeed = std::fmin(res_.min_airspeed, r.sim.airspeed());
        const double alpha = r.sim.aeroNow().wing_alpha * 57.29578;
        res_.max_alpha_deg = std::fmax(res_.max_alpha_deg, alpha);
        res_.max_yaw_rate = std::fmax(res_.max_yaw_rate, std::fabs(s.omega.z));
        roll_int_ += s.omega.x * dt;   // поворот навколо поздовжньої осі (у петлі кути Ейлера непридатні)
        res_.max_roll_dev_deg = std::fmax(res_.max_roll_dev_deg, std::fabs(roll_int_) * 57.29578);
        if (trick_ == fc::Trick::Stall) {
            if (alpha > 15.0) { if (stall_t_ == 0) stall_z_ = s.pos.z; stall_t_ += dt; }
            if (stall_t_ > 0) zmin_ = std::fmin(zmin_, s.pos.z);
        } else zmin_ = std::fmin(zmin_, s.pos.z);
        zmax_dev_ = std::fmax(zmax_dev_, std::fabs(s.pos.z - z0_));
        if (trick_ == fc::Trick::Loop) res_.loop_deg = p.loopAngle() * 57.29578;
    }
    TrickResult finish(const QpFlightRig& r, const fc::TrickPilot& p) {
        TrickResult o = finishImpl(r, p);
        if (p.aborted()) { o.crashed = true; o.completed = false; o.score = -100; o.extra += std::string(", захист перервав трюк: ") + p.abortReason(); }
        return o;
    }
private:
    TrickResult finishImpl(const QpFlightRig& r, const fc::TrickPilot& p) {
        TrickResult& o = res_;
        o.time_s = t_;
        const double ground = r.sim.derived().cg_z + 15.0;   // нижче 15 м над землею — вважаємо, що трюк не вдався
        o.crashed = r.sim.crashed() || r.sim.diverged() || zmin_ < ground;
        if (o.min_airspeed > 1e8) o.min_airspeed = 0;
        const double z1 = r.sim.state().pos.z;
        double dpsi = (r.sim.eulerRad().z - yaw0_) * 57.29578;
        while (dpsi > 180) dpsi -= 360;
        while (dpsi < -180) dpsi += 360;
        o.heading_deg = dpsi; o.alt_dev = zmax_dev_;
        const double gpen = 15.0 * std::fmax(0.0, o.max_g - 4.5);
        if (trick_ == fc::Trick::Roll) {
            o.alt_loss = std::fmax(0.0, z0_ - zmin_);
            o.roll_deg = p.rollAngle() * 57.29578;
            o.completed = p.succeeded() && !o.crashed;
            o.score = o.crashed ? -100 : (o.completed ? 100.0 : 60.0 * std::fmin(std::fmax(o.roll_deg, 0.0) / 360.0, 1.0))
                      - 1.5 * o.alt_loss - 1.0 * std::fabs(z1 - z0_) - 0.5 * std::fabs(dpsi) - gpen - 0.5 * o.time_s;
            char b[160]; std::snprintf(b, sizeof b, ", оберт %.0f°, курс змінився на %.0f°", o.roll_deg, dpsi); o.extra = b;
            o.score = o.crashed ? -100.0 : std::fmax(o.score, -95.0);
            return o;
        }
        if (trick_ == fc::Trick::Hammerhead) {
            o.alt_loss = std::fmax(0.0, z0_ - zmin_);
            o.completed = p.succeeded() && !o.crashed;
            const double dev = std::fabs(180.0 - std::fabs(dpsi));
            o.score = o.crashed ? -100 : (o.completed ? 100.0 : (p.pivotDone() ? 50.0 : 8.0 * std::min(p.phase(), 2)))
                      - (o.completed ? 0.4 * dev : 0.0) - 1.0 * std::fabs(z1 - z0_) - 1.0 * o.alt_loss - gpen - 0.3 * o.time_s;
            char b[200]; std::snprintf(b, sizeof b, ", розворот %s, курс змінився на %.0f° (ціль 180°), висота виходу %+.1f м", p.pivotDone() ? "так" : "ні", dpsi, z1 - z0_); o.extra = b;
            o.score = o.crashed ? -100.0 : std::fmax(o.score, -95.0);
            return o;
        }
        if (trick_ == fc::Trick::Harrier) {
            o.alt_loss = std::fmax(0.0, z0_ - zmin_);
            o.stop_dist = p.stopDistance(); o.drift = p.hoverDrift(); o.spun_deg = p.spunAngle() * 57.29578;
            o.completed = p.succeeded() && !o.crashed;
            o.score = o.crashed ? -100 : (o.completed ? 100.0 : 25.0 * p.stopped() + 25.0 * p.spinDone())
                      - 0.5 * std::fmax(0.0, o.stop_dist - 15.0) - 2.0 * std::fmax(0.0, o.alt_dev - 1.0) - 4.0 * o.drift - 0.3 * std::fabs(dpsi) - gpen - 0.2 * o.time_s;
            char b[220]; std::snprintf(b, sizeof b, ", гальмівний шлях %.0f м, пірует %.0f°, знесло %.1f м, найбільше відхилення висоти %.1f м", o.stop_dist, o.spun_deg, o.drift, o.alt_dev); o.extra = b;
            o.score = o.crashed ? -100.0 : std::fmax(o.score, -95.0);
            return o;
        }
        if (trick_ == fc::Trick::Cobra) {
            o.alt_loss = std::fmax(0.0, z0_ - zmin_);
            o.completed = p.succeeded() && !o.crashed;
            const double nm = p.noseMax(); o.nose_max_deg = nm;
            o.score = o.crashed ? -100 : (o.completed ? 100.0 : 50.0 * std::clamp(nm / 90.0, 0.0, 1.0))
                      - 2.0 * std::fmax(0.0, 85.0 - nm) - 1.0 * std::fmax(0.0, o.alt_dev - 8.0) - 0.4 * std::fabs(dpsi) - gpen - 0.3 * o.time_s
                      + 0.8 * std::fmax(0.0, 10.0 - o.min_airspeed);
            char b[200]; std::snprintf(b, sizeof b, ", ніс до %.0f°, швидкість впала до %.1f м/с, відхилення висоти %.1f м", nm, o.min_airspeed > 1e8 ? 0.0 : o.min_airspeed, o.alt_dev); o.extra = b;
            o.score = o.crashed ? -100.0 : std::fmax(o.score, -95.0);
            return o;
        }
        if (trick_ == fc::Trick::Tumble) {
            o.alt_loss = std::fmax(0.0, z0_ - zmin_);
            o.completed = p.succeeded() && !o.crashed;
            o.tumble_rot = p.tumbleRotations();
            const double drama = std::fmin(o.tumble_rot / 2.0, 1.0);
            o.score = o.crashed ? -100 : (o.completed ? 60.0 + 40.0 * drama : 20.0 * drama)
                      - 1.0 * std::fmax(0.0, o.alt_loss - 15.0) - gpen - 0.2 * o.time_s;
            char b[200]; std::snprintf(b, sizeof b, ", кульбітів %.1f, нижче входу на %.1f м, %s", o.tumble_rot, o.alt_loss, p.recovered() ? "врятувався" : "не врятувався"); o.extra = b;
            o.score = o.crashed ? -100.0 : std::fmax(o.score, -95.0);
            return o;
        }
        if (trick_ == fc::Trick::Flip) {
            o.alt_loss = std::fmax(0.0, z0_ - zmin_);
            o.completed = p.succeeded() && !o.crashed;
            const double fd = p.flipAngle() * 57.29578; o.flip_deg = fd; o.flip_loss = p.flipLoss();
            o.score = o.crashed ? -100 : (o.completed ? 100.0 : 50.0 * std::clamp(fd / 360.0, 0.0, 1.0) + 10.0 * p.stopped())
                      - 3.0 * std::fmax(0.0, p.flipLoss() - 1.0) - 0.3 * std::fabs(dpsi) - gpen - 0.2 * o.time_s;
            char b[200]; std::snprintf(b, sizeof b, ", сальто %.0f°, втрата висоти на сальто %.1f м", fd, p.flipLoss()); o.extra = b;
            o.score = o.crashed ? -100.0 : std::fmax(o.score, -95.0);
            return o;
        }
        if (trick_ == fc::Trick::Loop) {
            o.alt_loss = z0_ - std::fmin(z1, zmin_);
            o.completed = p.completedLoop() && !p.active() && !o.crashed;
            if (o.crashed) o.score = -100;
            else {
                o.score = o.completed ? 100.0 : 60.0 * std::fmin(p.loopAngle() / (2 * 3.14159265), 1.0);
                o.score -= 1.5 * std::fabs(z0_ - z1);                 // вийти на тій самій висоті
                o.score -= 0.3 * o.max_roll_dev_deg;                 // петля рівна, без крену
                o.score -= 15.0 * std::fmax(0.0, o.max_g - 4.5);     // перевантаження конструкції
                o.score -= 0.5 * o.time_s;                            // швидше — краще
            }
        } else {
            o.stalled = stall_t_ >= 0.3;
            o.alt_loss = o.stalled ? stall_z_ - zmin_ : 0;
            o.completed = o.stalled && !p.active() && p.phase() >= 4 && !o.crashed && r.sim.airspeed() > 8.0;
            if (o.crashed) o.score = -100;
            else if (!o.stalled) o.score = -50 + 30 * std::fmin(1.0, o.max_alpha_deg / 15.0);   // так і не звалився
            else {
                o.score = o.completed ? 100.0 : 20.0;
                o.score -= 3.0 * o.alt_loss;                              // вийти з мінімальною втратою висоти
                o.score -= 10.0 * std::fmax(0.0, o.max_yaw_rate - 1.0);   // без штопора
                o.score -= 0.2 * o.max_roll_dev_deg;
                o.score -= 15.0 * std::fmax(0.0, o.max_g - 4.5);
                o.score -= 0.5 * o.time_s;
            }
        }
        return o;
    }

private:
    fc::Trick trick_ = fc::Trick::Loop;
    TrickResult res_;
    double z0_ = 0, zmin_ = 0, roll_int_ = 0, stall_t_ = 0, stall_z_ = 0, t_ = 0, zmax_dev_ = 0, yaw0_ = 0;
    Vec3d v_prev_;
    bool first_ = true;
};

struct TrajPoint { float t, x, y, z; float n[3], up[3]; float tilt[4]; };

// Траєкторія в осях входу в трюк: x — уздовж початкового курсу, y — ліворуч, z — висота відносно входу
inline TrajPoint trajPoint(const QpFlightRig& r, const Vec3d& p0, double yaw0, double t0) {
    const QpState& s = r.sim.state();
    const double c = std::cos(yaw0), sn = std::sin(yaw0), dx = s.pos.x - p0.x, dy = s.pos.y - p0.y;
    const Vec3d nw = s.q.rotate(Vec3d{1, 0, 0}), uw = s.q.rotate(Vec3d{0, 0, 1});
    TrajPoint tp{float(r.sim.time() - t0), float(dx * c + dy * sn), float(-dx * sn + dy * c), float(s.pos.z - p0.z),
                 {float(nw.x * c + nw.y * sn), float(-nw.x * sn + nw.y * c), float(nw.z)}, {float(uw.x * c + uw.y * sn), float(-uw.x * sn + uw.y * c), float(uw.z)}, {}};
    for (int i = 0; i < 4; ++i) tp.tilt[i] = float(r.sim.tilt(i));
    return tp;
}

class TrickTrainer {
public:
    struct Episode { int index = 0; fc::Trick trick = fc::Trick::Loop; float u[fc::kTrickParams] = {}; TrickResult r; std::vector<TrajPoint> traj; };
    // Що ШІ «думає» після кожного покоління: розподіл, з якого бере спроби (середнє й розкид кожного параметра), до й після оновлення
    struct Generation {
        int index = 0;
        float mu_before[fc::kTrickParams], sd_before[fc::kTrickParams], mu_after[fc::kTrickParams], sd_after[fc::kTrickParams];
        float elite_mean[fc::kTrickParams];
        double elite_scores[3] = {0, 0, 0}, mean_score = 0;
        int completed = 0, crashed = 0, size = 0;
    };
    std::function<void(const Generation&)> on_generation;
    bool record_traj = false;
    float mu[fc::kTrickParams], sd[fc::kTrickParams];
    int population = 10, elite = 3;
    int threads = 0;   // скільки ядер рахують спроби одночасно; 0 — усі
    double trick_time_limit = 40;
    std::function<void(const Episode&, const Episode& best)> on_episode;   // прогрес (номер спроби, оцінка, найкраща)
    std::function<bool()> cancel;                                         // true — зупинити навчання

    TrickTrainer(const AircraftFile& a, const AeroTables& t) : a_(a), t_(t) {}

    // Апарат у польоті літаком: зліт, перехід, крейсер на 60 м, 8 с усталеного польоту. Будується раз і копіюється для кожної спроби.
    // robust > 0 — кожна спроба летить ще й у випадкових умовах (вітер до 6 м/с з поривами, вантаж до 0.5 кг, інші шуми датчиків):
    // оцінка — середня, а аварія в будь-яких умовах — провал. Так навчені параметри не «підігнані» під один штиль.
    int robust = 0;
    std::unique_ptr<QpFlightRig> cruiseRig(unsigned seed = 11, double wind = 0, double wind_dir = 0, double payload = 0) const {
        auto r = std::make_unique<QpFlightRig>(a_, t_, seed);
        if (wind > 0) { WindConfig w; w.mean20ft = Vec3d{wind * std::cos(wind_dir), wind * std::sin(wind_dir), 0}; w.turbulence = 1.2; r->setWind(w, seed + 5); }
        if (payload > 0) r->sim.setPayload(payload);
        using S = fc::QpStep;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30};
        S hv; hv.type = S::HoverAt; hv.p = {6000, 0, 60}; hv.hold_s = 5;
        r->setMission({tk, hv});
        r->run(150, [](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Cruise && g.ctl.phaseTime() > 8 && g.ctl.vtolWeight() == 0; });
        return r;
    }

    Episode run(fc::Trick trick, const float u[fc::kTrickParams]) {
        prepare();
        QpFlightRig r = *base_;   // та сама точка старту для кожної спроби
        Episode ep; ep.trick = trick;
        for (int i = 0; i < fc::kTrickParams; ++i) ep.u[i] = u[i];
        ep.r = fly(r, trick, u, record_traj ? &ep.traj : nullptr);
        if (!variants_.empty()) {
            const size_t k = size_t(variant_next_++) % variants_.size();
            QpFlightRig rv = *variants_[k];
            const TrickResult o = fly(rv, trick, u, nullptr);
            const bool crash = ep.r.crashed || o.crashed;
            ep.r.score = crash ? std::fmin(ep.r.score, o.score) : 0.5 * (ep.r.score + o.score);
            ep.r.crashed = crash; ep.r.completed = ep.r.completed && o.completed;
            ep.r.extra += o.crashed ? ", у вітрі/з вантажем — АВАРІЯ" : (o.completed ? ", у вітрі/з вантажем — вдалося" : ", у вітрі/з вантажем — не вийшло");
        }
        return ep;
    }
    void prepare() {
        std::lock_guard<std::mutex> lk(prep_mu_);
        if (!base_) base_ = cruiseRig();
        if (robust > 0 && variants_.empty()) {
            std::mt19937 g(777u);
            prng::uniform<double> U(0, 1);
            for (int i = 0; i < robust; ++i) variants_.push_back(cruiseRig(101u + unsigned(i) * 17u, 2.0 + 4.0 * U(g), 6.2832 * U(g), U(g) < 0.5 ? 0.0 : 0.5 * U(g)));
        }
    }

    // Політ трюку з поточного стану rig; керування повертається автопілоту після трюку
    TrickResult fly(QpFlightRig& r, fc::Trick trick, const float u[fc::kTrickParams], std::vector<TrajPoint>* traj = nullptr) const {
        const Vec3d p0 = r.sim.state().pos;
        const double yaw0 = r.sim.eulerRad().z, t0 = r.sim.time();
        fc::TrickPilot pilot;
        pilot.v_cruise = r.ctl.cfg.af.v_cruise; pilot.v_stall = r.ctl.cfg.af.v_stall; pilot.cfg = &r.ctl.cfg; pilot.guard.ground_z = r.ctl.home().z;
        pilot.start(trick, u);
        TrickJudge judge; judge.begin(r, trick);
        double last = r.sim.time();
        r.override_act = [&](const fc::QpEstimate& e, fc::QpActuators& act) { return pilot.update(0.002f, e, act); };
        const long n = long(trick_time_limit * 1000);
        for (long i = 0; i < n && pilot.active(); ++i) {
            r.step();
            if (i % 10 == 0) { judge.sample(r, pilot, r.sim.time() - last); last = r.sim.time(); }
            if (traj && i % 50 == 0) traj->push_back(trajPoint(r, p0, yaw0, t0));
            if (r.sim.crashed() || r.sim.diverged()) break;
        }
        r.override_act = nullptr;
        return judge.finish(r, pilot);
    }

    // Навчання: episodes спроб, повертає найкращу
    // start_u — продовжити з уже навченого: пошук починається навколо нього з меншим розкидом, а сам він спершу перелітається
    // (спроба 0) і стає найкращою, доки нова спроба не обжене його, — тож повторне навчання ніколи не погіршує результат.
    Episode train(fc::Trick trick, int episodes, unsigned seed = 1, const float* start_u = nullptr) {
        std::mt19937 rng(seed);
        prng::normal<float> N(0, 1);
        for (int i = 0; i < fc::kTrickParams; ++i) { mu[i] = start_u ? std::clamp(start_u[i], 0.0f, 1.0f) : 0.5f; sd[i] = start_u ? 0.12f : 0.3f; }
        int gen_index = 0;   // з нуля: нічого не знає — усе посередині, великий розкид
        Episode best; best.r.score = -1e9;
        std::vector<Episode> gen;
        prepare();
        if (start_u) {
            best = run(trick, start_u);
            best.index = 0;
            history.push_back(best);
            if (on_episode) on_episode(best, best);
        }
        for (int k = 0; k < episodes;) {
            if (cancel && cancel()) break;
            // спроби одного покоління незалежні — рахуються одночасно на всіх ядрах (порядок і результат ті самі, що й по одній)
            const int nb = std::min(population - int(gen.size()), episodes - k);
            std::vector<Episode> batch(static_cast<size_t>(nb));
            for (int b = 0; b < nb; ++b) {
                batch[size_t(b)].trick = trick;
                for (int i = 0; i < fc::kTrickParams; ++i) batch[size_t(b)].u[i] = std::clamp(mu[i] + sd[i] * N(rng), 0.0f, 1.0f);
            }
            std::atomic<int> next{0};
            auto worker = [&]() {
                for (int b = next++; b < nb; b = next++) batch[size_t(b)] = run(trick, batch[size_t(b)].u);
            };
            const int nt = std::clamp(threads > 0 ? threads : int(std::thread::hardware_concurrency()), 1, nb);
            std::vector<std::thread> pool;
            for (int t = 1; t < nt; ++t) pool.emplace_back(worker);
            worker();
            for (std::thread& th : pool) th.join();
            for (Episode& ep : batch) {
                ep.index = ++k;
                if (ep.r.score > best.r.score) best = ep;
                gen.push_back(ep);
                history.push_back(ep);
                if (on_episode) on_episode(ep, best);
            }
            if (int(gen.size()) == population) {   // нове покоління — навколо найкращих
                std::sort(gen.begin(), gen.end(), [](const Episode& x, const Episode& y) { return x.r.score > y.r.score; });
                Generation gi; gi.index = ++gen_index; gi.size = int(gen.size());
                for (int i = 0; i < fc::kTrickParams; ++i) { gi.mu_before[i] = mu[i]; gi.sd_before[i] = sd[i]; }
                for (const Episode& g : gen) { gi.mean_score += g.r.score / double(gen.size()); gi.completed += g.r.completed; gi.crashed += g.r.crashed; }
                for (int j = 0; j < elite && j < 3; ++j) gi.elite_scores[j] = gen[size_t(j)].r.score;
                for (int i = 0; i < fc::kTrickParams; ++i) {
                    float m = 0, v = 0;
                    for (int j = 0; j < elite; ++j) m += gen[size_t(j)].u[i];
                    m /= float(elite);
                    for (int j = 0; j < elite; ++j) v += (gen[size_t(j)].u[i] - m) * (gen[size_t(j)].u[i] - m);
                    mu[i] = 0.7f * m + 0.3f * mu[i];
                    sd[i] = std::fmax(0.04f, std::sqrt(v / float(elite)) + 0.02f);   // не стискатись до нуля — щоб ще шукати
                    gi.elite_mean[i] = m; gi.mu_after[i] = mu[i]; gi.sd_after[i] = sd[i];
                }
                if (on_generation) on_generation(gi);
                gen.clear();
            }
        }
        return best;
    }

    std::vector<Episode> history;

    // Файл навчених трюків: «loop = u0 … u6 оцінка»
    static bool save(const std::string& path, fc::Trick t, const float u[fc::kTrickParams], double score) {
        std::vector<std::string> lines;
        if (FILE* f = std::fopen(path.c_str(), "rb")) { char b[512]; while (std::fgets(b, sizeof b, f)) lines.push_back(b); std::fclose(f); }
        const std::string key = fc::trickKey(t);
        char row[256]; int k = std::snprintf(row, sizeof row, "%s =", key.c_str());
        for (int i = 0; i < fc::kTrickParams; ++i) k += std::snprintf(row + k, sizeof row - size_t(k), " %.4f", u[i]);
        std::snprintf(row + k, sizeof row - size_t(k), " %.1f\n", score);
        bool done = false;
        for (std::string& l : lines) if (l.rfind(key + " =", 0) == 0) { l = row; done = true; }
        if (!done) lines.push_back(row);
        FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) return false;
        std::fputs("# Навчені пілотажні трюки (tools/ai_tricks.cpp або GUI → «Трюки»): 7 параметрів u ∈ [0,1] і оцінка останнього навчання\n", f);
        for (const std::string& l : lines) if (l.rfind("#", 0) != 0) std::fputs(l.c_str(), f);
        std::fclose(f);
        return true;
    }
    static bool load(const std::string& path, fc::Trick t, float u[fc::kTrickParams], double* score = nullptr) {
        FILE* f = std::fopen(path.c_str(), "rb");
        if (!f) return false;
        const std::string key = fc::trickKey(t) + std::string(" =");
        char b[512]; bool ok = false;
        while (std::fgets(b, sizeof b, f)) {
            if (std::string(b).rfind(key, 0) != 0) continue;
            double v[fc::kTrickParams + 1] = {};
            const char* p = b + key.size(); int n = 0, used = 0;
            while (n < fc::kTrickParams + 1 && std::sscanf(p, "%lf%n", &v[n], &used) == 1) { p += used; ++n; }
            if (n >= fc::kTrickParams) { for (int i = 0; i < fc::kTrickParams; ++i) u[i] = float(v[i]); if (score) *score = n > fc::kTrickParams ? v[fc::kTrickParams] : 0; ok = true; }
        }
        std::fclose(f);
        return ok;
    }

private:
    const AircraftFile& a_;
    const AeroTables& t_;
    std::unique_ptr<QpFlightRig> base_;
    std::vector<std::unique_ptr<QpFlightRig>> variants_;
    std::atomic<int> variant_next_{0};
    std::mutex prep_mu_;
};

// Виконання трюків у польоті (GUI й тести): один трюк або програма авіашоу — трюки один за одним, між ними автопілот
// стабілізує політ літаком (щонайменше recover_s, рівно, висота від min_alt).
class AirShow {
public:
    std::vector<fc::Trick> program;
    float u[fc::kTrickCount][fc::kTrickParams] = {};
    double recover_s = 4.0, min_alt = 50.0, max_wait_s = 40.0;
    fc::TrickPilot pilot;
    TrickJudge judge;
    struct Done { fc::Trick trick; TrickResult r; };
    std::vector<Done> results;
    std::vector<TrajPoint> traj;
    bool running() const { return running_; }
    bool inTrick() const { return in_trick_; }
    int index() const { return idx_; }
    double waitTime() const { return wait_; }
    std::string lastText;

    void begin(QpFlightRig& r, std::vector<fc::Trick> prog, bool wait_first = false) {
        program = std::move(prog); results.clear(); idx_ = -1; running_ = true; in_trick_ = false; wait_ = wait_first ? 0.0 : 1e9; (void)r;
    }
    void stop(QpFlightRig& r) {
        if (in_trick_) { r.override_act = nullptr; resume(r); }
        running_ = false; in_trick_ = false;
    }
    bool ready(const QpFlightRig& r) const {
        const Vec3d eu = r.sim.eulerRad();
        return r.ctl.phase() == fc::QpPhase::Cruise && r.ctl.vtolWeight() == 0 && r.sim.state().pos.z > min_alt && std::fabs(eu.x) < 0.2 && std::fabs(eu.y) < 0.2;
    }
    // після кожного кроку моделі (1 мс)
    void after(QpFlightRig& r) {
        if (!running_) return;
        if (in_trick_) {
            if (++steps_ % 10 == 0) { judge.sample(r, pilot, r.sim.time() - last_t_); last_t_ = r.sim.time(); }
            if (steps_ % 50 == 0) traj.push_back(trajPoint(r, p0_, yaw0_, t0_));
            if (!pilot.active() || r.sim.crashed() || r.sim.diverged()) {
                r.override_act = nullptr;
                resume(r);
                results.push_back({program[size_t(idx_)], judge.finish(r, pilot)});
                lastText = std::string(fc::trickName(program[size_t(idx_)])) + ": " + results.back().r.text();
                in_trick_ = false; wait_ = 0;
                if (r.sim.crashed() || r.sim.diverged() || idx_ + 1 >= int(program.size())) running_ = false;
            }
            return;
        }
        wait_ += 0.001;
        if (!((wait_ >= recover_s && ready(r)) || (wait_ > max_wait_s && r.sim.state().pos.z > min_alt * 0.6))) return;
        ++idx_;
        if (idx_ >= int(program.size())) { running_ = false; return; }
        const fc::Trick t = program[size_t(idx_)];
        pilot = fc::TrickPilot{};
        pilot.v_cruise = r.ctl.cfg.af.v_cruise; pilot.v_stall = r.ctl.cfg.af.v_stall; pilot.cfg = &r.ctl.cfg; pilot.guard.ground_z = r.ctl.home().z;
        pilot.start(t, u[int(t)]);
        judge.begin(r, t);
        steps_ = 0; last_t_ = r.sim.time(); in_trick_ = true;
        p0_ = r.sim.state().pos; yaw0_ = r.sim.eulerRad().z; t0_ = r.sim.time(); traj.clear();
        fc::TrickPilot* pp = &pilot;
        QpFlightRig* rp = &r;
        r.override_act = [pp, rp](const fc::QpEstimate& e, fc::QpActuators& act) { pp->a_dist = rp->ctl.aiAccel(); return pp->update(0.002f, e, act); };
    }

private:
    bool running_ = false, in_trick_ = false;
    int idx_ = -1;
    double wait_ = 0, last_t_ = 0, t0_ = 0, yaw0_ = 0;
    long steps_ = 0;
    Vec3d p0_;
    static void resume(QpFlightRig& r) {
        fc::QpEstimate e; e.pos.z = float(r.sim.state().pos.z); e.airspeed = float(r.sim.airspeed());
        for (int i = 0; i < 4; ++i) e.tilt_meas[i] = float(r.sim.tilt(i));
        r.ctl.resumeAfterOverride(e);
    }
};

// Програма «Авіашоу аса»: усі трюки, фінал — «втрата керування»
inline std::vector<fc::Trick> airShowProgram() {
    return {fc::Trick::Loop, fc::Trick::Roll, fc::Trick::Hammerhead, fc::Trick::Cobra, fc::Trick::Harrier, fc::Trick::Flip, fc::Trick::Stall, fc::Trick::Tumble};
}

// Параметри, з якими кожен трюк гарантовано вдається (знайдені навчанням; запасний варіант, поки свого навчання немає)
inline const float* defaultTrickParams(fc::Trick t) {
    static const float d[fc::kTrickCount][fc::kTrickParams] = {
        {0.0f, 0.27f, 0.57f, 1.0f, 0.42f, 0.08f, 0.4f},
        {0.80f, 0.26f, 0.37f, 0.81f, 0.70f, 0.41f, 0.85f},
        {0.70f, 0.91f, 0.25f, 0.81f, 0.37f, 0.0f, 0.30f},
        {0.28f, 0.61f, 0.45f, 0.39f, 0.50f, 0.24f, 0.36f},
        {0.64f, 0.38f, 0.90f, 0.33f, 0.73f, 0.69f, 0.34f},
        {0.0f, 1.0f, 0.4f, 0.1f, 0.0f, 0.2f, 1.0f},
        {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
        {0.5f, 0.5f, 0.5f, 0.5f, 0.4f, 0.6f, 0.6f},
    };
    return d[int(t)];
}

}  // namespace sim
