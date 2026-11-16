#pragma once
// ШІ-енергоменеджер (бортовий): сам вимірює в польоті, скільки потужності апарат бере на кожній швидкості, запам'ятовує вітер
// на кожній висоті і обирає повітряну швидкість і висоту крейсеру, за яких на кілометр шляху до цілі йде найменше енергії.
//  - крива потужності — таблиця по 1 м/с: середня електрична потужність (напруга × струм) у рівному усталеному польоті.
//    На першому довгому відрізку апарат «пробує» кілька швидкостей по 12 с (так він вивчає саме себе, без моделі),
//    далі зрідка перевіряє сусідні швидкості, щоб таблиця лишалась точною (вантаж, батарея, знос гвинтів);
//  - вітер за висотою: шари по 10 м, вимір у польоті літаком (земна − повітряна швидкість);
//  - швидкість: мінімум P(V)/V_земна уздовж шляху з урахуванням кута упередження в бічному вітрі;
//  - висота: лише між виміряними шарами, якщо енергія на решту шляху разом з набором менша щонайменше на 4%.
#include <algorithm>
#include <array>
#include <cmath>
#include "../fc/qp_mission.h"

namespace ai {

class EnergyManager {
public:
    bool on = false;
    float v_min = 13.0f, v_max = 21.0f;
    float alt_min = 40.0f, alt_max = 120.0f;   // м над домом
    float climb_eff = 0.45f;                   // частка електроенергії, що йде в потенціальну на наборі
    float min_gain = 0.04f;
    float time_weight = 0.0f;                  // Вт·год за годину польоту: 0 — лише енергія; більше — цінувати й час

    float v_best = 0, alt_best = 0, wh_per_km = 0;
    int bins_known = 0;
    bool exploring = false;

    static constexpr int kBins = 10;           // 13 … 22 м/с
    float binPower(int i, float* sec = nullptr) const { if (sec) *sec = float(bin_t_[size_t(i)]); return bin_t_[size_t(i)] > 0 ? float(bin_p_[size_t(i)]) : 0.0f; }
    float binSpeed(int i) const { return v0_ + float(i); }

    void update(float dt, const fc::QpEstimate& e, const fc::QpFlight& fl) {
        const fc::QpPhase ph = fl.phase();
        const fc::Euler eu = fc::eulerOf(e.q);
        const float h = e.pos.z - fl.home().z;
        if (ph == fc::QpPhase::Cruise && e.airspeed > 8) {
            const float wx = e.vel.x - e.airspeed * std::cos(eu.yaw), wy = e.vel.y - e.airspeed * std::sin(eu.yaw);
            layer(h).add(wx, wy, dt / 6.0f);
        }
        // таблиця потужності: рівний усталений політ, швидкість тримається біля цілі
        const float p = e.v_bus * std::fmax(e.bus_current, 0.0f);
        p_f_ += (p - p_f_) * std::fmin(1.0f, dt / 2.0f);
        v_f_ += (e.airspeed - v_f_) * std::fmin(1.0f, dt / 2.0f);
        // у поривах миттєва швидкість гуляє на ±1–2 м/с — тож у таблицю йде потужність за ЗАДАНОЇ швидкості, коли вона тримається ≥ 6 с
        if (std::fabs(v_best - cmd_) > 0.01f) { cmd_ = v_best; cmd_t_ = 0; } else cmd_t_ += dt;
        hold_t_ = (std::fabs(e.vel.z) < 1.0f && std::fabs(eu.roll) < 0.35f) ? hold_t_ + dt : 0.0f;
        if (ph == fc::QpPhase::Cruise && fl.vtolWeight() < 0.01f && cmd_ > 0 && cmd_t_ > 6.0f && hold_t_ > 3.0f && !fl.assisting()) {
            const int b = int(std::lround(cmd_ - v0_));
            if (b >= 0 && b < kBins) {
                const double k = std::fmin(1.0, dt / std::fmax(1.0, bin_t_[size_t(b)]));   // середнє, а після 60 с — ковзне (≈ хвилина пам'яті)
                bin_p_[size_t(b)] = bin_t_[size_t(b)] > 0 ? bin_p_[size_t(b)] + (p_f_ - bin_p_[size_t(b)]) * std::fmax(k, dt / 60.0) : p_f_;
                bin_t_[size_t(b)] = std::fmin(bin_t_[size_t(b)] + dt, 600.0);
            }
        }
        if (ph != fc::QpPhase::Cruise) { v_best = 0; alt_best = 0; exploring = false; probe_ = -1; return; }
        cruise_t_ += dt;
        dec_t_ += dt;
        if (dec_t_ < 1.0f) return;
        dec_t_ = 0;
        bins_known = 0;
        for (int i = 0; i < kBins; ++i) bins_known += bin_t_[size_t(i)] > 5;
        if (!on) { v_best = 0; alt_best = 0; return; }
        const fc::Vec3 tgt = target(fl, e);
        const float dx = tgt.x - e.pos.x, dy = tgt.y - e.pos.y, d = std::hypot(dx, dy);
        if (d < 1) { v_best = 0; alt_best = 0; return; }
        const float ux = dx / d, uy = dy / d;
        // вивчення: поки відомо менше трьох швидкостей — пробувати 14, 16, 18, 20 м/с; далі раз на 90 с — сусідню з найкращою
        if (probe_ >= 0) {
            probe_t_ += 1.0f;
            if (probe_t_ < 14.0f) { v_best = binSpeed(probe_); exploring = true; return; }
            probe_ = -1;
        }
        const float t_left = d / std::fmax(5.0f, fl.cfg.v_cruise);
        const Layer& here0 = layer(h);
        if (bins_known < 3 && t_left > 25) {   // перші виміри — довкола швидкості, яку обрав би звичайний автопілот (вітер + 8 м/с), і повільніше
            const float vdef = std::fmin(v_max, std::fmax(fl.cfg.v_cruise, std::hypot(here0.wx, here0.wy) + 8.0f));
            const int c = std::min(kBins - 1, std::max(0, int(std::lround(vdef - v0_))));
            const float vg_def = groundSpeed(binSpeed(c), here0.wx, here0.wy, ux, uy);
            for (int s : {c, c - 2, c - 4, c + 2}) {
                if (s < 0 || s >= kBins || bin_t_[size_t(s)] >= 5 || binSpeed(s) < v_min || binSpeed(s) > v_max) continue;
                if (groundSpeed(binSpeed(s), here0.wx, here0.wy, ux, uy) < std::fmax(6.0f, 0.6f * vg_def)) continue;   // у сильний зустрічний повільні швидкості не пробувати: майже стоїть на місці
                probe_ = s; probe_t_ = 0; v_best = binSpeed(s); exploring = true; return;
            }
        }
        exploring = false;
        const Layer& here = layer(h);
        float e_here = 0;
        const int bb = bestBin(here.wx, here.wy, ux, uy, &e_here);
        if (bb < 0 || groundSpeed(binSpeed(bb), here.wx, here.wy, ux, uy) < 8.0f) {   // сильний зустрічний: спробувати швидше, ніж уже міряли
            int fastest = -1;
            for (int s = 0; s < kBins; ++s) if (bin_t_[size_t(s)] > 5) fastest = s;
            for (int s = kBins - 1; s > std::max(bb, fastest); --s) if (bin_t_[size_t(s)] < 5 && binSpeed(s) <= v_max && binSpeed(s) >= v_min) { probe_ = s; probe_t_ = 0; v_best = binSpeed(s); exploring = true; return; }   // лише швидші за вже виміряні
            if (bb < 0) { v_best = 0; alt_best = 0; return; }
        }
        v_best = binSpeed(bb);
        wh_per_km = e_here;
        {   // звичайний автопілот летів би на «вітер + 8 м/с»: перевизначати лише з виграшем ≥ 3% за виміряною таблицею
            const float vdef = std::fmin(v_max, std::fmax(fl.cfg.v_cruise, std::hypot(here.wx, here.wy) + 8.0f));
            const int c = std::min(kBins - 1, std::max(0, int(std::lround(vdef - v0_))));
            if (bin_t_[size_t(c)] > 5) {
                const float vg = groundSpeed(binSpeed(c), here.wx, here.wy, ux, uy);
                const float e_def = vg > 1 ? (float(bin_p_[size_t(c)]) + time_weight) / vg * 1000.0f / 3600.0f : 1e9f;
                if (e_here > 0.97f * e_def) { v_best = 0; wh_per_km = e_def; }
            }
        }
        if (cruise_t_ - last_probe_ > 90.0f && t_left > 25) {   // перевірити сусідню швидкість, якої ще не міряли або давно
            last_probe_ = cruise_t_;
            for (int s : {bb + 1, bb - 1}) if (s >= 0 && s < kBins && binSpeed(s) >= v_min && binSpeed(s) <= v_max && bin_t_[size_t(s)] < 20) { probe_ = s; probe_t_ = 0; v_best = binSpeed(s); exploring = true; return; }
        }
        float best_cost = e_here * d / 1000.0f, best_h = alt_best > 0 ? alt_best : h;
        const float m = fl.massEstimate();
        for (int i = 0; i < kLayers; ++i) {
            const float hl = (i + 0.5f) * 10.0f;
            if (hl < alt_min || hl > alt_max || layers_[size_t(i)].w < 0.5f) continue;
            float el = 0;
            if (bestBin(layers_[size_t(i)].wx, layers_[size_t(i)].wy, ux, uy, &el) < 0) continue;
            const float dh = hl - h;
            const float cost = el * d / 1000.0f + (dh > 0 ? m * 9.81f * dh / climb_eff / 3600.0f : 0.0f);
            if (cost < best_cost * (1 - min_gain)) { best_cost = cost; best_h = hl; }
        }
        alt_best = best_h;
    }

    static constexpr int kLayers = 16;
    int knownBins() const { int n = 0; for (double t : bin_t_) n += t > 5; return n; }
    void importTable(const EnergyManager& o) { bin_p_ = o.bin_p_; bin_t_ = o.bin_t_; }   // виміряне в попередніх польотах
    float layerWind(int i, float* wx, float* wy) const { *wx = layers_[size_t(i)].wx; *wy = layers_[size_t(i)].wy; return layers_[size_t(i)].w; }
    float powerAt(float v) const {   // з таблиці (між виміряними — лінійно); 0 — невідомо
        int lo = -1, hi = -1;
        for (int i = 0; i < kBins; ++i) if (bin_t_[size_t(i)] > 5) { if (binSpeed(i) <= v) lo = i; if (binSpeed(i) >= v && hi < 0) hi = i; }
        if (lo < 0 || hi < 0) return 0.0f;
        if (lo == hi) return float(bin_p_[size_t(lo)]);
        const float f = (v - binSpeed(lo)) / (binSpeed(hi) - binSpeed(lo));
        return float(bin_p_[size_t(lo)] + f * (bin_p_[size_t(hi)] - bin_p_[size_t(lo)]));
    }

private:
    struct Layer { float wx = 0, wy = 0, w = 0; void add(float x, float y, float k) { k = std::fmin(k, 1.0f); wx += (x - wx) * k; wy += (y - wy) * k; w = std::fmin(10.0f, w + k); } };
    Layer& layer(float h) { return layers_[size_t(std::min(kLayers - 1, std::max(0, int(h / 10.0f))))]; }
    const Layer& layer(float h) const { return layers_[size_t(std::min(kLayers - 1, std::max(0, int(h / 10.0f))))]; }

    static float groundSpeed(float v, float wx, float wy, float ux, float uy) {
        const float wa = wx * ux + wy * uy, wc = -wx * uy + wy * ux;
        if (std::fabs(wc) >= v) return 0.0f;
        return std::sqrt(v * v - wc * wc) + wa;
    }
    // найкраща серед виміряних швидкостей; між ними не вгадує — лише те, що апарат справді пролітав
    int bestBin(float wx, float wy, float ux, float uy, float* e_out) const {
        int best = -1; float eb = 1e9f;
        for (int i = 0; i < kBins; ++i) {
            const float v = binSpeed(i);
            if (bin_t_[size_t(i)] <= 5 || v < v_min || v > v_max) continue;
            const float vg = groundSpeed(v, wx, wy, ux, uy);
            if (vg < 6.0f) continue;   // запас поступу: не повзти проти вітру
            const float ev = (float(bin_p_[size_t(i)]) + time_weight) / vg * 1000.0f / 3600.0f;
            if (ev < eb) { eb = ev; best = i; }
        }
        if (e_out) *e_out = eb;
        return best;
    }
    static fc::Vec3 target(const fc::QpFlight& fl, const fc::QpEstimate& e) {
        const fc::QpStep* s = fl.currentStep();
        return s ? s->p : fc::Vec3{fl.home().x, fl.home().y, e.pos.z};
    }

    static constexpr float v0_ = 13.0f;
    std::array<double, kBins> bin_p_{}, bin_t_{};
    std::array<Layer, kLayers> layers_{};
    float cmd_ = 0, cmd_t_ = 0;
    float p_f_ = 0, v_f_ = 0, hold_t_ = 0, dec_t_ = 0, cruise_t_ = 0, last_probe_ = 0, probe_t_ = 0;
    int probe_ = -1;
};

}  // namespace ai
