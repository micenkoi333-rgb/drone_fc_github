#pragma once
// Модель рульової машинки / приводу нахилу гондоли.
// Двигун постійного струму + редуктор + цифровий регулятор положення:
//  - швидкість залежить від навантаження: ω = ω0·(1 − M_проти/M_стоп) (лінійна характеристика колекторного мотора);
//  - навантаження, більше за момент утримання, провертає редуктор назад (машинка «не тримає»);
//  - мертва зона регулятора, люфт шестерень, піддатливість (вихід прогинається під моментом);
//  - нагрів обмоток від струму (∝ моменту²) і зниження моменту при перегріві;
//  - ослаблення (знос, просадка живлення) — множник моменту для випробувань і для ШІ-діагностики.
#include <algorithm>
#include <cmath>

namespace sim {

struct ServoSpec {
    double rate = 8;            // рад/с без навантаження
    double tau = 0.05;          // с, стала регулятора положення
    double stall_nm = 1e9;      // Н·м, момент зупинки (утримання)
    double deadband = 0;        // рад
    double backlash = 0;        // рад, повний люфт
    double compliance = 0;      // рад/(Н·м), прогин виходу під моментом
    double heat_c_per_s = 2;    // °C/с при моменті зупинки
    double cool_tau_s = 60;     // с
    double derate_from_c = 70;  // °C — далі момент падає на 1.2%/°C
    double inertia = 0;         // кг·м² навантаження на виході (гондола з мотором): розгін забирає частину моменту
};

class ServoActuator {
public:
    ServoActuator() = default;
    explicit ServoActuator(const ServoSpec& s) : s_(s) {}
    void reset(double pos) { gear_ = shaft_ = out_ = pos; temp_ = 25; rate_now_ = 0; }

    // load_nm — момент від потоку/гвинта на вихідному валу (додатний штовхає в бік +кута)
    void step(double dt, double target, double load_nm, double strength = 1.0) {
        const double prev = out_;
        const double stall = std::fmax(s_.stall_nm * strength * derate(), 1e-6);
        const double err = target - gear_;
        double w = 0;
        if (std::fabs(err) > s_.deadband || hunting_) {
            hunting_ = std::fabs(err) > 0.3 * s_.deadband;
            const double dir = err > 0 ? 1.0 : -1.0;
            const double against = -load_nm * dir;                      // >0 — навантаження заважає руху
            const double avail = std::clamp(1.0 - against / stall, 0.0, 1.3);
            const double w_max = s_.rate * avail;
            w = std::clamp(err * (1 - std::exp(-dt / s_.tau)) / dt, -w_max, w_max);
            if (s_.inertia > 0) {   // розгін гондоли: прискорення не більше за (вільний момент) / інерцію
                const double amax = std::fmax(stall - std::fabs(load_nm), 0.05 * stall) / s_.inertia;
                w = std::clamp(w, w_prev_ - amax * dt, w_prev_ + amax * dt);
            }
        }
        if (std::fabs(load_nm) > stall) {                               // перевантаження: редуктор провертається за моментом
            const double sl = (std::fabs(load_nm) - stall) / stall;
            w += (load_nm > 0 ? 1 : -1) * s_.rate * std::fmin(sl, 1.0);
        }
        gear_ += w * dt;
        w_prev_ = w;
        const double hb = s_.backlash / 2;
        if (gear_ - shaft_ > hb) shaft_ = gear_ - hb; else if (shaft_ - gear_ > hb) shaft_ = gear_ + hb;
        if (std::fabs(load_nm) > 1e-9 && s_.backlash > 0) {             // момент притискає вал до одного боку люфту
            const double side = load_nm > 0 ? 1.0 : -1.0;
            const double want = gear_ + side * hb;
            shaft_ += (want - shaft_) * std::fmin(1.0, dt / 0.01);
        }
        out_ = shaft_ + load_nm * s_.compliance;
        const double r = std::fmin(std::fabs(load_nm) / std::fmax(s_.stall_nm * strength, 1e-6), 1.5);
        temp_ += (s_.heat_c_per_s * (r * r + 0.05 * std::fabs(w) / std::fmax(s_.rate, 1e-6)) - (temp_ - 25) / s_.cool_tau_s) * dt;
        rate_now_ = (out_ - prev) / dt;
        last_load_ = load_nm;
    }
    void hold() { rate_now_ = 0; }   // заклинена: вихід стоїть

    double pos() const { return out_; }
    double rate() const { return rate_now_; }
    double temperature() const { return temp_; }
    double load() const { return last_load_; }
    double loadFrac() const { return std::fabs(last_load_) / std::fmax(s_.stall_nm, 1e-9); }
    const ServoSpec& spec() const { return s_; }
    ServoSpec& spec() { return s_; }
private:
    double derate() const { return std::clamp(1.0 - 0.012 * (temp_ - s_.derate_from_c), 0.25, 1.0); }
    ServoSpec s_;
    double gear_ = 0, shaft_ = 0, out_ = 0, temp_ = 25, rate_now_ = 0, last_load_ = 0, w_prev_ = 0;
    bool hunting_ = false;
};

}  // namespace sim
