// qp_battery.h — акумулятор 6S2P Li-ion 21700: модель Тевенена (OCV(SoC), R0(T), одна RC-ланка), ємність від температури, нагрів.
// Усі параметри — з aircraft/quadplane.ini [battery] ([О] там, де відсутні публічні дані — див. docs/AIRCRAFT.md).
#pragma once
#include <cmath>
#include <vector>
#include "aircraft_params.h"
#include "qp_prop.h"

namespace sim {

class PackBattery {
public:
    PackBattery() = default;
    explicit PackBattery(const AircraftFile& a) {
        cs_ = a.num("battery.cells_series"); cp_ = a.num("battery.cells_parallel");
        cap_ah_ = cp_ * a.num("battery.cell_capacity_ah");
        r0_ = a.num("battery.cell_r0_ohm") * cs_ / cp_;
        r1_ = r0_ * a.num("battery.r1_over_r0");
        tau_ = a.num("battery.tau1_s");
        cth_ = a.num("battery.heat_capacity_j_k"); cool_ = a.num("battery.cooling_w_per_k");
        t_amb_ = a.num("battery.ambient_c"); temp_ = t_amb_;
        v_cut_cell_ = a.num("battery.v_cutoff_cell");
        ocv_soc_ = a.nums("battery.ocv_soc"); ocv_v_ = a.nums("battery.ocv_v");
        rt_t_ = a.nums("battery.rt_temp_c"); rt_f_ = a.nums("battery.rt_factor");
        ct_t_ = a.nums("battery.ct_temp_c"); ct_f_ = a.nums("battery.ct_factor");
        if (ocv_soc_.size() != ocv_v_.size() || rt_t_.size() != rt_f_.size() || ct_t_.size() != ct_f_.size() || ocv_soc_.size() < 2)
            throw std::runtime_error("battery: таблиці різної довжини");
    }

    void setTemperature(double c) { temp_ = c; }
    void setSoc(double s) { soc_ = std::clamp(s, 0.0, 1.0); }
    double soc() const { return soc_; }
    double temperature() const { return temp_; }
    double usedWh() const { return used_wh_; }
    double ocv() const { return cs_ * interpClamp(ocv_soc_, ocv_v_, soc_); }
    double nominalBusV() const { return cs_ * 4.0; }   // опорна напруга таблиць і аналізу (6S → 24 В, як і раніше)
    double r0() const { return r0_ * r_scale_ * interpClamp(rt_t_, rt_f_, temp_); }
    double capacityAh() const { return cap_ah_ * cap_scale_ * interpClamp(ct_t_, ct_f_, temp_); }
    void setResistanceScale(double s) { r_scale_ = s; }   // відмова: «втомлена» батарея (внутрішній опір ×s)
    void setCapacityScale(double s) { cap_scale_ = s; }
    // напруга на клемах при струмі i (А, розряд +)
    double voltage(double i) const { return ocv() - i * r0() - v_rc_; }
    bool depleted() const { return soc_ <= 0.0 || voltage(0) / cs_ < v_cut_cell_; }

    void step(double dt, double i) {
        const double v = voltage(i);
        const double r1 = r1_ * interpClamp(rt_t_, rt_f_, temp_);
        soc_ = std::fmax(0.0, soc_ - i * dt / (3600.0 * capacityAh()));
        v_rc_ += (i * r1 - v_rc_) * (1 - std::exp(-dt / tau_));
        used_wh_ += v * i * dt / 3600.0;
        temp_ += dt * (i * i * (r0() + r1) - cool_ * (temp_ - t_amb_)) / cth_;
    }

private:
    double cs_ = 6, cp_ = 2, cap_ah_ = 7.2, r0_ = 0.048, r1_ = 0.029, tau_ = 25, cth_ = 840, cool_ = 0.8, t_amb_ = 25, temp_ = 25, v_cut_cell_ = 3.0;
    double soc_ = 1.0, v_rc_ = 0, used_wh_ = 0, r_scale_ = 1.0, cap_scale_ = 1.0;
    std::vector<double> ocv_soc_, ocv_v_, rt_t_, rt_f_, ct_t_, ct_f_;
};

}  // namespace sim
