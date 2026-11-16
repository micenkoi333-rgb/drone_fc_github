// aero_tables.h — таблиці перерізу cl/cd/cm(α) на −180…180° з aircraft/aero_tables.json (генерує tools/aero_gen.py).
// Інтерполяція лінійна по α (рад), α приводиться до (−π, π]. Cm — відносно чверті хорди, ніс угору +.
#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include "mini_json.h"

namespace sim {

struct SectionCoeffs { double cl = 0, cd = 0, cm = 0, w_plate = 0; };   // w_plate — частка пост-зривної моделі (0…1)

class AirfoilTable {
public:
    double re = 0, cd_max = 0, cl_max = 0, cl_min = 0, alpha_stall_pos = 0, alpha_stall_neg = 0, cl_alpha = 0, alpha_zero = 0;   // кути в радіанах

    static AirfoilTable fromJson(const Json& j) {
        AirfoilTable t;
        const double d2r = 3.14159265358979323846 / 180.0;
        for (double a : j.at("alpha_deg").numbers()) t.alpha_.push_back(a * d2r);
        t.cl_ = j.at("cl").numbers(); t.cd_ = j.at("cd").numbers(); t.cm_ = j.at("cm").numbers(); t.wp_ = j.at("w_plate").numbers();
        const size_t n = t.alpha_.size();
        if (n < 8 || t.cl_.size() != n || t.cd_.size() != n || t.cm_.size() != n || t.wp_.size() != n)
            throw std::runtime_error("таблиця профілю: різна довжина масивів або замало точок");
        for (size_t i = 1; i < n; ++i)
            if (!(t.alpha_[i] > t.alpha_[i - 1])) throw std::runtime_error("таблиця профілю: α має зростати");
        if (t.alpha_.front() > -3.1 || t.alpha_.back() < 3.1) throw std::runtime_error("таблиця профілю: треба охопити −180…180°");
        t.re = j.at("re").number(); t.cd_max = j.at("cd_max").number();
        t.cl_max = j.at("cl_max").number(); t.cl_min = j.at("cl_min").number();
        t.alpha_stall_pos = j.at("alpha_stall_pos_deg").number() * d2r; t.alpha_stall_neg = j.at("alpha_stall_neg_deg").number() * d2r;
        t.cl_alpha = j.at("cl_alpha_per_rad").number(); t.alpha_zero = j.at("alpha_zero_deg").number() * d2r;
        return t;
    }

    SectionCoeffs eval(double alpha) const {
        const double pi = 3.14159265358979323846;
        if (!std::isfinite(alpha)) return {};
        if (alpha <= -pi || alpha > pi) alpha = std::remainder(alpha, 2 * pi);   // (−π, π]
        auto it = std::upper_bound(alpha_.begin(), alpha_.end(), alpha);
        size_t hi = size_t(it - alpha_.begin());
        if (hi == 0) hi = 1;
        if (hi >= alpha_.size()) hi = alpha_.size() - 1;
        const size_t lo = hi - 1;
        const double f = std::clamp((alpha - alpha_[lo]) / (alpha_[hi] - alpha_[lo]), 0.0, 1.0);
        auto lerp = [f](double a, double b) { return a + (b - a) * f; };
        return {lerp(cl_[lo], cl_[hi]), lerp(cd_[lo], cd_[hi]), lerp(cm_[lo], cm_[hi]), lerp(wp_[lo], wp_[hi])};
    }

    size_t size() const { return alpha_.size(); }
    double alphaAt(size_t i) const { return alpha_[i]; }
    SectionCoeffs at(size_t i) const { return {cl_[i], cd_[i], cm_[i], wp_[i]}; }

private:
    std::vector<double> alpha_, cl_, cd_, cm_, wp_;
};

class AeroTables {
public:
    static AeroTables load(const std::string& path) {
        const Json j = Json::load(path);
        AeroTables t;
        for (const auto& [name, node] : j.at("airfoils").obj) t.tables_.emplace(name, AirfoilTable::fromJson(node));
        if (t.tables_.empty()) throw std::runtime_error(path + ": немає жодного профілю");
        return t;
    }
    const AirfoilTable& get(const std::string& name) const {
        auto it = tables_.find(name);
        if (it == tables_.end()) throw std::runtime_error("немає таблиці профілю " + name);
        return it->second;
    }

private:
    std::map<std::string, AirfoilTable> tables_;
};

}  // namespace sim
