// aircraft_params.h — завантаження aircraft/tiltrotor.ini, перевірка значень і похідні величини апарата.
// Тільки std: жодних зовнішніх бібліотек. Вся геометрія й маси беруться з файлу, у коді «магічних чисел» немає.
#pragma once
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <mutex>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim {

struct MassItem {
    std::string name;
    double m = 0, x = 0, y = 0, z = 0;   // кг, м (тіло FLU, початок — ЦМ за кресленням)
    double dx = 0, dy = 0, dz = 0;       // габарит однорідного паралелепіпеда для власної інерції (0 — точкова маса)
};

struct Inertia { double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0; };   // кг·м² відносно ЦМ

class AircraftFile {
public:
    std::vector<MassItem> items;                    // розділ [mass]
    std::map<std::string, std::vector<std::string>> values;   // «розділ.ключ» -> слова значення

    static AircraftFile load(const std::string& path) {
        std::ifstream f(path);
        if (!f) throw std::runtime_error("не вдалось відкрити " + path);
        return parse(f, path);
    }

    static AircraftFile parse(std::istream& f, const std::string& path) {
        AircraftFile a;
        std::string line, section;
        int n = 0;
        while (std::getline(f, line)) {
            ++n;
            if (auto h = line.find('#'); h != std::string::npos) line.erase(h);
            std::istringstream ls(line);
            std::string first;
            if (!(ls >> first)) continue;
            if (first.front() == '[') {
                if (first.back() != ']') throw a.err(path, n, "розділ без «]»");
                section = first.substr(1, first.size() - 2);
                continue;
            }
            std::string eq;
            if (!(ls >> eq) || eq != "=") throw a.err(path, n, "очікувалось «ключ = значення»");
            std::vector<std::string> words;
            for (std::string w; ls >> w;) words.push_back(w);
            if (words.empty()) throw a.err(path, n, "порожнє значення");
            if (section == "mass") {
                if (words.size() != 7) throw a.err(path, n, "у [mass] треба 7 чисел: маса x y z dx dy dz");
                MassItem it;
                it.name = first;
                double* dst[7] = {&it.m, &it.x, &it.y, &it.z, &it.dx, &it.dy, &it.dz};
                for (int i = 0; i < 7; ++i) *dst[i] = a.toNumber(words[i], path, n);
                a.items.push_back(it);
            } else {
                a.values[section + "." + first] = words;
            }
        }
        return a;
    }

    bool has(const std::string& key) const { return values.count(key) != 0; }
    double num(const std::string& key) const {
        auto it = values.find(key);
        if (it == values.end()) throw std::runtime_error("немає параметра " + key);
        return toNumber(it->second[0], key, 0);
    }
    std::vector<double> nums(const std::string& key) const {   // всі числа в значенні (таблиці)
        auto it = values.find(key);
        if (it == values.end()) throw std::runtime_error("немає параметра " + key);
        std::vector<double> v;
        for (const std::string& w : it->second) v.push_back(toNumber(w, key, 0));
        return v;
    }
    std::string str(const std::string& key) const {
        auto it = values.find(key);
        if (it == values.end()) throw std::runtime_error("немає параметра " + key);
        return it->second[0];
    }

private:
    static std::runtime_error err(const std::string& path, int line, const std::string& what) {
        return std::runtime_error(path + ":" + std::to_string(line) + ": " + what);
    }
    static double toNumber(const std::string& w, const std::string& where, int line) {
        size_t used = 0;
        double v = 0;
        try { v = std::stod(w, &used); } catch (...) { used = 0; }
        if (used != w.size() || !std::isfinite(v))
            throw std::runtime_error(where + (line ? ":" + std::to_string(line) : "") + ": «" + w + "» — не число");
        return v;
    }
};

// Файл навченого ШІ під цей апарат: "ai/tricks_learned.ini" → "ai/tricks_learned_qp5.ini", якщо у файлі апарата є ai.variant = qp5
// (старий апарат — без суфікса). fallback = true: якщо файлу для апарата ще немає — повернути основний (як початкове наближення).
inline std::string aiVariant(const AircraftFile& a) { return a.has("ai.variant") ? a.str("ai.variant") : (a.has("ai.force_weights") ? a.str("ai.force_weights") : std::string()); }
inline std::string aiFile(const AircraftFile& a, const std::string& base, bool fallback = false) {
    const std::string v = aiVariant(a);
    if (v.empty()) return base;
    const size_t dot = base.find_last_of('.');
    const std::string path = base.substr(0, dot) + "_" + v + base.substr(dot);
    if (fallback) { std::ifstream f(path); if (!f) return base; }
    return path;
}

// Який апарат зараз активний: змінна середовища DRONE_AIRCRAFT, інакше перший рядок aircraft/active.txt, інакше tiltrotor.ini.
// Інструменти й GUI беруть апарат звідси; тести — завжди свої файли явно.
inline std::string activeAircraftPath() {
    if (const char* e = std::getenv("DRONE_AIRCRAFT")) if (*e) return e;
    std::ifstream f("aircraft/active.txt");
    std::string line;
    if (f && std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\n')) line.pop_back();
        if (!line.empty()) return line.find('/') == std::string::npos && line.find('\\') == std::string::npos ? "aircraft/" + line : line;
    }
    return "aircraft/tiltrotor.ini";
}


// Похідні величини: маса, центр мас, інерція, стійкість, звалювання, потужність висіння
struct AircraftDerived {
    double mass_airframe = 0, mass_payload = 0, mass_total = 0;
    double cg_x = 0, cg_y = 0, cg_z = 0;
    Inertia inertia;
    double wing_area = 0, aspect = 0, wing_le_x = 0, wing_te_x = 0, wing_ac_x = 0;
    double v_stall = 0, re_cruise = 0, cl_cruise = 0, alpha_cruise_deg = 0;
    double tail_area = 0, tail_arm = 0, fin_area = 0, v_h = 0, v_v = 0, neutral_x = 0, static_margin = 0;
    double rear_prop_clear = 0, front_prop_clear = 0;
    double rw_area = 0, rw_aspect = 0, rw_ac_x = 0, rw_lift_frac = 0;   // заднє крило (0 — немає): площа, AR, фокус, частка ваги в польоті
    double clear_min = 1e9; std::string clear_what;                      // найменший зазор диска гвинта до конструкції/землі в діапазоні нахилу
    double hover_ideal_w = 0, hover_elec_w = 0, battery_wh = 0;
    std::vector<std::string> problems;   // порушені умови; порожньо — все гаразд
};

inline std::vector<MassItem> allMassItems(const AircraftFile& a) { return a.items; }

struct MassProps { double mass = 0, cg_x = 0, cg_y = 0, cg_z = 0; Inertia inertia; };

// Маса, ЦМ і тензор інерції відносно ЦМ для набору компонентів (точкові маси + однорідні паралелепіпеди)
inline MassProps massProperties(const std::vector<MassItem>& items) {
    MassProps p;
    double mx = 0, my = 0, mz = 0;
    for (const MassItem& it : items) { p.mass += it.m; mx += it.m * it.x; my += it.m * it.y; mz += it.m * it.z; }
    if (p.mass <= 0) return p;
    p.cg_x = mx / p.mass; p.cg_y = my / p.mass; p.cg_z = mz / p.mass;
    for (const MassItem& it : items) {   // перенос осей до ЦМ + власна інерція паралелепіпеда
        const double rx = it.x - p.cg_x, ry = it.y - p.cg_y, rz = it.z - p.cg_z, r2 = rx * rx + ry * ry + rz * rz;
        p.inertia.xx += it.m / 12 * (it.dy * it.dy + it.dz * it.dz) + it.m * (r2 - rx * rx);
        p.inertia.yy += it.m / 12 * (it.dx * it.dx + it.dz * it.dz) + it.m * (r2 - ry * ry);
        p.inertia.zz += it.m / 12 * (it.dx * it.dx + it.dy * it.dy) + it.m * (r2 - rz * rz);
        p.inertia.xy -= it.m * rx * ry; p.inertia.xz -= it.m * rx * rz; p.inertia.yz -= it.m * ry * rz;
    }
    return p;
}

// Геометрична перевірка перетинів: обід кожного диска гвинта (усі кути нахилу min…max гондоли, крок 1°) проти коробок конструкції
// (крила з товщиною профілю, фюзеляж, балка, панелі V-хвоста, ніжки), землі (площина низу ніжок) і обідів інших дисків.
struct ClearBox { const char* name; double x0, x1, y0, y1, z0, z1; };
inline double boxDist(const ClearBox& b, double x, double y, double z) {
    const double dx = std::fmax(std::fmax(b.x0 - x, x - b.x1), 0.0), dy = std::fmax(std::fmax(b.y0 - y, y - b.y1), 0.0), dz = std::fmax(std::fmax(b.z0 - z, z - b.z1), 0.0);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
inline void propClearance(const AircraftFile& a, AircraftDerived& d) {
    const double pi = 3.14159265358979, R = a.num("vtol_prop.diameter_m") / 2, off = a.num("tilt.prop_offset_m");
    const double t_min = a.num("tilt.min_deg"), t_max = a.num("tilt.max_deg");
    const bool pusher = a.num("tilt.rear_pusher") > 0.5;
    std::vector<ClearBox> boxes;
    const double c = a.num("geometry.wing_chord_m"), b = a.num("geometry.wing_span_m"), zw = a.num("geometry.wing_z_m"), th = 0.06 * c;   // ±половина товщини Clark Y 11.7%
    boxes.push_back({"основне крило", d.wing_te_x, d.wing_le_x, -b / 2, b / 2, zw - th, zw + th});
    if (a.has("rear_wing.span_m")) {
        const double c2 = a.num("rear_wing.chord_m"), b2 = a.num("rear_wing.span_m"), le2 = a.num("rear_wing.le_x_m"), z2 = a.num("rear_wing.z_m");
        boxes.push_back({"заднє крило", le2 - c2, le2, -b2 / 2, b2 / 2, z2 - 0.06 * c2, z2 + 0.06 * c2});
    }
    for (const MassItem& it : a.items)   // фюзеляж і балка — за габаритами з [mass]
        if (it.name == "fuselage_shell" || it.name == "tail_boom")
            boxes.push_back({it.name == "tail_boom" ? "хвостова балка" : "фюзеляж", it.x - it.dx / 2, it.x + it.dx / 2, it.y - it.dy / 2, it.y + it.dy / 2, it.z - it.dz / 2, it.z + it.dz / 2});
    {   // V-хвіст — панель смугами по 1 см уздовж розмаху
        const double len = a.num("tail.vtail_panel_m"), cv = a.num("tail.vtail_chord_m"), gam = a.num("tail.vtail_dihedral_deg") * pi / 180;
        const double le = a.num("tail.vtail_le_x_m"), z0 = a.num("tail.vtail_root_z_m");
        for (double r = 0; r <= len + 1e-9; r += 0.01)
            for (int sd : {1, -1}) boxes.push_back({"V-хвіст", le - cv, le, sd * r * std::cos(gam) - 0.005, sd * r * std::cos(gam) + 0.005, z0 + r * std::sin(gam) - 0.005, z0 + r * std::sin(gam) + 0.005});
    }
    const std::vector<double> gp = a.nums("ground.gear_points");
    double z_ground = 0;
    for (size_t i = 0; i + 2 < gp.size(); i += 3) {   // ніжка — вертикальний стрижень Ø1.5 см від точки опори до площини крила
        boxes.push_back({"ніжка шасі", gp[i] - 0.0075, gp[i] + 0.0075, gp[i + 1] - 0.0075, gp[i + 1] + 0.0075, gp[i + 2], zw});
        z_ground = std::fmin(z_ground, gp[i + 2]);
    }
    struct Rim { int rotor; double x, y, z; };
    std::vector<Rim> rims;
    const char* rn[4] = {"передній лівий", "передній правий", "задній лівий", "задній правий"};
    for (int i = 0; i < 4; ++i) {
        const bool front = i < 2;
        const double px = a.num(front ? "vtol_motor.front_x_m" : "vtol_motor.rear_x_m"), py = (i % 2 == 0 ? 1 : -1) * a.num(front ? "vtol_motor.front_y_m" : "vtol_motor.rear_y_m");
        const double pz = a.num(front ? "vtol_motor.front_z_m" : "vtol_motor.rear_z_m"), sg = (!front && pusher) ? -1.0 : 1.0;
        for (double td = t_min; td <= t_max + 1e-9; td += 1.0) {
            const double t = td * pi / 180, ax = std::sin(t), az = std::cos(t);
            const double cx = px + ax * off * sg, cz = pz + az * off * sg;
            for (int k = 0; k < 72; ++k) {
                const double ph = k * 2 * pi / 72, u = R * std::cos(ph), v = R * std::sin(ph);
                const double x = cx + u * az, y = py + v, z = cz - u * ax;   // площина диска: u уздовж (cos t, 0, −sin t), v уздовж Y
                rims.push_back({i, x, y, z});
                for (const ClearBox& bx : boxes) {
                    const double dd = boxDist(bx, x, y, z);
                    if (dd < d.clear_min) { d.clear_min = dd; d.clear_what = std::string(rn[i]) + " гвинт — " + bx.name + " (нахил " + std::to_string(int(td)) + "°)"; }
                }
                if (z - z_ground < d.clear_min) { d.clear_min = z - z_ground; d.clear_what = std::string(rn[i]) + " гвинт — земля (нахил " + std::to_string(int(td)) + "°)"; }
            }
        }
    }
    for (size_t p = 0; p < rims.size(); p += 7)   // обід проти обідів інших гвинтів (будь-які нахили — грубо, з проріджуванням)
        for (size_t q = p; q < rims.size(); q += 7) {
            if (rims[q].rotor <= rims[p].rotor) continue;
            const double dx = rims[p].x - rims[q].x, dy = rims[p].y - rims[q].y, dz = rims[p].z - rims[q].z, dd = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dd < d.clear_min) { d.clear_min = dd; d.clear_what = std::string(rn[rims[p].rotor]) + " — " + rn[rims[q].rotor] + " гвинт"; }
        }
}

inline AircraftDerived deriveAircraftImpl(const AircraftFile& a, double v_cruise);
inline std::string aircraftKey(const AircraftFile& a) {
    std::string k;
    k.reserve(8192);
    for (const auto& kv : a.values) { k += kv.first; k += '='; for (const std::string& w : kv.second) { k += w; k += ' '; } k += ';'; }
    for (const MassItem& it : a.items) {
        k += it.name; k += ':';
        const double v[7] = {it.m, it.x, it.y, it.z, it.dx, it.dy, it.dz};
        k.append(reinterpret_cast<const char*>(v), sizeof v);
    }
    return k;
}
inline AircraftDerived deriveAircraft(const AircraftFile& a, double v_cruise = 17.0) {
    static std::mutex mx;
    static std::map<std::string, AircraftDerived> cache;
    std::string key = aircraftKey(a);
    key.append(reinterpret_cast<const char*>(&v_cruise), sizeof v_cruise);
    {
        std::lock_guard<std::mutex> l(mx);
        const auto it = cache.find(key);
        if (it != cache.end()) return it->second;
    }
    AircraftDerived d = deriveAircraftImpl(a, v_cruise);
    std::lock_guard<std::mutex> l(mx);
    if (cache.size() > 64) cache.clear();
    cache[key] = d;
    return d;
}
inline AircraftDerived deriveAircraftImpl(const AircraftFile& a, double v_cruise) {
    AircraftDerived d;
    const double g = 9.80665, rho = a.num("aero.air_density"), pi = 3.14159265358979;
    const auto items = allMassItems(a);
    for (const MassItem& it : items) d.mass_airframe += it.m;
    const MassProps mp = massProperties(items);
    d.mass_total = mp.mass; d.cg_x = mp.cg_x; d.cg_y = mp.cg_y; d.cg_z = mp.cg_z; d.inertia = mp.inertia;
    // трапецієподібне крило (geometry.wing_tip_chord_m): площа за середньою хордою, далі скрізь c — САХ; чверть хорди пряма
    const double b = a.num("geometry.wing_span_m"), c_root = a.num("geometry.wing_chord_m");
    const double c_tip = a.has("geometry.wing_tip_chord_m") ? a.num("geometry.wing_tip_chord_m") : c_root, lam = c_tip / c_root;
    const double c = 2.0 / 3.0 * c_root * (1 + lam + lam * lam) / (1 + lam);
    d.wing_area = b * (c_root + c_tip) / 2; d.aspect = b * b / d.wing_area;
    d.wing_le_x = a.num("geometry.cg_chord_frac") * c; d.wing_te_x = d.wing_le_x - c; d.wing_ac_x = d.wing_le_x - 0.25 * c;
    const double aw = a.num("aero.cl_alpha_3d"), at = a.num("aero.tail_cl_alpha"), eta = a.num("aero.tail_efficiency");
    const double deps = 2 * aw / (pi * d.aspect);
    // заднє крило: фокус на чверті хорди; нахил CL(α) — з поправкою на подовження (Гельмбольд: a ∝ AR/(AR+2)) відносно основного
    double rw = 0;   // внесок заднього крила в Σ a·S (у сліді основного: η, 1 − dε/dα)
    if (a.has("rear_wing.span_m")) {
        const double b2 = a.num("rear_wing.span_m"), c2 = a.num("rear_wing.chord_m");
        d.rw_area = b2 * c2; d.rw_aspect = b2 * b2 / d.rw_area; d.rw_ac_x = a.num("rear_wing.le_x_m") - 0.25 * c2;
        rw = aw * (d.rw_aspect / (d.rw_aspect + 2)) / (d.aspect / (d.aspect + 2)) * eta * d.rw_area * (1 - deps);
        // рівновага моментів відносно ЦМ на звалюванні (CL ≈ CLmax): L1·x1 + L2·x2 + cm0·q·S·c = 0, L1 + L2 = W
        const double cm0 = a.has("aero.cm0_wing") ? a.num("aero.cm0_wing") : 0.0, m0_per_w = cm0 * c / a.num("aero.cl_max_3d");
        d.rw_lift_frac = (d.wing_ac_x - d.cg_x + m0_per_w) / (d.wing_ac_x - d.rw_ac_x);
    }
    // звалювання: основне крило досягає CLmax, несучи (1 − частка заднього) ваги
    d.v_stall = std::sqrt(2 * d.mass_total * g * (1 - d.rw_lift_frac) / (rho * d.wing_area * a.num("aero.cl_max_3d")));
    d.cl_cruise = 2 * d.mass_total * g / (rho * v_cruise * v_cruise * d.wing_area);
    d.re_cruise = v_cruise * c / 1.46e-5;
    // α до хорди: α = α0 + CL/CLα (α0 < 0 — нульова підйомна сила при від'ємному куті); кут тіла = α − кут установки крила
    d.alpha_cruise_deg = a.num("aero.alpha_zero_deg") + d.cl_cruise / a.num("aero.cl_alpha_3d") * 180 / pi;
    // оперення: V-хвіст — горизонтальна ефективна площа S·cos²Γ, вертикальна S·sin²Γ (нахил підйомної сили панелі в обох площинах);
    // об'єми V_H = Sh·lh/(S·c), V_V = Sv·lv/(S·b); плече — від ЦМ до чверті хорди
    const double gam = a.num("tail.vtail_dihedral_deg") * pi / 180, s_v = 2 * a.num("tail.vtail_panel_m") * a.num("tail.vtail_chord_m");
    d.tail_area = s_v * std::cos(gam) * std::cos(gam);
    const double tail_ac = a.num("tail.vtail_le_x_m") - 0.25 * a.num("tail.vtail_chord_m");
    d.tail_arm = d.cg_x - tail_ac;
    d.fin_area = s_v * std::sin(gam) * std::sin(gam);
    d.v_h = d.tail_area * d.tail_arm / (d.wing_area * c);
    d.v_v = d.fin_area * d.tail_arm / (d.wing_area * b);
    // нейтральна точка: основне крило + заднє крило + V-хвіст (без внеску фюзеляжу й гвинтів — оптимістична оцінка запасу)
    const double wt = aw * d.wing_area, tt = at * eta * d.tail_area * (1 - deps);
    d.neutral_x = (wt * d.wing_ac_x + rw * d.rw_ac_x + tt * tail_ac) / (wt + rw + tt);
    d.static_margin = (d.cg_x - d.neutral_x) / c;
    // зазори гвинтів: передній диск (у висінні над віссю гондоли) до передньої кромки крила; задній — до кінця фюзеляжу й між собою
    const double pr = a.num("vtol_prop.diameter_m") / 2;
    d.front_prop_clear = (a.num("vtol_motor.front_x_m") - pr) - d.wing_le_x;
    propClearance(a, d);
    d.rear_prop_clear = 2 * a.num("vtol_motor.rear_y_m") - 2 * pr;   // між задніми дисками
    const double area = a.num("vtol_motor.count") * pi * pr * pr, T = d.mass_total * g;
    d.hover_ideal_w = std::pow(T, 1.5) / std::sqrt(2 * rho * area);
    d.hover_elec_w = d.hover_ideal_w / 0.60 / (0.80);   // FoM 0.60, ККД мотор·ESC 0.80 (оцінка з docs/AERO_MODEL.md)
    d.battery_wh = a.num("battery.cells_series") * a.num("battery.cells_parallel") * a.num("battery.cell_capacity_ah") * a.num("battery.cell_nominal_v");

    auto fail = [&](const char* fmt, double v1, double v2) {
        char buf[200]; std::snprintf(buf, sizeof buf, fmt, v1, v2); d.problems.push_back(buf);
    };
    if (std::fabs(d.mass_total - a.num("targets.takeoff_mass_kg")) > 0.01) fail("маса %.3f кг ≠ злітна %.3f кг", d.mass_total, a.num("targets.takeoff_mass_kg"));
    if (std::fabs(d.cg_x) > 0.002 || std::fabs(d.cg_y) > 0.002) fail("ЦМ не в початку координат: x=%.4f y=%.4f м", d.cg_x, d.cg_y);
    if (std::fabs(a.num("geometry.nose_x_m") - a.num("geometry.tail_x_m") - a.num("geometry.overall_length_m")) > 0.001)
        fail("довжина %.3f м не збігається з nose_x − tail_x (%.3f)", a.num("geometry.overall_length_m"), a.num("geometry.nose_x_m") - a.num("geometry.tail_x_m"));
    if (d.rear_prop_clear < 0.02) fail("задні гвинти перекриваються: зазор %.4f м (%.0f)", d.rear_prop_clear, 0);
    if (d.front_prop_clear < 0) fail("передній гвинт заходить на крило: %.4f м (%.0f)", d.front_prop_clear, 0);
    if (d.clear_min < 0.02) fail("гвинт ближче 2 см до конструкції: %.1f мм (%.0f)", 1000 * d.clear_min, 0), d.problems.back() += ": " + d.clear_what;
    if (std::fabs(d.v_stall - a.num("targets.v_stall_mps")) > 0.05 * a.num("targets.v_stall_mps")) fail("V_stall %.2f м/с далеко від %.1f м/с", d.v_stall, a.num("targets.v_stall_mps"));
    if (d.static_margin < a.num("targets.sm_min_frac") || d.static_margin > a.num("targets.sm_max_frac"))
        fail("запас стійкості %.3f САХ поза межами %.2f", d.static_margin, a.num("targets.sm_min_frac"));
    if (std::fabs(d.battery_wh - a.num("battery.usable_wh_target")) > 0.05 * a.num("battery.usable_wh_target")) fail("батарея %.1f Вт·год ≠ %.0f Вт·год", d.battery_wh, a.num("battery.usable_wh_target"));
    return d;
}

inline void printAircraftReport(const AircraftFile& a, const AircraftDerived& d) {
    std::printf("=== Конвертоплан: маси (кг) і положення відносно креслення ===\n");
    for (const MassItem& it : allMassItems(a)) std::printf("  %-18s %6.3f   x %+7.3f  y %+6.3f  z %+6.3f\n", it.name.c_str(), it.m, it.x, it.y, it.z);
    std::printf("  разом %.3f кг\n", d.mass_total);
    std::printf("ЦМ: x %+.4f y %+.4f z %+.4f м\n", d.cg_x, d.cg_y, d.cg_z);
    std::printf("Інерція (кг·м²): Ixx %.4f Iyy %.4f Izz %.4f  Ixy %.5f Ixz %.5f Iyz %.5f\n", d.inertia.xx, d.inertia.yy, d.inertia.zz,
                d.inertia.xy, d.inertia.xz, d.inertia.yz);
    std::printf("Крило: S %.3f м², AR %.2f, передня кромка x %+.4f, задня %+.4f, фокус крила %+.4f м\n", d.wing_area, d.aspect, d.wing_le_x, d.wing_te_x, d.wing_ac_x);
    std::printf("Звалювання: %.2f м/с (CLmax3D %.2f); крейсер 17 м/с: CL %.2f, Re %.2e, α≈%.1f°\n", d.v_stall, a.num("aero.cl_max_3d"), d.cl_cruise, d.re_cruise, d.alpha_cruise_deg);
    if (d.rw_area > 0) std::printf("Заднє крило: S %.3f м², AR %.2f, фокус %+.4f м; на звалюванні несе ≈ %.0f%% ваги\n", d.rw_area, d.rw_aspect, d.rw_ac_x, 100 * d.rw_lift_frac);
    std::printf("V-хвіст: ефективна горизонтальна площа %.3f м², плече %.3f м, V_H %.3f; вертикальна %.3f м², V_V %.4f\n", d.tail_area, d.tail_arm, d.v_h, d.fin_area, d.v_v);
    std::printf("Нейтральна точка x %+.4f м → запас стійкості %.1f%% САХ (без внеску фюзеляжу й гвинтів)\n", d.neutral_x, 100 * d.static_margin);
    std::printf("Зазор гвинтів: між задніми дисками %.1f мм, передні до крила %.1f мм; найменший у діапазоні нахилу %.1f мм (%s)\n", 1000 * d.rear_prop_clear,
                1000 * d.front_prop_clear, 1000 * d.clear_min, d.clear_what.c_str());
    std::printf("Висіння: ідеально %.0f Вт, електрично ≈ %.0f Вт; батарея %.0f Вт·год → ≈ %.1f хв висіння\n", d.hover_ideal_w, d.hover_elec_w, d.battery_wh,
                60 * d.battery_wh / d.hover_elec_w);
    if (d.problems.empty()) std::printf("ПЕРЕВІРКА: усе в межах\n");
    for (const std::string& p : d.problems) std::printf("ПЕРЕВІРКА НЕ ПРОЙДЕНА: %s\n", p.c_str());
}

}  // namespace sim
