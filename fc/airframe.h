// airframe.h — аеродинаміка планера: обтічний корпус, крило, кіль, штовхальний (тяговий) мотор.
// Спільна модель: симулятор рахує нею «справжні» сили, контролер — свою оцінку підйомної сили.
//
// Система тіла: X — вперед (ніс), Y — вліво, Z — вгору. Позиції — від геометричного центру рами.
// Крило: модель Біарда–Макліна (Small Unmanned Aircraft, 2012): до зриву — лінійна підйомна сила
// і індуктивний опір, після зриву — плавний перехід до «пластини» (працює на будь-яких кутах, і коли
// вітер дме в спину чи знизу — це важливо для коптера, який висить і рухається в усі боки).
#pragma once
#include "fc_math.h"

namespace fc {

constexpr float kRho = 1.225f;  // кг/м³, густина повітря

struct WingConfig {
    bool enabled = false;
    float area = 0.50f;           // м², площа крила
    float span = 1.60f;           // м, розмах
    float incidence_deg = 8.0f;   // кут установки відносно рами (ніс вгору = +)
    float x = 0.0f;               // м, положення аеродинамічного фокусу (+ вперед від центру рами)
    float z = 0.06f;              // м, висота над центром рами
    float cd0 = 0.025f;           // профільний опір
    float cl_max = 1.15f;         // максимальна підйомна сила (далі — зрив)
    float oswald = 0.8f;          // ефективність розмаху
    float mass_per_m2 = 1.3f;     // кг/м²: пінопласт EPP/XPS + карбоновий лонжерон + обшивка
    float under_rotors = 0.0f;    // 0..1 — частка крила під гвинтами (потік від гвинтів б'є в крило згори)
    float aspect() const { return span * span / area; }
    float chord() const { return area / span; }
    float clAlpha() const { float A = aspect(); return 2 * kPi * A / (A + 2); }   // 1/рад
    float alphaStall() const { return cl_max / clAlpha(); }
    float mass() const { return enabled ? area * mass_per_m2 : 0.0f; }
};

struct FinConfig {             // вертикальний кіль ззаду: сам розвертає дрон носом проти вітру
    float area = 0.0f;         // м² (0 — немає)
    float arm = 0.55f;         // м, наскільки позаду центру
    float height = 0.25f;      // м
    float mass_per_m2 = 1.0f;
    float mass() const { return area * mass_per_m2; }
};

struct HTailConfig {           // горизонтальний стабілізатор ззаду (напр. між двома балками): стійкість по тангажу
    float area = 0.0f;         // м² (0 — немає)
    float span = 0.78f;        // м
    float arm = 1.12f;         // м, наскільки позаду центру мас (аеродинамічний фокус стабілізатора)
    float z = 0.05f;           // м
    float incidence_deg = -1.0f;
    float mass_per_m2 = 0.8f;
    float mass() const { return area * mass_per_m2; }
};

struct PusherConfig {          // штовхальний гвинт ззаду: тягне вперед без нахилу корпусу
    bool enabled = false;
    float max_thrust = 25.0f;  // Н (статична, при номінальній напрузі)
    float diameter = 0.33f;    // м (13")
    float x = -0.30f;          // м
    float mass = 0.40f;        // кг: мотор + регулятор + пропелер + кріплення
    float eff = 0.55f;         // ККД гвинт × мотор × регулятор
    float pitch_speed = 35.0f; // м/с — «крок» гвинта: на такій швидкості потоку тяга падає до нуля
    float thrustFactor(float u) const { return std::fmax(0.0f, 1.0f - std::fmax(u, 0.0f) / pitch_speed); }
};

// Рулі для літакового режиму (підйомні гвинти зупинені): елерони на крилі, руль висоти на стабілізаторі,
// рулі напряму на кілях. Ефективність — безрозмірні похідні моментів на радіан відхилення:
//   крен   L = q·S·b·Clδa·δa,   тангаж M = q·S·c·Cmδe·δe,   курс N = q·S·b·Cnδr·δr   (q = ρV²/2 — уздовж носа).
// Команди −1..1 задаються «за моментом»: +1 елеронів — праве крило вниз, +1 руля висоти — ніс униз,
// +1 рулів напряму — ніс уліво. Які машинки в який бік — розводиться на виході контролера (reverse).
struct SurfaceConfig {
    bool enabled = false;
    float ail_cl = 0.20f;      // Clδa: елерони ~40% розмаху, ~25% хорди
    float ele_cm = 0.85f;      // Cmδe: руль висоти на стабілізаторі ≈ CLα_ст · τ · (S_ст · l_ст)/(S · c)
    float rud_cn = 0.04f;      // Cnδr: рулі напряму на кілях ≈ CLα_кіль · τ · (S_кіль · l_кіль)/(S · b)
    float max_deg = 25.0f;     // найбільше відхилення
    float tau = 0.04f;         // с — швидкість машинки (~0.1 с/60°)
    float mass = 0.12f;        // кг — 5 машинок + тяги + петлі
};

// Моменти від рулів (система тіла, Н·м). qd — динамічний тиск уздовж носа, Па.
inline Vec3 surfaceMoments(const SurfaceConfig& sc, const WingConfig& w, float qd, float ail, float ele, float rud) {
    if (!sc.enabled || qd <= 0) return Vec3{};
    const float dm = sc.max_deg * kDeg2Rad;
    const float qS = qd * w.area;
    return Vec3{qS * w.span * sc.ail_cl * dm * clampf(ail, -1, 1),
                qS * w.chord() * sc.ele_cm * dm * clampf(ele, -1, 1),
                qS * w.span * sc.rud_cn * dm * clampf(rud, -1, 1)};
}

struct AirframeConfig {
    float body_cd = 0.10f;     // Н/(м/с)²: опір корпусу (0.10 — відкрита рама, 0.05 — обтічний кокон)
    WingConfig wing;
    FinConfig fin;
    HTailConfig htail;
    PusherConfig pusher;
    SurfaceConfig surfaces;
    bool pitot = true;         // датчик швидкості повітря (трубка Піто) — якщо є крило
    float extraMass() const { return wing.mass() + fin.mass() + htail.mass() + (pusher.enabled ? pusher.mass : 0.0f) + (surfaces.enabled ? surfaces.mass : 0.0f); }
    bool hasAero() const { return wing.enabled || fin.area > 0 || htail.area > 0; }
};

// Коефіцієнти підйомної сили і опору для будь-якого кута атаки (рад)
inline void wingCoeffs(const WingConfig& w, float alpha, float& cl, float& cd) {
    // Перехід до «пластини» центровано трохи за кутом звалювання, а лінійну частину м'яко обмежено CLmax:
    // інакше на самому куті звалювання суміш була б уже наполовину пластиною і реальний максимум CL
    // виходив ~0.66·CLmax (звалювання на 20–25% швидше, ніж закладено)
    const float M = 30.0f, a0 = w.alphaStall() + 2.0f / M;
    auto ex = [](float v) { return std::exp(clampf(v, -40.0f, 40.0f)); };
    const float e1 = ex(-M * (alpha - a0)), e2 = ex(M * (alpha + a0));
    const float sig = (1 + e1 + e2) / ((1 + e1) * (1 + e2));   // 0 — до зриву, 1 — зрив
    const float A = w.aspect();
    const float s = std::sin(alpha), c = std::cos(alpha);
    float cl_lin = w.clAlpha() * alpha;
    {
        const float k = 12.0f / w.cl_max, cm = w.cl_max, x = cl_lin;
        cl_lin = x > 0 ? x - std::log1p(std::exp(std::fmin(k * (x - cm), 40.0f))) / k
                       : x + std::log1p(std::exp(std::fmin(k * (-x - cm), 40.0f))) / k;
    }
    const float cl_fp = 2 * (alpha >= 0 ? 1.0f : -1.0f) * s * s * c;
    cl = (1 - sig) * cl_lin + sig * cl_fp;
    const float cd_lin = w.cd0 + cl_lin * cl_lin / (kPi * w.oswald * A);
    const float cd_fp = w.cd0 + 1.3f * s * s;
    cd = (1 - sig) * std::fmin(cd_lin, 2.0f) + sig * cd_fp;
}

struct AeroResult {
    Vec3 force;            // Н, система тіла
    Vec3 torque;           // Н·м, відносно точки ref (центр мас)
    float alpha = 0;       // рад, кут атаки крила
    float airspeed = 0;    // м/с у площині крила
    float lift = 0;        // Н, підйомна сила (уздовж осі Z тіла)
    float wing_drag = 0;   // Н
    float fin_force = 0;   // Н, бічна сила кіля
    float download = 0;    // Н, тиск потоку від гвинтів на крило
    float tail_force = 0;  // Н, сила на стабілізаторі (вгору +)
};

// va_body — швидкість дрона відносно повітря (система тіла), omega — кутова швидкість,
// ref — точка, відносно якої рахуємо моменти (центр мас), rotor_thrust — загальна тяга гвинтів
inline AeroResult airframeForces(const AirframeConfig& af, const Vec3& va_body, const Vec3& omega, const Vec3& ref,
                                 float rotor_thrust = 0, float rotor_disk_area = 0.114f) {
    AeroResult r;
    const WingConfig& w = af.wing;
    float wing_cl = 0;   // для скосу потоку на стабілізаторі
    if (w.enabled) {
        const Vec3 pos{w.x, 0, w.z};
        const Vec3 v = va_body + omega.cross(pos);
        const float V = std::sqrt(v.x * v.x + v.z * v.z);
        r.airspeed = V;
        if (V > 0.05f) {
            r.alpha = std::atan2(-v.z, v.x) + w.incidence_deg * kDeg2Rad;
            r.alpha = std::remainder(r.alpha, 2 * kPi);
            float cl, cd;
            wingCoeffs(w, r.alpha, cl, cd);
            wing_cl = cl;
            const float q = 0.5f * kRho * V * V;
            const float L = q * w.area * cl, D = q * w.area * cd;
            const Vec3 ld{-v.z / V, 0, v.x / V};     // перпендикулярно потоку, «вгору» відносно крила
            const Vec3 dd{-v.x / V, 0, -v.z / V};    // уздовж потоку
            Vec3 F = ld * L + dd * D;
            // демпфування крену: опущене крило отримує більший кут атаки
            Vec3 Md{-0.5f * kRho * V * w.clAlpha() * w.area * w.span * w.span * omega.x / 12.0f, 0, 0};
            r.force += F;
            r.torque += (pos - ref).cross(F) + Md;
            r.lift = F.z;
            r.wing_drag = D;
        }
        // крило під гвинтами: швидкий потік від гвинтів тисне на нього згори (у русі потік зноситься назад)
        if (w.under_rotors > 0 && rotor_thrust > 0) {
            const float vh2 = rotor_thrust / 4 / (2 * kRho * rotor_disk_area);
            const float k = vh2 / (vh2 + va_body.x * va_body.x + va_body.y * va_body.y);
            r.download = 1.2f * w.under_rotors * w.area * (rotor_thrust / 4) / rotor_disk_area * 0.5f * k;
            r.force.z -= r.download;
            r.torque += (pos - ref).cross(Vec3{0, 0, -r.download});
        }
    }
    if (af.htail.area > 0) {   // стабілізатор: маленьке крило ззаду; крило попереду скошує йому потік донизу
        const HTailConfig& h = af.htail;
        const Vec3 pos{-h.arm, 0, h.z};
        const Vec3 v = va_body + omega.cross(pos);   // обертання по тангажу -> інший кут атаки -> демпфування
        const float V = std::sqrt(v.x * v.x + v.z * v.z);
        if (V > 0.05f) {
            WingConfig t;
            t.enabled = true; t.area = h.area; t.span = h.span; t.cd0 = 0.02f; t.cl_max = 1.0f; t.incidence_deg = 0;
            // скіс потоку від крила ε = 2·CL_крила / (π·A): від реальної підйомної сили — плавно, і після зриву
            // крила сам зникає (раніше був стрибок на 15° при куті потоку 0.5 рад)
            const float eps = w.enabled ? 2 * wing_cl / (kPi * w.aspect()) : 0.0f;
            float a = std::remainder(std::atan2(-v.z, v.x), 2 * kPi);
            a += h.incidence_deg * kDeg2Rad - eps;
            float cl, cd;
            wingCoeffs(t, a, cl, cd);
            const float q = 0.5f * kRho * V * V;
            const Vec3 ld{-v.z / V, 0, v.x / V}, dd{-v.x / V, 0, -v.z / V};
            Vec3 F = ld * (q * t.area * cl) + dd * (q * t.area * cd);
            r.force += F;
            r.torque += (pos - ref).cross(F);
            r.tail_force = F.z;
        }
    }
    if (af.fin.area > 0) {   // кіль: «крило», поставлене вертикально
        const Vec3 pos{-af.fin.arm, 0, af.fin.height * 0.5f};
        const Vec3 v = va_body + omega.cross(pos);
        const float V = std::sqrt(v.x * v.x + v.y * v.y);
        if (V > 0.05f) {
            WingConfig f;
            f.enabled = true; f.area = af.fin.area; f.span = af.fin.height; f.cd0 = 0.02f; f.cl_max = 1.0f; f.incidence_deg = 0;
            const float beta = std::remainder(std::atan2(v.y, v.x), 2 * kPi);
            float cl, cd;
            wingCoeffs(f, beta, cl, cd);
            const float q = 0.5f * kRho * V * V;
            const Vec3 ld{v.y / V, -v.x / V, 0}, dd{-v.x / V, -v.y / V, 0};
            Vec3 F = ld * (q * f.area * cl) + dd * (q * f.area * cd);
            r.force += F;
            r.torque += (pos - ref).cross(F);
            r.fin_force = F.y;
        }
    }
    return r;
}

// Тяговий гвинт: сила і споживана потужність при швидкості потоку u (м/с уздовж осі гвинта)
inline float pusherPower(const PusherConfig& p, float thrust, float u) {
    if (thrust <= 0) return 0;
    const float A = kPi * p.diameter * p.diameter / 4;
    u = std::fmax(u, 0.0f);
    const float vi = 0.5f * (-u + std::sqrt(u * u + 2 * thrust / (kRho * A)));
    return thrust * (u + vi) / p.eff;
}

}  // namespace fc
