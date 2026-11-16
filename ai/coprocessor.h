// ai/coprocessor.h — «другий мозок»: ШІ-співпроцесор, що приймає кадри від польотного контролера, стежить за станом апарата
// й шле поради (AdviceFrame) та повідомлення людині (MessageFrame). Не залежить від корпусу: усе, що він знає про апарат, —
// «норма», яку він сам запам'ятовує в перших хвилинах польоту, і порівняння вузлів між собою.
// Нейромережеві частини підключаються через слоти (force_model, vib_model) — ваги з tools/ai_export.py; без них працюють
// класичні ознаки (features.h) з порогами.
#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "features.h"
#include "link.h"
#include "nn.h"

namespace ai {

enum class Code : uint16_t {
    PropImbalance = 101,   // вібрація 1× обертів зросла: дисбаланс або пошкодження гвинта
    PropLoose = 102,       // 0.5× / 2× обертів зросли: люфт кріплення гвинта чи мотора
    MotorLoad = 110,       // мотор бере більше струму за інших при тих самих обертах
    ServoSlow = 120,       // привід нахилу гондоли запізнюється
    ServoBacklash = 121,   // люфт механізму нахилу
    BatteryAging = 130,    // внутрішній опір батареї зріс
};

struct CoprocessorConfig {
    float imu_hz = 1000, telem_hz = 100;
    float baseline_s = 60;          // скільки секунд польоту запам'ятовувати «норму»
    float vib_ratio = 2.5f;         // у скільки разів вібрація має перевищити норму
    float vib_min = 0.3f;           // м/с² — менші амплітуди не турбують людину
    float motor_load_ratio = 1.15f; // мотор бере на 15% більше, ніж за нормою відносно інших
    float servo_lag_s = 0.12f;
    float servo_backlash_rad = 0.0175f;   // 1°
    float battery_r_ratio = 1.6f;
    float persist_s = 3.0f;         // ознака має триматися стільки, щоб стати повідомленням
};

class AiCoprocessor {
public:
    CoprocessorConfig cfg;
    std::function<void(const MessageFrame&)> on_message;   // куди слати повідомлення (лінк або журнал)
    // Слот нейромережі моделі сил: вхід — вектор ознак телеметрії, вихід — 6 чисел (сила й момент у тілі). Порожній — порад немає.
    std::function<bool(const TelemetryFrame&, float force[3], float moment[3], float& confidence)> force_model;
    OpCounter ops;

    // --- вхідні кадри
    void onImu(const ImuBatchFrame& f) {
        for (int i = 0; i < 10; ++i) {
            const float a[3] = {f.acc[i][0] * f.acc_scale, f.acc[i][1] * f.acc_scale, f.acc[i][2] * f.acc_scale};
            orders_.update(a, rpm_, 1.0f / cfg.imu_hz);
            spectrum_.push(a);
        }
    }
    void onTelemetry(const TelemetryFrame& t) {
        last_ = t; have_ = true;
        const float dt = 1.0f / cfg.telem_hz;
        now_ = float(t.t_ms) * 1e-3f;
        for (int m = 0; m < 4; ++m) rpm_[m] = t.rpm[m];
        const bool air = t.flags & 1;
        if (air) air_s_ += dt;
        // приводи нахилу: 200 Гц у залізі, тут — з частотою телеметрії
        for (int m = 0; m < 4; ++m) { servo_[m].dt = dt; servo_[m].update(t.tilt_cmd[m], t.tilt_meas[m]); }
        if (std::fabs(t.i_bus) > 0.5f) battery_.update(t.v_bus, t.i_bus);
        if (air) learnOrCheck(t, dt);
    }

    // --- вихід: порада для контролера (50 Гц)
    AdviceFrame advice() {
        AdviceFrame a; a.seq = ++seq_;
        if (have_ && force_model) {
            float c = 0;
            if (force_model(last_, a.force, a.moment, c)) { a.status = 2; a.confidence = c; }
        }
        return a;
    }

    // --- для інтерфейсу й тестів
    const OrderTracker& orders() const { return orders_; }
    const BandSpectrum& spectrum() const { return spectrum_; }
    const ServoMonitor& servo(int m) const { return servo_[m]; }
    const BatteryRls& battery() const { return battery_; }
    bool baselineReady() const { return base_ready_; }
    const std::vector<MessageFrame>& messages() const { return sent_; }

    // Текст повідомлення для людини (на наземній станції: по лінку йдуть лише код, вузол і число)
    static std::string text(const MessageFrame& m) {
        char b[512];
        const int n = m.node;
        switch (Code(m.code)) {
            case Code::PropImbalance: std::snprintf(b, sizeof b, "Мотор %d: вібрація на частоті обертів у %.1f раза більша за норму — перевірте гвинт (пошкодження, баланс) і затягування.", n, m.value); break;
            case Code::PropLoose: std::snprintf(b, sizeof b, "Мотор %d: вібрація на половині/подвійній частоті обертів у %.1f раза більша за норму — схоже на люфт: перевірте, наскільки міцно затиснутий гвинт і мотор.", n, m.value); break;
            case Code::MotorLoad: std::snprintf(b, sizeof b, "Мотор %d бере на %.0f%% більше струму за інших при тих самих обертах — перевірте гвинт (тріщина, бруд), підшипники й ESC.", n, (m.value - 1) * 100); break;
            case Code::ServoSlow: std::snprintf(b, sizeof b, "Гондола %d: привід нахилу запізнюється на %.2f с — перевірте сервопривід, живлення й тертя в шарнірі.", n, m.value); break;
            case Code::ServoBacklash: std::snprintf(b, sizeof b, "Гондола %d: люфт механізму нахилу %.1f° — перевірте шарнір і кріплення сервоприводу.", n, m.value * 57.29578f); break;
            case Code::BatteryAging: std::snprintf(b, sizeof b, "Батарея: внутрішній опір у %.1f раза більший, ніж на початку польоту — батарея гріється або старіє, плануйте коротший політ.", m.value); break;
            default: std::snprintf(b, sizeof b, "Код %u, вузол %d, значення %.3f", unsigned(m.code), n, m.value); break;
        }
        return b;
    }

private:
    OrderTracker orders_;
    BandSpectrum spectrum_;
    ServoMonitor servo_[4];
    BatteryRls battery_;
    TelemetryFrame last_{};
    bool have_ = false, base_ready_ = false;
    float rpm_[4] = {0, 0, 0, 0}, now_ = 0, air_s_ = 0;
    uint32_t seq_ = 0;
    // норма: середні значення ознак за перші baseline_s польоту
    double b_n_ = 0, b_vib_[4][3] = {}, b_load_[4] = {}, b_r_ = 0, b_rn_ = 0;
    float persist_[4][8] = {};   // скільки секунд тримається кожна ознака [вузол][ознака]
    std::vector<MessageFrame> sent_;

    // відносне навантаження мотора: струм×напруга / (оберти³), поділене на середнє по всіх моторах — не залежить від режиму польоту
    void relativeLoad(const TelemetryFrame& t, float out[4]) const {
        float r[4], mean = 0; int k = 0;
        for (int m = 0; m < 4; ++m) {
            const float w = t.rpm[m] / 1000.0f;
            r[m] = w > 1.0f ? t.motor_i[m] / (w * w * w) : 0.0f;
            if (r[m] > 0) { mean += r[m]; ++k; }
        }
        mean = k ? mean / float(k) : 0.0f;
        for (int m = 0; m < 4; ++m) out[m] = mean > 0 ? r[m] / mean : 0.0f;
    }

    void learnOrCheck(const TelemetryFrame& t, float dt) {
        float load[4]; relativeLoad(t, load);
        if (!base_ready_) {   // спершу вчимося, яка «норма» саме в цього апарата (після 5 с — фільтри ознак устаталися)
            if (air_s_ < 5.0f) return;
            b_n_ += 1;
            for (int m = 0; m < 4; ++m) { for (int k = 0; k < 3; ++k) b_vib_[m][k] += orders_.amplitude(m, k); b_load_[m] += load[m]; }
            if (battery_.valid()) { b_r_ += battery_.R; b_rn_ += 1; }
            if (air_s_ >= 5.0f + cfg.baseline_s) {
                for (int m = 0; m < 4; ++m) { for (int k = 0; k < 3; ++k) b_vib_[m][k] /= b_n_; b_load_[m] /= b_n_; }
                b_r_ = b_rn_ > 0 ? b_r_ / b_rn_ : 0.0;   // струм майже не мінявся — норми опору немає, батарею не оцінюємо
                base_ready_ = true;
            }
            return;
        }
        for (int m = 0; m < 4; ++m) {
            const float r1 = ratio(orders_.amplitude(m, 1), b_vib_[m][1]);
            const float r05 = ratio(orders_.amplitude(m, 0), b_vib_[m][0]), r2 = ratio(orders_.amplitude(m, 2), b_vib_[m][2]);
            const bool loose = std::fmax(r05, r2) > cfg.vib_ratio && std::fmax(orders_.amplitude(m, 0), orders_.amplitude(m, 2)) > cfg.vib_min;
            check(m, 0, r1 > cfg.vib_ratio && orders_.amplitude(m, 1) > cfg.vib_min && !loose, dt, Code::PropImbalance, 1, r1);
            check(m, 1, loose, dt, Code::PropLoose, 1, std::fmax(r05, r2));
            const float lr = b_load_[m] > 0 ? float(load[m] / b_load_[m]) : 0.0f;
            check(m, 2, lr > cfg.motor_load_ratio, dt, Code::MotorLoad, 1, lr);
            check(m, 3, servo_[m].estimates > 0 && servo_[m].lag_s > cfg.servo_lag_s, dt, Code::ServoSlow, 1, servo_[m].lag_s);
            check(m, 4, servo_[m].estimates > 0 && servo_[m].backlash_rad > cfg.servo_backlash_rad, dt, Code::ServoBacklash, 1, servo_[m].backlash_rad);
        }
        const float rr = b_r_ > 1e-4 && battery_.valid() ? float(battery_.R / b_r_) : 0.0f;
        check(0, 5, rr > cfg.battery_r_ratio, dt, Code::BatteryAging, 0, rr, 10);
    }
    static float ratio(float v, double base) { return float(v / std::fmax(base, 0.02)); }

    // ознака має протриматися persist_s; кожне (вузол, код) повідомляється раз за політ
    void check(int m, int feat, bool on, float dt, Code code, uint8_t severity, float value, int node = -1) {
        float& p = persist_[m][feat];
        p = on ? p + dt : 0.0f;
        if (p < cfg.persist_s) return;
        const uint8_t nd = uint8_t(node >= 0 ? node : m + 1);
        for (const MessageFrame& s : sent_) if (s.node == nd && s.code == uint16_t(code)) return;
        MessageFrame f; f.t_ms = uint32_t(now_ * 1000); f.severity = severity; f.node = nd; f.code = uint16_t(code); f.value = value;
        sent_.push_back(f);
        if (on_message) on_message(f);
    }
};

}  // namespace ai
