// ai/force_model.h — нейромережа «модель сил»: з того, що бачить польотний контролер (повітряна швидкість, швидкість у тілі, кути,
// оберти й нахил гондол, керма), передбачає аеродинамічну силу на планер (крило, оперення, корпус) у тілі. Контролер віднімає
// її наперед (fc::VtolController::a_dist) — як Neural-Fly (Caltech): не чекати, поки порив чи підйомна сила крила
// накопичиться в похибці положення й інтеграторах.
// Ознаки — лише те, що є на реальному апараті; вітру мережа не знає (бачить його через трубу Піто й рух відносно землі).
// Ваги — ai/weights_force.h (tools/ai_train.py → tools/ai_export.py); без файлу модель вимкнена.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include "nn.h"
#if defined(__has_include)
#if __has_include("weights_force.h")
#include "weights_force.h"
#define AI_HAS_FORCE_WEIGHTS 1
#endif
#if __has_include("weights_force_qp5.h")
#include "weights_force_qp5.h"
#define AI_HAS_FORCE_WEIGHTS_QP5 1
#endif
#endif

namespace ai {

constexpr int kForceFeatures = 17, kForceOutputs = 3;

struct ForceInputs {
    float airspeed = 0;                 // м/с, труба Піто
    float v_body[3] = {0, 0, 0};        // м/с, швидкість відносно землі в осях тіла (GPS/EKF)
    float roll = 0, pitch_down = 0;     // рад
    float rps[4] = {0, 0, 0, 0};        // об/с, телеметрія ESC
    float tilt[4] = {0, 0, 0, 0};       // рад, енкодери гондол
    float elevator = 0, aileron = 0;    // рад, останні команди кермам
    float mass_ratio = 1;               // оцінка маси автопілотом / маса без вантажу (вантаж)
};

// Вектор ознак (масштаби — щоб усі були порядку одиниці ще до нормалізації)
inline void forceFeatures(const ForceInputs& in, float f[kForceFeatures]) {
    int k = 0;
    f[k++] = in.airspeed / 10.0f;
    f[k++] = in.airspeed * in.airspeed / 100.0f;   // сили крила ∝ V²
    for (int i = 0; i < 3; ++i) f[k++] = in.v_body[i] / 10.0f;
    f[k++] = in.roll; f[k++] = in.pitch_down;
    for (int i = 0; i < 4; ++i) f[k++] = in.rps[i] / 100.0f;
    for (int i = 0; i < 4; ++i) f[k++] = in.tilt[i];
    f[k++] = in.elevator; f[k++] = in.aileron;
}

// Пам'ять мережі (так само в tools/ai_train.py): кадри 50 Гц. Для 11 «рухливих» ознак (швидкості, кути, оберти) — різниця
// «зараз − тоді» 0.1, 0.3 і 1.0 с тому; для виміряної сили (акселерометр − таблиця тяги, частки ваги) — зараз, 0.1, 0.3 і 1.0 с тому.
constexpr int kForceDyn = 11;
constexpr int kForceLagDyn[3] = {5, 15, 50};
constexpr int kForceLagMeas[4] = {0, 5, 15, 50};
constexpr int kForceHistInputs = kForceFeatures + kForceDyn * 3 + 3 * 4;   // 62
constexpr int kForceMassInputs = kForceHistInputs + 1;                     // 63: + маса (вантаж)
constexpr int kForceRing = 64;                                             // ≥ 51 кадр (1 с)

// Мережа: вихід — сила в тілі в частках ваги (m·g); predict повертає ньютони.
// Два формати ваг: 17 входів (лише поточна мить) або 62 (з пам'яттю 1 с) — визначається за розміром першого шару.
class ForceModel {
public:
    bool loaded() const { return loaded_; }
    bool hasMemory() const { return hist_; }
    int inputs() const { return net_.inputs(); }
    void setWeight(float m_g) { mg_ = m_g; }
    OpCounter ops;
    ForceModel() { select(""); }
    // набір ваг під апарат: "" — основний (ai/weights_force.h), "qp5" — QP-5 (ai/weights_force_qp5.h; ключ ai.force_weights у файлі апарата).
    // Якщо набору для апарата ще немає — модель вимкнена: чужі ваги гірші, ніж жодних
    const std::string& variant() const { return variant_; }
    void select(const std::string& variant) {
        variant_ = variant; loaded_ = false;
        bool have = false;
        if (variant.empty()) {
#ifdef AI_HAS_FORCE_WEIGHTS
            net_ = ai_w_force::modelInt8(); have = true;
#endif
        } else if (variant == "qp5") {
#ifdef AI_HAS_FORCE_WEIGHTS_QP5
            net_ = ai_w_force_qp5::modelInt8(); have = true;
#endif
        }
        if (!have) return;
        hist_ = net_.inputs() == kForceHistInputs || net_.inputs() == kForceMassInputs;
        mass_in_ = net_.inputs() == kForceMassInputs;
        loaded_ = (net_.inputs() == kForceFeatures || hist_) && net_.outputs() == kForceOutputs;
    }
    void reset() { count_ = 0; head_ = 0; }
    // новий кадр 50 Гц: те, що бачать датчики зараз, і виміряна сила (Н)
    void push(const ForceInputs& in, const float meas_n[3]) {
        head_ = (head_ + 1) % kForceRing;
        forceFeatures(in, ring_[size_t(head_)].f);
        for (int i = 0; i < 3; ++i) ring_[size_t(head_)].m[i] = meas_n[i] / mg_;
        ring_[size_t(head_)].mass = in.mass_ratio;
        count_ = std::min(count_ + 1, kForceRing);
    }
    int memoryFrames() const { return count_; }
    // вхід мережі з останнього кадру й пам'яті
    int buildInput(float* x) const {
        const Frame& now = frame(0);
        int k = 0;
        for (int i = 0; i < kForceFeatures; ++i) x[k++] = now.f[i];
        if (!hist_) return k;
        for (int L : kForceLagDyn) { const Frame& p = frame(L); for (int i = 0; i < kForceDyn; ++i) x[k++] = now.f[i] - p.f[i]; }
        for (int L : kForceLagMeas) { const Frame& p = frame(L); for (int i = 0; i < 3; ++i) x[k++] = p.m[i]; }
        if (mass_in_) x[k++] = now.mass;
        return k;
    }
    bool predict(float force_n[3]) {
        if (!loaded_ || count_ == 0) return false;
        float x[kForceMassInputs], y[kForceOutputs];
        buildInput(x);
        net_.forward(x, y, &ops);
        for (int i = 0; i < 3; ++i) force_n[i] = std::isfinite(y[i]) ? y[i] * mg_ : 0.0f;
        return true;
    }
    // без пам'яті (старий виклик): один кадр
    bool predict(const ForceInputs& in, float force_n[3]) {
        const float zero[3] = {0, 0, 0};
        push(in, zero);
        return predict(force_n);
    }

    // Повний «знімок думки» мережі: вхід, нормалізований вхід, обидва приховані шари, вихід (Н)
    struct Trace { float feat[kForceMassInputs], norm[kForceMassInputs], h1[kMaxWidth], h2[kMaxWidth], out[kForceOutputs]; int n_in = 0, n1 = 0, n2 = 0; };
    bool trace(Trace& t) const {
        if (!loaded_ || net_.count != 3 || count_ == 0) return false;
        t.n_in = buildInput(t.feat);
        float y[kForceOutputs];
        float* outs[3] = {t.h1, t.h2, nullptr};
        net_.forwardTrace(t.feat, y, t.norm, outs);
        t.n1 = net_.layers[0].out; t.n2 = net_.layers[1].out;
        for (int i = 0; i < 3; ++i) t.out[i] = std::isfinite(y[i]) ? y[i] * mg_ : 0.0f;
        return true;
    }
    const Mlp<DenseQ>& net() const { return net_; }

private:
    std::string variant_;
    struct Frame { float f[kForceFeatures]; float m[3]; float mass = 1; };
    std::array<Frame, kForceRing> ring_{};
    int head_ = 0, count_ = 0;
    const Frame& frame(int lag) const { const int l = std::min(lag, count_ - 1); return ring_[size_t((head_ - l + kForceRing) % kForceRing)]; }
    Mlp<DenseQ> net_;
    bool loaded_ = false, hist_ = false, mass_in_ = false;
    float mg_ = 23.4f;
};

}  // namespace ai
