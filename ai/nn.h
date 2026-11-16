// ai/nn.h — маленька бібліотека нейромереж для ШІ-співпроцесора (ESP32-S3 / STM32H7) і симулятора.
// Тільки std, без динамічної пам'яті в розрахунку: той самий код працює в симуляторі й на мікроконтролері.
// Формат ваг — як у PyTorch (Linear: W[out][in]; GRU: weight_ih[3H][I], weight_hh[3H][H], порядок воріт r, z, n),
// ваги приходять з tools/ai_export.py у вигляді заголовка з масивами.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ai {

constexpr int kMaxWidth = 256;   // найширший шар (вистачає для мереж до ≈ 300 тис. параметрів)

// Лічильник множень-додавань: оцінка навантаження на мікроконтролер (бюджет ESP32-S3 — див. docs/AI.md)
struct OpCounter {
    uint64_t macs = 0;
    void add(uint64_t n) { macs += n; }
};

enum class Act : uint8_t { None, Relu, Tanh, Sigmoid };

inline float activate(float v, Act a) {
    switch (a) {
        case Act::Relu: return v > 0 ? v : 0;
        case Act::Tanh: return std::tanh(v);
        case Act::Sigmoid: return 1.0f / (1.0f + std::exp(-v));
        default: return v;
    }
}

// Повнозв'язний шар з дійсними вагами: y = act(W x + b)
struct DenseF {
    int in = 0, out = 0;
    const float* w = nullptr;   // [out][in]
    const float* b = nullptr;   // [out] (може бути nullptr)
    Act act = Act::None;
};

inline void run(const DenseF& L, const float* x, float* y, OpCounter* ops = nullptr) {
    for (int o = 0; o < L.out; ++o) {
        const float* row = L.w + size_t(o) * size_t(L.in);
        float acc = L.b ? L.b[o] : 0.0f;
        for (int i = 0; i < L.in; ++i) acc += row[i] * x[i];
        y[o] = activate(acc, L.act);
    }
    if (ops) ops->add(uint64_t(L.in) * uint64_t(L.out));
}

// Повнозв'язний шар з 8-бітними вагами (симетричне квантування по рядках): W[o][i] ≈ wq[o][i]·w_scale[o].
// Вхід квантується динамічно (одна шкала на вектор), накопичення — у int32, як в ESP-DL / CMSIS-NN.
struct DenseQ {
    int in = 0, out = 0;
    const int8_t* w = nullptr;      // [out][in]
    const float* w_scale = nullptr; // [out]
    const float* b = nullptr;       // [out] (може бути nullptr)
    Act act = Act::None;
};

inline void run(const DenseQ& L, const float* x, float* y, OpCounter* ops = nullptr) {
    float amax = 0;
    for (int i = 0; i < L.in; ++i) amax = std::max(amax, std::fabs(x[i]));
    const float xs = amax > 0 ? amax / 127.0f : 1.0f;
    std::array<int8_t, kMaxWidth> xq{};
    for (int i = 0; i < L.in; ++i) xq[size_t(i)] = int8_t(std::lround(std::clamp(x[i] / xs, -127.0f, 127.0f)));
    for (int o = 0; o < L.out; ++o) {
        const int8_t* row = L.w + size_t(o) * size_t(L.in);
        int32_t acc = 0;
        for (int i = 0; i < L.in; ++i) acc += int32_t(row[i]) * int32_t(xq[size_t(i)]);
        y[o] = activate(float(acc) * xs * L.w_scale[o] + (L.b ? L.b[o] : 0.0f), L.act);
    }
    if (ops) ops->add(uint64_t(L.in) * uint64_t(L.out));
}

// Квантування рядків дійсної матриці до int8 (для перевірок і для експорту без Python)
inline void quantizeRows(const float* w, int out, int in, std::vector<int8_t>& wq, std::vector<float>& scale) {
    wq.assign(size_t(out) * size_t(in), 0);
    scale.assign(size_t(out), 1.0f);
    for (int o = 0; o < out; ++o) {
        float amax = 0;
        for (int i = 0; i < in; ++i) amax = std::max(amax, std::fabs(w[size_t(o) * size_t(in) + size_t(i)]));
        const float s = amax > 0 ? amax / 127.0f : 1.0f;
        scale[size_t(o)] = s;
        for (int i = 0; i < in; ++i) wq[size_t(o) * size_t(in) + size_t(i)] = int8_t(std::lround(w[size_t(o) * size_t(in) + size_t(i)] / s));
    }
}

// GRU-комірка (як torch.nn.GRU, один шар): r = σ(Wir x + bir + Whr h + bhr), z = σ(Wiz x + biz + Whz h + bhz),
// n = tanh(Win x + bin + r ⊙ (Whn h + bhn)), h' = (1 − z) ⊙ n + z ⊙ h
struct GruF {
    int in = 0, hidden = 0;
    const float* w_ih = nullptr;   // [3H][in]
    const float* w_hh = nullptr;   // [3H][H]
    const float* b_ih = nullptr;   // [3H]
    const float* b_hh = nullptr;   // [3H]
};

inline void step(const GruF& G, const float* x, float* h, OpCounter* ops = nullptr) {
    const int H = G.hidden;
    std::array<float, 3 * kMaxWidth> gi{}, gh{};
    for (int k = 0; k < 3 * H; ++k) {
        float a = G.b_ih ? G.b_ih[k] : 0.0f, c = G.b_hh ? G.b_hh[k] : 0.0f;
        const float* ri = G.w_ih + size_t(k) * size_t(G.in);
        const float* rh = G.w_hh + size_t(k) * size_t(H);
        for (int i = 0; i < G.in; ++i) a += ri[i] * x[i];
        for (int j = 0; j < H; ++j) c += rh[j] * h[j];
        gi[size_t(k)] = a; gh[size_t(k)] = c;
    }
    for (int j = 0; j < H; ++j) {
        const float r = activate(gi[size_t(j)] + gh[size_t(j)], Act::Sigmoid);
        const float z = activate(gi[size_t(H + j)] + gh[size_t(H + j)], Act::Sigmoid);
        const float n = std::tanh(gi[size_t(2 * H + j)] + r * gh[size_t(2 * H + j)]);
        h[j] = (1 - z) * n + z * h[j];
    }
    if (ops) ops->add(uint64_t(3 * H) * uint64_t(G.in + H));
}

// Нормалізація входу так само, як при навчанні: x' = (x − mean) / std
struct Norm {
    int n = 0;
    const float* mean = nullptr;
    const float* inv_std = nullptr;
    void apply(const float* x, float* y) const { for (int i = 0; i < n; ++i) y[i] = (x[i] - mean[i]) * inv_std[i]; }
};

// Послідовна мережа (MLP) з шарів одного типу (DenseF або DenseQ), до 8 шарів
template <class Layer>
struct Mlp {
    Norm norm;
    std::array<Layer, 8> layers{};
    int count = 0;
    int inputs() const { return count ? layers[0].in : 0; }
    int outputs() const { return count ? layers[size_t(count - 1)].out : 0; }
    void forward(const float* x, float* y, OpCounter* ops = nullptr) const {
        std::array<float, kMaxWidth> a{}, b{};
        if (norm.n) norm.apply(x, a.data()); else std::copy(x, x + inputs(), a.begin());
        for (int k = 0; k < count; ++k) {
            run(layers[size_t(k)], a.data(), b.data(), ops);
            std::swap(a, b);
        }
        std::copy(a.begin(), a.begin() + outputs(), y);
    }
    // Те саме, що forward, але зберігає вхід після нормалізації і виходи кожного шару (для візуалізації)
    void forwardTrace(const float* x, float* y, float* norm_in, float* const* layer_out) const {
        std::array<float, kMaxWidth> a{}, b{};
        if (norm.n) norm.apply(x, a.data()); else std::copy(x, x + inputs(), a.begin());
        std::copy(a.begin(), a.begin() + inputs(), norm_in);
        for (int k = 0; k < count; ++k) {
            run(layers[size_t(k)], a.data(), b.data(), nullptr);
            std::swap(a, b);
            if (layer_out[k]) std::copy(a.begin(), a.begin() + layers[size_t(k)].out, layer_out[k]);
        }
        std::copy(a.begin(), a.begin() + outputs(), y);
    }
    size_t params() const {
        size_t p = 0;
        for (int k = 0; k < count; ++k) p += size_t(layers[size_t(k)].in + 1) * size_t(layers[size_t(k)].out);
        return p;
    }
};

}  // namespace ai
