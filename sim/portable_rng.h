#pragma once
// Однакові випадкові числа на Windows (MSVC) і Linux (g++): std::mt19937 у стандарті визначений точно, а prng::normal
// і prng::uniform — ні, і кожна бібліотека рахує по-своєму. Через це ті самі тести з тим самим seed летіли
// в різному вітрі й шумі. Тут — свої розподіли поверх генератора: Бокс–Мюллер і рівномірний з 32 біт.
#include <cmath>
#include <cstdint>
#include "portable_rng.h"

namespace prng {

template <class E>
inline double uni01(E& e) {   // (0, 1), без нуля — для логарифма
    const double r = double(E::max() - E::min()) + 1.0;
    return (double(e() - E::min()) + 0.5) / r;
}

template <class T = double>
struct normal {
    T m = 0, s = 1;
    normal() = default;
    normal(T mean, T sd) : m(mean), s(sd) {}
    template <class E>
    T operator()(E& e) const {
        const double u1 = uni01(e), u2 = uni01(e);
        return T(double(m) + double(s) * std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2));
    }
};

template <class T = double>
struct uniform {
    T a = 0, b = 1;
    uniform() = default;
    uniform(T lo, T hi) : a(lo), b(hi) {}
    template <class E>
    T operator()(E& e) const { return T(double(a) + (double(b) - double(a)) * uni01(e)); }
};

}  // namespace prng
