// battery.h — батарея: крива Li-ion, оцінка заряду, пороги попереджень
#pragma once
#include "fc_math.h"

namespace fc {

// Напруга розімкненого кола Li-ion елемента (21700, NMC) залежно від заряду 0..1
inline float liionOcv(float soc) {
    static const float tab[] = {2.90f, 3.15f, 3.30f, 3.38f, 3.45f, 3.50f, 3.55f, 3.59f, 3.62f, 3.66f, 3.70f,
                                3.74f, 3.78f, 3.82f, 3.86f, 3.90f, 3.95f, 4.00f, 4.05f, 4.10f, 4.15f};
    float x = clampf(soc, 0, 1) * 20;
    int i = int(x);
    if (i >= 20) return tab[20];
    return tab[i] + (tab[i + 1] - tab[i]) * (x - i);
}
// Обернена функція: заряд за напругою спокою
inline float liionSocFromOcv(float v_cell) {
    float lo = 0, hi = 1;
    for (int it = 0; it < 30; ++it) {
        float m = 0.5f * (lo + hi);
        (liionOcv(m) < v_cell ? lo : hi) = m;
    }
    return 0.5f * (lo + hi);
}

struct BatteryConfig {
    int cells = 6;
    float capacity_ah = 18.0f;
    float v_nominal_cell = 3.6f;   // при якій напрузі задано max_thrust моторів
    float warn_soc = 0.30f;        // попередження
    float rtl_soc = 0.20f;         // повернення додому + посадка
    float land_soc = 0.10f;        // негайна посадка де є
    float land_cell_v = 3.10f;     // або якщо напруга під навантаженням падає нижче (на елемент)
    float rtl_speed = 8.0f;        // м/с — якщо вітер невідомий
    // Економна швидкість повернення («speed-to-fly»): у штиль найменше енергії на кілометр — на ~11 м/с
    // повітряної швидкості; проти вітру летимо швидше відносно повітря (+½ зустрічного вітру), за вітром — повільніше.
    // Заміряно в симуляторі: за вітру 4 м/с у спину −32% енергії проти фіксованих 8 м/с, у штиль −7%.
    bool rtl_speed_to_fly = true;
    float best_range_airspeed = 11.0f;  // м/с
    float rtl_speed_max = 16.0f;   // м/с відносно землі
    float land_speed = 1.2f;       // м/с, ближче до землі — повільніше
};

}  // namespace fc
