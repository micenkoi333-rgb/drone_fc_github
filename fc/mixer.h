// mixer.h — розподіл тяги і моментів між 4 моторами (квадрокоптер X)
// Пріоритет при насиченні: roll/pitch > загальна тяга > yaw (щоб дрон не перевернувся і не втратив висоту).
#pragma once
#include "fc_math.h"

namespace fc {

struct MixerConfig {
    float arm_length = 0.35f;        // м, від центру до мотора (рама ~700 мм, пропелери 15")
    float k_yaw = 0.025f;            // м, реактивний момент = k_yaw * тяга
    float max_thrust = 28.0f;        // Н на мотор при cmd=1 (~2.9 кг тяги; разом 11.4 кг -> T/W 3.8 порожнім, 2.5 з 1.5 кг вантажу)
    float idle = 0.05f;              // мінімум, щоб мотори не зупинялись у польоті
    // Мотори не обов'язково стоять квадратом: VTOL-літак має їх на двох балках (прямокутник).
    // arm_x — від центру мас до мотора вздовж корпусу, arm_y — убік. 0 — квадрат з arm_length.
    float arm_x = 0, arm_y = 0;
    float arm_y_rear = 0;            // конвертоплан: задні мотори ближче до осі (кінці V-хвоста); 0 — як передні
    float arm_x_rear = 0;            // задні мотори від ЦМ уздовж корпусу, якщо ЦМ не посередині; 0 — як передні
    float max_thrust_rear = 0;       // Н: задні мотори слабші/інші (0 — як передні)
    float maxOf(int i) const { return (max_thrust_rear > 0 && kQuadX[i].sx < 0) ? max_thrust_rear : max_thrust; }
    float totalMax() const { return 2 * max_thrust + 2 * (max_thrust_rear > 0 ? max_thrust_rear : max_thrust); }
    float dx() const { return arm_x > 0 ? arm_x : arm_length * 0.70710678f; }
    float dy() const { return arm_y > 0 ? arm_y : arm_length * 0.70710678f; }
    float dyRear() const { return arm_y_rear > 0 ? arm_y_rear : dy(); }
    float dxRear() const { return arm_x_rear > 0 ? arm_x_rear : dx(); }
};

struct MixerResult {
    float cmd[4] = {0, 0, 0, 0};
    float thrust[4] = {0, 0, 0, 0};
    bool sat_attitude = false;  // не вистачило діапазону для roll/pitch/yaw
    bool sat_thrust = false;    // довелось змінити загальну тягу
};

inline MixerResult mixQuadX(float total_thrust, const Vec3& torque, const MixerConfig& c) {
    MixerResult r;
    const float dxf = c.dx(), dxr = c.dxRear(), dyf = c.dy(), dyr = c.dyRear();
    // ЦМ не посередині між гвинтами: у рівновазі передні несуть частку dxr/(dxf+dxr) ваги, задні — dxf/(dxf+dxr)
    float share[4];
    for (int i = 0; i < 4; ++i) share[i] = 2.0f * (kQuadX[i].sx > 0 ? dxr : dxf) / (dxf + dxr);
    const float sy2 = 2 * (dyf * dyf + dyr * dyr);   // крен — пропорційно плечу кожного мотора (мінімальна норма; без побічного тангажу й тяги)
    float tmn[4], tmx[4], avail = 1e9f;
    for (int i = 0; i < 4; ++i) { tmx[i] = c.maxOf(i); tmn[i] = c.idle * tmx[i]; avail = std::fmin(avail, tmx[i] - tmn[i]); }
    float rp[4], yw[4];
    for (int i = 0; i < 4; ++i) {
        const float dyi = kQuadX[i].sx > 0 ? dyf : dyr;
        rp[i] = torque.x * kQuadX[i].sy * dyi / sy2 - torque.y * kQuadX[i].sx / (2 * (dxf + dxr));
        // рискання різницею тяг діагональних пар; при різних плечах (dyf ≠ dyr) пари мають різну частку, щоб не з'являвся крен:
        // передня пара ±a, задня ±b, a·dyf = b·dyr, 2k(a + b) = τz
        yw[i] = torque.z * kQuadX[i].yaw_sign / (2 * c.k_yaw) * (kQuadX[i].sx > 0 ? dyr : dyf) / (dyf + dyr);
    }
    float lo = 1e9f, hi = -1e9f;
    for (float v : rp) { lo = v < lo ? v : lo; hi = v > hi ? v : hi; }
    // 1) roll/pitch: якщо навіть вони не влазять — масштабуємо їх і повністю віддаємо діапазон їм
    float rp_k = 1.0f, yaw_k = 1.0f;
    if (hi - lo > avail) {
        rp_k = avail / (hi - lo);
        lo *= rp_k; hi *= rp_k;
        yaw_k = 0;
        r.sat_attitude = true;
    }
    // 2) загальна тяга: зсуваємо, щоб roll/pitch влізли
    float base = total_thrust / 4;   // середня тяга мотора; мотор i — base·share[i]
    float bmin = -1e9f, bmax = 1e9f;
    for (int i = 0; i < 4; ++i) { bmin = std::fmax(bmin, (tmn[i] - rp_k * rp[i]) / share[i]); bmax = std::fmin(bmax, (tmx[i] - rp_k * rp[i]) / share[i]); }
    float base_c = bmin <= bmax ? clampf(base, bmin, bmax) : 0.5f * (bmin + bmax);
    if (std::fabs(base_c - base) > 1e-4f) r.sat_thrust = true;
    // 3) yaw — найнижчий пріоритет: беремо стільки, скільки влазить, не чіпаючи тягу
    for (int i = 0; i < 4 && yaw_k > 0; ++i) {
        float v = base_c * share[i] + rp_k * rp[i];
        if (yw[i] > 1e-9f) yaw_k = std::fmin(yaw_k, (tmx[i] - v) / yw[i]);
        else if (yw[i] < -1e-9f) yaw_k = std::fmin(yaw_k, (tmn[i] - v) / yw[i]);
    }
    if (yaw_k < 1.0f - 1e-4f) r.sat_attitude = true;
    yaw_k = clampf(yaw_k, 0, 1);
    for (int i = 0; i < 4; ++i) {
        r.thrust[i] = clampf(base_c * share[i] + rp_k * rp[i] + yaw_k * yw[i], tmn[i], tmx[i]);
        r.cmd[i] = r.thrust[i] / tmx[i];
    }
    return r;
}

}  // namespace fc
