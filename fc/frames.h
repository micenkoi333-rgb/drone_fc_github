#pragma once
// Системи координат. У цьому проєкті (симулятор і автопілот):
//   світ  — ENU: X схід, Y північ, Z угору; 0 — земля в точці старту;
//   тіло  — FLU: X уперед, Y ліворуч, Z угору; кватерніон q повертає тіло → світ;
//   кути  — крен + правим крилом униз, «pitch_down» + носом униз, курс yaw від осі X (схід) проти годинникової.
// ArduPilot / PX4 / MAVLink / більшість IMU-драйверів: світ NED (північ, схід, униз), тіло FRD (уперед, праворуч, униз),
// тангаж + носом угору, курс від півночі за годинниковою. Перетворення нижче — єдине місце, де вони стикуються:
// драйвери датчиків на залізі та міст SITL (tools/sitl_bridge.cpp) мають користуватись саме ними.
#include <cmath>
#include "fc_math.h"

namespace fc {

inline Vec3 enuToNed(const Vec3& v) { return {v.y, v.x, -v.z}; }
inline Vec3 nedToEnu(const Vec3& v) { return {v.y, v.x, -v.z}; }
inline Vec3 fluToFrd(const Vec3& v) { return {v.x, -v.y, -v.z}; }
inline Vec3 frdToFlu(const Vec3& v) { return {v.x, -v.y, -v.z}; }

// q_ned: FRD → NED. q_enu = R(NED→ENU) · q_ned · R(FLU→FRD); обидві сталі матриці — повороти на 180° (дзеркала осей не потрібні).
inline Quat qNedFrdToEnuFlu(const Quat& q_ned) {
    const Quat ned2enu(0.0f, 0.70710678f, 0.70710678f, 0.0f);   // поворот на 180° навколо (1,1,0)/√2: (N,E,D) → (E,N,U)
    const Quat flu2frd(0.0f, 1.0f, 0.0f, 0.0f);                 // поворот на 180° навколо X: (F,L,U) → (F,R,D)
    return ned2enu * q_ned * flu2frd;
}
inline Quat qEnuFluToNedFrd(const Quat& q_enu) {
    const Quat enu2ned(0.0f, 0.70710678f, 0.70710678f, 0.0f);
    const Quat frd2flu(0.0f, 1.0f, 0.0f, 0.0f);
    return enu2ned * q_enu * frd2flu;
}

// кути ArduPilot (крен, тангаж носом угору, курс від півночі за годинниковою) з нашого кватерніона
struct EulerNed { float roll, pitch, yaw; };
inline EulerNed eulerNed(const Quat& q_enu) {
    const Quat q = qEnuFluToNedFrd(q_enu);
    const float r = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    const float p = std::asin(std::fmax(-1.0f, std::fmin(1.0f, 2 * (q.w * q.y - q.z * q.x))));
    const float y = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return {r, p, y};
}

}  // namespace fc
