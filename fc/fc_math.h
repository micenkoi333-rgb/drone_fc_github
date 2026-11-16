// fc_math.h — мінімальна векторна/кватерніонна математика (float, без залежностей)
// Система координат: FLU — X вперед, Y вліво, Z вгору. Світ: ENU-подібний, Z вгору.
#pragma once
#include <cmath>

namespace fc {

constexpr float kPi = 3.14159265358979f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;
constexpr float kGravity = 9.80665f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vec3 {
    float x = 0, y = 0, z = 0;
    constexpr Vec3() = default;
    constexpr Vec3(float a, float b, float c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    Vec3 mul(const Vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }  // поелементно
    float norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const { float n = norm(); return n > 1e-9f ? *this / n : Vec3{}; }
    float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};
inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

// Кватерніон орієнтації: перетворює вектор з тіла (body) у світ (world).
struct Quat {
    float w = 1, x = 0, y = 0, z = 0;
    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat fromEuler(float roll, float pitch, float yaw) {  // ZYX
        float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
        return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
    }
    Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }
    Quat conj() const { return {w, -x, -y, -z}; }
    void normalize() {
        float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-9f) { *this = Quat(); return; }
        w /= n; x /= n; y /= n; z /= n;
    }
    Vec3 rotate(const Vec3& v) const {  // body -> world
        Vec3 u{x, y, z};
        Vec3 t = u.cross(v) * 2.0f;
        return v + t * w + u.cross(t);
    }
    Vec3 rotateInv(const Vec3& v) const { return conj().rotate(v); }  // world -> body

    // Інтегрування кутової швидкості тіла (рад/с) за dt
    void integrate(const Vec3& omega_body, float dt) {
        Vec3 a = omega_body * dt;
        float ang = a.norm();
        Quat dq;
        if (ang > 1e-9f) {
            float s = std::sin(ang * 0.5f) / ang;
            dq = {std::cos(ang * 0.5f), a.x * s, a.y * s, a.z * s};
        }
        *this = *this * dq;
        normalize();
    }
    void toEuler(float& roll, float& pitch, float& yaw) const {
        roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        pitch = std::asin(clampf(2 * (w * y - z * x), -1.f, 1.f));
        yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }
    Vec3 euler() const { Vec3 e; toEuler(e.x, e.y, e.z); return e; }
    // cos кута нахилу (вісь Z тіла проти вертикалі)
    float cosTilt() const { return 1 - 2 * (x * x + y * y); }
};

// Розташування моторів квадрокоптера X (спільне для контролера і симулятора)
//  idx  позиція      обертання   знак реактивного моменту по Z
//   0   передній-правий  CCW        -1
//   1   задній-лівий     CCW        -1
//   2   передній-лівий   CW         +1
//   3   задній-правий    CW         +1
struct MotorGeom { float sx, sy, yaw_sign; };
constexpr MotorGeom kQuadX[4] = {{+1, -1, -1}, {-1, +1, -1}, {+1, +1, +1}, {-1, -1, +1}};

}  // namespace fc
