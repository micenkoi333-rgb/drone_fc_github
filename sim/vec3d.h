// vec3d.h — double-вектори, матриці 3×3 й кватерніон для моделі QuadPlane (double потрібен для тримування й лінеаризації).
// Система координат тіла FLU: X вперед, Y вліво, Z вгору. Позитивні обертання — за правилом правої руки навколо осей FLU
// (тобто +Y — ніс ВНИЗ, +X — правий крен, +Z — ніс ліворуч).
#pragma once
#include <cmath>

namespace sim {

struct Vec3d {
    double x = 0, y = 0, z = 0;
    constexpr Vec3d() = default;
    constexpr Vec3d(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3d operator+(const Vec3d& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3d operator-(const Vec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3d operator-() const { return {-x, -y, -z}; }
    Vec3d operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3d operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3d& operator+=(const Vec3d& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3d& operator-=(const Vec3d& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    double dot(const Vec3d& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3d cross(const Vec3d& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3d normalized() const { const double n = norm(); return n > 1e-12 ? *this / n : Vec3d{}; }
    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};
inline Vec3d operator*(double s, const Vec3d& v) { return v * s; }

struct Mat3d {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Vec3d operator*(const Vec3d& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    double det() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
    Mat3d inverse() const {   // для тензора інерції; якщо матриця виродилась — одинична
        const double d = det();
        Mat3d r;
        if (std::fabs(d) < 1e-18) return r;
        r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / d; r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / d;
        r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / d; r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / d;
        r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / d; r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / d;
        r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / d; r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / d;
        r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / d;
        return r;
    }
    static Mat3d diag(double a, double b, double c) { Mat3d r; r.m[0][0] = a; r.m[1][1] = b; r.m[2][2] = c; return r; }
};

struct Quatd {   // тіло -> світ
    double w = 1, x = 0, y = 0, z = 0;
    Quatd() = default;
    Quatd(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    static Quatd fromEuler(double roll, double pitch, double yaw) {   // ZYX; pitch + = ніс ВНИЗ (обертання навколо +Y FLU)
        const double cr = std::cos(roll / 2), sr = std::sin(roll / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
        const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
        return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
    }
    Quatd operator*(const Quatd& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }
    Quatd conj() const { return {w, -x, -y, -z}; }
    void normalize() {
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12) { *this = Quatd(); return; }
        w /= n; x /= n; y /= n; z /= n;
    }
    Vec3d rotate(const Vec3d& v) const {   // тіло -> світ
        const Vec3d u{x, y, z};
        const Vec3d t = u.cross(v) * 2.0;
        return v + t * w + u.cross(t);
    }
    Vec3d rotateInv(const Vec3d& v) const { return conj().rotate(v); }   // світ -> тіло
    void integrate(const Vec3d& omega_body, double dt) {   // точне інтегрування сталої кутової швидкості за крок
        const Vec3d a = omega_body * dt;
        const double ang = a.norm();
        Quatd dq;
        if (ang > 1e-12) {
            const double s = std::sin(ang / 2) / ang;
            dq = {std::cos(ang / 2), a.x * s, a.y * s, a.z * s};
        }
        *this = *this * dq;
        normalize();
    }
    Vec3d euler() const {   // крен, тангаж (+ ніс вниз), курс
        Vec3d e;
        e.x = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        e.y = std::asin(std::fmax(-1.0, std::fmin(1.0, 2 * (w * y - z * x))));
        e.z = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        return e;
    }
};

}  // namespace sim
