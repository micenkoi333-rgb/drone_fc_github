// hal.h — шар абстракції заліза (Hardware Abstraction Layer).
// Польотний контролер бачить тільки IImu / IBaro / IMotorOutput.
// Під ними — драйвери для ЦИФРОВИХ (I2C/SPI, сирі int16) та АНАЛОГОВИХ (АЦП) сенсорів.
// Зараз ці шини реалізує симулятор; на реальному залізі ви реалізуєте IRawImuBus / IAdc
// під свій мікроконтролер (STM32, ESP32, Arduino...) — решта коду не змінюється.
#pragma once
#include <cstdint>
#include "fc_math.h"

namespace fc {

// ---------- Інтерфейси, які використовує контролер ----------
struct ImuSample {
    Vec3 gyro;   // рад/с, система тіла
    Vec3 accel;  // м/с², «specific force» (у спокої = +9.81 по Z)
};

class IImu {
public:
    virtual ~IImu() = default;
    virtual bool read(ImuSample& out) = 0;  // false = немає даних / помилка шини
};

class IBaro {
public:
    virtual ~IBaro() = default;
    virtual bool read(float& altitude_m) = 0;  // true лише коли є НОВИЙ відлік
};

class IMotorOutput {
public:
    virtual ~IMotorOutput() = default;
    virtual void write(const float cmd[4]) = 0;  // 0..1 (далі -> PWM/DShot)
    virtual void writeAux(float pusher) { (void)pusher; }  // 0..1 — тяговий (штовхальний) гвинт, якщо є
    // Рулі (літаковий режим VTOL): −1..1 «за моментом» — див. SurfaceConfig у airframe.h
    virtual void writeSurfaces(float aileron, float elevator, float rudder) { (void)aileron; (void)elevator; (void)rudder; }
};

// Датчик швидкості повітря (трубка Піто + диференційний датчик тиску, напр. MS4525DO): м/с уздовж носа
class IAirspeed {
public:
    virtual ~IAirspeed() = default;
    virtual bool read(float& ias) = 0;  // true — новий відлік
};

// GPS-приймач (u-blox NEO-M8/M10 тощо): широта/довгота + швидкість
struct GpsFix {
    double lat_deg = 0, lon_deg = 0;  // градуси
    float alt_m = 0;
    Vec3 vel;              // м/с у локальній системі: X — схід, Y — північ, Z — вгору
    float h_acc = 99;      // м, оцінка точності від приймача
    int sats = 0;
    bool fix3d = false;
};
class IGps {
public:
    virtual ~IGps() = default;
    virtual bool read(GpsFix& out) = 0;  // true лише коли є НОВИЙ відлік (зазвичай 5–10 Гц)
};

// Магнітометр (компас): поле в системі тіла, будь-які одиниці (важливий лише напрям)
class IMag {
public:
    virtual ~IMag() = default;
    virtual bool read(Vec3& field_body) = 0;
};

// Оптичний потік (камера дивиться вниз: PMW3901 / MTF-01 / візуальна одометрія на бортовому комп'ютері).
// rate — видимий рух поверхні, рад/с, у системі тіла; містить і поступальний рух, і власне обертання дрона:
//   rate.x = v_x / d - ω_y,   rate.y = v_y / d + ω_x     (d — відстань до землі вздовж осі камери)
// Для свого сенсора підберіть знаки в драйвері так, щоб виконувалась ця формула.
struct FlowSample {
    float rate_x = 0, rate_y = 0;  // рад/с
    float quality = 0;             // 0..1 (текстура поверхні, освітлення)
};
class IFlow {
public:
    virtual ~IFlow() = default;
    virtual bool read(FlowSample& out) = 0;  // true лише коли є новий відлік
};

// Лазерний далекомір вниз (TFmini / TF02 / LightWare): відстань уздовж осі Z тіла
class IRangefinder {
public:
    virtual ~IRangefinder() = default;
    virtual bool read(float& dist_m) = 0;  // false — немає нового відліку або поза діапазоном
};

// Перетворення широта/довгота -> метри відносно «дому» (точки старту)
struct GeoRef {
    double lat0 = 0, lon0 = 0;
    bool set = false;
    static constexpr double kR = 6378137.0;
    void setHome(double lat, double lon) { lat0 = lat; lon0 = lon; set = true; }
    Vec3 toLocal(double lat, double lon) const {
        const double d2r = 3.14159265358979 / 180.0;
        return {float((lon - lon0) * d2r * kR * std::cos(lat0 * d2r)), float((lat - lat0) * d2r * kR), 0};
    }
    void toGeo(const Vec3& p, double& lat, double& lon) const {
        const double d2r = 3.14159265358979 / 180.0;
        lat = lat0 + p.y / kR / d2r;
        lon = lon0 + p.x / (kR * std::cos(lat0 * d2r)) / d2r;
    }
};

// Модуль живлення: напруга й струм батареї (АЦП з дільником + шунт/датчик Холла)
class IBattery {
public:
    virtual ~IBattery() = default;
    virtual bool read(float& volts, float& amps) = 0;  // true — новий відлік
};

// ---------- Низькорівневі шини ----------
// Цифровий IMU (MPU6050/ICM42688/BMI088...): сирі 16-бітні регістри
class IRawImuBus {
public:
    virtual ~IRawImuBus() = default;
    virtual bool readRaw(int16_t gyro[3], int16_t accel[3]) = 0;
};

// Аналогові сенсори через АЦП
class IAdc {
public:
    virtual ~IAdc() = default;
    virtual bool sample(uint16_t* counts, int n_channels) = 0;
};

// ---------- Драйвер цифрового IMU ----------
class DigitalImu : public IImu {
public:
    struct Config {
        float gyro_lsb_per_dps = 16.4f;   // MPU6050 @ ±2000 °/с
        float accel_lsb_per_g = 4096.0f;  // MPU6050 @ ±8 g
        int axis_map[3] = {0, 1, 2};      // як осі чипа лягають на осі дрона
        int axis_sign[3] = {1, 1, 1};
    };
    DigitalImu(IRawImuBus& bus, const Config& c) : bus_(bus), c_(c) {}
    DigitalImu(IRawImuBus& bus) : bus_(bus) {}

    bool read(ImuSample& out) override {
        int16_t g[3], a[3];
        if (!bus_.readRaw(g, a)) return false;
        for (int i = 0; i < 3; ++i) {
            int s = c_.axis_map[i];
            out.gyro[i] = c_.axis_sign[i] * (g[s] / c_.gyro_lsb_per_dps) * kDeg2Rad;
            out.accel[i] = c_.axis_sign[i] * (a[s] / c_.accel_lsb_per_g) * kGravity;
        }
        return true;
    }
private:
    IRawImuBus& bus_;
    Config c_;
};

// ---------- Драйвер аналогового IMU (наприклад, ADXL335 + аналоговий гіроскоп) ----------
class AnalogImu : public IImu {
public:
    struct Config {
        int adc_bits = 12;
        float vref = 3.3f;
        float gyro_zero_v = 1.65f, gyro_v_per_dps = 0.0025f;  // 2.5 мВ/(°/с)
        float accel_zero_v = 1.65f, accel_v_per_g = 0.30f;    // 300 мВ/g
        int ch_gyro[3] = {0, 1, 2};
        int ch_accel[3] = {3, 4, 5};
    };
    AnalogImu(IAdc& adc, const Config& c) : adc_(adc), c_(c) {}
    AnalogImu(IAdc& adc) : adc_(adc) {}

    bool read(ImuSample& out) override {
        uint16_t cnt[6];
        if (!adc_.sample(cnt, 6)) return false;
        const float lsb = c_.vref / float((1 << c_.adc_bits) - 1);
        for (int i = 0; i < 3; ++i) {
            float vg = cnt[c_.ch_gyro[i]] * lsb;
            float va = cnt[c_.ch_accel[i]] * lsb;
            out.gyro[i] = (vg - c_.gyro_zero_v) / c_.gyro_v_per_dps * kDeg2Rad;
            out.accel[i] = (va - c_.accel_zero_v) / c_.accel_v_per_g * kGravity;
        }
        return true;
    }
    const Config& config() const { return c_; }
private:
    IAdc& adc_;
    Config c_;
};

}  // namespace fc
