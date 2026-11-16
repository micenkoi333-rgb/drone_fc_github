// ai/link.h — зв'язок «польотний контролер ↔ ШІ-співпроцесор» (UART 921600 бод або CAN) і сторож на боці контролера.
// Кадр: 0xA5 0x5A | тип | довжина (1 байт) | дані | CRC16-CCITT (від типу до кінця даних, little-endian).
// Ті самі структури й той самий розбір кадрів працюють у симуляторі й на залізі; порядок байтів — little-endian (STM32, ESP32, x86).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace ai {

enum class FrameType : uint8_t {
    Telemetry = 1,   // ПК → ШІ, 100 Гц: стан, команди, мотори, гондоли, батарея
    ImuBatch = 2,    // ПК → ШІ, 100 Гц: 10 сирих вибірок IMU (1 кГц) — для аналізу вібрацій
    Advice = 3,      // ШІ → ПК, 50 Гц: поправки сил/моментів і стан ШІ
    Message = 4,     // ШІ → ПК → наземна станція: діагностичне повідомлення (код, вузол, значення)
};

#pragma pack(push, 1)
struct TelemetryFrame {
    uint32_t t_ms = 0;
    uint8_t phase = 0;               // фаза польоту (fc::QpPhase)
    uint8_t flags = 0;               // біт 0 — у повітрі, біт 1 — GPS є
    float euler[3] = {0, 0, 0};      // крен, тангаж (+ ніс униз), курс, рад
    float gyro[3] = {0, 0, 0};       // рад/с, тіло
    float vel[3] = {0, 0, 0};        // м/с, світ ENU
    float airspeed = 0, alt = 0;     // м/с, м над домом
    float v_bus = 0, i_bus = 0;      // В, А
    float rpm[4] = {0, 0, 0, 0};     // об/хв з телеметрії ESC
    float motor_i[4] = {0, 0, 0, 0}; // А з телеметрії ESC
    float thrust_cmd[4] = {0, 0, 0, 0};   // Н — що контролер хоче від кожного гвинта
    float tilt_cmd[4] = {0, 0, 0, 0};     // рад — команда приводу нахилу
    float tilt_meas[4] = {0, 0, 0, 0};    // рад — енкодер гондоли
};
struct ImuBatchFrame {
    uint32_t t_ms = 0;               // час першої вибірки
    float acc_scale = 0.01f;         // м/с² на одиницю int16
    int16_t acc[10][3] = {};         // 10 вибірок × 3 осі, 1 кГц
};
struct AdviceFrame {
    uint32_t seq = 0;
    uint8_t status = 0;              // 0 — вимкнено, 1 — тінь (лише рахує), 2 — радить
    float force[3] = {0, 0, 0};      // Н, тіло: передбачена «зайва» сила, яку контролер має компенсувати
    float moment[3] = {0, 0, 0};     // Н·м, тіло
    float confidence = 0;            // 0…1
};
struct MessageFrame {
    uint32_t t_ms = 0;
    uint8_t severity = 0;            // 0 — до відома, 1 — перевірити після посадки, 2 — сісти найближчим часом
    uint8_t node = 0;                // вузол: 1…4 — мотор/гвинт/гондола №, 10 — батарея, 20 — рама
    uint16_t code = 0;               // що саме (ai::Code у coprocessor.h)
    float value = 0;                 // величина (у одиницях, що їх знає текст коду)
};
#pragma pack(pop)

inline uint16_t crc16(const uint8_t* p, size_t n, uint16_t c = 0xFFFF) {   // CRC16-CCITT (0x1021)
    for (size_t i = 0; i < n; ++i) {
        c ^= uint16_t(p[i]) << 8;
        for (int b = 0; b < 8; ++b) c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
    }
    return c;
}

template <class T>
inline void encode(FrameType type, const T& payload, std::vector<uint8_t>& out) {
    static_assert(sizeof(T) <= 255, "кадр має вміщатися в 255 байт");
    const size_t start = out.size();
    out.push_back(0xA5); out.push_back(0x5A); out.push_back(uint8_t(type)); out.push_back(uint8_t(sizeof(T)));
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&payload);
    out.insert(out.end(), p, p + sizeof(T));
    const uint16_t c = crc16(out.data() + start + 2, sizeof(T) + 2);
    out.push_back(uint8_t(c & 0xFF)); out.push_back(uint8_t(c >> 8));
}

// Розбір потоку байтів (з будь-якого шматкування): повертає кадри цілими, биті кадри відкидає й шукає наступний заголовок
class FrameParser {
public:
    struct Frame { FrameType type; std::array<uint8_t, 255> data; uint8_t len; };
    uint32_t good = 0, bad_crc = 0, dropped_bytes = 0;

    // Подати байти; для кожного цілого кадру викликається cb(const Frame&)
    template <class Cb>
    void feed(const uint8_t* p, size_t n, Cb&& cb) {
        for (size_t i = 0; i < n; ++i) push(p[i], cb);
    }
    template <class T>
    static bool as(const Frame& f, T& out) {
        if (f.len != sizeof(T)) return false;
        std::memcpy(&out, f.data.data(), sizeof(T));
        return true;
    }

private:
    enum State { Sync1, Sync2, Type, Len, Data, Crc1, Crc2 } st_ = Sync1;
    Frame cur_{};
    size_t got_ = 0;
    uint16_t crc_rx_ = 0;
    template <class Cb>
    void push(uint8_t b, Cb& cb) {
        switch (st_) {
            case Sync1: if (b == 0xA5) st_ = Sync2; else ++dropped_bytes; break;
            case Sync2: if (b == 0x5A) st_ = Type; else { ++dropped_bytes; st_ = b == 0xA5 ? Sync2 : Sync1; } break;
            case Type: cur_.type = FrameType(b); st_ = Len; break;
            case Len: cur_.len = b; got_ = 0; st_ = b ? Data : Crc1; break;
            case Data: cur_.data[got_++] = b; if (got_ == cur_.len) st_ = Crc1; break;
            case Crc1: crc_rx_ = b; st_ = Crc2; break;
            case Crc2: {
                crc_rx_ |= uint16_t(b) << 8;
                std::array<uint8_t, 257> buf{};
                buf[0] = uint8_t(cur_.type); buf[1] = cur_.len;
                std::memcpy(buf.data() + 2, cur_.data.data(), cur_.len);
                if (crc16(buf.data(), size_t(cur_.len) + 2) == crc_rx_) { ++good; cb(cur_); } else ++bad_crc;
                st_ = Sync1;
                break;
            }
        }
    }
};

// Сторож на боці польотного контролера: ШІ лише радить, а що з тієї поради дійде до керування — вирішує контролер.
// Поради немає довше за timeout → нуль; поправка обмежена за величиною й швидкістю зміни; режим «тінь» — рахує, не впливає.
struct GuardConfig {
    float timeout_s = 0.05f;          // 50 мс без поради — вимикаємо
    float force_max_n = 4.5f;         // ≈ 20% ваги апарата 2.3 кг (задати з маси: 0.2·m·g)
    float moment_max_nm = 0.4f;       // ≈ 20% запасу моменту
    float force_rate = 20.0f;         // Н/с — швидкість зміни поправки сили
    float moment_rate = 2.0f;         // Н·м/с
    float min_confidence = 0.3f;      // нижче — порада ігнорується
};

class AiGuard {
public:
    enum Mode : uint8_t { Off = 0, Shadow = 1, On = 2 };
    GuardConfig cfg;
    Mode mode = Off;

    void onAdvice(const AdviceFrame& a, float now_s) { last_ = a; last_t_ = now_s; have_ = true; }
    // Аварійне вимкнення до посадки (наприклад, запобіжник побачив, що з порадами політ гірший)
    void latchOff() { latched_ = true; }
    bool latched() const { return latched_; }
    bool fresh(float now_s) const { return have_ && now_s - last_t_ <= cfg.timeout_s; }

    // Що реально йде в контролер на цьому кроці (тіло, Н і Н·м)
    void output(float now_s, float dt, float force[3], float moment[3]) {
        float tf[3] = {0, 0, 0}, tm[3] = {0, 0, 0};
        const bool use = mode == On && !latched_ && fresh(now_s) && last_.status == 2 && last_.confidence >= cfg.min_confidence;
        if (use) {
            for (int k = 0; k < 3; ++k) { tf[k] = std::isfinite(last_.force[k]) ? last_.force[k] : 0; tm[k] = std::isfinite(last_.moment[k]) ? last_.moment[k] : 0; }
            clampVec(tf, cfg.force_max_n); clampVec(tm, cfg.moment_max_nm);
        }
        for (int k = 0; k < 3; ++k) {   // і вмикання, і вимикання — плавно, без стрибка сили
            f_[k] += std::clamp(tf[k] - f_[k], -cfg.force_rate * dt, cfg.force_rate * dt);
            m_[k] += std::clamp(tm[k] - m_[k], -cfg.moment_rate * dt, cfg.moment_rate * dt);
            force[k] = f_[k]; moment[k] = m_[k];
        }
    }

private:
    AdviceFrame last_{};
    float last_t_ = -1e9f, f_[3] = {0, 0, 0}, m_[3] = {0, 0, 0};
    bool have_ = false, latched_ = false;
    static void clampVec(float v[3], float lim) {
        const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n > lim && n > 0) for (int k = 0; k < 3; ++k) v[k] *= lim / n;
    }
};

}  // namespace ai
