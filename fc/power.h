// power.h — бортова електроніка: хто скільки їсть і що можна вимикати.
// Контролер сам керує живленням периферії (через IPowerSwitch): вимикає те, що зараз не потрібне.
#pragma once
#include "fc_math.h"

namespace fc {

enum PowerDev {
    PD_FC,        // польотний контролер + IMU + барометр + компас
    PD_GPS,
    PD_FLOW,      // камера оптичного потоку
    PD_RANGE,     // лазерний далекомір
    PD_RC,        // приймач пульта
    PD_TELEM,     // радіо телеметрії
    PD_VIDEO,     // цифровий відеопередавач
    PD_COMPUTER,  // бортовий комп'ютер (Jetson Orin Nano / аналог)
    PD_CAM_WIDE,  // ширококутна камера
    PD_CAM_PTZ,   // камера з зумом на підвісі (камера + мотори підвісу)
    PD_COUNT
};

// Режим корисного навантаження (комп'ютер + камери) — його обирає оператор
enum class PayloadMode { Full, Eco, Sleep };

struct PowerState {
    bool on[PD_COUNT];
    float video_tx = 0.3f;              // 0..1 — потужність відеопередавача (за відстанню)
    PayloadMode payload = PayloadMode::Full;
    PowerState() { for (bool& b : on) b = true; }
};

// Типове споживання (Вт) — з паспортів компонентів
struct AvionicsSpec {
    float fc = 1.0f, gps = 0.4f, flow = 0.35f, range = 0.6f, rc = 0.3f, telem = 0.5f;
    float video_base = 2.0f, video_tx_max = 5.0f;          // цифровий відеолінк: 25 мВт .. 1 Вт ефірної потужності
    float computer_full = 15.0f, computer_eco = 7.0f, computer_sleep = 2.5f;  // Orin Nano: 15 Вт / 7 Вт / простій
    float cam_wide = 1.5f, cam_ptz = 4.0f;                  // PTZ: камера 2 Вт + підвіс 2 Вт
    float bec_eff = 0.88f;                                  // перетворювач 24 В -> 5/12 В
};

// Скільки забирає електроніка з батареї
inline float avionicsWatts(const PowerState& s, const AvionicsSpec& a = AvionicsSpec()) {
    float w = 0;
    if (s.on[PD_FC]) w += a.fc;
    if (s.on[PD_GPS]) w += a.gps;
    if (s.on[PD_FLOW]) w += a.flow;
    if (s.on[PD_RANGE]) w += a.range;
    if (s.on[PD_RC]) w += a.rc;
    if (s.on[PD_TELEM]) w += a.telem;
    if (s.on[PD_VIDEO]) w += a.video_base + a.video_tx_max * clampf(s.video_tx, 0, 1);
    if (s.on[PD_COMPUTER])
        w += s.payload == PayloadMode::Full ? a.computer_full : s.payload == PayloadMode::Eco ? a.computer_eco : a.computer_sleep;
    if (s.on[PD_CAM_WIDE]) w += a.cam_wide;
    if (s.on[PD_CAM_PTZ]) w += a.cam_ptz;
    return w / a.bec_eff;
}

// Інтерфейс керування живленням (на залізі — MOSFET-ключі / GPIO enable / команди по UART)
class IPowerSwitch {
public:
    virtual ~IPowerSwitch() = default;
    virtual void apply(const PowerState& s) = 0;
};

}  // namespace fc
