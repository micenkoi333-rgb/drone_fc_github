#pragma once
// Бортовий журнал контролера: тільки те, що бачить і робить сам автопілот (оцінки, команди, стан), без даних симулятора.
// Кільцевий буфер записів і подій; на залізі — той самий формат, який скидається на SD-карту (sink).
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "qp_control.h"

namespace fc {

struct QpLogRec {
    float t;
    uint8_t phase, flags;           // flags: 1 армований, 2 GPS-аварія, 4 геозона, 8 пульт, 16 Піто, 32 мотор, 64 гондола, 128 ручне
    float px, py, pz, vx, vy, vz;
    float roll, pitch, yaw;         // рад
    float p, q, r;                  // рад/с
    float airspeed, v_bus, amps;
    float duty[4], tilt[4];         // команди
    float ail, ele, rud, cruise;
    float rps[4], tilt_meas[4];     // телеметрія ESC і датчиків гондол
};

struct QpLogEvent { float t; std::string text; };

class QpLog {
public:
    bool on = true;
    float rate_hz = 25.0f;
    size_t capacity = 30000;        // записів у пам'яті (≈ 20 хв при 25 Гц)
    std::function<void(const QpLogRec&)> sink;   // потік на носій (SD), якщо задано

    void record(float dt, const QpLogRec& r) {
        if (!on) return;
        acc_ += dt;
        if (acc_ + 1e-6f < 1.0f / rate_hz) return;
        acc_ = 0;
        if (recs_.size() < capacity) recs_.push_back(r); else { recs_[head_] = r; head_ = (head_ + 1) % capacity; }
        if (sink) sink(r);
    }
    void event(float t, const std::string& s) { if (on) { events_.push_back({t, s}); if (events_.size() > 2000) events_.erase(events_.begin()); } }
    size_t size() const { return recs_.size(); }
    const QpLogRec& at(size_t i) const { return recs_[(head_ + i) % recs_.size()]; }
    const std::vector<QpLogEvent>& events() const { return events_; }
    void clear() { recs_.clear(); events_.clear(); head_ = 0; acc_ = 0; }

    bool writeCsv(const std::string& path) const {
        FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) return false;
        std::fprintf(f, "t,phase,flags,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,airspeed,v_bus,amps,d1,d2,d3,d4,tilt1,tilt2,tilt3,tilt4,ail,ele,rud,cruise,rps1,rps2,rps3,rps4,tm1,tm2,tm3,tm4\n");
        for (size_t i = 0; i < size(); ++i) {
            const QpLogRec& r = at(i);
            std::fprintf(f, "%.2f,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f\n",
                         r.t, r.phase, r.flags, r.px, r.py, r.pz, r.vx, r.vy, r.vz, r.roll, r.pitch, r.yaw, r.p, r.q, r.r, r.airspeed, r.v_bus, r.amps,
                         r.duty[0], r.duty[1], r.duty[2], r.duty[3], r.tilt[0], r.tilt[1], r.tilt[2], r.tilt[3], r.ail, r.ele, r.rud, r.cruise,
                         r.rps[0], r.rps[1], r.rps[2], r.rps[3], r.tilt_meas[0], r.tilt_meas[1], r.tilt_meas[2], r.tilt_meas[3]);
        }
        std::fprintf(f, "# події\n");
        for (const QpLogEvent& ev : events_) std::fprintf(f, "# %.2f %s\n", ev.t, ev.text.c_str());
        std::fclose(f);
        return true;
    }
private:
    std::vector<QpLogRec> recs_;
    std::vector<QpLogEvent> events_;
    size_t head_ = 0;
    float acc_ = 0;
};

}  // namespace fc
