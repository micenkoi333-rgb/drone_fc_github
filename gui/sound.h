// sound.h — звук симулятора, синтезований зі стану моделі (без сторонніх бібліотек: Windows waveOut, 44.1 кГц, моно).
// Підйомні гвинти: тон проходу лопатей (2 лопаті → f = 2·n) з гармоніками + свист обмоток мотора (f = 7·n, 14 полюсів),
// гучність ∝ (n / n_висіння)²; маршовий гвинт — так само, окремий рівень; шум потоку ∝ тязі; посвист вітру ∝ V²;
// сигнал тривоги (880 Гц) поки є аварійний стан; удар при аварії. Гучність спадає з відстанню камери.
#pragma once
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
#include "../sim/portable_rng.h"
#pragma comment(lib, "winmm.lib")

struct SoundParams {
    float rps[5] = {0, 0, 0, 0, 0};   // 0–3 підйомні, 4 маршовий, об/с
    float thrust = 0;                 // сумарна тяга, Н
    float airspeed = 0;               // м/с
    float distance = 5;               // м від камери до апарата
    bool alert = false, crash = false, active = false;
    float volume = 0.7f;
};

class EngineSound {
public:
    bool open() {
        WAVEFORMATEX f{};
        f.wFormatTag = WAVE_FORMAT_PCM; f.nChannels = 1; f.nSamplesPerSec = kRate; f.wBitsPerSample = 16;
        f.nBlockAlign = 2; f.nAvgBytesPerSec = kRate * 2;
        if (waveOutOpen(&wo_, WAVE_MAPPER, &f, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) { wo_ = nullptr; return false; }
        for (int i = 0; i < kBufs; ++i) {
            data_[i].assign(kLen, 0);
            hdr_[i] = WAVEHDR{};
            hdr_[i].lpData = reinterpret_cast<LPSTR>(data_[i].data());
            hdr_[i].dwBufferLength = DWORD(kLen * sizeof(int16_t));
            waveOutPrepareHeader(wo_, &hdr_[i], sizeof(WAVEHDR));
            waveOutWrite(wo_, &hdr_[i], sizeof(WAVEHDR));
        }
        return true;
    }
    void close() {
        if (!wo_) return;
        waveOutReset(wo_);
        for (int i = 0; i < kBufs; ++i) waveOutUnprepareHeader(wo_, &hdr_[i], sizeof(WAVEHDR));
        waveOutClose(wo_); wo_ = nullptr;
    }
    bool ok() const { return wo_ != nullptr; }

    // викликати щокадру: дозаповнює відіграні буфери з плавним переходом від попередніх параметрів
    void pump(const SoundParams& p) {
        if (!wo_) return;
        if (p.crash && !was_crash_) thud_ = 1.0f;
        was_crash_ = p.crash;
        for (int i = 0; i < kBufs; ++i) {
            if (!(hdr_[i].dwFlags & WHDR_DONE)) continue;
            fill(data_[i], p);
            waveOutWrite(wo_, &hdr_[i], sizeof(WAVEHDR));
        }
    }

private:
    static constexpr int kRate = 44100, kLen = 1024, kBufs = 6;
    HWAVEOUT wo_ = nullptr;
    WAVEHDR hdr_[kBufs]{};
    std::vector<int16_t> data_[kBufs];
    SoundParams prev_;
    double ph_[5] = {0, 0, 0, 0, 0}, phw_[5] = {0, 0, 0, 0, 0}, ph_alert_ = 0;
    float lp1_ = 0, lp2_ = 0, lp3_ = 0, thud_ = 0, alert_t_ = 0, gain_ = 0;
    bool was_crash_ = false;
    std::mt19937 rng_{12345};
    prng::uniform<float> u_{-1.0f, 1.0f};

    void fill(std::vector<int16_t>& buf, const SoundParams& p) {
        const float dt = 1.0f / kRate, two_pi = 6.2831853f;
        const float dist_k = 1.0f / (1.0f + std::max(0.0f, p.distance - 3.0f) / 12.0f);
        const float target_gain = p.active ? p.volume * dist_k : 0.0f;
        for (int n = 0; n < kLen; ++n) {
            const float a = float(n) / kLen;   // інтерполяція параметрів у межах буфера
            gain_ += (target_gain - gain_) * 0.0005f;
            float s = 0;
            for (int i = 0; i < 5; ++i) {
                const float rps = prev_.rps[i] + (p.rps[i] - prev_.rps[i]) * a;
                if (rps < 2) continue;
                const float k = rps / (i < 4 ? 115.0f : 150.0f), amp = std::min(1.5f, k * k) * (i < 4 ? 0.10f : 0.13f);
                ph_[i] += two_pi * 2.0 * rps * dt; if (ph_[i] > two_pi) ph_[i] -= two_pi;
                phw_[i] += two_pi * 7.0 * rps * dt; if (phw_[i] > two_pi) phw_[i] -= two_pi;
                const float ph = float(ph_[i]);
                s += amp * (std::sin(ph) + 0.45f * std::sin(2 * ph + 0.3f) + 0.22f * std::sin(3 * ph + 1.1f)) + amp * 0.12f * std::sin(float(phw_[i]));
            }
            // шум потоку від гвинтів і вітру (білий шум через одно-полюсні ФНЧ)
            const float thrust = prev_.thrust + (p.thrust - prev_.thrust) * a, v = prev_.airspeed + (p.airspeed - prev_.airspeed) * a;
            const float wn = u_(rng_);
            lp1_ += (wn - lp1_) * 0.10f;          // ≈ 750 Гц
            lp2_ += (wn - lp2_) * 0.035f;         // ≈ 250 Гц
            s += lp1_ * std::min(0.25f, thrust / 400.0f) + lp2_ * std::min(0.5f, (v / 22.0f) * (v / 22.0f)) * 0.8f;
            // удар при аварії
            if (thud_ > 0.001f) { lp3_ += (wn - lp3_) * 0.02f; s += lp3_ * thud_ * 3.0f + thud_ * 0.6f * std::sin(two_pi * 55.0f * (1.0f - thud_)); thud_ *= 0.99985f; }
            float out = std::tanh(s * gain_ * 1.6f);
            // сигнал тривоги: 0.15 с звук / 0.6 с пауза (не залежить від відстані камери)
            if (p.alert && p.active) {
                alert_t_ += dt; if (alert_t_ > 0.75f) alert_t_ = 0;
                if (alert_t_ < 0.15f) { ph_alert_ += two_pi * 880.0 * dt; out += 0.12f * p.volume * float(std::sin(ph_alert_)); }
            } else alert_t_ = 0;
            buf[size_t(n)] = int16_t(std::clamp(out, -1.0f, 1.0f) * 30000.0f);
        }
        prev_ = p;
    }
};
