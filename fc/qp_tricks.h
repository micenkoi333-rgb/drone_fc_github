// qp_tricks.h — пілотажні трюки конвертоплана в режимі літака: мертва петля й звалювання з виходом.
// TrickPilot — «пілот трюку»: на час трюку сам керує рулями, газом і нахилом гондол (контури кутових швидкостей як у FwController).
// Як саме виконувати трюк, задають 7 параметрів u ∈ [0, 1] (швидкість обертання, газ на різних ділянках, кут гондол…) —
// їх підбирає навчання з підкріпленням (sim/qp_tricks.h, метод перехресної ентропії): спроба → оцінка → кращі спроби → нові спроби.
// Знаки: тангаж + ніс угору (pitch_up = −pitch_down), q_up = −ω_y; elevator > 0 — ніс УНИЗ; крен + правий, ω_x + — крен праворуч.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "qp_mission.h"

namespace fc {

enum class Trick : int { Loop = 0, Stall = 1, Roll = 2, Hammerhead = 3, Harrier = 4, Cobra = 5, Flip = 6, Tumble = 7 };
constexpr int kTrickParams = 7, kTrickCount = 8;
inline const char* trickKey(Trick t) { static const char* k[] = {"loop", "stall", "roll", "hammerhead", "harrier", "cobra", "flip", "tumble"}; return k[int(t)]; }

struct TrickParamSpec { const char* name; float lo, hi; };

inline const TrickParamSpec* trickSpec(Trick t) {
    static const TrickParamSpec loop[kTrickParams] = {
        {"розгін перед петлею, с", 0.0f, 4.0f},
        {"швидкість обертання, рад/с", 0.6f, 2.5f},
        {"газ на підйомі", 0.3f, 1.0f},
        {"обертання вгорі (множник)", 0.5f, 1.5f},
        {"газ угорі", 0.0f, 1.0f},
        {"гондоли вгорі, °", 30.0f, 92.0f},
        {"газ на виході", 0.2f, 0.9f},
    };
    static const TrickParamSpec stall[kTrickParams] = {
        {"ніс угору на вході, °", 10.0f, 45.0f},
        {"газ на вході", 0.0f, 0.5f},
        {"тримати після зриву, с", 0.0f, 1.5f},
        {"ніс униз на виході, °", 0.0f, 45.0f},
        {"газ на виході", 0.2f, 1.0f},
        {"гондоли на виході, °", 0.0f, 92.0f},
        {"вирівнювання, рад/с", 0.3f, 1.5f},
    };
    static const TrickParamSpec roll[kTrickParams] = {
        {"ніс угору перед бочкою, °", 0.0f, 25.0f},
        {"швидкість обертання, рад/с", 1.2f, 5.0f},
        {"газ", 0.3f, 1.0f},
        {"допомога гондолами (0…1)", 0.0f, 1.0f},
        {"утримання носа (жорсткість)", 0.5f, 5.0f},
        {"газ униз головою", 0.0f, 1.0f},
        {"газ на виході", 0.2f, 0.9f},
    };
    static const TrickParamSpec hammer[kTrickParams] = {
        {"вихід на вертикаль, рад/с", 0.8f, 2.5f},
        {"тяга на вертикалі, частка ваги", 0.0f, 0.9f},
        {"швидкість початку розвороту, м/с", 1.0f, 9.0f},
        {"швидкість розвороту, рад/с", 0.8f, 3.5f},
        {"тяга на розвороті, частка ваги", 0.2f, 1.2f},
        {"швидкість виводу з піке, м/с", 9.0f, 20.0f},
        {"вивід з піке, рад/с", 0.5f, 2.0f},
    };
    static const TrickParamSpec harrier[kTrickParams] = {
        {"гальмування, м/с²", 1.0f, 8.0f},
        {"ніс угору при гальмуванні, °", 0.0f, 30.0f},
        {"утримання висоти (жорсткість)", 0.5f, 3.0f},
        {"швидкість піруету, рад/с", 0.8f, 4.0f},
        {"рискання гондолами (частка)", 0.0f, 1.0f},
        {"розгін, м/с²", 1.5f, 6.0f},
        {"швидкість переходу в літак, м/с", 10.0f, 16.0f},
    };
    static const TrickParamSpec cobra[kTrickParams] = {
        {"швидкість входу, м/с", 14.0f, 24.0f},
        {"швидкість підйому носа, рад/с", 2.0f, 8.0f},
        {"найбільший кут носа, °", 60.0f, 130.0f},
        {"утримання, с", 0.0f, 1.5f},
        {"гондоли в кобрі (відн. корпусу), °", 0.0f, 90.0f},
        {"тяга в кобрі, частка ваги", 0.0f, 1.3f},
        {"швидкість опускання носа, рад/с", 1.0f, 4.0f},
    };
    static const TrickParamSpec tumble[kTrickParams] = {
        {"удар: крен, рад/с", 0.0f, 9.0f},
        {"удар: тангаж, рад/с", 0.0f, 9.0f},
        {"удар: рискання, рад/с", 0.0f, 6.0f},
        {"тривалість удару, с", 0.2f, 1.0f},
        {"вільне падіння, с", 0.5f, 3.0f},
        {"порятунок: тяга, частка ваги", 1.0f, 2.0f},
        {"порятунок: різкість вирівнювання", 3.0f, 9.0f},
    };
    static const TrickParamSpec flip[kTrickParams] = {
        {"ривок угору, частка ваги", 1.2f, 2.2f},
        {"тривалість ривка, с", 0.2f, 1.2f},
        {"швидкість сальто, рад/с", 4.0f, 12.0f},
        {"тяга під час сальто, частка ваги", 0.0f, 0.7f},
        {"ловля: тяга, частка ваги", 1.0f, 2.0f},
        {"розгін після, м/с²", 2.0f, 6.0f},
        {"швидкість переходу в літак, м/с", 10.0f, 16.0f},
    };
    switch (t) {
        case Trick::Loop: return loop;
        case Trick::Stall: return stall;
        case Trick::Roll: return roll;
        case Trick::Hammerhead: return hammer;
        case Trick::Harrier: return harrier;
        case Trick::Cobra: return cobra;
        case Trick::Flip: return flip;
        default: return tumble;
    }
}
inline const char* trickName(Trick t) {
    static const char* n[] = {"мертва петля", "звалювання й вихід", "векторна бочка", "хамерхед (розворот на вертикалі)", "«Харрієр»: стоп у повітрі, пірует, старт",
                              "кобра Пугачова", "сальто з висіння", "«втрата керування»: кульбіти й порятунок"};
    return n[int(t)];
}
inline const char* trickAbout(Trick t) {
    static const char* n[] = {
        "Повне коло у вертикальній площині. Рулі й газ; угорі гондоли можуть допомагати.",
        "Ніс угору, газ прибрано: крило втрачає підйомну силу (зрив). Вихід — ніс униз, газ, гондоли.",
        "Оберт 360° навколо поздовжньої осі. Гондоли одного боку підіймаються й допомагають елеронам.",
        "Свічка до нуля швидкості, на вершині розворот на 180° лише різницею тяги гвинтів, піке, вихід назад.",
        "Гондоли перекидаються назад і гальмують до висіння, пірует 360° нахилом гондол, розгін і знову літак.",
        "Ніс різко вгору до 90°+ без набору висоти: корпус як повітряне гальмо, гондоли тримають апарат. Потім ніс униз — і далі.",
        "Зупинка в повітрі, ривок угору, повне сальто через ніс у режимі коптера, ловля, розгін і знову літак.",
        "Свічка вгору, різкий удар обертання — і мотори вимикаються: апарат кувиркається у вільному падінні, ніби мертвий. Потім ШІ знаходить, де верх, ловить його, гасить падіння і знову летить літаком.",
    };
    return n[int(t)];
}
inline float trickValue(Trick t, const float u[kTrickParams], int i) {
    const TrickParamSpec& s = trickSpec(t)[i];
    return s.lo + clampf(u[i], 0.0f, 1.0f) * (s.hi - s.lo);
}

// Захист під час трюку: трюк перехоплює керування, але аварійні межі діють і далі.
// Спрацювання → трюк перервано, керування одразу повертається автопілоту (він виводить у рівний політ).
struct TrickGuard {
    bool on = true;
    float ground_z = 0;      // висота дому (оцінка), м
    float min_agl = 15.0f;   // нижче — перервати
    float v_min = 0;         // В, напруга без просідання; 0 — з cfg->v_low
    float motor_dead_s = 0.25f;
};

class TrickPilot {
public:
    float v_cruise = 15.0f, v_stall = 7.9f;
    TrickGuard guard;
    bool aborted() const { return abort_why_ != nullptr; }
    const char* abortReason() const { return abort_why_ ? abort_why_ : ""; }

    void start(Trick t, const float u[kTrickParams]) {
        trick_ = t;
        for (int i = 0; i < kTrickParams; ++i) p_[i] = trickValue(t, u, i);
        phase_ = 0; t_ = 0; t_phase_ = 0; loop_ = 0; qi_ = 0; settle_ = 0; active_ = true; stalled_est_ = false;
        init_ = false; roll_acc_ = 0; spun_ = 0; stop_dist_ = 0; drift_ = 0; pivot_done_ = false; stopped_ = false; spin_done_ = false; ok_ = false;
        nose_max_ = -90; flip_ = 0; z_flip_ = 0; flip_loss_ = 0; wi_ = {}; tumble_rot_ = 0; tumble_up_deg_ = 0; recovered_ = false; z_min_ = 1e9f;
        abort_why_ = nullptr; low_v_t_ = 0; lim_ = 1.0f; for (float& d : dead_t_) d = 0;
    }
    Trick trick() const { return trick_; }
    float param(int i) const { return p_[i]; }
    Vec3 cmdRates() const { return last_w_; }
    float cmdThrust(int i) const { return last_T_[i]; }
    float cmdTilt(int i) const { return last_tilt_[i]; }
    float noseMax() const { return nose_max_; }
    float flipAngle() const { return flip_; }
    float flipLoss() const { return flip_loss_; }
    float tumbleRotations() const { return tumble_rot_ / (2 * kPi); }
    bool recovered() const { return recovered_; }
    float lowestAlt() const { return z_min_ - z0_; }
    int phaseCount() const { static const int n[] = {3, 4, 3, 6, 4, 5, 7, 7}; return n[int(trick_)]; }
    const char* phaseName(int k) const { return phaseLabel(trick_, k); }
    const QpFlightConfig* cfg = nullptr;
    Vec3 a_dist{0, 0, 0};   // прискорення від повітря, передбачене ШІ «модель сил» (світ, м/с²); у висінні трюку віднімається наперед
    bool active() const { return active_; }
    int phase() const { return phase_; }
    float loopAngle() const { return loop_; }   // рад: наскільки повернувся ніс у площині петлі
    bool completedLoop() const { return trick_ == Trick::Loop && loop_ >= kLoopDone; }
    bool stallDetected() const { return stalled_est_; }
    float rollAngle() const { return roll_acc_; }
    float spunAngle() const { return spun_; }
    float stopDistance() const { return stop_dist_; }
    float hoverDrift() const { return drift_; }
    bool pivotDone() const { return pivot_done_; }
    bool stopped() const { return stopped_; }
    bool spinDone() const { return spin_done_; }
    bool succeeded() const { return ok_; }
    float startYaw() const { return yaw0_; }
    float time() const { return t_; }
    const char* phaseText() const { return phaseLabel(trick_, phase_); }
    static const char* phaseLabel(Trick trick, int phase) {
        static const char* lp[] = {"розгін", "петля", "вихід у горизонт", "готово"};
        static const char* st[] = {"гасіння швидкості", "звалювання", "вихід: розгін", "вихід: вирівнювання", "готово"};
        static const char* rl[] = {"ніс угору", "бочка", "вирівнювання", "готово"};
        static const char* hh[] = {"вихід на вертикаль", "свічка", "розворот гондолами", "піке", "вивід з піке", "вирівнювання", "готово"};
        static const char* hr[] = {"гальмування гондолами назад", "пірует", "розгін", "літак: вирівнювання", "готово"};
        static const char* cb[] = {"вихід на швидкість", "ніс угору", "кобра", "ніс униз", "вихід у політ", "готово"};
        static const char* fl[] = {"гальмування до висіння", "висіння", "ривок угору", "сальто", "ловля", "розгін", "літак: вирівнювання", "готово", "готово"};
        static const char* tu[] = {"свічка", "удар обертання", "вільне падіння", "порятунок: де верх?", "гасіння падіння", "розгін", "літак: вирівнювання", "готово"};
        switch (trick) {
            case Trick::Tumble: return tu[std::min(phase, 7)];
            case Trick::Loop: return lp[std::min(phase, 3)];
            case Trick::Stall: return st[std::min(phase, 4)];
            case Trick::Roll: return rl[std::min(phase, 3)];
            case Trick::Hammerhead: return hh[std::min(phase, 6)];
            case Trick::Harrier: return hr[std::min(phase, 4)];
            case Trick::Cobra: return cb[std::min(phase, 5)];
            default: return fl[std::min(phase, 7)];
        }
    }

    // Що пілот робить зараз і чому — з поточними числами й вивченими порогами
    std::string thought() const {
        char b[400];
        const float as = s_as_, vz = s_vz_, nose = s_nose_, alt = s_alt_, vh = s_vh_;
        b[0] = 0;
        if (!active_) { std::snprintf(b, sizeof b, ok_ ? "Трюк виконано. Віддаю керування автопілоту." : "Трюк перервано. Віддаю керування автопілоту."); return b; }
        switch (trick_) {
            case Trick::Loop:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Розганяюсь на повному газі ще %.1f с (вивчено: %.1f с розгону). Швидкість %.1f м/с — запас енергії на петлю.", std::fmax(0.0f, p_[0] - t_phase_), p_[0], as);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "Тягну ніс на себе %.2f рад/с. Пройдено %.0f° з 360°. %s Швидкість %.1f м/с, висота %+.1f м від входу.",
                                                     p_[1], loop_ * kRad2Deg, (loop_ > 0.6f * kPi && loop_ < 1.4f * kPi) ? "Я вгорі, вниз головою: газ і гондоли — як вивчено для вершини." : "Газ — як вивчено для підйому.", as, alt);
                else std::snprintf(b, sizeof b, "Петля замкнулась — виводжу в горизонт, газ %.0f%%. Тангаж %.0f°.", p_[6] * 100, nose);
                break;
            case Trick::Stall:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Задираю ніс на %.0f° і прибираю газ до %.0f%%: гашу швидкість, чекаю зриву. Зараз %.1f м/с (зрив ≈ %.1f м/с).", p_[0], p_[1] * 100, as, 0.85f * v_stall);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "Крило зірвалось — тримаю ручку ще %.1f с, щоб зрив був справжній. Падаю %.1f м/с.", std::fmax(0.0f, p_[2] - t_phase_), -vz);
                else if (phase_ == 2) std::snprintf(b, sizeof b, "Вихід: ніс униз на %.0f°, газ %.0f%%, гондоли %.0f°. Набираю швидкість до %.1f м/с — зараз %.1f.", p_[3], p_[4] * 100, p_[5], 1.25f * v_stall, as);
                else std::snprintf(b, sizeof b, "Вирівнююсь з обмеженою швидкістю %.1f рад/с, щоб не перевантажити крило.", p_[6]);
                break;
            case Trick::Roll:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Піднімаю ніс на %.0f° перед бочкою — у перевернутому стані ніс просяде, тож беру запас. Зараз %.0f°.", p_[0], nose);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "Обертаюсь %.1f рад/с. Пройдено %.0f° з 360°. Гондоли допомагають на %.0f%%. Курс тримаю: відхилення %.0f°.", p_[1], roll_acc_ * kRad2Deg, p_[3] * 100, s_yaw_);
                else std::snprintf(b, sizeof b, "Оберт завершено, вирівнюю крила, газ %.0f%%.", p_[6] * 100);
                break;
            case Trick::Hammerhead:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Виходжу на вертикаль %.1f рад/с. Ніс %.0f° (треба 90°).", p_[0], nose);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "Свічка вгору з тягою %.0f%% ваги. Чекаю, поки вертикальна швидкість впаде до %.1f м/с (вивчений поріг) — зараз %.1f м/с.", p_[1] * 100, p_[2], vz);
                else if (phase_ == 2) std::snprintf(b, sizeof b, "Майже зупинився в повітрі — кермо вже не діє. Розвертаюсь %.1f рад/с різницею тяги лівих і правих гвинтів. Ніс %.0f° (ціль −90°).", p_[3], nose);
                else if (phase_ == 3) std::snprintf(b, sizeof b, "Піке вертикально вниз, газ прибрано. Чекаю швидкості %.1f м/с — зараз %.1f.", p_[5], as);
                else if (phase_ == 4) std::snprintf(b, sizeof b, "Виводжу з піке %.1f рад/с. Ніс %.0f°, висота %+.1f м від входу.", p_[6], nose, alt);
                else std::snprintf(b, sizeof b, "Вирівнююсь. Курс змінився на %.0f° — ціль 180°.", s_yaw_);
                break;
            case Trick::Harrier:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Гондоли назад — гальмую %.1f м/с² і тримаю висоту. Ніс угору до %.0f°, щоб корпус теж гальмував. Земна швидкість %.1f м/с.", p_[0], p_[1], vh);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "Вишу на місці. Пірует %.1f рад/с: гондоли лівого й правого боку нахилені в різні боки. Повернувся на %.0f° з 360°, знесло %.1f м.", p_[3], spun_ * kRad2Deg, drift_);
                else if (phase_ == 2) std::snprintf(b, sizeof b, "Гондоли вперед — розгін %.1f м/с². Повітряна швидкість %.1f м/с; на %.1f м/с переходжу в літак.", p_[5], as, p_[6]);
                else std::snprintf(b, sizeof b, "Летю літаком, гондоли вперед. Чекаю стійкого польоту: %.1f м/с, Vz %.1f м/с.", as, vz);
                break;
            case Trick::Cobra:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Виходжу на швидкість входу %.1f м/с (вивчено) — зараз %.1f. Кобра з'їсть майже всю.", p_[0], as);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "Ривком задираю ніс %.1f рад/с до %.0f°. Зараз %.0f°. Гондоли під %.0f° до корпусу, тяга %.0f%% ваги.", p_[1], p_[2], nose, p_[4], p_[5] * 100);
                else if (phase_ == 2) std::snprintf(b, sizeof b, "Кобра! Ніс %.0f°, корпус гальмує як щит: швидкість %.1f м/с. Висоту тримають гвинти. Ще %.1f с.", nose, as, std::fmax(0.0f, p_[3] - t_phase_));
                else if (phase_ == 3) std::snprintf(b, sizeof b, "Опускаю ніс %.1f рад/с. Зараз %.0f°, висота %+.1f м від входу.", p_[6], nose, alt);
                else std::snprintf(b, sizeof b, "Розганяюсь назад до польоту: %.1f м/с. Гвинти підтримують, поки крило не понесе.", as);
                break;
            case Trick::Tumble:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Свічка вгору — набираю запас висоти для падіння. Висота %+.1f м від входу, ніс %.0f°.", alt, nose);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "УДАР: закручую себе %.1f / %.1f / %.1f рад/с (крен / тангаж / рискання) різницею тяги гвинтів.", p_[0], p_[1], p_[2]);
                else if (phase_ == 2) std::snprintf(b, sizeof b, "Мотори вимкнено — падаю, як камінь, і кувиркаюсь. Ще %.1f с. Уже %.0f обертів, падаю %.1f м/с. Гіроскоп пам'ятає, де я.", std::fmax(0.0f, p_[4] - t_phase_), tumble_rot_ / (2 * kPi), -vz);
                else if (phase_ == 3) std::snprintf(b, sizeof b, "ПОРЯТУНОК: шукаю, де верх (зараз «дах» дивиться на %.0f° від неба). Тяга — лише коли гвинти дивляться вгору, інакше штовхну себе в землю.", tumble_up_deg_);
                else if (phase_ == 4) std::snprintf(b, sizeof b, "Рівно! Гашу падіння тягою %.0f%% ваги: падаю %.1f м/с. Втрачено %.1f м від входу.", p_[5] * 100, -vz, -std::fmin(alt, 0.0f));
                else if (phase_ == 5) std::snprintf(b, sizeof b, "Врятувався. Гондоли вперед — розгін, %.1f м/с.", as);
                else std::snprintf(b, sizeof b, "Знову літак. %.1f м/с.", as);
                break;
            default:
                if (phase_ == 0) std::snprintf(b, sizeof b, "Гальмую гондолами до висіння. Земна швидкість %.1f м/с.", vh);
                else if (phase_ == 1) std::snprintf(b, sizeof b, "Вишу, збираюсь. Ще %.1f с.", std::fmax(0.0f, 1.5f - t_phase_));
                else if (phase_ == 2) std::snprintf(b, sizeof b, "Ривок угору: %.0f%% ваги ще %.2f с — щоб мати висоту на сальто. Vz %.1f м/с.", p_[0] * 100, std::fmax(0.0f, p_[1] - t_phase_), vz);
                else if (phase_ == 3) std::snprintf(b, sizeof b, "САЛЬТО %.1f рад/с! Пройдено %.0f° з 360°. Тягу скинуто до %.0f%% ваги, щоб вниз головою не тягнути в землю.", p_[2], flip_ * kRad2Deg, p_[3] * 100);
                else if (phase_ == 4) std::snprintf(b, sizeof b, "Ловлю: рівно й тяга %.0f%% ваги. Втрачено висоти %.1f м.", p_[4] * 100, flip_loss_);
                else if (phase_ == 5) std::snprintf(b, sizeof b, "Гондоли вперед — розгін %.1f м/с². %.1f м/с, на %.1f — у літак.", p_[5], as, p_[6]);
                else std::snprintf(b, sizeof b, "Летю літаком, вирівнююсь. %.1f м/с.", as);
        }
        return b;
    }

    // Один крок (dt) — команди виконавчим механізмам. Повертає false, коли трюк закінчено (керування — назад автопілоту).
    bool update(float dt, const QpEstimate& e, QpActuators& out) {
        if (!active_) return false;
        if (guard.on) {
            if (const char* why = guardCheck(dt, e)) { abort_why_ = why; active_ = false; return false; }
        }
        const bool r = updateImpl(dt, e, out);
        if (r && guard.on && cfg) {   // обмежувач струму батареї діє й у трюку (як у звичайному польоті)
            if (e.bus_current > cfg->i_max) lim_ = std::fmax(0.5f, lim_ - 3.0f * dt * (e.bus_current / cfg->i_max - 1.0f + 0.1f));
            else lim_ = std::fmin(1.0f, lim_ + 0.5f * dt);
            for (float& d : out.duty_vtol) d *= lim_;
            out.duty_cruise *= lim_;
        }
        return r;
    }
    float currentLimit() const { return lim_; }
private:
    float lim_ = 1.0f;
public:
    bool updateImpl(float dt, const QpEstimate& e, QpActuators& out) {
        t_ += dt; t_phase_ += dt;
        if (trick_ != Trick::Loop && trick_ != Trick::Stall) return updateVector(dt, e, out);
        const Euler eu = eulerOf(e.q);
        const float pitch_up = -eu.pitch_down, q_up = -e.omega.y;
        float q_cmd = 0, p_cmd = 0, thr = 0.5f, tilt = 1.57f;
        auto level = [&](float pitch_target, float q_max) {   // тримати тангаж і крила рівно
            q_cmd = clampf(2.5f * (pitch_target - pitch_up), -q_max, q_max);
            p_cmd = clampf(3.0f * (0.0f - eu.roll), -1.6f, 1.6f);
        };
        if (trick_ == Trick::Loop) {
            switch (phase_) {
                case 0:   // розгін по горизонту
                    level(0, 1.0f); thr = 1.0f;
                    if (t_phase_ >= p_[0]) next();
                    break;
                case 1: {   // петля: обертання з заданою швидкістю, вгорі — свій газ, гондоли й швидкість обертання
                    loop_ += q_up * dt;
                    const bool top = loop_ > 0.6f * kPi && loop_ < 1.4f * kPi;
                    q_cmd = p_[1] * (top ? p_[3] : 1.0f);
                    p_cmd = 0;   // площина петлі: крен не рухаємо
                    thr = top ? p_[4] : p_[2];
                    tilt = top ? p_[5] * kDeg2Rad : 1.57f;
                    if (loop_ >= kLoopDone) next();
                    if (t_phase_ > 15.0f) { phase_ = 3; active_ = false; }   // не вийшло обернутись
                    break;
                }
                case 2:   // вихід у горизонтальний політ
                    level(0, 1.0f); thr = p_[6];
                    settle_ = (std::fabs(pitch_up) < 0.09f && std::fabs(q_up) < 0.3f) ? settle_ + dt : 0.0f;
                    if (settle_ > 1.0f || t_phase_ > 8.0f) { phase_ = 3; active_ = false; }
                    break;
                default: active_ = false;
            }
        } else {
            const Vec3 vb = e.q.rotateInv(e.vel);
            const float alpha = std::atan2(-vb.z, std::fmax(vb.x, 0.5f));   // кут атаки (оцінка з GPS/EKF; вітер не враховано)
            switch (phase_) {
                case 0:   // ніс угору, газ малий — швидкість падає
                    level(p_[0] * kDeg2Rad, 1.0f); thr = p_[1];
                    if (alpha > 14.0f * kDeg2Rad || e.airspeed < 0.85f * v_stall) { stalled_est_ = true; next(); }
                    if (t_phase_ > 10.0f) { phase_ = 4; active_ = false; }   // так і не звалився
                    break;
                case 1:   // тримати ручку — справжній зрив
                    level(p_[0] * kDeg2Rad, 1.0f); thr = p_[1];
                    if (t_phase_ >= p_[2]) next();
                    break;
                case 2:   // вихід: ніс униз, газ, гондоли (0° — гвинти тягнуть угору, як коптер)
                    level(-p_[3] * kDeg2Rad, 1.5f); thr = p_[4]; tilt = p_[5] * kDeg2Rad;
                    if (e.airspeed > 1.25f * v_stall || t_phase_ > 6.0f) next();
                    break;
                case 3:   // вирівнювання з обмеженою швидкістю (перевантаження)
                    level(0, p_[6]); thr = 0.6f; tilt = 1.57f;
                    settle_ = (std::fabs(pitch_up) < 0.09f && std::fabs(e.vel.z) < 1.0f && e.airspeed > 1.1f * v_stall) ? settle_ + dt : 0.0f;
                    if (settle_ > 1.0f || t_phase_ > 8.0f) { phase_ = 4; active_ = false; }
                    break;
                default: active_ = false;
            }
        }
        // контури кутових швидкостей → керма (ефективність ∝ V², як у FwController)
        const float v = std::fmax(e.airspeed, 6.0f);
        const float scale = clampf((v_cruise * v_cruise) / (v * v), 0.5f, 2.5f);
        const float err = (q_cmd - q_up) * scale;
        qi_ = clampf(qi_ + 0.10f * err * dt, -0.25f, 0.25f);
        out.elevator = -clampf(0.12f * err + qi_, -0.43f, 0.43f);
        out.aileron = clampf(0.09f * (p_cmd - e.omega.x) * scale, -0.43f, 0.43f);
        out.rudder = clampf(-0.12f * e.omega.z * scale, -0.43f, 0.43f);
        thr = clampf(thr, 0.0f, 1.0f);
        for (int i = 0; i < 4; ++i) { out.duty_vtol[i] = thr; out.tilt[i] = tilt; last_tilt_[i] = tilt; last_T_[i] = cfg ? thr * cfg->vtol_map.maxThrust() : thr * 15.0f; }
        out.duty_cruise = 0;
        last_w_ = {p_cmd, -q_cmd, 0};
        snap(e);
        return active_;
    }

    static constexpr float kLoopDone = 2.0f * kPi - 0.35f;

private:
    Trick trick_ = Trick::Loop;
    float p_[kTrickParams] = {};
    int phase_ = 0;
    float t_ = 0, t_phase_ = 0, loop_ = 0, qi_ = 0, settle_ = 0;
    const char* guardCheck(float dt, const QpEstimate& e) {
        if (e.pos.z - guard.ground_z < guard.min_agl) return "мала висота";
        const float vmin = guard.v_min > 0 ? guard.v_min : (cfg ? cfg->v_low : 0.0f);
        if (vmin > 0) {
            const float v_rest = e.v_bus + std::fmax(e.bus_current, 0.0f) * (cfg ? cfg->batt_r_est : 0.06f);
            low_v_t_ = v_rest < vmin ? low_v_t_ + dt : 0.0f;
            if (low_v_t_ > 1.0f) return "низька напруга батареї";
        }
        for (int i = 0; i < 4; ++i) {   // мотор не крутиться, хоча трюк просить тягу
            const bool dead = e.vtol_rps[i] >= 0 && last_T_[i] > 2.0f && e.vtol_rps[i] < 15.0f;
            dead_t_[i] = dead ? dead_t_[i] + dt : 0.0f;
            if (dead_t_[i] > guard.motor_dead_s) return "відмова мотора";
        }
        return nullptr;
    }
    const char* abort_why_ = nullptr;
    float low_v_t_ = 0, dead_t_[4] = {0, 0, 0, 0};
    bool active_ = false, stalled_est_ = false;
    void next() { ++phase_; t_phase_ = 0; settle_ = 0; }

    bool init_ = false, pivot_done_ = false, stopped_ = false, spin_done_ = false, ok_ = false;
    float yaw0_ = 0, z0_ = 0, roll_acc_ = 0, spun_ = 0, stop_dist_ = 0, drift_ = 0, roll_ref_ = 0;
    float nose_max_ = -90, flip_ = 0, z_flip_ = 0, flip_loss_ = 0, z_hold_ = 0;
    float tumble_rot_ = 0, tumble_up_deg_ = 0, z_min_ = 0;
    bool recovered_ = false;
    Vec3 p0_, ph_, wi_, last_w_;
    float last_T_[4] = {}, last_tilt_[4] = {};
    float s_as_ = 0, s_vz_ = 0, s_nose_ = 0, s_alt_ = 0, s_vh_ = 0, s_yaw_ = 0;
    bool s_init_ = false; float s_z0_ = 0, s_yaw0_ = 0;
    void snap(const QpEstimate& e) {
        if (!s_init_) { s_init_ = true; s_z0_ = e.pos.z; s_yaw0_ = eulerOf(e.q).yaw; }
        const Vec3 n = e.q.rotate(Vec3{1, 0, 0});
        const float yw = init_ ? yaw0_ : s_yaw0_;
        s_as_ = e.airspeed; s_vz_ = e.vel.z; s_alt_ = e.pos.z - (init_ ? z0_ : s_z0_); s_vh_ = std::hypot(e.vel.x, e.vel.y);
        s_nose_ = std::atan2(n.z, n.x * std::cos(yw) + n.y * std::sin(yw)) * kRad2Deg;
        s_yaw_ = wrapPi(eulerOf(e.q).yaw - yw) * kRad2Deg;
    }

    void finish(bool ok) { ok_ = ok; active_ = false; phase_ = 99; }
    void next2() { next(); wi_ = {}; qi_ = 0; }

    Vec3 attRate(const Quat& q, const Quat& qt, float k) const {
        Quat qe = q.conj() * qt;
        if (qe.w < 0) { qe.w = -qe.w; qe.x = -qe.x; qe.y = -qe.y; qe.z = -qe.z; }
        return Vec3{qe.x, qe.y, qe.z} * (2.0f * k);
    }
    static Vec3 pointNose(const Quat& q, const Vec3& dir_world, float k) {
        const Vec3 t = q.rotateInv(dir_world);
        return {0, -k * t.z, k * t.y};
    }

    void surfaces(const QpEstimate& e, const Vec3& w, QpActuators& out, float dt) {
        const float v = std::fmax(e.airspeed, 6.0f);
        const float scale = clampf((v_cruise * v_cruise) / (v * v), 0.5f, 2.5f);
        const float ey = (w.y - e.omega.y) * scale;
        qi_ = clampf(qi_ + 0.10f * ey * dt, -0.25f, 0.25f);
        out.elevator = clampf(0.12f * ey + qi_, -0.43f, 0.43f);
        out.aileron = clampf(0.09f * (w.x - e.omega.x) * scale, -0.43f, 0.43f);
        out.rudder = clampf(0.12f * (w.z - e.omega.z) * scale, -0.43f, 0.43f);
    }

    // Розподіл на 4 гвинти з гондолами: кожен дає силу (fx уперед, fz угору) у площині x–z тіла.
    // Fz і моменти крену/тангажу — різницею fz, сила вперед і рискання — різницею fx (кут гондоли = atan2(fx, fz)).
    void rotors(const QpEstimate& e, const Vec3& tau, float Fx, float Fz, float react, QpActuators& out) {
        const MixerConfig& m = cfg->af.mixer;
        const float dx = m.dx(), dyf = m.dy(), dyr = m.dyRear();
        float X[4], Y[4];
        float sx2 = 0, sy2 = 0;
        for (int i = 0; i < 4; ++i) { X[i] = kQuadX[i].sx * dx; Y[i] = kQuadX[i].sy * (kQuadX[i].sx > 0 ? dyf : dyr); sx2 += X[i] * X[i]; sy2 += Y[i] * Y[i]; }
        float fz[4], fx[4], lo = 1e9f;
        for (int i = 0; i < 4; ++i) {
            const float tz = clampf(tau.z, -2.0f, 2.0f);
            fz[i] = Fz / 4 + tau.x * Y[i] / sy2 - tau.y * X[i] / sx2 + clampf(react * tz * kQuadX[i].yaw_sign / (4 * m.k_yaw), -1.5f, 1.5f);
            fx[i] = Fx / 4 - (1 - react) * tz * Y[i] / sy2;
            lo = std::fmin(lo, fz[i]);
        }
        if (lo < 0 && Fz < 1.0f) for (float& f : fz) f -= lo;
        for (float& f : fz) f = std::fmax(f, 0.0f);
        const float Tm = cfg->vtol_map.maxThrust();
        for (int i = 0; i < 4; ++i) {
            const float th = clampf(std::hypot(fx[i], fz[i]) > 0.2f ? std::atan2(fx[i], fz[i]) : 1.57f, cfg->tilt_min, cfg->tilt_max);
            const float T = clampf(fx[i] * std::sin(th) + fz[i] * std::cos(th), 0.0f, Tm);
            const float v_ax = std::fmax(e.airspeed * std::sin(th), 0.0f);
            out.tilt[i] = th;
            out.duty_vtol[i] = T > 0.05f ? clampf(cfg->vtol_map2.toDuty(T, v_ax, e.v_bus), 0.0f, 1.0f) : 0.0f;
            last_T_[i] = T; last_tilt_[i] = th;
        }
        out.duty_cruise = 0;
    }

    Vec3 torqueFor(const QpEstimate& e, const Vec3& w, float dt) {
        const Vec3 err = w - e.omega;
        wi_ = Vec3{clampf(wi_.x + err.x * dt, -0.5f, 0.5f), clampf(wi_.y + err.y * dt, -0.5f, 0.5f), clampf(wi_.z + err.z * dt, -0.5f, 0.5f)};
        const Vec3 acc = err * 14.0f + wi_ * 8.0f;
        return acc.mul(cfg->af.inertia);
    }

    void fly(const QpEstimate& e, const Vec3& w, float Fx, float Fz, float rotor_auth, float react, float dt, QpActuators& out) {
        last_w_ = w;
        surfaces(e, w, out, dt);
        rotors(e, torqueFor(e, w, dt) * rotor_auth, Fx, Fz, react, out);
    }

    // Висіння й рух гондолами: бажане прискорення у світі → сила в тілі (x — нахил гондол, z — тяга), y — креном
    void hover(const QpEstimate& e, const Vec3& a_w, float pitch_up, float yaw_t, float yaw_ff, float react, float dt, QpActuators& out) {
        const float m = cfg->af.mass;
        const float cy = std::cos(yaw_t), sy = std::sin(yaw_t);
        const float a_left = -a_w.x * sy + a_w.y * cy;
        const float roll = clampf(-std::atan2(a_left, kGravity + a_w.z), -0.35f, 0.35f);
        const Quat qt = Quat::fromEuler(roll, -pitch_up, yaw_t);
        Vec3 w = attRate(e.q, qt, 5.0f);
        w.x = clampf(w.x, -3, 3); w.y = clampf(w.y, -3, 3); w.z = clampf(w.z + yaw_ff, -4.5f, 4.5f);
        const Vec3 Fb = e.q.rotateInv(Vec3{a_w.x - a_dist.x, a_w.y - a_dist.y, a_w.z - a_dist.z + kGravity} * m);
        fly(e, w, Fb.x, std::fmax(Fb.z, 0.0f), 1.0f, react, dt, out);
    }

    float thrustOf(float thr) const { return clampf(thr, 0.0f, 1.0f) * 4.0f * cfg->vtol_map.maxThrust(); }
    float weightOf(float f) const { return std::fmin(f * cfg->af.mass * kGravity, 4.0f * cfg->vtol_map.maxThrust()); }

    void planeOut(const QpEstimate& e, const Euler& eu, float dt, QpActuators& out, float z_ref) {
        (void)eu;
        const float Fx = clampf(4.0f * cfg->vtol_map.maxThrust() * (0.35f + 0.08f * (v_cruise - e.airspeed)), 0.0f, 4.0f * cfg->vtol_map.maxThrust());
        Vec3 w = attRate(e.q, Quat::fromEuler(0, -clampf(3 * kDeg2Rad + 0.05f * (z_ref - e.pos.z), -0.15f, 0.25f), yaw0_), 3.0f);
        w = Vec3{clampf(w.x, -1.5f, 1.5f), clampf(w.y, -1.0f, 1.0f), clampf(w.z, -0.6f, 0.6f)};
        fly(e, w, Fx, 0, 0.3f, 0, dt, out);
        for (float& t : out.tilt) t = std::fmax(t, 1.45f);
    }

    bool updateVector(float dt, const QpEstimate& e, QpActuators& out) {
        if (!cfg) { finish(false); return false; }
        const Euler eu = eulerOf(e.q);
        if (!init_) { init_ = true; yaw0_ = eu.yaw; z0_ = e.pos.z; p0_ = e.pos; roll_ref_ = eu.roll; wi_ = {}; z_hold_ = e.pos.z; }
        snap(e);
        const Vec3 nose = e.q.rotate(Vec3{1, 0, 0});
        const float pitch_up = -eu.pitch_down;
        auto levelRates = [&](float pitch_t, float k) {
            Vec3 w = attRate(e.q, Quat::fromEuler(0, -pitch_t, eu.yaw), k);
            return Vec3{clampf(w.x, -2.0f, 2.0f), clampf(w.y, -1.2f, 1.2f), clampf(w.z, -1.0f, 1.0f)};
        };
        if (trick_ == Trick::Roll) {
            switch (phase_) {
                case 0: {
                    const float pt = p_[0] * kDeg2Rad;
                    fly(e, levelRates(pt, 3.0f), thrustOf(p_[2]), 0, 0.3f, 0, dt, out);
                    if ((std::fabs(pitch_up - pt) < 2 * kDeg2Rad && t_phase_ > 0.3f) || t_phase_ > 2.5f) { next(); roll_ref_ = eu.roll; roll_acc_ = 0; }
                    break;
                }
                case 1: {
                    roll_ref_ += p_[1] * dt;
                    roll_acc_ += e.omega.x * dt;
                    const Quat qt = Quat::fromEuler(roll_ref_, -p_[0] * kDeg2Rad * std::fmax(0.0f, 1.0f - t_phase_ * p_[1] / (2 * kPi)), yaw0_);
                    Vec3 w = attRate(e.q, qt, p_[4]);
                    w.x += p_[1];
                    w.x = clampf(w.x, -6, 6); w.y = clampf(w.y, -2, 2); w.z = clampf(w.z, -2, 2);
                    const bool inverted = e.q.rotate(Vec3{0, 0, 1}).z < 0;
                    fly(e, w, thrustOf(inverted ? p_[5] : p_[2]), 0, p_[3], 0, dt, out);
                    if (roll_acc_ >= 2 * kPi - 0.15f) next();
                    if (t_phase_ > 10.0f) finish(false);
                    break;
                }
                case 2: {
                    fly(e, levelRates(0, 3.0f), thrustOf(p_[6]), 0, 0.3f, 0, dt, out);
                    settle_ = (std::fabs(eu.roll) < 0.1f && std::fabs(pitch_up) < 0.12f && std::fabs(e.omega.x) < 0.3f) ? settle_ + dt : 0.0f;
                    if (settle_ > 1.0f) finish(true);
                    if (t_phase_ > 8.0f) finish(false);
                    break;
                }
                default: finish(false);
            }
            return active_;
        }
        if (trick_ == Trick::Hammerhead) {
            switch (phase_) {
                case 0: {
                    Vec3 w{clampf(3.0f * (0 - eu.roll), -1.5f, 1.5f), -p_[0], 0};
                    fly(e, w, weightOf(p_[1] + 0.3f), 0, 0.5f, 0, dt, out);
                    if (nose.z > 0.97f) next();
                    if (t_phase_ > 6.0f) finish(false);
                    break;
                }
                case 1: {
                    const Vec3 w = pointNose(e.q, Vec3{0, 0, 1}, 4.0f);
                    fly(e, w, weightOf(p_[1]), 0, 1.0f, 0, dt, out);
                    if (e.vel.z < p_[2]) next();
                    if (t_phase_ > 10.0f) finish(false);
                    break;
                }
                case 2: {
                    const Vec3 w{0, 0, p_[3]};
                    fly(e, w, weightOf(p_[4]), 0, 1.0f, 0, dt, out);
                    if (nose.z < -0.85f) { pivot_done_ = true; next(); }
                    if (t_phase_ > 8.0f) finish(false);
                    break;
                }
                case 3: {
                    const Vec3 w = pointNose(e.q, Vec3{0, 0, -1}, 4.0f);
                    fly(e, w, thrustOf(0.05f), 0, 1.0f, 0, dt, out);
                    if (e.airspeed > p_[5] || std::fabs(e.vel.z) > p_[5]) next();
                    if (t_phase_ > 6.0f) finish(false);
                    break;
                }
                case 4: {
                    Vec3 w{0, -p_[6], 0};
                    if (nose.z > -0.6f) w.x = clampf(3.0f * (0 - eu.roll), -1.5f, 1.5f);
                    fly(e, w, thrustOf(0.4f), 0, 0.5f, 0, dt, out);
                    if (nose.z > -0.03f) next();
                    if (t_phase_ > 6.0f) finish(false);
                    break;
                }
                case 5: {
                    fly(e, levelRates(0, 3.0f), thrustOf(0.35f), 0, 0.3f, 0, dt, out);
                    settle_ = (std::fabs(eu.roll) < 0.12f && std::fabs(pitch_up) < 0.12f) ? settle_ + dt : 0.0f;
                    if (settle_ > 1.0f) finish(pivot_done_);
                    if (t_phase_ > 8.0f) finish(false);
                    break;
                }
                default: finish(false);
            }
            return active_;
        }
        if (trick_ == Trick::Cobra) {
            const float hx = std::cos(yaw0_), hy = std::sin(yaw0_);
            const float nose_ang = std::atan2(nose.z, nose.x * hx + nose.y * hy);
            nose_max_ = std::fmax(nose_max_, nose_ang * kRad2Deg);
            auto inPlane = [&](float ang, float k) {
                const Vec3 dir{std::cos(ang) * hx, std::cos(ang) * hy, std::sin(ang)};
                Vec3 w = pointNose(e.q, dir, k);
                const Vec3 wing = e.q.rotate(Vec3{0, 1, 0});
                w.x = clampf(-4.0f * wing.z, -2.0f, 2.0f);
                return w;
            };
            switch (phase_) {
                case 0:
                    fly(e, levelRates(0, 3.0f), thrustOf(clampf(0.35f + 0.12f * (p_[0] - e.airspeed), 0.0f, 1.0f)), 0, 0.3f, 0, dt, out);
                    if ((std::fabs(e.airspeed - p_[0]) < 0.7f && t_phase_ > 0.5f) || t_phase_ > 3.0f) next2();
                    break;
                case 1: {
                    Vec3 w = inPlane(nose_ang, 0);
                    w.y = -p_[1];
                    const float tl = p_[4] * kDeg2Rad;
                    const float T = weightOf(p_[5]);
                    fly(e, w, T * std::sin(tl), T * std::cos(tl) * clampf(nose_ang / (p_[2] * kDeg2Rad), 0.0f, 1.0f), 1.0f, 0, dt, out);
                    if (nose_ang >= p_[2] * kDeg2Rad) next2();
                    if (t_phase_ > 4.0f) finish(false);
                    break;
                }
                case 2: {
                    const Vec3 w = inPlane(p_[2] * kDeg2Rad, 5.0f);
                    const float tl = p_[4] * kDeg2Rad;
                    const float T = weightOf(p_[5]);
                    fly(e, w, T * std::sin(tl), T * std::cos(tl), 1.0f, 0, dt, out);
                    if (t_phase_ >= p_[3]) next2();
                    break;
                }
                case 3: {
                    Vec3 w = inPlane(nose_ang, 0);
                    w.y = p_[6];
                    if (nose_ang < 0.35f) { const Vec3 wl = levelRates(0, 3.0f); w.y = std::fmin(w.y, wl.y + 0.5f); }
                    const float T = weightOf(std::fmax(p_[5], 0.6f));
                    const float tl = clampf(nose_ang, 0.0f, 1.57f);
                    fly(e, w, T * std::sin(tl) + thrustOf(0.2f), T * std::cos(tl), 1.0f, 0, dt, out);
                    if (nose_ang < 0.05f) next2();
                    if (t_phase_ > 5.0f) finish(false);
                    break;
                }
                case 4: {
                    fly(e, levelRates(2 * kDeg2Rad + clampf(0.04f * (z0_ - e.pos.z), -0.15f, 0.2f), 3.0f), thrustOf(clampf(0.35f + 0.1f * (v_cruise - e.airspeed), 0.1f, 0.9f)), weightOf(clampf((1.25f * v_stall - e.airspeed) / (0.6f * v_stall), 0.0f, 1.0f)), 0.6f, 0, dt, out);
                    settle_ = (std::fabs(eu.roll) < 0.12f && std::fabs(pitch_up) < 0.15f && e.airspeed > 1.2f * v_stall) ? settle_ + dt : 0.0f;
                    if (settle_ > 1.0f) finish(nose_max_ > 75);
                    if (t_phase_ > 10.0f) finish(false);
                    break;
                }
                default: finish(false);
            }
            return active_;
        }
        if (trick_ == Trick::Tumble) {
            z_min_ = std::fmin(z_min_, e.pos.z);
            const Vec3 up_b = e.q.rotateInv(Vec3{0, 0, 1});
            tumble_up_deg_ = std::acos(clampf(up_b.z, -1.0f, 1.0f)) * kRad2Deg;
            switch (phase_) {
                case 0:
                    fly(e, levelRates(35 * kDeg2Rad, 3.0f), thrustOf(0.2f), 0, 0.5f, 0, dt, out);
                    if (e.airspeed < 11.0f || t_phase_ > 3.0f) next2();
                    break;
                case 1: {
                    const Vec3 w{p_[0], -p_[1], p_[2]};
                    fly(e, w, 0, weightOf(0.45f), 1.0f, 0.3f, dt, out);
                    tumble_rot_ += e.omega.norm() * dt;
                    if (t_phase_ >= p_[3]) next2();
                    break;
                }
                case 2:
                    tumble_rot_ += e.omega.norm() * dt;
                    for (int i = 0; i < 4; ++i) { out.duty_vtol[i] = 0; out.tilt[i] = 0.3f; last_T_[i] = 0; last_tilt_[i] = 0.3f; }
                    out.duty_cruise = 0; out.elevator = out.aileron = out.rudder = 0;
                    last_w_ = {};
                    if (t_phase_ >= p_[4]) next2();
                    break;
                case 3: {
                    tumble_rot_ += e.omega.norm() * dt;
                    Vec3 w = attRate(e.q, Quat::fromEuler(0, 0, eulerOf(e.q).yaw), p_[6]);
                    w = Vec3{clampf(w.x, -9, 9), clampf(w.y, -9, 9), clampf(w.z, -3, 3)};
                    const float k = clampf((up_b.z - 0.2f) / 0.6f, 0.0f, 1.0f);
                    fly(e, w, 0, weightOf(0.25f + k * (p_[5] - 0.25f)), 1.0f, 0.3f, dt, out);
                    for (float& t : out.tilt) t = 0;
                    if (up_b.z > 0.9f && e.omega.norm() < 1.5f) { recovered_ = true; next2(); }
                    if (t_phase_ > 6.0f) finish(false);
                    break;
                }
                case 4: {
                    const Vec3 a{-1.2f * e.vel.x, -1.2f * e.vel.y, clampf(2.2f * (0.0f - e.vel.z), -3.0f, kGravity * (p_[5] - 1.0f) + 2.0f)};
                    hover(e, a, 0, eulerOf(e.q).yaw, 0, 0.3f, dt, out);
                    if (std::fabs(e.vel.z) < 0.8f && std::hypot(e.vel.x, e.vel.y) < 2.0f) { z_hold_ = e.pos.z; yaw0_ = eulerOf(e.q).yaw; next2(); }
                    if (t_phase_ > 8.0f) finish(false);
                    break;
                }
                case 5: {
                    const float az = clampf(1.5f * (1.5f * (z_hold_ - e.pos.z) - 1.8f * e.vel.z), -5.0f, 6.0f);
                    const float cy = std::cos(yaw0_), sy = std::sin(yaw0_);
                    const float v_left = -e.vel.x * sy + e.vel.y * cy;
                    Vec3 a{4.0f * cy + 1.5f * v_left * sy, 4.0f * sy - 1.5f * v_left * cy, az};
                    hover(e, a, 2 * kDeg2Rad, yaw0_, 0, 0.3f, dt, out);
                    if (e.airspeed > 13.0f) next2();
                    if (t_phase_ > 12.0f) finish(false);
                    break;
                }
                case 6:
                    planeOut(e, eulerOf(e.q), dt, out, z_hold_);
                    settle_ = (std::fabs(e.vel.z) < 1.0f && e.airspeed > 1.3f * v_stall && std::fabs(eulerOf(e.q).roll) < 0.12f) ? settle_ + dt : 0.0f;
                    if (settle_ > 1.5f) finish(recovered_);
                    if (t_phase_ > 10.0f) finish(false);
                    break;
                default: finish(false);
            }
            return active_;
        }
        if (trick_ == Trick::Flip) {
            const float az = clampf(1.5f * (1.5f * (z_hold_ - e.pos.z) - 1.8f * e.vel.z), -5.0f, 6.0f);
            switch (phase_) {
                case 0: {
                    const float vh = std::hypot(e.vel.x, e.vel.y);
                    Vec3 a{0, 0, az};
                    if (vh > 0.3f) { a.x = -5.0f * e.vel.x / vh; a.y = -5.0f * e.vel.y / vh; }
                    hover(e, a, 12 * kDeg2Rad * clampf(vh / 6.0f, 0.0f, 1.0f), yaw0_, 0, 0.3f, dt, out);
                    if (vh < 0.8f) { stopped_ = true; ph_ = e.pos; z_hold_ = e.pos.z; next2(); }
                    if (t_phase_ > 15.0f) finish(false);
                    break;
                }
                case 1: {
                    Vec3 a{clampf(1.2f * (ph_.x - e.pos.x) - 1.8f * e.vel.x, -3, 3), clampf(1.2f * (ph_.y - e.pos.y) - 1.8f * e.vel.y, -3, 3), az};
                    hover(e, a, 0, yaw0_, 0, 0.3f, dt, out);
                    if (t_phase_ > 1.5f) { z_flip_ = e.pos.z; next2(); }
                    break;
                }
                case 2: {
                    const Vec3 w = levelRates(0, 4.0f);
                    fly(e, w, 0, weightOf(p_[0]), 1.0f, 0.3f, dt, out);
                    for (float& t : out.tilt) t = 0;
                    if (t_phase_ >= p_[1]) { flip_ = 0; next2(); }
                    break;
                }
                case 3: {
                    flip_ += e.omega.y * dt;
                    const Vec3 w{0, p_[2], 0};
                    fly(e, w, 0, weightOf(p_[3]), 1.0f, 0.3f, dt, out);
                    for (float& t : out.tilt) t = 0;
                    if (flip_ >= 2 * kPi - 0.6f) next2();
                    if (t_phase_ > 3.0f) finish(false);
                    break;
                }
                case 4: {
                    if (t_phase_ < 0.6f) {
                        Vec3 w = attRate(e.q, Quat::fromEuler(0, 0, eu.yaw), 7.0f);
                        w = Vec3{clampf(w.x, -6, 6), clampf(w.y, -6, 6), clampf(w.z, -2, 2)};
                        fly(e, w, 0, weightOf(p_[4]), 1.0f, 0.3f, dt, out);
                        for (float& t : out.tilt) t = 0;
                    } else {
                        if (z_hold_ > e.pos.z + 1) z_hold_ = std::fmax(e.pos.z, z_hold_ - 3.0f * dt);
                        Vec3 a{-1.8f * e.vel.x, -1.8f * e.vel.y, clampf(1.5f * (1.5f * (z_hold_ - e.pos.z) - 1.8f * e.vel.z), -5.0f, 6.0f)};
                        hover(e, a, 0, eu.yaw, 0, 0.3f, dt, out);
                        flip_loss_ = std::fmax(flip_loss_, z_flip_ - e.pos.z);
                        if (t_phase_ > 2.5f && std::fabs(e.vel.z) < 0.6f) { z_hold_ = e.pos.z; next2(); }
                    }
                    flip_loss_ = std::fmax(flip_loss_, z_flip_ - e.pos.z);
                    if (t_phase_ > 8.0f) finish(false);
                    break;
                }
                case 5: {
                    const float cy = std::cos(yaw0_), sy = std::sin(yaw0_);
                    const float v_left = -e.vel.x * sy + e.vel.y * cy;
                    Vec3 a{p_[5] * cy + 1.5f * v_left * sy, p_[5] * sy - 1.5f * v_left * cy, az};
                    hover(e, a, 2 * kDeg2Rad, yaw0_, 0, 0.3f, dt, out);
                    if (e.airspeed > p_[6]) next2();
                    if (t_phase_ > 12.0f) finish(false);
                    break;
                }
                case 6: {
                    planeOut(e, eu, dt, out, z_hold_);
                    settle_ = (std::fabs(e.vel.z) < 1.0f && e.airspeed > 1.3f * v_stall && std::fabs(eu.roll) < 0.12f) ? settle_ + dt : 0.0f;
                    if (settle_ > 1.5f) finish(flip_ >= 2 * kPi - 0.6f);
                    if (t_phase_ > 10.0f) finish(false);
                    break;
                }
                default: finish(false);
            }
            return active_;
        }
        // «Харрієр»
        const float kz = p_[2];
        const float az = clampf(kz * (1.5f * (z0_ - e.pos.z) - 1.8f * e.vel.z), -5.0f, 6.0f);
        drift_ = phase_ == 1 ? std::fmax(drift_, std::hypot(e.pos.x - ph_.x, e.pos.y - ph_.y)) : drift_;
        switch (phase_) {
            case 0: {
                const float vh = std::hypot(e.vel.x, e.vel.y);
                Vec3 a{0, 0, az};
                if (vh > 0.3f) { a.x = -p_[0] * e.vel.x / vh; a.y = -p_[0] * e.vel.y / vh; }
                const float pu = p_[1] * kDeg2Rad * clampf(vh / 6.0f, 0.0f, 1.0f);
                hover(e, a, pu, yaw0_, 0, 0.3f, dt, out);
                stop_dist_ = std::hypot(e.pos.x - p0_.x, e.pos.y - p0_.y);
                if (vh < 0.8f) { stopped_ = true; ph_ = e.pos; next(); }
                if (t_phase_ > 15.0f) finish(false);
                break;
            }
            case 1: {
                const float ang = std::fmin(t_phase_ * p_[3], 2 * kPi);
                spun_ = ang;
                const float ff = ang < 2 * kPi ? p_[3] : 0.0f;
                Vec3 a{clampf(1.2f * (ph_.x - e.pos.x) - 1.8f * e.vel.x, -3, 3), clampf(1.2f * (ph_.y - e.pos.y) - 1.8f * e.vel.y, -3, 3), az};
                hover(e, a, 0, yaw0_ + ang, ff, 1.0f - p_[4], dt, out);
                if (ang >= 2 * kPi && std::fabs(wrapPi(eu.yaw - yaw0_)) < 0.15f && std::fabs(e.omega.z) < 0.4f) { spin_done_ = true; next(); }
                if (t_phase_ > 2 * kPi / p_[3] + 6.0f) finish(false);
                break;
            }
            case 2: {
                const float cy = std::cos(yaw0_), sy = std::sin(yaw0_);
                const float v_left = -e.vel.x * sy + e.vel.y * cy;
                Vec3 a{p_[5] * cy + 1.5f * v_left * sy, p_[5] * sy - 1.5f * v_left * cy, az};
                hover(e, a, 2 * kDeg2Rad, yaw0_, 0, 0.3f, dt, out);
                if (e.airspeed > p_[6]) next();
                if (t_phase_ > 12.0f) finish(false);
                break;
            }
            case 3: {
                planeOut(e, eu, dt, out, z0_);
                settle_ = (std::fabs(e.vel.z) < 1.0f && e.airspeed > 1.3f * v_stall && std::fabs(eu.roll) < 0.12f) ? settle_ + dt : 0.0f;
                if (settle_ > 1.5f) finish(stopped_ && spin_done_);
                if (t_phase_ > 10.0f) finish(false);
                break;
            }
            default: finish(false);
        }
        return active_;
    }
};

}  // namespace fc
