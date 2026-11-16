// tests/test_ai.h — тести ШІ-співпроцесора (ai/): нейромережа проти еталона numpy, квантування, протокол, сторож,
// ознаки вібрацій / приводів / батареї й наскрізна діагностика на синтетичному польоті. Від корпусу апарата не залежать.
// Підключається з tests/test_main.cpp (там визначено CHECK і SECTION).
#pragma once
#include "../ai/coprocessor.h"
#include "../ai/weights_demo.h"
#include "../ai/force_model.h"
#include "../sim/qp_tricks.h"
#include "../sim/portable_rng.h"

static void testAi() {
    SECTION("ШІ-співпроцесор: нейромережа, квантування, протокол, сторож, діагностика");
#ifdef AI_HAS_FORCE_WEIGHTS
    {   // 0. Навчена модель сил (tools/ai_train.py): 8-бітна мережа в C++ відповідає еталону numpy
        const auto mq = ai_w_force::modelInt8();
        double e = 0, ymax = 0;
        for (int t = 0; t < ai_w_force::kTests; ++t) {
            float y[ai_w_force::kOutputs];
            mq.forward(ai_w_force::TEST_X + t * ai_w_force::kInputs, y);
            for (int o = 0; o < ai_w_force::kOutputs; ++o) {
                const double ref = ai_w_force::TEST_Y[t * ai_w_force::kOutputs + o];
                e = std::fmax(e, std::fabs(y[o] - ref)); ymax = std::fmax(ymax, std::fabs(ref));
            }
        }
        std::printf("  модель сил %d-64-64-3 (%zu параметрів): int8 у C++ проти numpy — похибка %.3f частки ваги\n", ai_w_force::kInputs, mq.params(), e);
        // критерій: похибка квантування має бути меншою за половину власної похибки мережі (RMS ≈ 1.5 Н ≈ 0.064 ваги на перевірочних польотах)
        CHECK(e < 0.032, "модель сил int8: похибка %.3f частки ваги", e);
    }
    {   // Висіння у вітрі 6 м/с з поривами (польоти, яких мережа не бачила): з порадами ШІ висота тримається щонайменше на 10% краще (модель сил навчена ще без висіння на крилі),
        // горизонталь — не гірше (+10% допуск); без аварій. Поправка — лише вертикальна і до ≈ 9 м/с повітряної швидкості (там мережа точна);
        // з гвинтами в косому потоці (елемент лопаті) горизонтальна поправка й поправка на 9 м/с вже не покращували. Повне порівняння — tools/ai_eval.cpp.
        const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
        const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
        double alt[3] = {0, 0, 0}, hor[3] = {0, 0, 0}; bool crash = false;
        for (unsigned seed : {900u, 901u})
            for (int mode : {0, 2}) {
                QpFlightRig rig(a, tb, seed);
                rig.noise = SensorNoise::realistic();   // повний бортовий ланцюжок датчиків
                WindConfig w; w.mean20ft = Vec3d{6.0, 2.4, 0}; rig.setWind(w, seed + 77);
                fc::QpStep tk; tk.type = fc::QpStep::Takeoff; tk.p = {0, 0, 30}; rig.setMission({tk});
                rig.ai_mode = mode;
                double sa = 0, sh = 0, n = 0;
                rig.run(150, [&](const QpFlightRig& g) {
                    if (g.sim.time() > 40) { const Vec3d p = g.sim.state().pos; const double dz = p.z - g.sim.derived().cg_z - 30; sa += dz * dz; sh += p.x * p.x + p.y * p.y; n += 1; }
                    return false;
                });
                alt[mode] += std::sqrt(sa / std::fmax(n, 1)) / 2; hor[mode] += std::sqrt(sh / std::fmax(n, 1)) / 2;
                crash = crash || rig.sim.crashed();
            }
        std::printf("  висіння, вітер 6 м/с: висота RMS %.2f → %.2f м, горизонталь RMS %.2f → %.2f м (без ШІ → з ШІ)\n", alt[0], alt[2], hor[0], hor[2]);
        CHECK(!crash && alt[2] < 0.9 * alt[0] && hor[2] < 1.1 * hor[0], "поради ШІ мають покращувати висіння у вітрі");
    }
#endif
    {   // 1. MLP: дійсні ваги збігаються з numpy, 8-бітні — у межах 2% діапазону виходу
        const auto mf = ai_w_demo::modelFloat();
        const auto mq = ai_w_demo::modelInt8();
        double ef = 0, eq = 0, ymax = 0;
        ai::OpCounter ops;
        for (int t = 0; t < ai_w_demo::kTests; ++t) {
            float yf[ai_w_demo::kOutputs], yq[ai_w_demo::kOutputs];
            mf.forward(ai_w_demo::TEST_X + t * ai_w_demo::kInputs, yf, &ops);
            mq.forward(ai_w_demo::TEST_X + t * ai_w_demo::kInputs, yq);
            for (int o = 0; o < ai_w_demo::kOutputs; ++o) {
                const double ref = ai_w_demo::TEST_Y[t * ai_w_demo::kOutputs + o];
                ef = std::fmax(ef, std::fabs(yf[o] - ref)); eq = std::fmax(eq, std::fabs(yq[o] - ref)); ymax = std::fmax(ymax, std::fabs(ref));
            }
        }
        std::printf("  MLP 12-32-32-6 (%zu параметрів, %llu множень на прохід): похибка float %.1e, int8 %.2f%% від діапазону виходу\n",
                    mf.params(), (unsigned long long)(ops.macs / ai_w_demo::kTests), ef, 100 * eq / ymax);
        CHECK(ef < 1e-4, "MLP float не збігається з numpy: %.2e", ef);
        CHECK(eq < 0.02 * ymax, "MLP int8: похибка %.3f при діапазоні %.3f", eq, ymax);
        float h[ai_w_demo_gru::kHidden] = {};
        const ai::GruF g = ai_w_demo_gru::cell();
        for (int s = 0; s < ai_w_demo_gru::kSteps; ++s) ai::step(g, ai_w_demo_gru::SEQ_X + s * ai_w_demo_gru::kInputs, h);
        double eg = 0;
        for (int j = 0; j < ai_w_demo_gru::kHidden; ++j) eg = std::fmax(eg, std::fabs(h[j] - ai_w_demo_gru::H_END[j]));
        CHECK(eg < 1e-4, "GRU не збігається з numpy: %.2e", eg);
    }
    {   // 2. Протокол: кадри в потоці зі сміттям і довільним шматкуванням; битий кадр відкидається, наступний — цілий
        ai::TelemetryFrame t; t.t_ms = 12345; t.rpm[2] = 6012.5f; t.tilt_meas[3] = 1.2f; t.v_bus = 23.9f;
        ai::AdviceFrame adv; adv.seq = 7; adv.force[1] = -1.25f; adv.status = 2;
        ai::MessageFrame msg; msg.node = 3; msg.code = uint16_t(ai::Code::PropImbalance); msg.value = 2.7f;
        std::vector<uint8_t> s = {0x00, 0xA5, 0x13, 0x5A};
        ai::encode(ai::FrameType::Telemetry, t, s);
        s.push_back(0xFF);
        ai::encode(ai::FrameType::Advice, adv, s);
        const size_t bad_at = s.size() + 10;
        ai::encode(ai::FrameType::Telemetry, t, s);   // цей зіпсуємо
        ai::encode(ai::FrameType::Message, msg, s);
        s[bad_at] ^= 0x40;
        ai::FrameParser p;
        int n_t = 0, n_a = 0, n_m = 0; bool same = true;
        for (size_t i = 0; i < s.size(); i += 7)
            p.feed(s.data() + i, std::min<size_t>(7, s.size() - i), [&](const ai::FrameParser::Frame& f) {
                ai::TelemetryFrame t2; ai::AdviceFrame a2; ai::MessageFrame m2;
                if (f.type == ai::FrameType::Telemetry && ai::FrameParser::as(f, t2)) { ++n_t; same = same && t2.t_ms == 12345 && t2.rpm[2] == 6012.5f && t2.tilt_meas[3] == 1.2f; }
                if (f.type == ai::FrameType::Advice && ai::FrameParser::as(f, a2)) { ++n_a; same = same && a2.seq == 7 && a2.force[1] == -1.25f; }
                if (f.type == ai::FrameType::Message && ai::FrameParser::as(f, m2)) { ++n_m; same = same && m2.node == 3 && m2.value == 2.7f; }
            });
        std::printf("  протокол: телеметрія %zu Б, пачка IMU %zu Б (≈ %.0f КБ/с з 100 Гц кожна, UART 921600 — 92 КБ/с); прийнято %u, битих %u\n",
                    sizeof(ai::TelemetryFrame) + 6, sizeof(ai::ImuBatchFrame) + 6, (sizeof(ai::TelemetryFrame) + sizeof(ai::ImuBatchFrame) + 12) * 100 / 1024.0, p.good, p.bad_crc);
        CHECK(n_t == 1 && n_a == 1 && n_m == 1 && p.bad_crc == 1 && same, "розбір кадрів: телеметрія %d, порада %d, повідомлення %d, битих %u", n_t, n_a, n_m, p.bad_crc);
    }
    {   // 3. Сторож: межа величини, швидкість зміни, таймаут 50 мс, «тінь», низька впевненість, NaN
        ai::AiGuard g; g.mode = ai::AiGuard::On;
        ai::AdviceFrame a; a.status = 2; a.confidence = 0.9f; a.force[0] = 10.0f;
        float f[3], m[3], t = 0;
        g.onAdvice(a, t); t += 0.01f; g.output(t, 0.1f, f, m);   // свіжа порада, крок 0.1 с: за крок не більше 20 Н/с · 0.1 с
        const float f_ramp = f[0];
        for (int i = 0; i < 20; ++i) { g.onAdvice(a, t); t += 0.02f; g.output(t, 0.02f, f, m); }
        const float f_lim = f[0];
        for (int i = 0; i < 30; ++i) { t += 0.02f; g.output(t, 0.02f, f, m); }   // порад немає 0.6 с
        const float f_timeout = f[0];
        ai::AiGuard sh; sh.mode = ai::AiGuard::Shadow; sh.onAdvice(a, 0); sh.output(0.01f, 1.0f, f, m);
        const float f_shadow = f[0];
        ai::AiGuard lc; lc.mode = ai::AiGuard::On; ai::AdviceFrame bad = a; bad.confidence = 0.1f; lc.onAdvice(bad, 0); lc.output(0.01f, 1.0f, f, m);
        const float f_conf = f[0];
        ai::AiGuard nn; nn.mode = ai::AiGuard::On; ai::AdviceFrame nanv = a; nanv.force[0] = std::nanf(""); nn.onAdvice(nanv, 0); nn.output(0.01f, 1.0f, f, m);
        std::printf("  сторож: за 0.1 с %.1f Н (межа швидкості 2 Н), далі %.2f Н (межа 4.5 Н), без порад → %.2f Н; тінь %.1f, низька впевненість %.1f, NaN %.1f\n",
                    f_ramp, f_lim, f_timeout, f_shadow, f_conf, f[0]);
        CHECK(f_ramp <= 2.0f + 1e-4f && std::fabs(f_lim - 4.5f) < 1e-3f && f_timeout == 0.0f && f_shadow == 0.0f && f_conf == 0.0f && f[0] == 0.0f,
              "сторож пропустив зайве");
    }
    {   // 4. Вібрації: дисбаланс мотора 3 (1×) серед інших моторів на близьких обертах і шуму
        ai::OrderTracker ot;
        std::mt19937 rng(5); prng::normal<float> nz(0.0f, 0.5f);
        const float rpm[4] = {5800, 6000, 6200, 6100}, amp[4] = {0.2f, 0.2f, 1.0f, 0.2f};
        double ph[4] = {0, 0.5, 1.0, 1.5};
        for (int i = 0; i < 8000; ++i) {
            float a[3] = {nz(rng), nz(rng), 9.81f + nz(rng)};
            for (int mo = 0; mo < 4; ++mo) {
                ph[mo] += 2 * 3.14159265358979 * rpm[mo] / 60.0 * 0.001;
                a[0] += amp[mo] * float(std::cos(ph[mo])); a[1] += amp[mo] * float(std::sin(ph[mo]));   // відцентрова сила дисбалансу обертається в площині диска
            }
            ot.update(a, rpm, 0.001f);
        }
        // обертовий вектор a·(cos, sin) у двох осях: сумарна амплітуда за двома осями = a·√2
        const float k = 1.0f / std::sqrt(2.0f);
        std::printf("  вібрації 1× (м/с²): мотори %.2f %.2f %.2f %.2f (задано 0.2 0.2 1.0 0.2)\n", k * ot.amplitude(0, 1), k * ot.amplitude(1, 1), k * ot.amplitude(2, 1), k * ot.amplitude(3, 1));
        CHECK(std::fabs(k * ot.amplitude(2, 1) - 1.0f) < 0.15f && k * ot.amplitude(0, 1) < 0.35f && k * ot.amplitude(3, 1) < 0.35f, "відстеження порядків не розділило мотори");
    }
    {   // 5. Привід нахилу: запізнення 0.10 с і люфт 1.5°
        ai::ServoMonitor sm; sm.dt = 0.005f;
        const double d2r = 3.14159265358979 / 180;
        std::vector<double> hist;
        double meas = 0;
        const double bl = 1.5 * d2r;
        for (int i = 0; i < 4000; ++i) {
            const double tt = i * 0.005;
            const double cmd = 10 * d2r * std::sin(2 * 3.14159265358979 * 0.3 * tt) + 4 * d2r * std::sin(2 * 3.14159265358979 * 0.7 * tt + 1);
            hist.push_back(cmd);
            const double cd = hist.size() > 20 ? hist[hist.size() - 21] : 0.0;   // 20 кроків = 0.10 с
            if (cd - meas > bl / 2) meas = cd - bl / 2; else if (meas - cd > bl / 2) meas = cd + bl / 2;   // мертва зона
            sm.update(float(cmd), float(meas));
        }
        std::printf("  привід нахилу: запізнення %.3f с (задано 0.100), люфт %.2f° (задано 1.50), оцінок %d\n", sm.lag_s, sm.backlash_rad / d2r, sm.estimates);
        CHECK(std::fabs(sm.lag_s - 0.10f) < 0.02f && std::fabs(sm.backlash_rad / d2r - 1.5) < 0.3, "оцінка приводу: %.3f с, %.2f°", sm.lag_s, sm.backlash_rad / d2r);
    }
    {   // 6. Батарея: R = 0.050 Ом
        ai::BatteryRls b;
        std::mt19937 rng(9); prng::uniform<float> cur(5, 40); prng::normal<float> nz(0, 0.02f);
        for (int i = 0; i < 3000; ++i) { const float I = cur(rng); b.update(24.5f - 0.05f * I + nz(rng), I); }
        std::printf("  батарея: ЕРС %.2f В (24.50), опір %.4f Ом (0.0500)\n", b.E, b.R);
        CHECK(std::fabs(b.R - 0.05f) < 0.005f && std::fabs(b.E - 24.5f) < 0.1f, "оцінка батареї");
    }
    // 7. Наскрізно: синтетичний політ 10 хв через протокол. Норма 1 хв, далі — несправність; без несправності — жодного повідомлення.
    struct Fault { int prop = -1; int servo = -1; int loose = -1; };
    auto fly = [](Fault fault, unsigned seed) {
        ai::AiCoprocessor co;
        std::vector<ai::MessageFrame> got;
        ai::FrameParser parser;
        std::mt19937 rng(seed); prng::normal<float> nz(0, 1);
        double ph[4] = {0, 1, 2, 3}, meas[4] = {0, 0, 0, 0};
        std::vector<double> cmd_hist[4];
        const double d2r = 3.14159265358979 / 180, onset = 240;
        ai::ImuBatchFrame imu;
        for (int ms = 0; ms < 600000; ++ms) {
            const double t = ms * 1e-3;
            float rpm[4];
            for (int m = 0; m < 4; ++m) rpm[m] = float(6000 + 150 * (m - 1.5) + 80 * std::sin(0.3 * t + m));
            float a[3] = {0.4f * nz(rng), 0.4f * nz(rng), 9.81f + 0.4f * nz(rng)};
            for (int m = 0; m < 4; ++m) {
                ph[m] += 2 * 3.14159265358979 * rpm[m] / 60.0 * 1e-3;
                float amp = 0.15f;
                if (fault.prop == m && t > onset) amp = float(0.15 + 0.75 * std::fmin(1.0, (t - onset) / 20));
                a[0] += amp * float(std::cos(ph[m])); a[1] += amp * float(std::sin(ph[m]));
                if (fault.loose == m && t > onset) {   // люфт кріплення: удари раз на два оберти (0.5×) і подвійна частота
                    const float g = float(std::fmin(1.0, (t - onset) / 20));
                    a[2] += 0.6f * g * float(std::cos(0.5 * ph[m])) + 0.3f * g * float(std::cos(2 * ph[m] + 0.4));
                }
            }
            const int k = ms % 10;
            for (int ax = 0; ax < 3; ++ax) imu.acc[k][ax] = int16_t(std::lround(a[ax] / imu.acc_scale));
            if (k == 0) imu.t_ms = uint32_t(ms);
            std::vector<uint8_t> bytes;
            if (k == 9) {
                ai::encode(ai::FrameType::ImuBatch, imu, bytes);
                ai::TelemetryFrame tf; tf.t_ms = uint32_t(ms); tf.flags = 1; tf.v_bus = 24.0f - 0.04f * 30.0f;
                tf.i_bus = float(30 + 5 * std::sin(0.5 * t)); tf.v_bus = 24.0f - 0.04f * tf.i_bus + 0.01f * nz(rng);
                for (int m = 0; m < 4; ++m) {
                    tf.rpm[m] = rpm[m] * (1 + 0.003f * nz(rng));
                    const float w = rpm[m] / 1000.0f;
                    tf.motor_i[m] = 0.035f * w * w * w * (1 + 0.02f * nz(rng));
                    const double c = 5 * d2r * std::sin(2 * 3.14159265358979 * 0.5 * t + m);
                    cmd_hist[m].push_back(c);
                    const double cd = cmd_hist[m].size() > 4 ? cmd_hist[m][cmd_hist[m].size() - 5] : 0.0;   // 0.04 с — справний привід
                    const double bl = (fault.servo == m && t > onset) ? 2.0 * d2r : 0.2 * d2r;
                    if (cd - meas[m] > bl / 2) meas[m] = cd - bl / 2; else if (meas[m] - cd > bl / 2) meas[m] = cd + bl / 2;
                    tf.tilt_cmd[m] = float(c); tf.tilt_meas[m] = float(meas[m] + 0.001 * nz(rng));
                }
                ai::encode(ai::FrameType::Telemetry, tf, bytes);
                parser.feed(bytes.data(), bytes.size(), [&](const ai::FrameParser::Frame& f) {
                    ai::ImuBatchFrame ib; ai::TelemetryFrame tt;
                    if (f.type == ai::FrameType::ImuBatch && ai::FrameParser::as(f, ib)) co.onImu(ib);
                    if (f.type == ai::FrameType::Telemetry && ai::FrameParser::as(f, tt)) co.onTelemetry(tt);
                });
            }
        }
        return co.messages();
    };
    {
        const auto ok = fly(Fault{}, 31);
        std::printf("  справний політ 10 хв: повідомлень %zu\n", ok.size());
        for (const auto& m : ok) std::printf("      хибне: %s\n", ai::AiCoprocessor::text(m).c_str());
        CHECK(ok.empty(), "справний апарат — жодного повідомлення, а є %zu", ok.size());
        const auto pr = fly(Fault{2, -1}, 32);
        bool found = false; double t_det = 0;
        for (const auto& m : pr) { std::printf("      t=%.0f с: %s\n", m.t_ms * 1e-3, ai::AiCoprocessor::text(m).c_str()); if (m.node == 3 && m.code == uint16_t(ai::Code::PropImbalance)) { found = true; t_det = m.t_ms * 1e-3; } }
        std::printf("  дисбаланс гвинта мотора 3 з t = 240 с (наростає 20 с): виявлено %s через %.0f с\n", found ? "так" : "НІ", t_det - 240);
        CHECK(found && pr.size() == 1 && t_det - 240 < 30, "дисбаланс мотора 3: знайдено %d, повідомлень %zu", int(found), pr.size());
        const auto sv = fly(Fault{-1, 1}, 33);
        bool sfound = false;
        for (const auto& m : sv) { std::printf("      t=%.0f с: %s\n", m.t_ms * 1e-3, ai::AiCoprocessor::text(m).c_str()); if (m.node == 2 && m.code == uint16_t(ai::Code::ServoBacklash)) sfound = true; }
        CHECK(sfound && sv.size() == 1, "люфт гондоли 2: знайдено %d, повідомлень %zu", int(sfound), sv.size());
        const auto lo = fly(Fault{-1, -1, 0}, 34);
        bool lfound = false;
        for (const auto& m : lo) { std::printf("      t=%.0f с: %s\n", m.t_ms * 1e-3, ai::AiCoprocessor::text(m).c_str()); if (m.node == 1 && m.code == uint16_t(ai::Code::PropLoose)) lfound = true; }
        CHECK(lfound && lo.size() == 1, "люфт гвинта мотора 1: знайдено %d, повідомлень %zu", int(lfound), lo.size());
    }
}

// Пілотажні трюки (fc/qp_tricks.h, sim/qp_tricks.h): пілот трюку з розумними параметрами виконує мертву петлю
// й виходить зі звалювання без аварії; оцінка спроби розрізняє вдалу й невдалу (навчання — tools/ai_tricks.cpp або GUI → «Трюки»).
static void testTricks() {
    SECTION("Трюки: петля, звалювання, бочка, хамерхед, «Харрієр», кобра, сальто, авіашоу (пілот трюку, оцінка спроби)");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    TrickTrainer tr(a, tb);
    const float loop_u[fc::kTrickParams] = {0.0f, 0.27f, 0.57f, 1.0f, 0.42f, 0.08f, 0.4f};
    const auto lp = tr.run(fc::Trick::Loop, loop_u);
    std::printf("  петля: %s\n", lp.r.text().c_str());
    CHECK(lp.r.completed && !lp.r.crashed && lp.r.loop_deg > 330 && lp.r.max_g < 7, "петля з розумними параметрами має вдаватись");
    float stall_u[fc::kTrickParams] = {0.80f, 0.26f, 0.37f, 0.81f, 0.70f, 0.41f, 0.85f};
    TrickTrainer::load("ai/tricks_learned.ini", fc::Trick::Stall, stall_u);   // навчені параметри (підібрані вручну колись — запасні)
    const auto st = tr.run(fc::Trick::Stall, stall_u);
    std::printf("  звалювання: %s\n", st.r.text().c_str());
    CHECK(st.r.stalled && st.r.completed && !st.r.crashed && st.r.max_alpha_deg > 15, "звалювання з виходом має вдаватись");
    const float bad_u[fc::kTrickParams] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // занадто повільно, без газу — петля не виходить
    const auto bad = tr.run(fc::Trick::Loop, bad_u);
    std::printf("  петля з поганими параметрами: %s\n", bad.r.text().c_str());
    CHECK(bad.r.score < lp.r.score, "оцінка має розрізняти вдалу й невдалу спробу");
    const float roll_u[fc::kTrickParams] = {0.70f, 0.91f, 0.25f, 0.81f, 0.37f, 0.0f, 0.30f};
    const auto rl = tr.run(fc::Trick::Roll, roll_u);
    std::printf("  векторна бочка: %s\n", rl.r.text().c_str());
    CHECK(rl.r.completed && !rl.r.crashed && rl.r.roll_deg > 330 && std::fabs(rl.r.heading_deg) < 30, "бочка з гондолами має вдаватись без зміни курсу");
    const float hh_u[fc::kTrickParams] = {0.28f, 0.61f, 0.45f, 0.39f, 0.50f, 0.24f, 0.36f};
    const auto hh = tr.run(fc::Trick::Hammerhead, hh_u);
    std::printf("  хамерхед: %s\n", hh.r.text().c_str());
    CHECK(hh.r.completed && !hh.r.crashed && std::fabs(hh.r.heading_deg) > 150, "хамерхед: розворот на вертикалі, вихід у зворотному напрямку");
    const float hr_u[fc::kTrickParams] = {0.64f, 0.38f, 0.90f, 0.33f, 0.73f, 0.69f, 0.34f};
    const auto hr = tr.run(fc::Trick::Harrier, hr_u);
    std::printf("  «Харрієр»: %s\n", hr.r.text().c_str());
    CHECK(hr.r.completed && !hr.r.crashed && hr.r.spun_deg > 350 && hr.r.drift < 3 && hr.r.stop_dist < 60, "«Харрієр»: зупинка гондолами, пірует на місці, назад у літак");
    const auto cb = tr.run(fc::Trick::Cobra, defaultTrickParams(fc::Trick::Cobra));
    std::printf("  кобра: %s\n", cb.r.text().c_str());
    CHECK(cb.r.completed && !cb.r.crashed && cb.r.nose_max_deg > 85, "кобра: ніс вище 85° і вихід у політ");
    const auto fl = tr.run(fc::Trick::Flip, defaultTrickParams(fc::Trick::Flip));
    std::printf("  сальто з висіння: %s\n", fl.r.text().c_str());
    CHECK(fl.r.completed && !fl.r.crashed && fl.r.flip_deg > 300 && fl.r.flip_loss < 3, "сальто: повний оберт, ловля без великої втрати висоти, назад у літак");
    const auto tu = tr.run(fc::Trick::Tumble, defaultTrickParams(fc::Trick::Tumble));
    std::printf("  «втрата керування»: %s\n", tu.r.text().c_str());
    CHECK(tu.r.completed && !tu.r.crashed && tu.r.tumble_rot > 0.7, "«втрата керування»: кульбіти з вимкненими моторами, порятунок і назад у літак");
    {   // авіашоу: усі трюки поспіль, між ними автопілот вирівнює політ
        auto rig = tr.cruiseRig();
        AirShow show;
        for (int k = 0; k < fc::kTrickCount; ++k) for (int i = 0; i < fc::kTrickParams; ++i) show.u[k][i] = defaultTrickParams(fc::Trick(k))[i];
        show.begin(*rig, airShowProgram());
        const double t0 = rig->sim.time();
        while (show.running() && rig->sim.time() - t0 < 400 && !rig->sim.crashed()) { rig->step(); show.after(*rig); }
        int ok = 0; for (const auto& d : show.results) ok += d.r.completed;
        std::printf("  авіашоу: %d з %zu трюків виконано за %.0f с, висота в кінці %.0f м\n", ok, show.results.size(), rig->sim.time() - t0, rig->sim.state().pos.z);
        CHECK(!rig->sim.crashed() && show.results.size() == airShowProgram().size() && ok == int(airShowProgram().size()), "авіашоу: усі трюки без аварії");
    }
    const float zero_u[fc::kTrickParams] = {};
    CHECK(tr.run(fc::Trick::Harrier, zero_u).r.score < hr.r.score && tr.run(fc::Trick::Hammerhead, zero_u).r.score < hh.r.score, "оцінка нових трюків розрізняє вдалу й невдалу спробу");
}


// Вантаж, про який автопілот не знає: маса оцінюється на зльоті й у висінні, висота у висінні тримається, посадка без аварії
static void testPayload() {
    SECTION("Вантаж: автопілот сам оцінює масу (0, 0.6, 1.2 кг), висіння й посадка");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    for (double kg : {0.0, 0.6, 1.2}) {
        double rms[2] = {0, 0}, mest = 0, mtrue = 0; bool ok = true;
        for (int adapt = 0; adapt < 2; ++adapt) for (unsigned seed : {5u, 6u}) {   // два польоти на кожен варіант — висіння в поривах шумне
            QpFlightRig r(a, tb, seed);
            r.noise = SensorNoise::realistic();   // повний бортовий ланцюжок датчиків
            WindConfig w; w.mean20ft = Vec3d{4, 1.5, 0}; w.turbulence = 1.5; r.setWind(w, seed - 2);
            using S = fc::QpStep; S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {800, 0, 40}; hv.hold_s = 15; S ld; ld.type = S::Land;
            r.setMission({tk, hv, ld});
            r.sim.setPayload(kg); r.ctl.mass_adapt = adapt; r.ai_mode = 2;
            double z2 = 0; int n = 0; bool reached = false;   // лише утримання висоти: після переходу апарат ще знижується з 60 до 40 м — це не про масу
            r.run(400, [&](const QpFlightRig& g) {
                const double dz = g.sim.state().pos.z - g.sim.derived().cg_z - 40;
                if (g.ctl.phase() == fc::QpPhase::VtolHold && std::fabs(dz) < 0.5) reached = true;
                if (g.ctl.phase() == fc::QpPhase::VtolHold && reached) { z2 += dz * dz; ++n; }
                return g.ctl.phase() == fc::QpPhase::Landed;
            });
            rms[adapt] += std::sqrt(z2 / std::max(n, 1)) / 2;
            ok = ok && !r.sim.crashed() && r.ctl.phase() == fc::QpPhase::Landed;
            if (adapt) { mest += r.ctl.massEstimate() / 2; mtrue = r.sim.mass(); }
        }
        std::printf("  вантаж %.1f кг: маса %.2f кг, оцінка %.2f (%+.0f%%); висота у висінні RMS %.2f → %.2f м (без оцінки → з оцінкою)\n", kg, mtrue, mest, 100 * (mest / mtrue - 1), rms[0], rms[1]);
        CHECK(ok, "з вантажем %.1f кг — політ і посадка без аварії", kg);
        CHECK(std::fabs(mest / mtrue - 1) < 0.2, "оцінка маси в межах 20%%");
        CHECK(rms[1] < 0.3 && rms[1] <= rms[0] + 0.05, "з оцінкою маси висота тримається не гірше");
    }
}
