// tools/ai_tricks.cpp — навчання пілотажних трюків на своєму комп'ютері (без вікна; те саме робить GUI → «Трюки»).
// Збирання: cl /std:c++17 /O2 /EHsc /utf-8 tools\ai_tricks.cpp /Fe:ai_tricks.exe   (або g++ -std=c++17 -O2 -o ai_tricks tools/ai_tricks.cpp)
// Запуск:   ai_tricks [трюк|all|both] [спроб, типово 100] [seed] [new|cont — з нуля / продовжити з навченого] [ядер, 0 — усі] [умов вітру/вантажу, 0 — лише штиль]   (both — петля й звалювання, all — усі)
// Результат: ai/tricks_learned.ini (параметри найкращої спроби) і data/tricks_<трюк>.csv (історія навчання: спроба, оцінка, найкраща).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "../sim/console_utf8.h"
#include "../sim/qp_tricks.h"

using namespace sim;

int main(int argc, char** argv) {
    const std::string which = argc > 1 ? argv[1] : "all";
    const int episodes = argc > 2 ? std::atoi(argv[2]) : 100;
    const unsigned seed = argc > 3 ? unsigned(std::atoi(argv[3])) : 1u;
    const bool fresh = argc > 4 && std::string(argv[4]) == "new";   // «new» — з нуля, інакше — продовжити з ai/tricks_learned.ini
    const int threads = argc > 5 ? std::atoi(argv[5]) : 0;          // ядер (0 — усі; 1 — тихо, на одному ядрі)
    const int robust = argc > 6 ? std::atoi(argv[6]) : 0;           // умов з вітром і вантажем, у яких перевіряється кожна спроба (0 — лише штиль)
    const AircraftFile a = AircraftFile::load(activeAircraftPath());
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    const std::string out_path = aiFile(a, "ai/tricks_learned.ini"), in_path = aiFile(a, "ai/tricks_learned.ini", true), var = aiVariant(a);
    std::printf("Апарат: %s; навчене — %s\n", activeAircraftPath().c_str(), out_path.c_str());
    for (int ti = 0; ti < fc::kTrickCount; ++ti) {
        const fc::Trick t = fc::Trick(ti);
        const bool pick = which == "all" || which == fc::trickKey(t) || (which == "both" && ti < 2);
        if (!pick) continue;
        std::printf("=== Навчання: %s, %d спроб ===\n", fc::trickName(t), episodes);
        TrickTrainer tr(a, tb);
        tr.threads = threads;
        tr.robust = robust;
        tr.on_episode = [](const TrickTrainer::Episode& e, const TrickTrainer::Episode& best) {
            std::printf("спроба %3d: оцінка %6.1f %s | найкраща %6.1f (спроба %d)\n", e.index, e.r.score,
                        e.r.crashed ? "АВАРІЯ   " : (e.r.completed ? "виконано " : "не вийшло"), best.r.score, best.index);
            std::fflush(stdout);
        };
        float prev[fc::kTrickParams]; double prev_score = 0;
        const bool cont = !fresh && TrickTrainer::load(in_path, t, prev, &prev_score);
        if (cont) std::printf("Продовжую з навченого раніше (оцінка тоді %.1f): спершу перелітаю його, далі шукаю поруч\n", prev_score);
        else std::printf("Навчання з нуля\n");
        const TrickTrainer::Episode best = tr.train(t, episodes, seed, cont ? prev : nullptr);
        std::printf("Найкраща спроба %d (0 — навчене раніше): %s\nПараметри:\n", best.index, best.r.text().c_str());
        for (int i = 0; i < fc::kTrickParams; ++i) std::printf("  %-32s %.2f\n", fc::trickSpec(t)[i].name, fc::trickValue(t, best.u, i));
        TrickTrainer::save(out_path, t, best.u, best.r.score);
        std::string csv = std::string("data/tricks_") + fc::trickKey(t) + (var.empty() ? "" : "_" + var) + ".csv";
        if (FILE* f = std::fopen(csv.c_str(), "wb")) {
            std::fputs("episode,score,completed,crashed,alt_loss_m,max_g,min_airspeed\n", f);
            for (const auto& e : tr.history) std::fprintf(f, "%d,%.1f,%d,%d,%.2f,%.2f,%.2f\n", e.index, e.r.score, int(e.r.completed), int(e.r.crashed), e.r.alt_loss, e.r.max_g, e.r.min_airspeed);
            std::fclose(f);
        }
        // успішність за поколіннями (по 10 спроб) — видно, чи вчиться
        std::printf("Успішних спроб у кожному десятку: ");
        for (size_t g = 0; g < tr.history.size(); g += 10) {
            int ok = 0;
            for (size_t j = g; j < std::min(g + 10, tr.history.size()); ++j) ok += tr.history[j].r.completed;
            std::printf("%d ", ok);
        }
        std::printf("\nЗбережено %s і %s\n\n", out_path.c_str(), csv.c_str());
    }
    return 0;
}
