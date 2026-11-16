#!/usr/bin/env python3
"""ai_pipeline.py — повний цикл навчання моделі сил на своєму комп'ютері: дані → навчання → (порівняння).

Запускається з train_ai.bat (він спершу збирає ai_datagen.exe і ai_eval.exe), або вручну з кореня проєкту:
    python tools/ai_pipeline.py --flights 120 --jobs 8 [--epochs 40] [--keep]

--flights  скільки польотів симулювати (більше — точніше: 120 ≈ 5.5 год польоту; 300–500 — помітно краще)
--jobs     скільки процесів одночасно (типово — кількість ядер процесора)
--epochs   скільки разів пройти по всіх даних під час навчання
--keep     не генерувати дані заново, а взяти вже наявні data/*.bin (наприклад, щоб навчити з іншими параметрами)

Потрібні: Python 3.9+ і numpy (pip install numpy). PyTorch і відеокарта не потрібні.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = ".exe" if os.name == "nt" else ""


def aircraft_variant():
    """Активний апарат (DRONE_AIRCRAFT або aircraft/active.txt) → (ai.force_weights з його ini, вага m·g)."""
    path = os.environ.get("DRONE_AIRCRAFT", "")
    if not path and os.path.exists("aircraft/active.txt"):
        name = open("aircraft/active.txt", encoding="utf-8").readline().strip()
        if name:
            path = name if ("/" in name or "\\" in name) else "aircraft/" + name
    path = path or "aircraft/tiltrotor.ini"
    variant, mass, sec = "", 2.386, ""
    for line in open(path, encoding="utf-8"):
        line = line.split("#")[0].strip()
        if line.startswith("["):
            sec = line.strip("[]")
        elif "=" in line:
            k, v = [x.strip() for x in line.split("=", 1)]
            if sec == "ai" and k == "force_weights":
                variant = v
            if sec == "targets" and k == "takeoff_mass_kg":
                mass = float(v.split()[0])
    print(f"Апарат: {path}, набір ваг: {'основний' if not variant else variant}, маса {mass:.3f} кг")
    return variant, mass * 9.80665


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flights", type=int, default=120)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--seed", type=int, default=100)
    a = ap.parse_args()
    os.chdir(ROOT)
    os.makedirs("data", exist_ok=True)
    gen = os.path.join(ROOT, "ai_datagen" + EXE)

    if not a.keep:
        if not os.path.exists(gen):
            sys.exit("немає ai_datagen" + EXE + " — запустіть train_ai.bat (він збирає) або зберіть tools/ai_datagen.cpp")
        for f in glob.glob("data/part_*.bin") + glob.glob("data/part_*.log"):
            os.remove(f)
        jobs = max(1, min(a.jobs, a.flights))
        per = [a.flights // jobs + (1 if i < a.flights % jobs else 0) for i in range(jobs)]
        procs, seed = [], a.seed
        print(f"1/3 Генерація даних: {a.flights} польотів у {jobs} процесах (один політ ≈ 25–40 с на ядро)…")
        t0 = time.time()
        for i, n in enumerate(per):
            log = open(f"data/part_{i}.log", "w", encoding="utf-8")
            procs.append((subprocess.Popen([gen, str(n), str(seed), f"data/part_{i}.bin"], stdout=log, stderr=subprocess.STDOUT), log))
            seed += 1000   # різні seed — різні вітри й місії в кожному процесі
        while any(p.poll() is None for p, _ in procs):
            time.sleep(10)
            done = sum(open(f"data/part_{i}.log", encoding="utf-8", errors="ignore").read().count("політ") for i in range(jobs))
            el = time.time() - t0
            eta = el / max(done, 1) * (a.flights - done)
            print(f"   готово {done}/{a.flights} польотів, минуло {el / 60:.1f} хв, лишилось ≈ {eta / 60:.0f} хв", end="\r")
        print()
        for p, log in procs:
            log.close()
            if p.returncode != 0:
                sys.exit("генератор завершився з помилкою — див. data/part_*.log")
    files = sorted(glob.glob("data/part_*.bin"))
    if not files:
        sys.exit("немає даних data/part_*.bin")

    variant, mg = aircraft_variant()
    out = "ai/weights_force.h" if not variant else f"ai/weights_force_{variant}.h"
    print(f"2/3 Навчання на {len(files)} файлах, {a.epochs} епох…")
    subprocess.check_call([sys.executable, "tools/ai_train.py", *files, "-o", out, "--epochs", str(a.epochs),
                           "--name", "force" if not variant else "force_" + variant, "--mg", f"{mg:.2f}"])

    print(f"3/3 Нові ваги — {out}. Перезберіть симулятор (build.bat / build_gui.bat), щоб він їх використовував.")


if __name__ == "__main__":
    main()
