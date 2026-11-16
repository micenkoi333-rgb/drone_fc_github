#!/usr/bin/env python3
"""aero_gen.py — офлайн-генерація таблиць профілю cl/cd/cm(α) на −180…180° для aircraft/aero_tables.json.

Запуск вручну (потрібні aerosandbox і neuralfoil; симулятор від Python не залежить, готовий JSON комітиться):
    python tools/aero_gen.py [вихідний_файл]

До зриву — NeuralFoil (xlarge) при заданому Re; після зриву — екстраполяція Вітерни–Корріґана (Viterna), що неперервно
зшивається в точці зриву й дає плоску пластину при α → 90°; для |α| > 90° — дзеркальне відображення через 180°.
Скінченне подовження крила (індукований кут і опір) додає C++-код, таблиці — суто 2D-перерізи.
Джерела й обмеження — docs/AERO_MODEL.md. Cm — відносно чверті хорди, позитивний — ніс угору.
"""
import json
import math
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")

# (ім'я таблиці, профіль, Re, подовження поверхні для CDmax)
TABLES = [
    ("clarky_re3e5", "clarky", 3.0e5, 5.71),      # крило
    ("naca0010_re1p5e5", "naca0010", 1.5e5, 3.47),   # стабілізатор і кілі
    ("sd7037_re2p5e5", "sd7037", 2.5e5, 4.04),       # крило QP-5 TiltQuad v3 (у JSON поки оцінка з Clark Y — див. поле estimated)
]
BLEND_DEG = 6.0        # ширина зшивки NeuralFoil → Вітерна після зриву
NF_RANGE = (-20.0, 30.0)
STEP_FINE, STEP_COARSE = 0.5, 1.0


SMOOTH_SIGMA_DEG = 1.0   # згладжування Гаусом: NeuralFoil на низьких Re дає «сходинку» cl біля α≈0 (ламінарна бульбашка)


def smooth(y, sigma_deg, step_deg):
    """Гаусове згладжування рівномірної послідовності; краї продовжуються сталим значенням."""
    half = int(round(3 * sigma_deg / step_deg))
    k = np.exp(-0.5 * (np.arange(-half, half + 1) * step_deg / sigma_deg) ** 2)
    k /= k.sum()
    padded = np.concatenate([np.full(half, y[0]), y, np.full(half, y[-1])])
    return np.convolve(padded, k, mode="valid")


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def viterna(alpha_rad, alpha_s, cl_s, cd_s, cd_max):
    """Вітерна для α у (0, 90°]; повертає cl, cd. alpha_s > 0 — кут зриву, cl_s, cd_s — значення в ньому."""
    a1 = cd_max / 2
    sa, ca = math.sin(alpha_s), math.cos(alpha_s)
    a2 = (cl_s - cd_max * sa * ca) * sa / (ca * ca)
    b1 = cd_max
    b2 = (cd_s - cd_max * sa * sa) / ca
    s, c = np.sin(alpha_rad), np.cos(alpha_rad)
    cl = a1 * np.sin(2 * alpha_rad) + a2 * c * c / np.maximum(s, 1e-6)
    cd = b1 * s * s + b2 * c
    return cl, np.maximum(cd, 0.0)


def first_peak(scan, cl, direction):
    """Індекс першого локального максимуму cl (direction=+1, від α=0 до +) або мінімуму (−1) — зрив.
    NeuralFoil після зриву може знову «підіймати» cl, тому глобальний максимум брати не можна."""
    win, drop = 8, 0.02   # ±2° і падіння принаймні 0.02 за наступні 2°
    idx = [i for i in range(len(scan)) if (scan[i] > 0 if direction > 0 else scan[i] < 0)]
    idx = idx if direction > 0 else idx[::-1]
    sgn = float(direction)
    for n, i in enumerate(idx):
        j = idx[n + win] if n + win < len(idx) else None
        if j is None:
            break
        near = [sgn * cl[k] for k in idx[max(0, n - win):n + win + 1]]
        if sgn * cl[i] >= max(near) and sgn * cl[i] - sgn * cl[j] >= drop:
            return i
    return idx[int(np.argmax([sgn * cl[k] for k in idx]))]


def plate_cm(cl, cd, alpha_rad):
    cn = cl * np.cos(alpha_rad) + cd * np.sin(alpha_rad)
    s = np.abs(np.sin(alpha_rad))
    # центр тиску від передньої кромки: при обтіканні спереду 0.25 → 0.5 (90°); від задньої кромки (|α| > 90°) 0.75 → 0.5
    xcp = np.where(np.abs(alpha_rad) > math.pi / 2, 0.75 - 0.25 * s, 0.25 + 0.25 * s)
    return -cn * (xcp - 0.25)


def build_table(name, airfoil, re, ar):
    import aerosandbox as asb
    af = asb.Airfoil(airfoil)
    scan = np.arange(NF_RANGE[0], NF_RANGE[1] + 1e-9, 0.25)
    nf = af.get_aero_from_neuralfoil(alpha=scan, Re=re, mach=0.05, model_size="xlarge")
    cl_nf, cd_nf, cm_nf = (smooth(np.asarray(nf[k], dtype=float), SMOOTH_SIGMA_DEG, 0.25) for k in ("CL", "CD", "CM"))
    ip, im = first_peak(scan, cl_nf, +1), first_peak(scan, cl_nf, -1)
    a_sp, a_sm = float(scan[ip]), float(scan[im])
    cl_sp, cd_sp, cl_sm, cd_sm = float(cl_nf[ip]), float(cd_nf[ip]), float(cl_nf[im]), float(cd_nf[im])
    cd_max = 1.11 + 0.018 * ar

    grid = np.unique(np.concatenate([np.arange(-30, 30 + 1e-9, STEP_FINE), np.arange(-180, 180 + 1e-9, STEP_COARSE)]))
    cl, cd, cm, wp = (np.zeros_like(grid) for _ in range(4))

    cd_min = float(cd_nf.min())

    def post_stall(a_deg):
        """Вітерна для довільного α (град). |α| ≤ 90° — з параметрами зриву відповідного знака;
        |α| > 90° (обтікання від задньої кромки) — плоска пластина cl = CDmax/2·sin2α, cd = CDmax·sin²α + cd_min."""
        a = abs(a_deg)
        if a > 90.0:
            ar_ = math.radians(a_deg)
            return cd_max / 2 * math.sin(2 * ar_), cd_max * math.sin(ar_) ** 2 + cd_min
        neg = a_deg < 0
        als, cls, cds = (math.radians(-a_sm), -cl_sm, cd_sm) if neg else (math.radians(a_sp), cl_sp, cd_sp)
        cl_v, cd_v = viterna(np.array([max(math.radians(a), 1e-4)]), als, cls, cds, cd_max)
        cl_v, cd_v = float(cl_v[0]), float(cd_v[0])
        return (-cl_v if neg else cl_v), cd_v

    for i, a in enumerate(grid):
        inside = a_sm <= a <= a_sp
        in_nf = NF_RANGE[0] <= a <= NF_RANGE[1]
        if inside:
            w = 0.0
        elif in_nf:
            dist = a - a_sp if a > 0 else a_sm - a
            w = float(smoothstep(dist / BLEND_DEG))
        else:
            w = 1.0
        cl_p, cd_p = post_stall(a)
        cm_p = float(plate_cm(cl_p, cd_p, math.radians(a)))
        if in_nf:
            cl_n, cd_n, cm_n = (float(np.interp(a, scan, arr)) for arr in (cl_nf, cd_nf, cm_nf))
        else:
            cl_n, cd_n, cm_n = cl_p, cd_p, cm_p
        cl[i] = (1 - w) * cl_n + w * cl_p
        cd[i] = (1 - w) * cd_n + w * cd_p
        cm[i] = (1 - w) * cm_n + w * cm_p
        wp[i] = w

    # нахил у лінійній ділянці (1/рад) і нульова підйомна сила
    m = (scan >= -2) & (scan <= 6)
    slope, icpt = np.polyfit(scan[m], cl_nf[m], 1)
    return {
        "airfoil": airfoil, "re": re, "ar_for_cdmax": ar, "cd_max": round(cd_max, 4),
        "alpha_stall_pos_deg": a_sp, "alpha_stall_neg_deg": a_sm, "cl_max": round(cl_sp, 4), "cl_min": round(cl_sm, 4),
        "cl_alpha_per_rad": round(float(slope) * 180 / math.pi, 4), "alpha_zero_deg": round(float(-icpt / slope), 3),
        "alpha_deg": [round(float(x), 3) for x in grid],
        "cl": [round(float(x), 5) for x in cl], "cd": [round(float(x), 5) for x in cd],
        "cm": [round(float(x), 5) for x in cm], "w_plate": [round(float(x), 4) for x in wp],
    }


def main():
    sys.stdout.reconfigure(encoding="utf-8")   # консоль Windows за замовчуванням cp1251
    out = sys.argv[1] if len(sys.argv) > 1 else "aircraft/aero_tables.json"
    try:
        import aerosandbox
        import neuralfoil
    except ImportError as e:
        sys.exit(f"Потрібні aerosandbox і neuralfoil ({e}). pip install aerosandbox neuralfoil")
    data = {
        "generator": "tools/aero_gen.py",
        "versions": {"aerosandbox": aerosandbox.__version__, "neuralfoil": getattr(neuralfoil, "__version__", "?")},
        "note": "2D-перерізи, α −180…180° (град), Cm відносно c/4 (ніс угору +). До зриву NeuralFoil, далі Вітерна; w_plate — частка пост-зривної моделі.",
        "airfoils": {},
    }
    for name, airfoil, re, ar in TABLES:
        t = build_table(name, airfoil, re, ar)
        data["airfoils"][name] = t
        print(f"{name}: CLmax {t['cl_max']} @ {t['alpha_stall_pos_deg']}°, CLα {t['cl_alpha_per_rad']} 1/рад, α0 {t['alpha_zero_deg']}°, "
              f"CDmax {t['cd_max']}, точок {len(t['alpha_deg'])}")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, separators=(",", ":"))
    print("записано", out)


if __name__ == "__main__":
    main()
