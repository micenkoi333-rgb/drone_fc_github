#!/usr/bin/env python3
"""ai_train.py — навчання моделі сил (ai/force_model.h) на даних tools/ai_datagen.cpp. Потрібен лише numpy (PyTorch не обов'язковий).

    python tools/ai_train.py data/*.bin -o ai/weights_force.h

Мережа: 62 входи (17 ознак зараз + пам'ять 1 с: як змінювались швидкості, кути й оберти і яку силу показували датчики) → 64 → 64 → 3
(tanh, tanh, лінійний вихід), Adam, міні-батчі; --hist 0 — лише 17 ознак без пам'яті. Польоти діляться на навчальні й перевірочні
(перевірка — на польотах, яких мережа не бачила). Після навчання ваги зберігаються в .npz і перетворюються на заголовок C++
через tools/ai_export.py (float + int8 + еталонні пари для тесту).
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np

NF0, NY = 17, 3
OUT_NAMES = ["Fx (вперед)", "Fy (вліво)", "Fz (вгору)"]

# Пам'ять мережі (так само в ai/force_model.h): записи 50 Гц. Для 11 «рухливих» ознак — різниця «зараз − тоді» 0.1, 0.3 і 1.0 с тому;
# для виміряної сили (акселерометр − таблиця тяги) — її значення зараз, 0.1, 0.3 і 1.0 с тому.
DYN = list(range(11))
LAG_DYN = [5, 15, 50]
LAG_MEAS = [0, 5, 15, 50]
NF_HIST = NF0 + len(DYN) * len(LAG_DYN) + 3 * len(LAG_MEAS) + 1   # + маса (оцінка автопілота / без вантажу)


def lag_index(F, L):
    n = len(F)
    change = np.r_[True, F[1:] != F[:-1]]
    starts = np.maximum.accumulate(np.where(change, np.arange(n), 0))
    return np.maximum(np.arange(n) - L, starts)


def lead_index(F, L):
    n = len(F)
    change = np.r_[F[1:] != F[:-1], True]
    ends = np.minimum.accumulate(np.where(change, np.arange(n), n - 1)[::-1])[::-1]
    return np.minimum(np.arange(n) + L, ends)


def onboard_label(M, F, win, ahead):
    """Мітка з бортового виміру (на залізі правди немає): згладжування без зсуву фази (±win кадрів, лише в межах польоту)
    і на ahead кадрів уперед — мережа вчиться бачити силу крізь шум і затримку фільтра, а не копіювати останній вимір."""
    acc = np.zeros_like(M, dtype=np.float64)
    for k in range(-win, win + 1):
        acc += M[lead_index(F, k) if k >= 0 else lag_index(F, -k)]
    S = (acc / (2 * win + 1)).astype(np.float32)
    return S[lead_index(F, ahead)]


def load(paths, hist, label_mode="auto", win=3, ahead=1):
    """→ X (ознаки, з пам'яттю чи без), Y (мітки), W (вага коптера), F (номер польоту)"""
    Xs, Ys, Ws, Fs = [], [], [], []
    for p in paths:
        raw = np.fromfile(p, dtype=np.float32)
        cols = 23
        if raw.size % 27 == 0:
            f22 = raw.reshape(-1, 27)[:, 22]
            if np.all(f22 == np.round(f22)) and len(np.unique(f22)) * 50 < len(f22):
                cols = 27
        if cols == 23 and raw.size % 26 == 0:
            f22 = raw.reshape(-1, 26)[:, 22]
            if np.all(f22 == np.round(f22)) and len(np.unique(f22)) * 50 < len(f22):
                cols = 26
        if hist and cols < 26:
            sys.exit(f"{p}: старий формат без виміряної сили — згенеруйте дані заново (ai_datagen)")
        d = raw.reshape(-1, cols).astype(np.float32)
        del raw
        F = d[:, 22]
        X = d[:, :NF0]
        if hist:
            parts = [X]
            for L in LAG_DYN:
                j = lag_index(F, L)
                parts.append(X[:, DYN] - X[j][:, DYN])
            M = d[:, 23:26]
            for L in LAG_MEAS:
                parts.append(M[lag_index(F, L)])
            parts.append(d[:, 26:27] if cols == 27 else np.ones((len(d), 1)))
            X = np.concatenate(parts, axis=1)
        Y = d[:, NF0:NF0 + NY]
        onboard = cols >= 26 and np.allclose(Y, d[:, 23:26], atol=1e-6)   # дані нового формату: мітка = бортовий вимір
        if label_mode == "smooth" or (label_mode == "auto" and onboard):
            Y = onboard_label(d[:, 23:26], F, win, ahead)
        Xs.append(X); Ys.append(Y); Ws.append(d[:, 20]); Fs.append(F)
    X, Y, W, F = (np.concatenate(v) for v in (Xs, Ys, Ws, Fs))
    ok = np.isfinite(X).all(axis=1) & np.isfinite(Y).all(axis=1)
    return X[ok], Y[ok], W[ok], F[ok]


class Mlp:
    def __init__(self, sizes, rng):
        self.W = [rng.standard_normal((o, i)).astype(np.float64) * np.sqrt(1.0 / i) for i, o in zip(sizes[:-1], sizes[1:])]
        self.b = [np.zeros(o) for o in sizes[1:]]
        self.m = [np.zeros_like(p) for p in self.W + self.b]
        self.v = [np.zeros_like(p) for p in self.W + self.b]
        self.t = 0

    def forward(self, x):
        acts = [x]
        a = x
        for k, (W, b) in enumerate(zip(self.W, self.b)):
            z = a @ W.T + b
            a = np.tanh(z) if k < len(self.W) - 1 else z
            acts.append(a)
        return acts

    def step(self, x, y, lr, wd=1e-5):
        acts = self.forward(x)
        n = len(x)
        g = 2.0 * (acts[-1] - y) / n   # d(MSE)/d(вихід)
        gW, gb = [], []
        for k in reversed(range(len(self.W))):
            gW.insert(0, g.T @ acts[k] + wd * self.W[k])
            gb.insert(0, g.sum(axis=0))
            if k > 0:
                g = (g @ self.W[k]) * (1 - acts[k] ** 2)
        self.t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        params = self.W + self.b
        grads = gW + gb
        for i, (p, gr) in enumerate(zip(params, grads)):
            self.m[i] = b1 * self.m[i] + (1 - b1) * gr
            self.v[i] = b2 * self.v[i] + (1 - b2) * gr * gr
            mh = self.m[i] / (1 - b1 ** self.t)
            vh = self.v[i] / (1 - b2 ** self.t)
            p -= lr * mh / (np.sqrt(vh) + eps)
        return float(np.mean((acts[-1] - y) ** 2))

    def predict(self, x):
        return self.forward(x)[-1]


def r2(y, p):
    return 1 - ((y - p) ** 2).sum(axis=0) / ((y - y.mean(axis=0)) ** 2).sum(axis=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data", nargs="+")
    ap.add_argument("-o", "--out", default="ai/weights_force.h")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--drop", default="", help="номери ознак, які обнулити (через кому), напр. 11,12,13,14 — кути гондол. Без збудження гондол у даних "
                    "(ai_datagen, 4-й аргумент 0) їх варто прибрати: контролер нахиляє гондоли проти вітру, і мережа вчилась би «нахил → сила» (наслідок замість причини)")
    ap.add_argument("--mg", type=float, default=23.4, help="вага апарата m·g, Н — лише для звіту в ньютонах")
    ap.add_argument("--hist", type=int, default=1, help="1 — пам'ять 1 с (ознаки минулого й виміряна сила), 0 — лише поточна мить")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--name", default="force", help="ім'я набору ваг (простір імен ai_w_<name>): force — основний апарат, force_qp5 — QP-5")
    ap.add_argument("--label", default="auto", help="auto — бортові дані: згладжений вимір наперед; smooth — завжди так; raw — мітка як записана")
    ap.add_argument("--smooth", type=int, default=3, help="півширина згладжування мітки, кадрів (50 Гц)")
    ap.add_argument("--ahead", type=int, default=1, help="на скільки кадрів уперед передбачати")
    ap.add_argument("--ckpt", default="", help="файл стану навчання: якщо є — продовжити з нього; з --time-limit — зберегти й вийти (код 3), коли час вийде")
    ap.add_argument("--time-limit", type=float, default=0, help="секунд на цей запуск (0 — без обмеження)")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    X, Y, W, F = load(a.data, a.hist, a.label, a.smooth, a.ahead)
    NF = X.shape[1]
    for c in (int(v) for v in a.drop.split(",") if v != ""):
        X[:, c] = 0.0   # ознаку прибрано (на борту теж подається 0) — щоб мережа не вчилась «наслідок → причина»
    flights = np.unique(F)
    np.random.default_rng(1).shuffle(flights)
    test_f = set(flights[: max(1, len(flights) // 6)].tolist())
    te = np.array([f in test_f for f in F])
    print(f"даних: {len(X)} записів з {len(flights)} польотів ({len(X) / 50 / 60:.0f} хв польоту), входів мережі {NF}; перевірка — {len(test_f)} польотів, яких мережа не бачить")
    xm, xs = X[~te].mean(axis=0), X[~te].std(axis=0) + 1e-6
    ym, ys = Y[~te].mean(axis=0), Y[~te].std(axis=0) + 1e-6
    Xn, Yn = ((X - xm) / xs).astype(np.float32), ((Y - ym) / ys).astype(np.float32)
    del X
    net = Mlp([NF, a.hidden, a.hidden, NY], rng)
    tr_idx = np.where(~te)[0]
    t0 = time.time()
    ep0 = 0
    if a.ckpt and os.path.exists(a.ckpt):
        import pickle
        with open(a.ckpt, "rb") as f:
            st = pickle.load(f)
        net.W, net.b, net.m, net.v, net.t = st["W"], st["b"], st["m"], st["v"], st["t"]
        rng.bit_generator.state = st["rng"]; tr_idx = st["idx"]; ep0 = st["ep"]
        print(f"продовжую з епохи {ep0 + 1}")
    for ep in range(ep0, a.epochs):
        if a.ckpt and a.time_limit > 0 and ep > ep0 and time.time() - t0 > a.time_limit:
            import pickle
            with open(a.ckpt, "wb") as f:
                pickle.dump({"W": net.W, "b": net.b, "m": net.m, "v": net.v, "t": net.t, "rng": rng.bit_generator.state, "idx": tr_idx, "ep": ep}, f)
            print(f"час вийшов — збережено стан після {ep} епох; запустіть ще раз для продовження")
            sys.exit(3)
        rng.shuffle(tr_idx)
        lr = a.lr * (0.5 * (1 + np.cos(np.pi * ep / a.epochs)) * 0.95 + 0.05)   # косинусне згасання
        loss = 0.0
        nb = 0
        for s in range(0, len(tr_idx), a.batch):
            b = tr_idx[s:s + a.batch]
            loss += net.step(Xn[b], Yn[b], lr)
            nb += 1
        if ep % 5 == 4 or ep == a.epochs - 1:
            pt = np.concatenate([net.predict(Xn[te][i:i + 65536]) for i in range(0, int(te.sum()), 65536)])
            print(f"  епоха {ep + 1:3d}: навчання MSE {loss / nb:.4f}, перевірка MSE {np.mean((pt - Yn[te]) ** 2):.4f} ({time.time() - t0:.0f} с)")
    # звіт на перевірочних польотах, у ньютонах
    P = np.concatenate([net.predict(Xn[i:i + 65536]) for i in range(0, len(Xn), 65536)]) * ys + ym
    for name, mask in (("усі режими", te), ("гвинти несуть вагу (w ≥ 0.99) — де поправка діє", te & (W >= 0.99))):
        if mask.sum() < 10:
            continue
        e = (P[mask] - Y[mask]) * a.mg
        rr = r2(Y[mask], P[mask])
        print(f"перевірка, {name}: {mask.sum()} записів")
        for k in range(NY):
            print(f"  {OUT_NAMES[k]:12s}: R² {rr[k]:.3f}, похибка RMS {np.sqrt(np.mean(e[:, k] ** 2)):.2f} Н, сила RMS {np.sqrt(np.mean((Y[mask, k] * a.mg) ** 2)):.2f} Н")
    # останній шар «розгортаємо» з нормалізації виходу: y = ys·(W a + b) + ym
    Ws = [w.copy() for w in net.W]
    for c in (int(v) for v in a.drop.split(",") if v != ""):   # прибрані ознаки: на борту їхнє значення не впливає
        Ws[0][:, c] = 0.0; xm[c] = 0.0; xs[c] = 1.0
    bs = [b.copy() for b in net.b]
    Ws[-1] = Ws[-1] * ys[:, None]
    bs[-1] = bs[-1] * ys + ym
    npz = os.path.splitext(a.out)[0] + ".npz"
    sd = {f"layer{i}.weight": Ws[i].astype(np.float32) for i in range(len(Ws))}
    sd.update({f"layer{i}.bias": bs[i].astype(np.float32) for i in range(len(bs))})
    sd["norm.mean"] = xm.astype(np.float32)
    sd["norm.std"] = xs.astype(np.float32)
    np.savez(npz, **sd)
    exp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ai_export.py")
    subprocess.check_call([sys.executable, exp, npz, "--name", a.name, "--acts", "tanh,tanh,none", "-o", a.out])


if __name__ == "__main__":
    main()
