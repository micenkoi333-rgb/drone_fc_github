#!/usr/bin/env python3
"""ai_export.py — експорт ваг нейромережі в заголовок C++ для ai/nn.h (симулятор і ESP32-S3 / STM32H7).

Потрібен лише numpy. Ваги — файл .npz з масивами у форматі PyTorch:
    layer0.weight [out, in], layer0.bias [out], layer1.weight, ... (+ необов'язково norm.mean, norm.std [in])
Як отримати з PyTorch (nn.Sequential з Linear-шарів):
    sd = {f"layer{i}.{k}": v.detach().numpy() for i, m in enumerate(lin_layers) for k, v in m.state_dict().items()}
    np.savez("force_model.npz", **sd, **{"norm.mean": mean, "norm.std": std})

Використання:
    python tools/ai_export.py model.npz --name force --acts relu,relu,none -o ai/weights_force.h
    python tools/ai_export.py --demo -o ai/weights_demo.h     # випадкова мережа + GRU з еталонними відповідями для тестів

У заголовок пишуться: дійсні ваги, 8-бітні ваги з масштабом на рядок (як квантує ai::quantizeRows), нормалізація входу
й еталонні пари вхід → вихід, порахувані тут numpy: тест C++ звіряє з ними свою реалізацію.
"""
import argparse
import sys

import numpy as np

ACT = {"none": "ai::Act::None", "relu": "ai::Act::Relu", "tanh": "ai::Act::Tanh", "sigmoid": "ai::Act::Sigmoid"}


def act_np(x, a):
    if a == "relu":
        return np.maximum(x, 0)
    if a == "tanh":
        return np.tanh(x)
    if a == "sigmoid":
        return 1 / (1 + np.exp(-x))
    return x


def quantize_rows(w):
    amax = np.abs(w).max(axis=1)
    s = np.where(amax > 0, amax / 127.0, 1.0).astype(np.float32)
    q = np.rint(w / s[:, None]).clip(-127, 127).astype(np.int8)
    return q, s


def arr(name, a, ctype):
    a = np.asarray(a).ravel()
    if ctype == "float":
        def lit(v):
            t = f"{float(v):.9g}"
            return (t if any(ch in t for ch in ".en") else t + ".0") + "f"   # 0 → 0.0f (у C++ «0f» — помилка)
        body = ", ".join(lit(v) for v in a)
    else:
        body = ", ".join(str(int(v)) for v in a)
    lines, line = [], ""
    for tok in body.split(", "):
        if len(line) + len(tok) > 110:
            lines.append(line)
            line = ""
        line += tok + ", "
    lines.append(line)
    return f"alignas(4) const {ctype} {name}[] = {{\n    " + "\n    ".join(lines).rstrip(", ") + "\n};\n"


def mlp_forward(layers, acts, mean, std, x):
    a = (x - mean) / std
    for (w, b), ac in zip(layers, acts):
        a = act_np(w @ a + b, ac)
    return a


def emit_mlp(name, layers, acts, mean, std, tests_x):
    n_in = layers[0][0].shape[1]
    out = [f"namespace ai_w_{name} {{\n", f"constexpr int kInputs = {n_in}, kOutputs = {layers[-1][0].shape[0]}, kLayers = {len(layers)};\n"]
    out.append(arr("NORM_MEAN", mean, "float"))
    out.append(arr("NORM_INV_STD", 1.0 / std, "float"))
    for i, (w, b) in enumerate(layers):
        q, s = quantize_rows(w)
        out += [arr(f"L{i}_W", w, "float"), arr(f"L{i}_B", b, "float"), arr(f"L{i}_Q", q, "int8_t"), arr(f"L{i}_S", s, "float")]
    for kind, layer in (("Float", "DenseF"), ("Int8", "DenseQ")):
        out.append(f"inline ai::Mlp<ai::{layer}> model{kind}() {{\n    ai::Mlp<ai::{layer}> m;\n")
        out.append(f"    m.norm.n = kInputs; m.norm.mean = NORM_MEAN; m.norm.inv_std = NORM_INV_STD;\n    m.count = kLayers;\n")
        for i, (w, _) in enumerate(layers):
            o, n = w.shape
            if layer == "DenseF":
                out.append(f"    m.layers[{i}] = ai::DenseF{{{n}, {o}, L{i}_W, L{i}_B, {ACT[acts[i]]}}};\n")
            else:
                out.append(f"    m.layers[{i}] = ai::DenseQ{{{n}, {o}, L{i}_Q, L{i}_S, L{i}_B, {ACT[acts[i]]}}};\n")
        out.append("    return m;\n}\n")
    ys = np.array([mlp_forward(layers, acts, mean, std, x) for x in tests_x], dtype=np.float32)
    out.append(f"constexpr int kTests = {len(tests_x)};\n")
    out.append(arr("TEST_X", tests_x, "float"))
    out.append(arr("TEST_Y", ys, "float"))
    out.append(f"}}  // namespace ai_w_{name}\n")
    return "".join(out)


def emit_gru(name, wih, whh, bih, bhh, seq):
    H = whh.shape[1]
    h = np.zeros(H, dtype=np.float64)
    for x in seq:
        gi, gh = wih @ x + bih, whh @ h + bhh
        r = 1 / (1 + np.exp(-(gi[:H] + gh[:H])))
        z = 1 / (1 + np.exp(-(gi[H:2 * H] + gh[H:2 * H])))
        n = np.tanh(gi[2 * H:] + r * gh[2 * H:])
        h = (1 - z) * n + z * h
    out = [f"namespace ai_w_{name} {{\n", f"constexpr int kInputs = {wih.shape[1]}, kHidden = {H}, kSteps = {len(seq)};\n"]
    out += [arr("W_IH", wih, "float"), arr("W_HH", whh, "float"), arr("B_IH", bih, "float"), arr("B_HH", bhh, "float")]
    out.append(f"inline ai::GruF cell() {{ return ai::GruF{{kInputs, kHidden, W_IH, W_HH, B_IH, B_HH}}; }}\n")
    out += [arr("SEQ_X", seq, "float"), arr("H_END", h, "float")]
    out.append(f"}}  // namespace ai_w_{name}\n")
    return "".join(out)


def header(body, src):
    return ("// Згенеровано tools/ai_export.py — не редагувати вручну. Джерело: " + src + "\n"
            "#pragma once\n#include <cstdint>\n#include \"nn.h\"\n\n" + body)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("npz", nargs="?")
    ap.add_argument("--name", default="model")
    ap.add_argument("--acts", default="")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()
    rng = np.random.default_rng(7)
    if a.demo:   # випадкові мережі для перевірки C++ (масштаб ваг — як після навчання з ініціалізацією Кеймінга)
        sizes, acts = [12, 32, 32, 6], ["relu", "tanh", "none"]
        layers = [((rng.standard_normal((o, i)) * np.sqrt(2.0 / i)).astype(np.float32), (rng.standard_normal(o) * 0.1).astype(np.float32))
                  for i, o in zip(sizes[:-1], sizes[1:])]
        mean = rng.standard_normal(12).astype(np.float32)
        std = (0.5 + rng.random(12)).astype(np.float32)
        tests = (mean + std * rng.standard_normal((5, 12))).astype(np.float32)
        body = emit_mlp("demo", layers, acts, mean, std, tests)
        I, H = 8, 16
        g = lambda *s: (rng.standard_normal(s) * 0.4).astype(np.float32)
        body += "\n" + emit_gru("demo_gru", g(3 * H, I), g(3 * H, H), g(3 * H), g(3 * H), g(20, I))
        src = "--demo (випадкові ваги, seed 7)"
    else:
        if not a.npz:
            ap.error("потрібен файл .npz або --demo")
        z = np.load(a.npz)
        layers, i = [], 0
        while f"layer{i}.weight" in z:
            layers.append((z[f"layer{i}.weight"].astype(np.float32), z[f"layer{i}.bias"].astype(np.float32)))
            i += 1
        if not layers:
            sys.exit("у файлі немає layer0.weight")
        acts = a.acts.split(",") if a.acts else ["relu"] * (len(layers) - 1) + ["none"]
        if len(acts) != len(layers) or any(x not in ACT for x in acts):
            sys.exit(f"--acts: потрібно {len(layers)} значень з {list(ACT)}")
        if max(max(w.shape) for w, _ in layers) > 256:
            sys.exit("шар ширший за ai::kMaxWidth = 256")
        n_in = layers[0][0].shape[1]
        mean = z["norm.mean"].astype(np.float32) if "norm.mean" in z else np.zeros(n_in, np.float32)
        std = z["norm.std"].astype(np.float32) if "norm.std" in z else np.ones(n_in, np.float32)
        tests = (mean + std * rng.standard_normal((5, n_in))).astype(np.float32)
        body = emit_mlp(a.name, layers, acts, mean, std, tests)
        src = a.npz
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(header(body, src))
    print("записано", a.out)


if __name__ == "__main__":
    main()
