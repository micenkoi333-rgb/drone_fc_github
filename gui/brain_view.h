// brain_view.h — вікно «Мозок ШІ» (підключається з main_gui.cpp всередині його простору імен, після App і функцій трюків).
// Три вкладки: «Навчання» — що ШІ пробує і як змінює переконання з кожним поколінням; «Політ трюку» — що пілот робить зараз і чому,
// команди проти факту, гондоли й тяга; «Нейромережа» — живі активації мережі сил, її передбачення проти правди, доказ впливу.

ImU32 scoreColor(double s, float alpha = 1.0f) {
    const float t = float(std::clamp((s + 60.0) / 160.0, 0.0, 1.0));
    const float r = t < 0.5f ? 1.0f : 1.0f - (t - 0.5f) * 1.6f, g = t < 0.5f ? 0.25f + t * 1.4f : 0.95f;
    return ImGui::ColorConvertFloat4ToU32(ImVec4(r * 0.92f, g * 0.85f, 0.25f, alpha));
}
ImU32 actColor(float a, float alpha = 1.0f) {
    const float m = std::clamp(std::fabs(a), 0.0f, 1.0f);
    const ImVec4 base(0.28f, 0.28f, 0.32f, alpha), pos(1.0f, 0.62f, 0.18f, alpha), neg(0.30f, 0.62f, 1.0f, alpha);
    const ImVec4& c = a >= 0 ? pos : neg;
    return ImGui::ColorConvertFloat4ToU32(ImVec4(base.x + (c.x - base.x) * m, base.y + (c.y - base.y) * m, base.z + (c.z - base.z) * m, alpha));
}

// Власний графік на ImDrawList (лінії ImPlot у цій збірці ImGui малюються пунктиром)
struct PS { std::vector<float> x, y; const char* name; ImU32 col; float w; bool dots; };
void chart(const char* id, const std::vector<PS>& ser, float height, const char* xlab, const char* ylab, const std::vector<float>& vlines = {}) {
    ImGui::PushID(id);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const float W = std::max(160.0f, ImGui::GetContentRegionAvail().x - 4), H = height, L = 52, B = 34, legw = 0;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + W, p0.y + H), IM_COL32(20, 22, 28, 255), 5);
    float x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f;
    for (const PS& s2 : ser) for (size_t i = 0; i < s2.x.size() && i < s2.y.size(); ++i) { x0 = std::min(x0, s2.x[i]); x1 = std::max(x1, s2.x[i]); if (std::isfinite(s2.y[i])) { y0 = std::min(y0, s2.y[i]); y1 = std::max(y1, s2.y[i]); } }
    if (x0 > x1) { ImGui::Dummy(ImVec2(W, H)); ImGui::PopID(); return; }
    if (y1 - y0 < 1e-3f) { y0 -= 1; y1 += 1; }
    const float pad = (y1 - y0) * 0.08f; y0 -= pad; y1 += pad;
    const float px0 = p0.x + L, px1 = p0.x + W - 6 - legw, py0 = p0.y + 22, py1 = p0.y + H - B;
    auto X = [&](float v) { return px0 + (v - x0) / std::max(x1 - x0, 1e-6f) * (px1 - px0); };
    auto Y = [&](float v) { return py1 - (v - y0) / (y1 - y0) * (py1 - py0); };
    for (int g = 0; g <= 4; ++g) {
        const float v = y0 + (y1 - y0) * float(g) / 4, yy = Y(v);
        dl->AddLine(ImVec2(px0, yy), ImVec2(px1, yy), IM_COL32(255, 255, 255, 18));
        char b[24]; std::snprintf(b, sizeof b, "%.3g", v);
        dl->AddText(ImVec2(p0.x + L - 6 - ImGui::CalcTextSize(b).x, yy - 8), IM_COL32(160, 165, 175, 255), b);
    }
    if (y0 < 0 && y1 > 0) dl->AddLine(ImVec2(px0, Y(0)), ImVec2(px1, Y(0)), IM_COL32(255, 255, 255, 50));
    for (int g = 0; g <= 5; ++g) {
        const float v = x0 + (x1 - x0) * float(g) / 5;
        char b[24]; std::snprintf(b, sizeof b, "%.4g", v);
        dl->AddText(ImVec2(X(v) - ImGui::CalcTextSize(b).x * 0.5f, py1 + 2), IM_COL32(160, 165, 175, 255), b);
    }
    for (float v : vlines) if (v > x0 && v < x1) dl->AddLine(ImVec2(X(v), py0), ImVec2(X(v), py1), IM_COL32(255, 255, 255, 30));
    dl->AddText(ImVec2(px1 - ImGui::CalcTextSize(xlab).x, py1 + 16), IM_COL32(140, 145, 155, 255), xlab);
    dl->AddText(ImVec2(p0.x + 6, p0.y + 3), IM_COL32(140, 145, 155, 255), ylab);
    dl->PushClipRect(ImVec2(px0, py0 - 2), ImVec2(px1, py1 + 2), true);
    std::vector<ImVec2> pts;
    for (const PS& s2 : ser) {
        pts.clear();
        const size_t n = std::min(s2.x.size(), s2.y.size()), step = std::max<size_t>(1, n / 1200);
        for (size_t i = 0; i < n; i += step) pts.push_back(ImVec2(X(s2.x[i]), Y(s2.y[i])));
        if (s2.dots) for (const ImVec2& q : pts) dl->AddCircleFilled(q, 3.0f, s2.col, 8);
        else if (pts.size() > 1) dl->AddPolyline(pts.data(), int(pts.size()), s2.col, ImDrawFlags_None, s2.w);
    }
    dl->PopClipRect();
    float lx = px0 + 6 + ImGui::CalcTextSize(ylab).x;
    for (const PS& s2 : ser) {
        if (!s2.name || !s2.name[0]) continue;
        dl->AddRectFilled(ImVec2(lx, p0.y + 7), ImVec2(lx + 12, p0.y + 15), s2.col);
        dl->AddText(ImVec2(lx + 16, p0.y + 3), IM_COL32(215, 220, 230, 255), s2.name);
        lx += 30 + ImGui::CalcTextSize(s2.name).x;
    }
    ImGui::Dummy(ImVec2(W, H + 4));
    ImGui::PopID();
}
template <class D> std::vector<float> fv(const D& d) { return std::vector<float>(d.begin(), d.end()); }

void brainSample(App& A) {
    const fc::TrickPilot& p = A.show.pilot;
    const QpState& s = A.rig->sim.state();
    const float t = float(A.rig->sim.time());
    App::RateHist& h = A.rh;
    h.t.push_back(t);
    const fc::Vec3 wc = p.cmdRates();
    const float wcv[3] = {wc.x, wc.y, wc.z};
    for (int k = 0; k < 3; ++k) { h.wc[k].push_back(wcv[k]); h.w[k].push_back(float(s.omega[k])); }
    while (!h.t.empty() && h.t.back() - h.t.front() > 8.0f) { h.t.pop_front(); for (int k = 0; k < 3; ++k) { h.wc[k].pop_front(); h.w[k].pop_front(); } }
    if (A.trick_trail.empty() || (s.pos - A.trick_trail.back()).norm() > 0.6) { A.trick_trail.push_back(s.pos); if (A.trick_trail.size() > 3000) A.trick_trail.pop_front(); }
}

void aiSample(App& A) {
    QpFlightRig& r = *A.rig;
    if (!r.ai_trace_valid || r.ai_mode == 0) return;
    const float t = float(r.sim.time());
    App::AiHist& h = A.aih;
    if (!h.t.empty() && t - h.t.back() < 0.05f) return;
    if (!h.t.empty() && t < h.t.back()) h = App::AiHist{};
    h.t.push_back(t); h.pz.push_back(r.ai_trace.out[2]); h.tz.push_back(r.ai_truth[2]); h.px.push_back(r.ai_trace.out[0]); h.tx.push_back(r.ai_truth[0]); h.used.push_back(r.ai_used_acc[2]);
    h.mz.push_back(float(r.ai_meas[2])); h.trust.push_back(r.ai_trust * 100);
    while (h.t.back() - h.t.front() > 30.0f) { h.t.pop_front(); h.pz.pop_front(); h.tz.pop_front(); h.px.pop_front(); h.tx.pop_front(); h.used.pop_front(); h.mz.pop_front(); h.trust.pop_front(); }
}

// Рядок «переконання» про один параметр: сіре поле — весь діапазон, світла хмара — звідки ШІ зараз бере спроби (середнє ± розкид),
// кружечки — спроби останнього покоління (колір — оцінка), золотий ромб — найкраща спроба, біла риска — середнє
void beliefRow(ImDrawList* dl, ImVec2 p, float w, float h, float mu, float sd, const std::vector<std::pair<float, double>>& dots, float best_u, bool has_best) {
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(38, 40, 48, 255), 4);
    for (int x = 0; x < int(w); x += 2) {
        const float u = (float(x) + 1.0f) / w, z = (u - mu) / std::fmax(sd, 0.01f);
        const float a = std::exp(-0.5f * z * z);
        if (a > 0.02f) dl->AddRectFilled(ImVec2(p.x + float(x), p.y + 2), ImVec2(p.x + float(x) + 2, p.y + h - 2), IM_COL32(120, 190, 255, int(150 * a)));
    }
    dl->AddLine(ImVec2(p.x + mu * w, p.y), ImVec2(p.x + mu * w, p.y + h), IM_COL32(255, 255, 255, 230), 2);
    for (const auto& d : dots) dl->AddCircleFilled(ImVec2(p.x + std::clamp(d.first, 0.0f, 1.0f) * w, p.y + h * 0.5f), 4.5f, scoreColor(d.second), 10);
    if (has_best) {
        const ImVec2 c(p.x + best_u * w, p.y + h * 0.5f);
        dl->AddQuadFilled(ImVec2(c.x, c.y - 7), ImVec2(c.x + 6, c.y), ImVec2(c.x, c.y + 7), ImVec2(c.x - 6, c.y), IM_COL32(255, 210, 60, 255));
        dl->AddQuad(ImVec2(c.x, c.y - 7), ImVec2(c.x + 6, c.y), ImVec2(c.x, c.y + 7), ImVec2(c.x - 6, c.y), IM_COL32(60, 40, 0, 255));
    }
}

struct TrajDraw { const std::vector<TrajPoint>* tr; ImU32 col; float thick; bool planes; };

// Траєкторії в осях входу в трюк: view 0 — збоку (уздовж курсу × висота), 1 — згори (уздовж × убік)
void trajCanvas(const char* id, const std::vector<TrajDraw>& ts, int view, ImVec2 size) {
    ImGui::InvisibleButton(id, size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0 = ImGui::GetItemRectMin(), p1 = ImGui::GetItemRectMax();
    dl->AddRectFilled(p0, p1, IM_COL32(24, 28, 36, 255), 6);
    float xmin = -5, xmax = 5, ymin = -5, ymax = 5;
    for (const TrajDraw& t : ts) for (const TrajPoint& q : *t.tr) {
        const float a = q.x, b = view == 0 ? q.z : q.y;
        xmin = std::fmin(xmin, a); xmax = std::fmax(xmax, a); ymin = std::fmin(ymin, b); ymax = std::fmax(ymax, b);
    }
    const float pad = 14, W = p1.x - p0.x - 2 * pad, H = p1.y - p0.y - 2 * pad - 14;
    const float sc = std::fmin(W / std::fmax(xmax - xmin, 1.0f), H / std::fmax(ymax - ymin, 1.0f));
    const float ex = std::clamp(0.45f * H / std::fmax((ymax - ymin) * sc, 1.0f), 1.0f, 4.0f);
    const float scy = sc * ex;
    const float cx = p0.x + pad + (W - (xmax - xmin) * sc) * 0.5f, cy = p0.y + pad + 14 + (H + (ymax - ymin) * scy) * 0.5f;
    auto P = [&](float a, float b) { return ImVec2(cx + (a - xmin) * sc, cy - (b - ymin) * scy); };
    float step = 5; for (float s2 : {5.f, 10.f, 20.f, 50.f, 100.f, 200.f}) { step = s2; if ((xmax - xmin) / s2 < 8 && (ymax - ymin) / s2 < 6) break; }
    for (float g = std::ceil(xmin / step) * step; g <= xmax; g += step) dl->AddLine(P(g, ymin), P(g, ymax), IM_COL32(255, 255, 255, 16));
    for (float g = std::ceil(ymin / step) * step; g <= ymax; g += step) dl->AddLine(P(xmin, g), P(xmax, g), IM_COL32(255, 255, 255, g == 0 ? 60 : 16));
    char lab[128]; std::snprintf(lab, sizeof lab, "%s   сітка %.0f м%s", view == 0 ? "ЗБОКУ: уперед → , висота ↑" : "ЗГОРИ: уперед → , ліворуч ↑", step, ex > 1.05f ? (view == 0 ? "   (висота розтягнута)" : "   (убік розтягнуто)") : "");
    dl->AddText(ImVec2(p0.x + 8, p0.y + 4), IM_COL32(200, 205, 215, 255), lab);
    dl->AddCircleFilled(P(0, 0), 4, IM_COL32(255, 255, 255, 200));
    std::vector<ImVec2> pts;
    for (const TrajDraw& t : ts) {
        pts.clear();
        for (const TrajPoint& q : *t.tr) pts.push_back(P(q.x, view == 0 ? q.z : q.y));
        if (pts.size() > 1) dl->AddPolyline(pts.data(), int(pts.size()), t.col, ImDrawFlags_None, t.thick);
        if (!t.planes) continue;
        float last = -10;
        for (const TrajPoint& q : *t.tr) {
            if (q.t - last < 0.8f) continue;
            last = q.t;
            const float L = 9.0f;
            const float na = q.n[0], nb = view == 0 ? q.n[2] : q.n[1], ua = q.up[0], ub = view == 0 ? q.up[2] : q.up[1];
            const ImVec2 c = P(q.x, view == 0 ? q.z : q.y);
            const float nn = std::fmax(std::hypot(na, nb * ex), 0.2f);
            const ImVec2 dn(na / nn * L, -nb * ex / nn * L);
            dl->AddLine(ImVec2(c.x - dn.x, c.y - dn.y), ImVec2(c.x + dn.x, c.y + dn.y), IM_COL32(255, 255, 255, 235), 2.5f);
            dl->AddCircleFilled(ImVec2(c.x + dn.x, c.y + dn.y), 2.5f, IM_COL32(255, 90, 90, 255));
            if (view == 0) dl->AddLine(ImVec2(c.x - dn.x * 0.8f, c.y - dn.y * 0.8f), ImVec2(c.x - dn.x * 0.8f + ua * 6, c.y - dn.y * 0.8f - ub * 6), IM_COL32(120, 220, 255, 255), 2);
        }
    }
}

void brainTrain(App& A) {
    std::lock_guard<std::mutex> l(A.train_mx);
    App::BrainTrain& b = A.br;
    if (!b.have) {
        ImGui::TextWrapped("Тут видно, як ШІ вчиться трюку. Вкладка «Трюки» → обери трюк → «Навчити».");
        ImGui::TextWrapped("Ти побачиш: кожну спробу (траєкторію збоку й згори), що ШІ вирішив після кожних 10 спроб, і як звужується його «переконання» щодо кожного параметра.");
        return;
    }
    const fc::Trick t = b.trick;
    const fc::TrickParamSpec* sp = fc::trickSpec(t);
    const int n = int(b.eps.size());
    const double best_score = b.best >= 0 ? b.eps[size_t(b.best)].r.score : 0;
    if (A.big_font) ImGui::PushFont(A.big_font, 22.0f);
    ImGui::Text("%s — спроба %d з %d, покоління %d", fc::trickName(t), n, A.train_total, int(b.gens.size()) + 1);
    if (A.big_font) ImGui::PopFont();
    if (b.best >= 0) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.82f, 0.25f, 1), "  найкраща: %.0f (спроба %d)", best_score, b.eps[size_t(b.best)].index); }
    if (A.train_running) { ImGui::SameLine(); ImGui::TextColored(ImVec4(0.5f, 0.9f, 1, 1), "  • вчиться"); }

    const float colw = ImGui::GetContentRegionAvail().x;
    const bool wide = colw > 900;
    ImGui::BeginChild("bt_left", ImVec2(wide ? colw * 0.47f : colw, wide ? 0 : 420), ImGuiChildFlags_None);
    ImGui::SeparatorText("Переконання ШІ: з якого діапазону він бере наступні спроби");
    ImGui::TextDisabled("хмара — де шукає (вужча = впевненіший), риска — середнє, кружечки — останні спроби (зелений = добре), ромб — найкраща");
    const int off = (n > 0 && b.eps[0].index == 0) ? 1 : 0;
    const int g0 = n > off ? off + (n - off) - ((n - off - 1) % 10 + 1) : 0;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float rowW = ImGui::GetContentRegionAvail().x;
    for (int i = 0; i < fc::kTrickParams; ++i) {
        const float val = sp[i].lo + std::clamp(b.mu[i], 0.0f, 1.0f) * (sp[i].hi - sp[i].lo), spr = b.sd[i] * (sp[i].hi - sp[i].lo);
        const float conf = std::clamp(1.0f - (b.sd[i] - 0.04f) / 0.26f, 0.0f, 1.0f);
        ImGui::TextUnformatted(sp[i].name);
        char vb[96]; std::snprintf(vb, sizeof vb, "%.2f ± %.2f   впевненість %.0f%%", val, spr, conf * 100);
        ImGui::SameLine(std::fmax(ImGui::GetCursorPosX() + 10, rowW - ImGui::CalcTextSize(vb).x));
        ImGui::TextColored(ImVec4(0.6f + 0.4f * (1 - conf), 0.85f, 1.0f - 0.5f * (1 - conf), 1), "%s", vb);
        const ImVec2 cp = ImGui::GetCursorScreenPos();
        std::vector<std::pair<float, double>> dots;
        for (int k = std::max(0, g0); k < n; ++k) dots.push_back({b.eps[size_t(k)].u[i], b.eps[size_t(k)].r.score});
        const float lw = 44;
        beliefRow(dl, ImVec2(cp.x + lw, cp.y), rowW - 2 * lw, 24, b.mu[i], b.sd[i], dots, b.best >= 0 ? b.eps[size_t(b.best)].u[i] : 0, b.best >= 0);
        char lo[32], hi[32]; std::snprintf(lo, sizeof lo, "%.3g", sp[i].lo); std::snprintf(hi, sizeof hi, "%.3g", sp[i].hi);
        dl->AddText(ImVec2(cp.x + lw - 6 - ImGui::CalcTextSize(lo).x, cp.y + 4), IM_COL32(140, 145, 155, 255), lo);
        dl->AddText(ImVec2(cp.x + rowW - lw + 6, cp.y + 4), IM_COL32(140, 145, 155, 255), hi);
        ImGui::Dummy(ImVec2(rowW, 30));
    }
    ImGui::SeparatorText("Оцінки спроб");
    if (!A.tr_x.empty()) {
        std::vector<float> vl;
        for (const auto& g : b.gens) vl.push_back(float(g.index * g.size) + 0.5f);
        chart("learn", {{fv(A.tr_x), fv(A.tr_score), "спроба", IM_COL32(120, 180, 255, 255), 1, true}, {fv(A.tr_x), fv(A.tr_best), "найкраща", IM_COL32(255, 210, 60, 255), 2.5f, false}}, 180, "спроба", "оцінка", vl);
    }
    ImGui::EndChild();
    if (wide) ImGui::SameLine();
    ImGui::BeginChild("bt_right", ImVec2(0, 0), ImGuiChildFlags_None);
    ImGui::SeparatorText("Спроби: як летів апарат");
    if (A.view_ep >= n) A.view_ep = -1;
    const int sel = A.view_ep >= 0 ? A.view_ep : b.best;
    std::vector<TrajDraw> ts;
    for (int k = std::max(0, n - 30); k < n; ++k) {
        const bool cur = k >= g0;
        ts.push_back({&b.eps[size_t(k)].traj, b.eps[size_t(k)].r.crashed ? IM_COL32(230, 60, 60, cur ? 200 : 70) : scoreColor(b.eps[size_t(k)].r.score, cur ? 0.75f : 0.22f), cur ? 1.8f : 1.0f, false});
    }
    if (sel >= 0 && sel < n) ts.push_back({&b.eps[size_t(sel)].traj, IM_COL32(255, 210, 60, 255), 3.0f, true});
    const float cw = ImGui::GetContentRegionAvail().x;
    trajCanvas("##side", ts, 0, ImVec2(cw, std::fmin(300.0f, cw * 0.55f)));
    trajCanvas("##top", ts, 1, ImVec2(cw, std::fmin(220.0f, cw * 0.38f)));
    ImGui::TextDisabled("золота — %s; білі рисочки — положення корпусу кожні 0.8 с (червона точка — ніс, блакитна — «дах»)", A.view_ep >= 0 ? "вибрана спроба" : "найкраща спроба");
    if (n > 0) {
        int v = sel >= 0 ? sel + 1 : n;
        ImGui::SetNextItemWidth(cw * 0.45f);
        if (ImGui::SliderInt("спроба №", &v, 1, n)) A.view_ep = v - 1;
        ImGui::SameLine(); if (ImGui::Button("Найкраща")) A.view_ep = -1;
        const TrickTrainer::Episode& e = b.eps[size_t(std::clamp(v - 1, 0, n - 1))];
        ImGui::TextWrapped("%s", e.r.text().c_str());
        if (A.train_running) ImGui::BeginDisabled();
        if (ImGui::Button("Показати цю спробу в 3D (той самий старт, ті самі параметри)", ImVec2(-1, 0))) {
            for (int i = 0; i < fc::kTrickParams; ++i) A.replay_u[i] = e.u[i];
            A.replay_trick = t; A.replay_req = true;
        }
        if (A.train_running) ImGui::EndDisabled();
    }
    ImGui::SeparatorText("Думки ШІ після кожного покоління");
    ImGui::BeginChild("thoughts", ImVec2(0, 0), ImGuiChildFlags_Borders);
    for (int i = int(b.thoughts.size()) - 1; i >= 0; --i) { ImGui::TextWrapped("%s", b.thoughts[size_t(i)].c_str()); ImGui::Separator(); }
    ImGui::EndChild();
    ImGui::EndChild();
}

void nacelleGauge(ImDrawList* dl, ImVec2 c, float r, float tilt_act, float tilt_cmd, float thrust, float thrust_cmd, float tmax, const char* name) {
    dl->AddCircleFilled(c, r, IM_COL32(32, 36, 46, 255));
    dl->AddCircle(c, r, IM_COL32(90, 95, 110, 255), 32, 1.5f);
    dl->AddLine(ImVec2(c.x - r, c.y), ImVec2(c.x + r, c.y), IM_COL32(255, 255, 255, 25));
    dl->AddLine(ImVec2(c.x, c.y - r), ImVec2(c.x, c.y + r), IM_COL32(255, 255, 255, 25));
    auto dir = [&](float th, float L) { return ImVec2(c.x + std::sin(th) * L, c.y - std::cos(th) * L); };
    dl->AddLine(c, dir(tilt_cmd, r * 0.95f), IM_COL32(255, 255, 255, 110), 1.5f);
    dl->AddLine(c, dir(tilt_act, r * 0.8f), IM_COL32(255, 160, 50, 255), 6);
    const float k = std::clamp(thrust / std::fmax(tmax, 1.0f), 0.0f, 1.0f);
    const ImVec2 tip = dir(tilt_act, r * 0.8f), arrow = dir(tilt_act, r * 0.8f + 4 + 40 * k);
    dl->AddLine(ImVec2(tip.x, tip.y), arrow, IM_COL32(110, 230, 120, 255), 3);
    char b[64]; std::snprintf(b, sizeof b, "%.0f°  %.1f Н", tilt_act * 57.29578f, thrust);
    dl->AddText(ImVec2(c.x - ImGui::CalcTextSize(name).x * 0.5f, c.y + r + 4), IM_COL32(200, 205, 215, 255), name);
    dl->AddText(ImVec2(c.x - ImGui::CalcTextSize(b).x * 0.5f, c.y + r + 20), IM_COL32(255, 200, 120, 255), b);
    const float bw = r * 1.6f, by = c.y + r + 40;
    dl->AddRectFilled(ImVec2(c.x - bw / 2, by), ImVec2(c.x + bw / 2, by + 6), IM_COL32(50, 54, 64, 255));
    dl->AddRectFilled(ImVec2(c.x - bw / 2, by), ImVec2(c.x - bw / 2 + bw * k, by + 6), IM_COL32(110, 230, 120, 255));
    const float kc = std::clamp(thrust_cmd / std::fmax(tmax, 1.0f), 0.0f, 1.0f);
    dl->AddLine(ImVec2(c.x - bw / 2 + bw * kc, by - 2), ImVec2(c.x - bw / 2 + bw * kc, by + 8), IM_COL32(255, 255, 255, 220), 2);
}

void brainFlight(App& A) {
    const AirShow& sh = A.show;
    const fc::TrickPilot& p = sh.pilot;
    if (!sh.running() && sh.results.empty()) {
        ImGui::TextWrapped("Тут видно, що робить ШІ-пілот під час трюку: у якій він фазі, що думає й чому, що командує гондолам і моторам і як апарат слухається.");
        ImGui::TextWrapped("Вкладка «Трюки» → «Виконати трюк» або «АВІАШОУ АСА» (у польоті літаком). Або у вкладці «Навчання» — «Показати спробу в 3D».");
        return;
    }
    if (sh.program.size() > 1) {
        ImGui::SeparatorText("Програма авіашоу");
        if (ImGui::BeginTable("prog", 4, ImGuiTableFlags_SizingStretchSame)) {
            for (size_t i = 0; i < sh.program.size(); ++i) {
                ImGui::TableNextColumn();
                const bool done = i < sh.results.size(), cur = int(i) == sh.index() && sh.inTrick();
                if (done) ImGui::TextColored(sh.results[i].r.completed ? ImVec4(0.45f, 0.9f, 0.5f, 1) : ImVec4(1, 0.45f, 0.4f, 1), "%s %s — %.0f", sh.results[i].r.completed ? "[+]" : "[x]", fc::trickName(sh.program[i]), sh.results[i].r.score);
                else if (cur) ImGui::TextColored(ImVec4(1, 0.75f, 0.3f, 1), ">> %s", fc::trickName(sh.program[i]));
                else ImGui::TextDisabled("%d. %s", int(i) + 1, fc::trickName(sh.program[i]));
            }
            ImGui::EndTable();
        }
        if (sh.running() && !sh.inTrick()) ImGui::TextColored(ImVec4(0.5f, 0.85f, 1, 1), "Автопілот вирівнює політ і набирає висоту перед наступним трюком… %.0f с", sh.waitTime());
    }
    if (A.big_font) ImGui::PushFont(A.big_font, 24.0f);
    ImGui::TextColored(ImVec4(1, 0.85f, 0.4f, 1), "%s", fc::trickName(p.trick()));
    if (A.big_font) ImGui::PopFont();
    {   // фази
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const int np = p.phaseCount();
        const float W = ImGui::GetContentRegionAvail().x, h = 30, gap = 4, w = (W - gap * float(np - 1)) / float(np);
        const ImVec2 o = ImGui::GetCursorScreenPos();
        const int ph = p.active() ? p.phase() : np;
        for (int k = 0; k < np; ++k) {
            const ImVec2 a(o.x + float(k) * (w + gap), o.y), b2(a.x + w, a.y + h);
            const ImU32 c = k < ph ? IM_COL32(50, 120, 70, 255) : (k == ph ? IM_COL32(220, 140, 40, 255) : IM_COL32(48, 52, 62, 255));
            dl->AddRectFilled(a, b2, c, 5);
            const char* nm = p.phaseName(k);
            ImGui::PushClipRect(a, b2, true);
            const ImVec2 ts = ImGui::CalcTextSize(nm);
            dl->AddText(ImVec2(a.x + std::fmax(3.0f, (w - ts.x) * 0.5f), a.y + (h - ts.y) * 0.5f), IM_COL32(255, 255, 255, 240), nm);
            ImGui::PopClipRect();
        }
        ImGui::Dummy(ImVec2(W, h + 4));
    }
    ImGui::SeparatorText("Що я роблю зараз і чому");
    if (A.big_font) ImGui::PushFont(A.big_font, 19.0f);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.97f, 1.0f, 1));
    ImGui::TextWrapped("%s", sh.inTrick() || !sh.running() ? p.thought().c_str() : "Чекаю: автопілот стабілізує політ.");
    ImGui::PopStyleColor();
    if (A.big_font) ImGui::PopFont();

    ImGui::SeparatorText("Гондоли й тяга (вигляд збоку: 0° — угору, 90° — уперед)");
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const float W = ImGui::GetContentRegionAvail().x, r = std::fmin(46.0f, W / 11.0f);
        const ImVec2 o = ImGui::GetCursorScreenPos();
        const char* nm[4] = {"передній правий", "задній лівий", "передній лівий", "задній правий"};
        const int order[4] = {2, 0, 1, 3};
        const float tmax = A.rig->ctl.cfg.vtol_map.maxThrust();
        for (int j = 0; j < 4; ++j) {
            const int i = order[j];
            const ImVec2 c(o.x + W * (0.125f + 0.25f * float(j)), o.y + r + 30);
            nacelleGauge(dl, c, r, float(A.rig->sim.tilt(i)), sh.inTrick() ? p.cmdTilt(i) : float(A.rig->sim.tilt(i)), float(A.rig->sim.vtolThrust(i)), sh.inTrick() ? p.cmdThrust(i) : 0.0f, tmax, nm[i]);
        }
        ImGui::Dummy(ImVec2(W, 2 * r + 86));
        ImGui::TextDisabled("помаранчева — де гондола зараз, біла тонка — куди ШІ її командує; зелена стрілка й смужка — тяга (біла риска — команда)");
    }
    ImGui::SeparatorText("Обертання: команда ШІ проти факту");
    if (A.rh.t.size() > 2) {
        const std::vector<float> tx = fv(A.rh.t);
        const char* nm[3] = {"крен", "тангаж", "рискання"};
        const ImU32 col[3] = {IM_COL32(100, 175, 255, 255), IM_COL32(255, 155, 60, 255), IM_COL32(120, 230, 120, 255)};
        const ImU32 colc[3] = {IM_COL32(100, 175, 255, 120), IM_COL32(255, 155, 60, 120), IM_COL32(120, 230, 120, 120)};
        std::vector<PS> ps;
        for (int k = 0; k < 3; ++k) { ps.push_back({tx, fv(A.rh.wc[k]), "", colc[k], 4.0f, false}); ps.push_back({tx, fv(A.rh.w[k]), nm[k], col[k], 1.6f, false}); }
        chart("rates", ps, 210, "с", "рад/с");
        ImGui::TextDisabled("широка бліда лінія — що командує ШІ, тонка — як апарат реально обертається");
    }
    ImGui::SeparatorText("Вивчені параметри, з якими летить");
    if (ImGui::BeginTable("bp", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
        for (int i = 0; i < fc::kTrickParams; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(fc::trickSpec(p.trick())[i].name);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", p.param(i));
        }
        ImGui::EndTable();
    }
    if (!sh.traj.empty()) {
        ImGui::SeparatorText("Траєкторія цього трюку");
        std::vector<TrajDraw> ts{{&sh.traj, IM_COL32(255, 210, 60, 255), 2.5f, true}};
        trajCanvas("##ftraj", ts, 0, ImVec2(ImGui::GetContentRegionAvail().x, 240));
    }
    if (!A.trick_result.empty()) ImGui::TextWrapped("Оцінка: %s", A.trick_result.c_str());
}

void startCompare(App& A) {
    if (A.cmp_running) return;
    if (A.cmp_thread.joinable()) A.cmp_thread.join();
    A.cmp_running = true; A.cmp_done = false;
    { std::lock_guard<std::mutex> l(A.cmp_mx); A.cmp_t.clear(); A.cmp_off.clear(); A.cmp_on.clear(); A.cmp_msg = "Рахую два польоти одночасно: без ШІ і з ШІ…"; }
    const AircraftFile f = A.file;
    const AeroTables tb = A.tables;
    A.cmp_thread = std::thread([&A, f, tb]() {
        std::vector<float> t, z[2];
        double rms[2] = {0, 0};
        auto fly = [&](int mode) {
            QpFlightRig rig(f, tb, 900u);
            rig.noise.gyro_rad_s = 0.01; rig.noise.airspeed_mps = 0.3; rig.noise.pos_m = 0.5; rig.noise.vel_mps = 0.1; rig.noise.att_rad = 0.004;
            rig.noise.delay_att_s = 0.02; rig.noise.delay_gps_s = 0.15; rig.noise.gps_hz = 10; rig.noise.rpm_frac = 0.01;
            WindConfig w; w.mean20ft = Vec3d{6.0, 2.4, 0}; rig.setWind(w, 977);
            fc::QpStep tk; tk.type = fc::QpStep::Takeoff; tk.p = {0, 0, 30}; rig.setMission({tk});
            rig.ai_mode = mode == 0 ? 0 : 2;
            double s2 = 0, nn = 0;
            rig.run(100, [&](const QpFlightRig& g) {
                const double tt = g.sim.time();
                if (tt > 30) {
                    const double dz = g.sim.state().pos.z - g.sim.derived().cg_z - 30;
                    if (mode == 0) t.push_back(float(tt));
                    z[mode].push_back(float(dz)); s2 += dz * dz; nn += 1;
                }
                return A.train_cancel.load();
            });
            rms[mode] = std::sqrt(s2 / std::fmax(nn, 1));
        };
        { std::thread other(fly, 1); fly(0); other.join(); }   // обидва польоти одночасно, на двох ядрах
        { std::lock_guard<std::mutex> l(A.cmp_mx); A.cmp_msg.clear(); }
        std::lock_guard<std::mutex> l(A.cmp_mx);
        A.cmp_t = t; A.cmp_off = z[0]; A.cmp_on = z[1]; A.cmp_rms[0] = rms[0]; A.cmp_rms[1] = rms[1];
        A.cmp_done = true; A.cmp_running = false;
    });
}

void brainNet(App& A) {
    QpFlightRig& r = *A.rig;
    {
        const auto& nt = r.advisor.net.net();
        ImGui::TextWrapped("Нейромережа «модель сил» (%d входів → %d → %d → 3 виходи) дивиться на те, що бачить контролер%s, і передбачає, з якою силою повітря зараз штовхає апарат. "
                           "Контролер віднімає цю силу наперед, а не чекає, поки апарат зсунеться. Кожні 0.02 с вона перевіряє себе: що передбачила — проти того, що показали датчики.",
                           nt.inputs(), nt.count ? nt.layers[0].out : 0, nt.count > 1 ? nt.layers[1].out : 0, r.advisor.net.hasMemory() ? ", і пам'ятає останню секунду" : "");
    }
    const char* am[] = {"вимкнено", "тінь — рахує, не керує", "радить контролеру"};
    for (int i = 0; i < 3; ++i) { if (ImGui::RadioButton(am[i], A.ai_mode == i)) { A.ai_mode = i; r.ai_mode = i; } if (i < 2) ImGui::SameLine(); }
    if (!r.advisor.net.loaded()) { ImGui::TextColored(ImVec4(1, 0.7f, 0.3f, 1), "Ваг мережі немає (ai/weights_force.h)."); return; }
    if (A.ai_mode == 0 || !r.ai_trace_valid) {
        ImGui::TextColored(ImVec4(0.6f, 0.85f, 1, 1), "%s", A.ai_mode == 0 ? "Мережа вимкнена. Обери «тінь» (вона рахує, але не керує) або «радить» — і запусти політ." : "Мережа ввімкнена — запусти політ («Старт»), щоб побачити її думки.");
    } else {
        const ai::ForceModel::Trace& tr = r.ai_trace;
        const auto& net = r.advisor.net.net();
        ImGui::SeparatorText("Що мережа «думає» зараз: активність нейронів (помаранчевий +, синій −), лінії — найсильніші сигнали");
        const int nin = tr.n_in;
        const float W = ImGui::GetContentRegionAvail().x, H = nin > 20 ? 640.0f : 400.0f;
        ImGui::InvisibleButton("##net", ImVec2(W, H));
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 o = ImGui::GetItemRectMin();
        dl->AddRectFilled(o, ImVec2(o.x + W, o.y + H), IM_COL32(20, 23, 30, 255), 6);
        const char* fn[ai::kForceFeatures] = {"V повітря", "V²", "Vx тіла", "Vy тіла", "Vz тіла", "крен", "тангаж", "оберти 1", "оберти 2", "оберти 3", "оберти 4",
                                              "гондола 1", "гондола 2", "гондола 3", "гондола 4", "руль висоти", "елерони"};
        const float xin = o.x + 170, x1 = o.x + W * 0.42f, x2 = o.x + W * 0.66f, xout = o.x + W - 150;
        auto ypos = [&](int i, int n) { return o.y + 16 + (H - 32) * (float(i) + 0.5f) / float(n); };
        auto yin = [&](int i) {   // входи: поточні ознаки — ширше (підписи), пам'ять — щільніше
            if (nin <= ai::kForceFeatures) return ypos(i, nin);
            const float big = 2.2f, tot = big * ai::kForceFeatures + float(nin - ai::kForceFeatures);
            const float u = i < ai::kForceFeatures ? big * (float(i) + 0.5f) : big * ai::kForceFeatures + float(i - ai::kForceFeatures) + 0.5f;
            return o.y + 16 + (H - 32) * u / tot;
        };
        auto drawLayer = [&](const ai::DenseQ& L, const float* a, float xa, float xb, int na, int nb, float rel, bool first = false) {
            struct C { float v; int i, j; };
            static std::vector<C> cs;
            cs.clear();
            for (int j = 0; j < nb; ++j) for (int i = 0; i < na; ++i) {
                const float w = float(L.w[size_t(j) * size_t(L.in) + size_t(i)]) * L.w_scale[j];
                const float v = w * a[i];
                if (std::fabs(v) > 1e-3f) cs.push_back({v, i, j});
            }
            const size_t keep = std::min<size_t>(cs.size(), 260);
            std::partial_sort(cs.begin(), cs.begin() + long(keep), cs.end(), [](const C& p, const C& q) { return std::fabs(p.v) > std::fabs(q.v); });
            const float vmax = keep ? std::fabs(cs[0].v) : 1.0f;
            for (size_t k = keep; k-- > 0;) {
                const float m = std::fabs(cs[k].v) / std::fmax(vmax, 1e-6f);
                dl->AddLine(ImVec2(xa, first ? yin(cs[k].i) : ypos(cs[k].i, na)), ImVec2(xb, ypos(cs[k].j, nb)), cs[k].v > 0 ? IM_COL32(255, 160, 60, int(25 + 170 * m * rel)) : IM_COL32(80, 160, 255, int(25 + 170 * m * rel)), 0.6f + 1.6f * m);
            }
        };
        drawLayer(net.layers[0], tr.norm, xin, x1, nin, tr.n1, 1.0f, true);
        drawLayer(net.layers[1], tr.h1, x1, x2, tr.n1, tr.n2, 1.0f);
        drawLayer(net.layers[2], tr.h2, x2, xout, tr.n2, 3, 1.0f);
        int unused = 0;
        const char* grp[7] = {"зміна за 0.1 с", "зміна за 0.3 с", "зміна за 1 с", "датчики: сила зараз", "сила 0.1 с тому", "сила 0.3 с тому", "сила 1 с тому"};
        for (int i = 0; i < nin; ++i) {
            bool off = true;
            for (int j = 0; j < tr.n1 && off; ++j) off = net.layers[0].w[size_t(j) * size_t(net.layers[0].in) + size_t(i)] == 0;
            unused += off;
            const ImVec2 c(xin, yin(i));
            dl->AddCircleFilled(c, nin > 20 ? 4.5f : 6.0f, off ? IM_COL32(60, 60, 66, 255) : actColor(std::tanh(tr.norm[i] * 0.7f)));
            const char* lab = nullptr;
            if (i < ai::kForceFeatures) lab = fn[i];
            else if (i < ai::kForceFeatures + 33 && (i - ai::kForceFeatures) % 11 == 0) lab = grp[(i - ai::kForceFeatures) / 11];
            else if (i == ai::kForceHistInputs) lab = "маса (вантаж)";
            else if (i >= ai::kForceFeatures + 33 && i < ai::kForceHistInputs && (i - ai::kForceFeatures - 33) % 3 == 0) lab = grp[3 + (i - ai::kForceFeatures - 33) / 3];
            if (lab) {
                const ImVec2 ts = ImGui::CalcTextSize(lab);
                const bool g = i >= ai::kForceFeatures;
                dl->AddText(ImVec2(c.x - 12 - ts.x, c.y - ts.y * 0.5f), off ? IM_COL32(110, 110, 118, 255) : (g ? IM_COL32(150, 210, 255, 255) : IM_COL32(205, 210, 220, 255)), lab);
                if (g) dl->AddLine(ImVec2(c.x - 10, c.y - 6), ImVec2(xin - 10, c.y - 6), IM_COL32(150, 210, 255, 60));
            }
        }
        for (int i = 0; i < tr.n1; ++i) dl->AddCircleFilled(ImVec2(x1, ypos(i, tr.n1)), 3.2f, actColor(tr.h1[i]));
        for (int i = 0; i < tr.n2; ++i) dl->AddCircleFilled(ImVec2(x2, ypos(i, tr.n2)), 3.2f, actColor(tr.h2[i]));
        const char* on[3] = {"сила вперед", "сила вбік", "сила вгору"};
        const float mg = float(r.ctl.massNominal() * QuadPlaneSim::kG);
        for (int k = 0; k < 3; ++k) {
            const ImVec2 c(xout, ypos(k, 3));
            dl->AddCircleFilled(c, 10, actColor(std::clamp(tr.out[k] / (0.3f * mg), -1.0f, 1.0f)));
            char b[96]; std::snprintf(b, sizeof b, "%s\nШІ: %+.1f Н\nправда: %+.1f Н", on[k], tr.out[k], r.ai_truth[k]);
            dl->AddText(ImVec2(c.x + 16, c.y - 24), IM_COL32(230, 232, 240, 255), b);
        }
        dl->AddText(ImVec2(xin - 40, o.y + H - 18), IM_COL32(150, 150, 160, 255), "входи (датчики)");
        dl->AddText(ImVec2(x1 - 40, o.y + H - 18), IM_COL32(150, 150, 160, 255), "шар 1 (64)");
        dl->AddText(ImVec2(x2 - 40, o.y + H - 18), IM_COL32(150, 150, 160, 255), "шар 2 (64)");
        if (unused) ImGui::TextDisabled("сірі входи (%d) мережа свідомо не використовує: інакше вона вчилась би «наслідок → причина»", unused);
        else ImGui::TextDisabled("усі входи працюють; блакитні підписи — пам'ять: як змінювався політ і що показували датчики сили останню секунду");

        ImGui::SeparatorText("Самоперевірка: передбачила → подивилась, що сталось → наскільки собі довіряю");
        {
            const float* pp = r.ai_check_pred; const float* mm = r.ai_check_meas;
            ImGui::TextWrapped("0.02 с тому я передбачила силу вгору %+.1f Н, вперед %+.1f Н. Датчики (акселерометр мінус тяга гвинтів) показали %+.1f і %+.1f Н — промах %.1f Н.",
                               pp[2], pp[0], mm[2], mm[0], std::sqrt((pp[0] - mm[0]) * (pp[0] - mm[0]) + (pp[1] - mm[1]) * (pp[1] - mm[1]) + (pp[2] - mm[2]) * (pp[2] - mm[2])));
            ImGui::TextWrapped("Середній промах за останні ~2 с: %.2f Н (датчики самі шумлять на ≈ 0.6 Н).", r.ai_check_rms);
            const float tr2 = r.ai_trust;
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, tr2 > 0.7f ? ImVec4(0.35f, 0.85f, 0.45f, 1) : (tr2 > 0.35f ? ImVec4(0.95f, 0.75f, 0.25f, 1) : ImVec4(0.95f, 0.35f, 0.3f, 1)));
            char tb[96]; std::snprintf(tb, sizeof tb, "довіра до себе %.0f%%", tr2 * 100);
            ImGui::ProgressBar(tr2, ImVec2(-1, 22), tb);
            ImGui::PopStyleColor();
            ImGui::TextDisabled("%s", tr2 > 0.7f ? "Передбачення збігаються з реальністю — контролер бере пораду повністю."
                                    : (tr2 > 0.35f ? "Промахуюсь частіше, ніж зазвичай — контролер бере лише частину поради."
                                                   : "Щось незнайоме (сильний порив, відмова, нове середовище) — я майже не втручаюсь, керує контролер."));
        }

        ImGui::SeparatorText("Що з цього отримує контролер");
        const float used = r.ai_used_acc[2];
        ImGui::TextWrapped("Мережа каже: повітря штовхає апарат %s на %.1f Н (%.0f%% ваги), правда моделі — %.1f Н. %s",
                           tr.out[2] >= 0 ? "вгору" : "вниз", std::fabs(tr.out[2]), 100 * std::fabs(tr.out[2]) / mg, r.ai_truth[2],
                           A.ai_mode == 2 ? (r.ai_fade > 0.01f ? "" : (r.ai_trust < 0.3f ? "Зараз поправка вимкнена: довіра до себе замала." : "Зараз поправка вимкнена: це швидкість польоту літаком — там мережа менш точна, і контролер її не слухає."))
                                          : "Режим «тінь»: контролер цю пораду не використовує.");
        if (A.ai_mode == 2 && r.ai_fade > 0.01f)
            ImGui::TextColored(ImVec4(0.5f, 1, 0.6f, 1), "Контролер отримує поправку %+.2f м/с² наперед (бере %.0f%% поради) — не чекає, поки апарат просяде чи підскочить.", used, r.ai_fade * 100);
    }
    if (A.aih.t.size() > 2) {
        const std::vector<float> tx = fv(A.aih.t);
        chart("aiz", {{tx, fv(A.aih.mz), "вгору: датчики", IM_COL32(120, 200, 255, 110), 1.0f, false}, {tx, fv(A.aih.tz), "вгору: правда", IM_COL32(255, 255, 255, 230), 2.5f, false},
                      {tx, fv(A.aih.pz), "вгору: ШІ", IM_COL32(255, 155, 50, 255), 2.0f, false},
                      {tx, fv(A.aih.tx), "вперед: правда", IM_COL32(170, 170, 180, 160), 1.5f, false}, {tx, fv(A.aih.px), "вперед: ШІ", IM_COL32(100, 175, 255, 230), 1.5f, false}},
              210, "с", "сила повітря, Н");
        chart("trust", {{tx, fv(A.aih.trust), "довіра до себе, %", IM_COL32(110, 230, 120, 255), 2.0f, false}}, 120, "с", "%");
    }
    if (r.ai_stats.n > 0) ImGui::Text("Точність у висінні: похибка %.2f Н при силі %.2f Н — мережа пояснює %.0f%% сили", r.aiRmsError(), r.aiRmsTruth(),
                                      100 * std::clamp(1 - std::pow(r.aiRmsError() / std::fmax(r.aiRmsTruth(), 1e-6), 2.0), 0.0, 1.0));

    ImGui::SeparatorText("Доказ: та сама хвилина висіння у вітрі 6 м/с — без ШІ і з ШІ");
    if (A.cmp_running) ImGui::BeginDisabled();
    if (ImGui::Button(A.cmp_running ? "Рахую…" : "Порівняти (кілька секунд)", ImVec2(-1, 0))) startCompare(A);
    if (A.cmp_running) ImGui::EndDisabled();
    if (!A.cmp_running && A.cmp_thread.joinable()) A.cmp_thread.join();
    std::lock_guard<std::mutex> l(A.cmp_mx);
    if (!A.cmp_msg.empty()) ImGui::TextDisabled("%s", A.cmp_msg.c_str());
    if (A.cmp_done && !A.cmp_t.empty()) {
        const double gain = 100 * (1 - A.cmp_rms[1] / std::fmax(A.cmp_rms[0], 1e-6));
        if (A.big_font) ImGui::PushFont(A.big_font, 20.0f);
        ImGui::TextColored(gain > 0 ? ImVec4(0.5f, 1, 0.6f, 1) : ImVec4(1, 0.6f, 0.4f, 1), "Похибка висоти: без ШІ %.2f м → з ШІ %.2f м (%s%.0f%%)", A.cmp_rms[0], A.cmp_rms[1], gain > 0 ? "−" : "+", std::fabs(gain));
        if (A.big_font) ImGui::PopFont();
        chart("cmp", {{A.cmp_t, A.cmp_off, "без ШІ", IM_COL32(190, 190, 205, 255), 1.5f, false}, {A.cmp_t, A.cmp_on, "з ШІ", IM_COL32(255, 155, 50, 255), 2.0f, false}}, 210, "с", "відхилення висоти, м");
    }
}

void panelBrain(App& A, float W, float H, float left, float right, float bottom) {
    if (!A.show_brain) return;
    if (A.brain_tab_req >= 0) A.brain_full = A.brain_tab_req != 1;
    static bool last_full = !A.brain_full;
    const bool re = last_full != A.brain_full;
    last_full = A.brain_full;
    const float bw = A.brain_full ? W - left - right : std::fmax(560.0f, (W - left - right) * 0.6f);
    ImGui::SetNextWindowPos(ImVec2(W - right - bw, 0), re ? ImGuiCond_Always : ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(bw, A.brain_full ? H : H - bottom), re ? ImGuiCond_Always : ImGuiCond_Appearing);
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (!ImGui::Begin("Мозок ШІ — що він думає і робить", &A.show_brain)) { ImGui::End(); return; }
    if (ImGui::SmallButton(A.brain_full ? "Зменшити (видно 3D)" : "Збільшити")) A.brain_full = !A.brain_full;
    ImGui::SameLine(); ImGui::Checkbox("Сили в 3D: зелені — тяга гвинтів, блакитна — повітря, рожева — що передбачає ШІ", &A.show_forces);
    if (ImGui::BeginTabBar("brain")) {
        const char* tabs[3] = {"Навчання трюку", "Політ трюку", "Нейромережа сил"};
        for (int k = 0; k < 3; ++k) {
            const ImGuiTabItemFlags fl = A.brain_tab_req == k ? ImGuiTabItemFlags_SetSelected : 0;
            if (ImGui::BeginTabItem(tabs[k], nullptr, fl)) {
                ImGui::BeginChild("bc");
                if (k == 0) brainTrain(A); else if (k == 1) brainFlight(A); else brainNet(A);
                if (A.replay_req) { A.replay_req = false; replayEpisode(A, A.replay_trick, A.replay_u); A.brain_tab_req = 1; }
                ImGui::EndChild();
                ImGui::EndTabItem();
            }
        }
        A.brain_tab_req = -1;
        ImGui::EndTabBar();
    }
    ImGui::End();
}
