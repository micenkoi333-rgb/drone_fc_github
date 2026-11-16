// mini_json.h — мінімальний JSON-читач (об'єкти, масиви, рядки, числа, true/false/null) для aircraft/aero_tables.json.
// Без зовнішніх бібліотек. Помилки — std::runtime_error з позицією у тексті.
#pragma once
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim {

struct Json {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double num = 0;
    std::string str;
    std::vector<Json> arr;
    std::map<std::string, Json> obj;

    const Json& at(const std::string& key) const {
        auto it = obj.find(key);
        if (type != Object || it == obj.end()) throw std::runtime_error("JSON: немає ключа «" + key + "»");
        return it->second;
    }
    bool has(const std::string& key) const { return type == Object && obj.count(key) != 0; }
    double number() const {
        if (type != Number) throw std::runtime_error("JSON: очікувалось число");
        return num;
    }
    std::vector<double> numbers() const {
        if (type != Array) throw std::runtime_error("JSON: очікувався масив чисел");
        std::vector<double> v;
        v.reserve(arr.size());
        for (const Json& e : arr) v.push_back(e.number());
        return v;
    }

    static Json parse(const std::string& text) {
        size_t i = 0;
        Json v = value(text, i);
        skip(text, i);
        if (i != text.size()) fail(text, i, "зайві символи після значення");
        return v;
    }
    static Json load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("не вдалось відкрити " + path);
        std::stringstream ss;
        ss << f.rdbuf();
        return parse(ss.str());
    }

private:
    [[noreturn]] static void fail(const std::string&, size_t i, const char* what) {
        throw std::runtime_error("JSON, позиція " + std::to_string(i) + ": " + what);
    }
    static void skip(const std::string& s, size_t& i) {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i;
    }
    static std::string quoted(const std::string& s, size_t& i) {
        if (i >= s.size() || s[i] != '"') fail(s, i, "очікувався рядок");
        std::string out;
        for (++i; i < s.size() && s[i] != '"'; ++i) {
            if (s[i] == '\\' && i + 1 < s.size()) {
                ++i;
                switch (s[i]) {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'u': i += 4; out += '?'; break;   // \uXXXX у наших даних лише в нотатках — не потрібні
                    default: out += s[i];
                }
            } else {
                out += s[i];
            }
        }
        if (i >= s.size()) fail(s, i, "рядок не закрито");
        ++i;
        return out;
    }
    static Json value(const std::string& s, size_t& i) {
        skip(s, i);
        if (i >= s.size()) fail(s, i, "несподіваний кінець");
        Json j;
        if (s[i] == '{') {
            j.type = Object;
            ++i;
            skip(s, i);
            if (i < s.size() && s[i] == '}') { ++i; return j; }
            for (;;) {
                skip(s, i);
                std::string k = quoted(s, i);
                skip(s, i);
                if (i >= s.size() || s[i] != ':') fail(s, i, "очікувалось «:»");
                ++i;
                j.obj[k] = value(s, i);
                skip(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; return j; }
                fail(s, i, "очікувалось «,» або «}»");
            }
        }
        if (s[i] == '[') {
            j.type = Array;
            ++i;
            skip(s, i);
            if (i < s.size() && s[i] == ']') { ++i; return j; }
            for (;;) {
                j.arr.push_back(value(s, i));
                skip(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == ']') { ++i; return j; }
                fail(s, i, "очікувалось «,» або «]»");
            }
        }
        if (s[i] == '"') { j.type = String; j.str = quoted(s, i); return j; }
        if (s.compare(i, 4, "true") == 0) { j.type = Bool; j.b = true; i += 4; return j; }
        if (s.compare(i, 5, "false") == 0) { j.type = Bool; i += 5; return j; }
        if (s.compare(i, 4, "null") == 0) { i += 4; return j; }
        size_t used = 0;
        try { j.num = std::stod(s.substr(i), &used); } catch (...) { fail(s, i, "очікувалось значення"); }
        if (!std::isfinite(j.num)) fail(s, i, "число не скінченне");
        j.type = Number;
        i += used;
        return j;
    }
};

}  // namespace sim
