#include "NgspiceRunner.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace Kirchhoff {
namespace {

std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
}

// Strip a "v(...)" / "i(...)" wrapper and any "plot." prefix, lower-cased — so average("v(Vout)") and
// average("vout") both resolve to the captured vector "vout" (ngspice may name it "tran1.vout").
std::string canonical_vec(std::string name) {
    name = to_lower(name);
    if (name.size() > 3 && (name.compare(0,2,"v(")==0 || name.compare(0,2,"i(")==0) && name.back()==')')
        name = name.substr(2, name.size() - 3);
    auto dot = name.rfind('.');
    if (dot != std::string::npos) name = name.substr(dot + 1);
    return name;
}

} // namespace

std::optional<double> NgspiceRunResult::average(const std::string& name, double from, double to) const {
    const std::string want = canonical_vec(name);
    const std::vector<double>* v = nullptr;
    for (const auto& kv : vectors) {
        if (canonical_vec(kv.first) == want) { v = &kv.second; break; }
    }
    if (!v || v->size() != time.size() || time.empty()) return std::nullopt;
    // Trapezoidal mean over [from, to].
    double area = 0.0, span = 0.0;
    for (size_t i = 1; i < time.size(); ++i) {
        double t0 = time[i-1], t1 = time[i];
        if (t1 <= from || t0 >= to) continue;
        double a = std::max(t0, from), b = std::min(t1, to);
        if (b <= a) continue;
        // linear interpolation of the sample values at the clipped sub-interval ends
        auto interp = [&](double t){
            double f = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
            return (*v)[i-1] + f * ((*v)[i] - (*v)[i-1]);
        };
        area += 0.5 * (interp(a) + interp(b)) * (b - a);
        span += (b - a);
    }
    if (span <= 0.0) return std::nullopt;
    return area / span;
}

} // namespace Kirchhoff

#ifdef ENABLE_NGSPICE
#include <ngspice/sharedspice.h>
#include <chrono>
#include <thread>

namespace Kirchhoff {
namespace {

// Per-run capture state, passed to libngspice via the userData pointer (avoids a process-global
// singleton; libngspice itself is global, so runs are serialized by the caller, but the state pointer
// keeps the callbacks self-contained).
struct Capture {
    std::vector<double> time;
    std::map<std::string, std::vector<double>> vectors;
    bool complete = false;
    bool error = false;
    std::string errorMessage;
};

extern "C" int kh_send_char(char* /*out*/, int /*id*/, void* /*ud*/) { return 0; }

extern "C" int kh_send_stat(char* out, int /*id*/, void* ud) {
    if (out && ud && std::strstr(out, "--ready--")) static_cast<Capture*>(ud)->complete = true;
    return 0;
}

extern "C" int kh_controlled_exit(int status, bool /*immediate*/, bool /*quit*/, int /*id*/, void* ud) {
    if (ud) {
        auto* c = static_cast<Capture*>(ud);
        c->complete = true;
        if (status != 0) { c->error = true; c->errorMessage = "ngspice exit status " + std::to_string(status); }
    }
    return status;
}

extern "C" int kh_send_data(pvecvaluesall vecs, int /*count*/, int /*id*/, void* ud) {
    if (!ud || !vecs) return 0;
    auto* c = static_cast<Capture*>(ud);
    for (int i = 0; i < vecs->veccount; ++i) {
        const char* nm = vecs->vecsa[i]->name;
        if (!nm) continue;
        std::string lname = to_lower(nm);
        bool isTime = (lname == "time") ||
                      (lname.size() > 5 && lname.compare(lname.size()-5, 5, ".time") == 0);
        if (isTime) c->time.push_back(vecs->vecsa[i]->creal);
        else        c->vectors[nm].push_back(vecs->vecsa[i]->creal);
    }
    return 0;
}

extern "C" int kh_send_initdata(pvecinfoall /*info*/, int /*id*/, void* /*ud*/) { return 0; }

extern "C" int kh_bg_running(bool running, int /*id*/, void* ud) {
    if (ud && !running) static_cast<Capture*>(ud)->complete = true;
    return 0;
}

// Drop a trailing `.control … .endc` block (case-insensitive) — we run via the API, not the deck's
// interactive block — and keep everything else (circuit + .options/.ic/.tran + .end).
std::string strip_control_block(const std::string& deck) {
    std::istringstream in(deck);
    std::ostringstream out;
    std::string line;
    bool inControl = false;
    while (std::getline(in, line)) {
        std::string t = to_lower(line);
        // left-trim for the directive check
        size_t p = t.find_first_not_of(" \t");
        std::string head = (p == std::string::npos) ? "" : t.substr(p);
        if (!inControl && head.compare(0, 8, ".control") == 0) { inControl = true; continue; }
        if (inControl) { if (head.compare(0, 5, ".endc") == 0) inControl = false; continue; }
        out << line << "\n";
    }
    return out.str();
}

} // namespace

bool ngspice_in_process_available() { return true; }

NgspiceRunResult run_ngspice_in_process(const std::string& deck, double timeoutSeconds) {
    NgspiceRunResult result;
    Capture cap;

    if (ngSpice_Init(kh_send_char, kh_send_stat, kh_controlled_exit,
                     kh_send_data, kh_send_initdata, kh_bg_running, &cap) != 0)
        throw std::runtime_error("libngspice: ngSpice_Init failed");

    // WASM workaround #1: under emscripten there is no /proc/meminfo, so ngspice's available-memory
    // probe returns 0 and aborts with a false "not enough memory". Disabling the check is harmless on
    // native (where memory is plentiful) and required for the in-browser run. (Workaround #2 — treating
    // a non-empty time vector as completion when the sync WASM build's `run` returns without firing the
    // bg-thread callback — is handled by the wait loop below via ngSpice_running() + cap.time.)
    ngSpice_Command(const_cast<char*>("set no_mem_check"));

    // Build the circuit line array. ngspice treats line 1 as the title, so prepend one so the first
    // real card is not swallowed. Storage must outlive the char* array (no realloc after .data()).
    std::vector<std::string> lines;
    lines.emplace_back("* kirchhoff in-process run");
    std::istringstream in(strip_control_block(deck));
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);

    std::vector<char*> cir;
    cir.reserve(lines.size() + 1);
    for (auto& l : lines) cir.push_back(const_cast<char*>(l.c_str()));
    cir.push_back(nullptr);

    if (ngSpice_Circ(cir.data()) != 0) {
        ngSpice_Command(const_cast<char*>("remcirc"));
        throw std::runtime_error("libngspice: ngSpice_Circ failed to load the deck");
    }

    ngSpice_Command(const_cast<char*>("run"));

    // The native shared library runs the transient on a background thread; wait for a completion
    // callback, with a fallback (data captured + thread idle) and a hard timeout.
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<long>(timeoutSeconds * 1000.0));
    while (!cap.complete) {
        if (!ngSpice_running() && !cap.time.empty()) break;   // finished, callback may not have fired
        if (std::chrono::steady_clock::now() > deadline) {
            ngSpice_Command(const_cast<char*>("bg_halt"));
            result.error = "timeout after " + std::to_string(timeoutSeconds) + "s";
            ngSpice_Command(const_cast<char*>("remcirc"));
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Free the circuit so the next run starts clean (libngspice keeps loaded circuits otherwise).
    ngSpice_Command(const_cast<char*>("remcirc"));

    result.time = std::move(cap.time);
    result.vectors = std::move(cap.vectors);
    result.error = cap.errorMessage;
    result.success = !cap.error && !result.time.empty();
    return result;
}

} // namespace Kirchhoff

#else  // ENABLE_NGSPICE not defined

namespace Kirchhoff {
bool ngspice_in_process_available() { return false; }
NgspiceRunResult run_ngspice_in_process(const std::string&, double) {
    throw std::runtime_error("Kirchhoff was built without libngspice (ENABLE_NGSPICE off); "
                             "use the ngspice CLI or rebuild with -DENABLE_NGSPICE=ON");
}
} // namespace Kirchhoff

#endif
