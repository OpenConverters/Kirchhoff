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

// Canonicalize a vector name for matching: lower-case; strip a "v(...)" / "i(...)" wrapper; strip any
// "plot." prefix (ngspice may name a vector "tran1.vout"); and strip a trailing "#branch" (ngspice names
// a source's current "VVin#branch"). So average("v(Vout)")/"vout"/"tran1.vout" and
// average("i(VVin)")/"VVin#branch" all resolve to their captured vector.
std::string canonical_vec(std::string name) {
    name = to_lower(name);
    if (name.size() > 3 && (name.compare(0,2,"v(")==0 || name.compare(0,2,"i(")==0) && name.back()==')')
        name = name.substr(2, name.size() - 3);
    auto dot = name.rfind('.');
    if (dot != std::string::npos) name = name.substr(dot + 1);
    auto br = name.rfind("#branch");
    if (br != std::string::npos && br + 7 == name.size()) name = name.substr(0, br);
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

void NgspiceRunResult::drop_samples_before(double tStart) {
    size_t k = 0;
    while (k < time.size() && time[k] < tStart) ++k;
    if (k == 0) return;
    time.erase(time.begin(), time.begin() + static_cast<std::ptrdiff_t>(k));
    for (auto& kv : vectors) {
        auto& v = kv.second;
        const size_t n = std::min(k, v.size());
        v.erase(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n));
    }
}

} // namespace Kirchhoff

#ifdef ENABLE_NGSPICE
#include <ngspice/sharedspice.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

namespace Kirchhoff {
namespace {

// Per-run capture state, passed to libngspice via the userData pointer. The ngspice transient runs on a
// BACKGROUND thread that writes this state concurrently with the wait loop reading it, so the shared flags
// are atomic and the sample buffers are mutex-guarded. The state is heap-allocated and kept alive past the
// function return (see g_activeCapture) so a lingering background thread can never write freed memory.
struct Capture {
    std::mutex mtx;                                            // guards time / vectors / errorMessage / console
    std::vector<double> time;
    std::map<std::string, std::vector<double>> vectors;
    std::string console;                                       // full SendChar text (for .meas parsing)
    std::atomic<bool> complete{false};
    std::atomic<bool> error{false};
    std::string errorMessage;
};

// Definitive ngspice signatures that the transient did NOT complete correctly. ngspice does not always
// call controlled_exit on a mid-run abort (the bg thread just stops), so these are detected from the
// character output — otherwise a crashed run returns partial data that looks like success.
bool line_is_fatal(const char* out) {
    static const char* const sigs[] = {
        "Timestep too small", "simulation(s) aborted", "singular matrix", "Fatal error"
    };
    for (const char* sig : sigs)
        if (std::strstr(out, sig)) return true;
    return false;
}

extern "C" int kh_send_char(char* out, int /*id*/, void* ud) {
    if (!out || !ud) return 0;
    auto* c = static_cast<Capture*>(ud);
    std::lock_guard<std::mutex> lk(c->mtx);
    // Capture the full console stream so run_ngspice_console can return the `.meas`
    // results (libngspice strips a leading "stdout "/"stderr " tag from each line).
    const char* text = out;
    if (std::strncmp(out, "stdout ", 7) == 0) text = out + 7;
    else if (std::strncmp(out, "stderr ", 7) == 0) text = out + 7;
    c->console.append(text);
    c->console.push_back('\n');
    if (line_is_fatal(out)) {
        c->error = true;
        if (c->errorMessage.empty()) c->errorMessage = out;
    }
    return 0;
}

extern "C" int kh_send_stat(char* out, int /*id*/, void* ud) {
    if (out && ud && std::strstr(out, "--ready--")) static_cast<Capture*>(ud)->complete = true;
    return 0;
}

extern "C" int kh_controlled_exit(int status, bool /*immediate*/, bool /*quit*/, int /*id*/, void* ud) {
    if (ud) {
        auto* c = static_cast<Capture*>(ud);
        if (status != 0) {
            std::lock_guard<std::mutex> lk(c->mtx);
            c->error = true;
            if (c->errorMessage.empty()) c->errorMessage = "ngspice exit status " + std::to_string(status);
        }
        c->complete = true;
    }
    return status;
}

extern "C" int kh_send_data(pvecvaluesall vecs, int /*count*/, int /*id*/, void* ud) {
    if (!ud || !vecs) return 0;
    auto* c = static_cast<Capture*>(ud);
    std::lock_guard<std::mutex> lk(c->mtx);
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

// The most recent run's capture state, held so it outlives run_ngspice_in_process even if a timed-out
// background thread is still (briefly) writing to it. libngspice serializes runs, so a single slot is
// enough; the next run replaces it only after that run has waited for its own thread to stop.
std::shared_ptr<Capture> g_activeCapture;

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
    // Heap-allocate the capture and publish it before wiring the callbacks. Replacing g_activeCapture here
    // drops the previous run's state (whose thread we already waited for) and keeps THIS run's state alive
    // for the whole function — and beyond, if we have to abandon a hung background thread.
    auto cap = std::make_shared<Capture>();
    g_activeCapture = cap;

    if (ngSpice_Init(kh_send_char, kh_send_stat, kh_controlled_exit,
                     kh_send_data, kh_send_initdata, kh_bg_running, cap.get()) != 0)
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
    auto timeHasSamples = [&] {                                // read shared buffer under the lock
        std::lock_guard<std::mutex> lk(cap->mtx);
        return !cap->time.empty();
    };
    while (!cap->complete) {
        if (!ngSpice_running() && timeHasSamples()) break;    // finished, callback may not have fired
        if (std::chrono::steady_clock::now() > deadline) {
            // bg_halt only REQUESTS a stop; wait for the background thread to actually stop before we let
            // the circuit and capture state go, so it can never write into freed memory.
            ngSpice_Command(const_cast<char*>("bg_halt"));
            for (int i = 0; i < 500 && ngSpice_running(); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            ngSpice_Command(const_cast<char*>("remcirc"));
            result.error = "timeout after " + std::to_string(timeoutSeconds) + "s";
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Free the circuit so the next run starts clean (libngspice keeps loaded circuits otherwise).
    ngSpice_Command(const_cast<char*>("remcirc"));

    // The thread has stopped (complete set by bg_running(false)/controlled_exit, or ngSpice_running()==false
    // above); copy the captured data out under the lock.
    {
        std::lock_guard<std::mutex> lk(cap->mtx);
        result.time = cap->time;
        result.vectors = cap->vectors;
        result.error = cap->errorMessage;
    }
    result.success = !cap->error && !result.time.empty();
    return result;
}

std::string run_ngspice_console(const std::string& deck, double timeoutSeconds) {
    // In-process replacement for `ngspice -b <deck>`: load the deck INCLUDING its
    // `.control … .endc` block, which libngspice executes on load (run/meas/wrdata),
    // and return the captured console text so the caller can parse `.meas` results.
    auto cap = std::make_shared<Capture>();
    g_activeCapture = cap;

    if (ngSpice_Init(kh_send_char, kh_send_stat, kh_controlled_exit,
                     kh_send_data, kh_send_initdata, kh_bg_running, cap.get()) != 0)
        throw std::runtime_error("libngspice: ngSpice_Init failed");
    ngSpice_Command(const_cast<char*>("set no_mem_check"));

    // Keep the .control block (do NOT strip it) — that is what runs the sim + meas.
    std::vector<std::string> lines;
    lines.emplace_back("* kirchhoff in-process console run");
    std::istringstream in(deck);
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);

    std::vector<char*> cir;
    cir.reserve(lines.size() + 1);
    for (auto& l : lines) cir.push_back(const_cast<char*>(l.c_str()));
    cir.push_back(nullptr);

    // ngSpice_Circ runs the .control block's foreground `run` synchronously, so by the
    // time it returns the sim + meas + wrdata have completed. Guard with a watchdog
    // thread that halts a runaway sim after the timeout (best-effort).
    std::atomic<bool> done{false};
    std::thread watchdog([&] {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(static_cast<long>(timeoutSeconds * 1000.0));
        while (!done) {
            if (std::chrono::steady_clock::now() > deadline) {
                ngSpice_Command(const_cast<char*>("bg_halt"));
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    if (ngSpice_Circ(cir.data()) != 0) {
        done = true;
        watchdog.join();
        ngSpice_Command(const_cast<char*>("remcirc"));
        throw std::runtime_error("libngspice: ngSpice_Circ failed to load the deck");
    }
    done = true;
    watchdog.join();

    ngSpice_Command(const_cast<char*>("remcirc"));

    std::lock_guard<std::mutex> lk(cap->mtx);
    return cap->console;
}

} // namespace Kirchhoff

#else  // ENABLE_NGSPICE not defined

namespace Kirchhoff {
bool ngspice_in_process_available() { return false; }
NgspiceRunResult run_ngspice_in_process(const std::string&, double) {
    throw std::runtime_error("Kirchhoff was built without libngspice (ENABLE_NGSPICE off); "
                             "use the ngspice CLI or rebuild with -DENABLE_NGSPICE=ON");
}
std::string run_ngspice_console(const std::string&, double) {
    throw std::runtime_error("Kirchhoff was built without libngspice (ENABLE_NGSPICE off); "
                             "rebuild with -DENABLE_NGSPICE=ON");
}
} // namespace Kirchhoff

#endif
