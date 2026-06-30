#pragma once

// Kirchhoff::Settings — project-wide configuration, mirroring MKF's OpenMagnetics::Settings singleton.
// One process-global instance (Settings::GetInstance()) carries the knobs that are not per-design: the
// default run engine, whether to prefer the in-process libngspice runner over the ngspice CLI, and
// verbosity. Per-design knobs stay in the TAS `config` object (KirchhoffConfig.hpp); Settings is for
// global policy. reset() restores the defaults (handy in tests).
//
// (Header-only inline singleton — the inline function-local static is one instance across all TUs.)

namespace Kirchhoff {

// Which engine computes a converter's operating point. SPICE (default) = run the ngspice deck (the
// in-process libngspice runner when available, else the CLI). ANALYTICAL = analytical_operating_point()
// — simulator-free, magnetics-independent, WASM-clean. Lives here (config), used by Analytical.hpp.
enum class RunEngine { SPICE, ANALYTICAL };

class Settings {
  public:
    static Settings& GetInstance() {
        static Settings instance;
        return instance;
    }

    void reset() {
        _runEngine = RunEngine::SPICE;
        _preferInProcessNgspice = true;
        _verbose = false;
    }

    // Default run engine (per-call overrides still win where a function takes an explicit engine).
    RunEngine get_run_engine() const { return _runEngine; }
    void set_run_engine(RunEngine v) { _runEngine = v; }

    // When SPICE is selected and Kirchhoff was built with ENABLE_NGSPICE, prefer the in-process
    // libngspice runner over shelling out to the `ngspice` CLI. Ignored when libngspice is unavailable.
    bool get_prefer_in_process_ngspice() const { return _preferInProcessNgspice; }
    void set_prefer_in_process_ngspice(bool v) { _preferInProcessNgspice = v; }

    bool get_verbose() const { return _verbose; }
    void set_verbose(bool v) { _verbose = v; }

  private:
    Settings() = default;
    Settings(const Settings&) = delete;
    Settings& operator=(const Settings&) = delete;

    RunEngine _runEngine = RunEngine::SPICE;
    bool _preferInProcessNgspice = true;
    bool _verbose = false;
};

} // namespace Kirchhoff
