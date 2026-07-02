#include "Cmc.hpp"
#include "NgspiceRunner.hpp"          // run_ngspice_in_process + ngspice_in_process_available
#include "processors/WaveformProcessor.h"  // the shared DSP (MKF), reused — not re-implemented

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

// CMC ngspice simulations — ported from MKF converter_models/CommonModeChoke.cpp
// (generate_realistic_cmc_circuit :767, simulate_realistic_cmc :890, generate_ngspice_circuit/LISN :490,
// simulate_and_extract_waveforms :608, simulate_and_extract_operating_points :695), adapted onto
// Kirchhoff::run_ngspice_in_process (which returns the raw transient vectors, so period extraction /
// tstart windowing is done by the .tran directive in the deck, exactly as the MKF decks already did).
//
// Solver options match the CMC spice_config MKF used (reltol=1e-3 abstol=1e-9 vntol=1e-6), which is also
// the value KH's TasAssembler emits — one consistent tolerance set across every KH deck.

namespace Kirchhoff {
using nlohmann::json;
using WP = OpenMagnetics::WaveformProcessor;

namespace {

const char* kCmcOptions = ".options reltol=1e-3 abstol=1e-9 vntol=1e-6\n";

// V_mains → CM excitation scaling, calibrated so 230 V is a no-op (mirrors ConverterAnalytical's
// cmc_excitation_scaling; dV/dt ∝ V_bus ≈ √2·V_mains → I_cm scales linearly with the mains voltage).
double cmc_excitation_scaling(double operatingVoltage) {
    return operatingVoltage <= 0.0 ? 0.0 : operatingVoltage / 230.0;
}

// Canonicalize an ngspice vector name for matching (lower-case; strip v()/i() wrapper, plot prefix, and
// #branch suffix) — same rule NgspiceRunner uses internally, replicated here for the by-name lookup.
std::string canon(std::string name) {
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c){ return std::tolower(c); });
    if (name.size() > 3 && (name.compare(0,2,"v(")==0 || name.compare(0,2,"i(")==0) && name.back()==')')
        name = name.substr(2, name.size() - 3);
    auto dot = name.rfind('.');
    if (dot != std::string::npos) name = name.substr(dot + 1);
    auto br = name.rfind("#branch");
    if (br != std::string::npos && br + 7 == name.size()) name = name.substr(0, br);
    return name;
}

// Look up a captured vector by (canonical) name; empty if absent.
std::vector<double> vec_of(const NgspiceRunResult& r, const std::string& name) {
    const std::string want = canon(name);
    for (const auto& kv : r.vectors)
        if (canon(kv.first) == want) return kv.second;
    return {};
}

// ── The "realistic"/ideal per-winding CM excitation deck (MKF generate_realistic_cmc_circuit) ─────────
// Each winding is driven by its own current source: DC line bias + sinusoidal CM ripple I_cm = C·dV/dt.
// The windings are UNCOUPLED (self-inductance L each) so the deck matches the closed-form analytical
// model (V = L·ω·I with no (1+k) factor). tstart cuts the transient to the steady-state window.
std::string ideal_cmc_deck(int numWindings, double inductance, double operatingCurrent,
                           double operatingVoltage, double excitationFreq, double parasiticCapPf,
                           double dvdtVPerNs, int numberOfPeriods, int numberOfSteadyStatePeriods) {
    if (numberOfPeriods < 1) numberOfPeriods = 1;
    if (numberOfSteadyStatePeriods < 0) numberOfSteadyStatePeriods = 0;
    const double cmNoiseCurrent = (parasiticCapPf * 1e-12) * (dvdtVPerNs * 1e9)
                                * cmc_excitation_scaling(operatingVoltage);
    const int totalPeriods = numberOfSteadyStatePeriods + numberOfPeriods;
    const double simTime = double(totalPeriods) / excitationFreq;
    const double tStart = double(numberOfSteadyStatePeriods) / excitationFreq;
    const double timeStep = 1.0 / (excitationFreq * 50.0);

    std::ostringstream c;
    c << "* CMC ideal per-winding CM excitation deck\n";
    c << "* Per-winding CM current source (DC line bias + sinusoidal CM ripple)\n";
    for (int w = 0; w < numWindings; ++w)
        c << "Icm" << w << " 0 src" << w << " SIN(" << operatingCurrent << " " << cmNoiseCurrent
          << " " << excitationFreq << " 0 0 0)\n";
    c << "* Current sense (0V in series)\n";
    for (int w = 0; w < numWindings; ++w)
        c << "Vsense" << w << " src" << w << " cmc_in" << w << " 0\n";
    c << "* CMC windings (uncoupled self-inductance, matches analytical)\n";
    for (int w = 0; w < numWindings; ++w)
        c << "Lcmc" << w << " cmc_in" << w << " cmc_out" << w << " " << std::scientific << inductance
          << std::defaultfloat << "\n";
    c << "* Winding return path to ground\n";
    for (int w = 0; w < numWindings; ++w)
        c << "Rgnd" << w << " cmc_out" << w << " 0 1u\n";
    c << "* Transient (tstart = steady-state window)\n";
    c << ".tran " << std::scientific << timeStep << " " << simTime << " " << tStart
      << std::defaultfloat << "\n";
    c << "* Output signals\n";
    for (int w = 0; w < numWindings; ++w)
        c << ".save v(cmc_in" << w << ") v(cmc_out" << w << ") i(Vsense" << w << ")\n";
    c << kCmcOptions << ".end\n";
    return c.str();
}

// ── The CISPR LISN test deck (MKF generate_ngspice_circuit) ───────────────────────────────────────────
// A 1 V CM noise source couples into every line through 100 pF; the coupled CMC (k≈0.99) blocks CM
// current; a simplified CISPR-16 LISN + 50 Ω receiver measures the residual. Extracts the attenuation
// and CM impedance per test frequency.
std::string lisn_cmc_deck(int numWindings, double inductance, double frequency,
                          double operatingVoltage, double operatingCurrent) {
    const double period = 1.0 / frequency;
    const int numPeriods = 20;
    const double simTime = numPeriods * period;
    const double stepTime = period / 100.0;
    const double loadResistance = operatingVoltage / operatingCurrent;

    std::ostringstream c;
    c << "* CMC LISN (CISPR-16) test deck @ " << (frequency / 1e3) << " kHz\n";
    c << "Vcm_noise cm_src 0 SIN(0 1 " << frequency << ")\n";
    c << "* CM noise coupling caps\n";
    for (int w = 0; w < numWindings; ++w)
        c << "Ccm" << w << " cm_src cmc_in" << w << " 100p\n";
    c << "* Common-mode choke (coupled); Vsense sits IN SERIES with each winding, so i(Vsense) IS the\n";
    c << "* winding CM current (a sense in the load branch would read the ~zero differential current).\n";
    for (int w = 0; w < numWindings; ++w) {
        c << "Vsense" << w << " cmc_in" << w << " cmc_w" << w << " 0\n";
        c << "Lcmc" << w << " cmc_w" << w << " cmc_out" << w << " " << std::scientific << inductance
          << std::defaultfloat << "\n";
    }
    for (int i = 0; i < numWindings; ++i)
        for (int j = i + 1; j < numWindings; ++j)
            c << "K" << i << "_" << j << " Lcmc" << i << " Lcmc" << j << " 0.99\n";
    c << "* Simplified LISN + 50ohm receiver\n";
    for (int w = 0; w < numWindings; ++w) {
        c << "Llisn" << w << " cmc_out" << w << " lisn_mid" << w << " 50u\n";
        c << "Clisn" << w << " lisn_mid" << w << " 0 1u\n";
        c << "Rlisn" << w << " lisn_mid" << w << " lisn_out" << w << " 5\n";
        c << "Rmeas" << w << " lisn_out" << w << " 0 50\n";
    }
    c << "* AC load (EUT) across the input lines (line-to-line; the neutral carries no load)\n";
    if (numWindings == 2) {
        c << "Rload cmc_in0 cmc_in1 " << loadResistance << "\n";
    } else {
        for (int w = 0; w < 3; ++w)
            c << "Rload" << w << " cmc_in" << w << " cmc_in" << ((w + 1) % 3)
              << " " << (loadResistance * 3) << "\n";
    }
    c << ".tran " << std::scientific << stepTime << " " << simTime << std::defaultfloat << "\n";
    c << ".save v(cm_src)";
    for (int w = 0; w < numWindings; ++w) c << " v(lisn_out" << w << ") i(Vsense" << w << ")";
    c << "\n" << kCmcOptions << ".end\n";
    return c.str();
}

// CMC winding names by count (MKF windingNames).
std::vector<std::string> winding_names(int n) {
    if (n == 2) return {"Line", "Neutral"};
    if (n == 3) return {"Phase A", "Phase B", "Phase C"};
    if (n == 4) return {"Phase A", "Phase B", "Phase C", "Neutral"};
    std::vector<std::string> v;
    for (int i = 0; i < n; ++i) v.push_back("Winding " + std::to_string(i + 1));
    return v;
}

double peak_abs(const std::vector<double>& v) {
    double m = 0.0;
    for (double x : v) m = std::max(m, std::abs(x));
    return m;
}

}  // namespace

// ═══ simulate_cmc_ideal_waveforms — MKF simulate_realistic_cmc (:890). One OperatingPoint whose
// per-winding excitations are the SIMULATED winding V (v_in − v_out) and I; converterWaveforms empty
// (the waveforms live in the operating point, matching the legacy shape). ════════════════════════════

json simulate_cmc_ideal_waveforms(const CmcDesign& d, double inductance, double parasiticCapPf,
                                  double dvdtVPerNs, int numberOfPeriods, int numberOfSteadyStatePeriods) {
    if (!ngspice_in_process_available())
        return json{{"success", false}, {"error", "Kirchhoff built without libngspice"}};
    // Same rule as build_cmc_inputs (Cmc.hpp) — the simulated operating point must excite at the
    // frequency the analytical design was made for, or the two contradict each other.
    const double excFreq = cmc_excitation_frequency(d);
    const std::string deck = ideal_cmc_deck(d.numberOfWindings, inductance, d.operatingCurrent,
                                            d.operatingVoltage, excFreq, parasiticCapPf, dvdtVPerNs,
                                            numberOfPeriods, numberOfSteadyStatePeriods);
    NgspiceRunResult r = run_ngspice_in_process(deck);
    if (!r.success)
        return json{{"success", false}, {"error", "CMC ideal simulation failed: " + r.error}};

    const auto names = winding_names(d.numberOfWindings);
    MAS::OperatingPoint op;
    for (int w = 0; w < d.numberOfWindings; ++w) {
        std::vector<double> vIn = vec_of(r, "cmc_in" + std::to_string(w));
        std::vector<double> vOut = vec_of(r, "cmc_out" + std::to_string(w));
        std::vector<double> current = vec_of(r, "vsense" + std::to_string(w));
        std::vector<double> time = r.time;
        if (time.empty() || current.empty() || vIn.size() != vOut.size() || vIn.empty()) continue;

        std::vector<double> voltage(vIn.size());
        for (size_t i = 0; i < vIn.size(); ++i) voltage[i] = vIn[i] - vOut[i];
        // Normalize the time origin to 0 (ngspice starts at tStart); the sampler assumes [0, 1/f).
        const double t0 = time.front();
        if (t0 != 0.0) for (auto& t : time) t -= t0;

        MAS::Waveform cw; cw.set_data(current); cw.set_time(time);
        MAS::Waveform vw; vw.set_data(voltage); vw.set_time(time);
        op.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(cw, vw, excFreq, names[static_cast<size_t>(w)]));
    }
    op.get_mutable_conditions().set_ambient_temperature(d.ambientTemperature);
    op.set_name("Simulated");

    json inputs;
    inputs["operatingPoints"] = json::array({json(op)});
    return json{
        {"success", true},
        {"inputs", std::move(inputs)},
        {"converterWaveforms", json::array()},   // legacy: ideal waveforms live in the operating point
        {"cmcDiagnostics", json{{"computedInductance", d.computedInductance}}},
    };
}

// ═══ simulate_cmc_lisn_waveforms — MKF simulate_and_extract_waveforms (:608) over the impedance-spec
// frequencies. converterWaveforms carries the raw per-frequency arrays + attenuation/impedance. ═══════

json simulate_cmc_lisn_waveforms(const CmcDesign& d, double inductance) {
    if (!ngspice_in_process_available())
        return json{{"success", false}, {"error", "Kirchhoff built without libngspice"}};

    std::vector<double> frequencies;
    for (const auto& pt : d.impedancePoints) frequencies.push_back(pt.get_frequency());
    if (frequencies.empty()) frequencies = {150000.0};

    json converterWaveforms = json::array();
    json failedFrequencies = json::array();   // every failed run is SURFACED, never silently skipped
    for (double frequency : frequencies) {
        const std::string deck = lisn_cmc_deck(d.numberOfWindings, inductance, frequency,
                                               d.operatingVoltage, d.operatingCurrent);
        NgspiceRunResult r = run_ngspice_in_process(deck);
        if (!r.success) {
            failedFrequencies.push_back(json{{"frequency", frequency}, {"error", r.error}});
            continue;
        }

        std::vector<double> inputVoltage = vec_of(r, "cm_src");
        std::vector<std::vector<double>> windingCurrents;
        std::vector<double> lisnVoltage;
        for (int w = 0; w < d.numberOfWindings; ++w) {
            windingCurrents.push_back(vec_of(r, "vsense" + std::to_string(w)));
            std::vector<double> lo = vec_of(r, "lisn_out" + std::to_string(w));
            if (lisnVoltage.empty()) lisnVoltage = lo;   // representative receiver node (MKF used any)
        }
        const double theoreticalImpedance = 2.0 * M_PI * frequency * inductance;
        double attenuation = 0.0, cmImpedance = theoreticalImpedance;
        const double vinPeak = peak_abs(inputVoltage);
        const double voutPeak = peak_abs(lisnVoltage);
        if (voutPeak > 0 && vinPeak > 0) attenuation = 20.0 * std::log10(vinPeak / voutPeak);
        double totalIcm = 0.0;
        for (const auto& wc : windingCurrents) totalIcm += peak_abs(wc);
        if (totalIcm > 1e-12 && vinPeak > 0) cmImpedance = vinPeak / totalIcm;

        converterWaveforms.push_back(json{
            {"frequency", frequency},
            {"time", r.time},
            {"inputVoltage", std::move(inputVoltage)},
            {"windingCurrents", std::move(windingCurrents)},
            {"lisnVoltage", std::move(lisnVoltage)},
            {"operatingPointName", "CMC_" + std::to_string(static_cast<int>(frequency / 1000)) + "kHz"},
            {"commonModeAttenuation", attenuation},
            {"commonModeImpedance", cmImpedance},
            {"theoreticalImpedance", theoreticalImpedance},
        });
    }
    if (converterWaveforms.empty() && !failedFrequencies.empty())
        return json{{"success", false},
                    {"error", "CMC LISN simulation failed at every test frequency: "
                              + failedFrequencies[0].at("error").get<std::string>()},
                    {"failedFrequencies", std::move(failedFrequencies)}};
    json out{{"success", true}, {"converterWaveforms", std::move(converterWaveforms)}};
    if (!failedFrequencies.empty()) out["failedFrequencies"] = std::move(failedFrequencies);
    return out;
}

} // namespace Kirchhoff
