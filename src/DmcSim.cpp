#include "Dmc.hpp"
#include "NgspiceRunner.hpp"          // run_ngspice_in_process + ngspice_in_process_available

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// DMC ngspice simulations — ported from MKF converter_models/DifferentialModeChoke.cpp
// (generate_ngspice_circuit :333 LC low-pass, simulate_and_extract_waveforms :423,
// verify_attenuation :572), adapted onto Kirchhoff::run_ngspice_in_process. Same tolerance set as the
// CMC decks (reltol=1e-3 abstol=1e-9 vntol=1e-6). Feeds the DMC wizard's EMI-spectrum + attenuation views.

namespace Kirchhoff {
using nlohmann::json;

namespace {

const char* kDmcOptions = ".options reltol=1e-3 abstol=1e-9 vntol=1e-6\n";

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
std::vector<double> vec_of(const NgspiceRunResult& r, const std::string& name) {
    const std::string want = canon(name);
    for (const auto& kv : r.vectors)
        if (canon(kv.first) == want) return kv.second;
    return {};
}

// Settle window before the measurement opens: the LC natural mode (at the CUTOFF, far below the
// test frequency) must ring down, or the sine turn-on transient dominates v(filter_out) and the
// "measured" attenuation reads tens of dB low. The ring-down is set by the ACTUAL RLC decay: with
// the load R across C the poles decay no slower than max(2RC, L/R) (underdamped: tau = 2RC;
// overdamped slow pole: R/L) — 10 cutoff periods alone is ~1 tau for high-Q filters. Callers use
// the SAME value to trim the captured samples (NgspiceRunResult::drop_samples_before — the data
// callback streams every timepoint; .tran's tstart does not trim what the runner captures).
double dmc_settle_time(double inductance, double filterCap, double loadResistance) {
    const double cutoff = 1.0 / (2.0 * M_PI * std::sqrt(inductance * filterCap));
    const double decayTau = std::max(2.0 * loadResistance * filterCap, inductance / loadResistance);
    return std::max(10.0 / cutoff, 8.0 * decayTau);
}

// LC low-pass filter test deck (MKF DifferentialModeChoke::generate_ngspice_circuit). DC bus + DM noise
// sine → DMC L → filter cap → load. filterCap is the CALLER'S capacitance — the deck must simulate the
// exact filter the caller designed/reports on, never re-size its own per test frequency.
std::string dmc_deck(double inductance, double filterCap, double frequency,
                     double operatingVoltage, double operatingCurrent) {
    const double period = 1.0 / frequency;
    const int numPeriods = 20;
    const double stepTime = period / 100.0;
    const double loadResistance = operatingVoltage / operatingCurrent;
    const double settleTime = dmc_settle_time(inductance, filterCap, loadResistance);
    const double simTime = settleTime + numPeriods * period;

    std::ostringstream c;
    c << "* DMC LC low-pass filter test @ " << (frequency / 1e3) << " kHz\n";
    c << "Vdc in_dc 0 " << operatingVoltage << "\n";
    c << "Vnoise noise_src in_dc SIN(0 " << (operatingVoltage * 0.1) << " " << frequency << ")\n";
    c << "Vsense noise_src dmc_in 0\n";
    c << "Ldmc dmc_in dmc_out " << std::scientific << inductance << std::defaultfloat << "\n";
    c << "Rdmc_esr dmc_out filter_out 0.01\n";
    c << "Cfilt filter_out 0 " << std::scientific << filterCap << std::defaultfloat << "\n";
    c << "Rload filter_out 0 " << loadResistance << "\n";
    c << ".tran " << std::scientific << stepTime << " " << simTime << std::defaultfloat << "\n";
    c << ".save v(noise_src) v(filter_out) i(Vsense)\n";
    c << kDmcOptions << ".end\n";
    return c.str();
}

// DM attenuation (dB) from the AC components (DC-subtracted peaks) of noise_src vs filter_out.
double dm_attenuation(const std::vector<double>& vin, const std::vector<double>& vout) {
    if (vin.empty() || vout.empty()) return 0.0;
    const double vinAvg = std::accumulate(vin.begin(), vin.end(), 0.0) / vin.size();
    const double voutAvg = std::accumulate(vout.begin(), vout.end(), 0.0) / vout.size();
    double vinAc = 0.0, voutAc = 0.0;
    for (double x : vin) vinAc = std::max(vinAc, std::abs(x - vinAvg));
    for (double x : vout) voutAc = std::max(voutAc, std::abs(x - voutAvg));
    return (voutAc > 0 && vinAc > 0) ? 20.0 * std::log10(vinAc / voutAc) : 0.0;
}

// The frequencies to test: the spec impedance points, else the switching-frequency harmonics.
// THROWS when neither exists — testing at 50/60 Hz instead would "verify" a spectrum with no
// switching content (the silent lineFrequency substitute MKF's require_input existed to prevent).
std::vector<double> test_frequencies(const DmcDesign& d) {
    std::vector<double> f;
    for (const auto& pt : d.impedancePoints) f.push_back(pt.get_frequency());
    if (f.empty()) {
        if (d.switchingFrequency <= 0)
            throw std::invalid_argument(
                "dmc simulation: no impedance points and no switchingFrequency — nothing defines "
                "the test frequencies");
        f = {d.switchingFrequency, d.switchingFrequency * 2, d.switchingFrequency * 5};
    }
    return f;
}

// The ONE filter capacitance every sim/verify row uses: the explicit argument, else the spec's
// filterCapacitance, else the fc = fsw/10 sizing clamped to [1nF, 100µF] (MKF's rule) — which needs
// the real switching frequency, so it THROWS when that is absent too.
double resolve_filter_capacitance(const DmcDesign& d, double inductance, double explicitCap) {
    if (explicitCap > 0) return explicitCap;
    if (d.filterCapacitance) return *d.filterCapacitance;
    if (d.switchingFrequency <= 0)
        throw std::invalid_argument(
            "dmc simulation: filter capacitance cannot be auto-sized — supply filterCapacitance "
            "(or the capacitance argument), or switchingFrequency for the fc = fsw/10 rule");
    const double cutoff = d.switchingFrequency / 10.0;
    const double filterCap = 1.0 / (4.0 * M_PI * M_PI * cutoff * cutoff * inductance);
    return std::min(std::max(filterCap, 1e-9), 100e-6);
}

}  // namespace

// ═══ simulate_dmc_waveforms — MKF simulate_and_extract_waveforms (:423) over the test frequencies.
// Every frequency simulates the SAME resolved LC filter; failed runs are surfaced. ═══════════════════

json simulate_dmc_waveforms(const DmcDesign& d, double inductance, double capacitance) {
    if (!ngspice_in_process_available())
        return json{{"success", false}, {"error", "Kirchhoff built without libngspice"}};
    const double filterCap = resolve_filter_capacitance(d, inductance, capacitance);

    json converterWaveforms = json::array();
    json failedFrequencies = json::array();
    const double settle = dmc_settle_time(inductance, filterCap, d.inputVoltage / d.operatingCurrent);
    for (double frequency : test_frequencies(d)) {
        NgspiceRunResult r = run_ngspice_in_process(
            dmc_deck(inductance, filterCap, frequency, d.inputVoltage, d.operatingCurrent));
        if (!r.success) {
            failedFrequencies.push_back(json{{"frequency", frequency}, {"error", r.error}});
            continue;
        }
        r.drop_samples_before(settle);   // measure steady state only (see dmc_settle_time)
        std::vector<double> inputVoltage = vec_of(r, "noise_src");
        std::vector<double> outputVoltage = vec_of(r, "filter_out");
        std::vector<double> inductorCurrent = vec_of(r, "vsense");
        converterWaveforms.push_back(json{
            {"frequency", frequency},
            {"time", r.time},
            {"inputVoltage", inputVoltage},
            {"outputVoltage", outputVoltage},
            {"inductorCurrent", std::move(inductorCurrent)},
            {"operatingPointName", "DMC_" + std::to_string(static_cast<int>(frequency / 1000)) + "kHz"},
            {"dmAttenuation", dm_attenuation(inputVoltage, outputVoltage)},
        });
    }
    if (converterWaveforms.empty() && !failedFrequencies.empty())
        return json{{"success", false},
                    {"error", "DMC simulation failed at every test frequency: "
                              + failedFrequencies[0].at("error").get<std::string>()},
                    {"failedFrequencies", std::move(failedFrequencies)}};
    json out{{"success", true}, {"converterWaveforms", std::move(converterWaveforms)}};
    if (!failedFrequencies.empty()) out["failedFrequencies"] = std::move(failedFrequencies);
    return out;
}

// ═══ verify_dmc_attenuation — MKF verify_attenuation (:572). Required attenuation per impedance point
// (20·log10(Z/Rload)); theoretical LC attenuation (40·log10(f/fc)); measured from ngspice ON THE SAME
// FILTER (resolved L + C); pass if ≥ 0.9·required. A row whose ngspice run failed (or a build without
// libngspice) judges on the theoretical value but SAYS so: simulated:false, measuredAttenuation:null,
// and a message that never claims "Measured" for a number that wasn't. ═══════════════════════════════

json verify_dmc_attenuation(const DmcDesign& d, double inductance, double capacitance) {
    const double filterCap = resolve_filter_capacitance(d, inductance, capacitance);
    const double loadImpedance = d.inputVoltage / d.operatingCurrent;

    // Test points: (frequency, required attenuation). From impedance points, else fsw harmonics
    // (test_frequencies already threw if neither exists).
    std::vector<std::pair<double, double>> testPoints;
    for (const auto& pt : d.impedancePoints) {
        const double z = pt.get_impedance().get_magnitude();
        testPoints.push_back({pt.get_frequency(), std::max(0.0, 20.0 * std::log10(z / loadImpedance))});
    }
    if (testPoints.empty()) {
        const std::vector<double> f = test_frequencies(d);
        testPoints = {{f[0], 20.0}, {f[1], 30.0}, {f[2], 40.0}};
    }

    const double cutoffFrequency = 1.0 / (2.0 * M_PI * std::sqrt(inductance * filterCap));
    const bool ngspiceAvailable = ngspice_in_process_available();

    json results = json::array();
    for (const auto& [frequency, requiredAttenuation] : testPoints) {
        const double theoretical = (frequency > cutoffFrequency)
            ? 40.0 * std::log10(frequency / cutoffFrequency) : 0.0;
        std::optional<double> measured;
        std::string simFailure;
        if (ngspiceAvailable) {
            NgspiceRunResult r = run_ngspice_in_process(
                dmc_deck(inductance, filterCap, frequency, d.inputVoltage, d.operatingCurrent));
            if (r.success) {
                r.drop_samples_before(
                    dmc_settle_time(inductance, filterCap, d.inputVoltage / d.operatingCurrent));
                measured = dm_attenuation(vec_of(r, "noise_src"), vec_of(r, "filter_out"));
            } else {
                simFailure = "ngspice run failed: " + r.error;
            }
        } else {
            simFailure = "built without libngspice";
        }
        const double effective = measured ? *measured : theoretical;
        const bool passed = effective >= requiredAttenuation * 0.9;   // 10% margin
        std::ostringstream msg;
        msg.setf(std::ios::fixed); msg.precision(1);
        msg << "At " << (frequency / 1e3) << " kHz: Required " << requiredAttenuation << " dB, ";
        if (measured)
            msg << "Measured " << *measured << " dB (Theoretical: " << theoretical << " dB)";
        else
            msg << "Theoretical " << theoretical << " dB (NOT simulated — " << simFailure << ")";
        msg << " - " << (passed ? "PASS" : "FAIL");
        results.push_back(json{
            {"frequency", frequency},
            {"requiredAttenuation", requiredAttenuation},
            {"measuredAttenuation", measured ? json(*measured) : json(nullptr)},
            {"theoreticalAttenuation", theoretical},
            {"simulated", measured.has_value()},
            {"passed", passed},
            {"message", msg.str()},
        });
    }
    return results;
}

} // namespace Kirchhoff
