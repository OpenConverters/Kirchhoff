#include "Dmc.hpp"
#include "NgspiceRunner.hpp"          // run_ngspice_in_process + ngspice_in_process_available

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>
#include <sstream>
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

// LC low-pass filter test deck (MKF DifferentialModeChoke::generate_ngspice_circuit). DC bus + DM noise
// sine → DMC L → filter cap → load. filterCapacitance sized for fc = frequency/10, clamped [1nF, 100µF].
std::string dmc_deck(double inductance, double frequency, double operatingVoltage, double operatingCurrent) {
    const double period = 1.0 / frequency;
    const int numPeriods = 20;
    const double simTime = numPeriods * period;
    const double stepTime = period / 100.0;
    const double cutoff = frequency / 10.0;
    double filterCap = 1.0 / (4.0 * M_PI * M_PI * cutoff * cutoff * inductance);
    filterCap = std::min(std::max(filterCap, 1e-9), 100e-6);
    const double loadResistance = operatingVoltage / operatingCurrent;

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
std::vector<double> test_frequencies(const DmcDesign& d) {
    std::vector<double> f;
    for (const auto& pt : d.impedancePoints) f.push_back(pt.get_frequency());
    if (f.empty()) {
        const double fsw = d.switchingFrequency > 0 ? d.switchingFrequency : d.lineFrequency;
        f = {fsw, fsw * 2, fsw * 5};
    }
    return f;
}

}  // namespace

// ═══ simulate_dmc_waveforms — MKF simulate_and_extract_waveforms (:423) over the test frequencies. ═══

json simulate_dmc_waveforms(const DmcDesign& d, double inductance) {
    if (!ngspice_in_process_available())
        return json{{"success", false}, {"error", "Kirchhoff built without libngspice"}};

    json converterWaveforms = json::array();
    for (double frequency : test_frequencies(d)) {
        NgspiceRunResult r = run_ngspice_in_process(
            dmc_deck(inductance, frequency, d.inputVoltage, d.operatingCurrent));
        if (!r.success) continue;
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
    return json{{"success", true}, {"converterWaveforms", std::move(converterWaveforms)}};
}

// ═══ verify_dmc_attenuation — MKF verify_attenuation (:572). Required attenuation per impedance point
// (20·log10(Z/Rload)); theoretical LC attenuation (40·log10(f/fc)); measured from ngspice (falls back to
// theoretical when unavailable); pass if measured ≥ 0.9·required. ════════════════════════════════════

json verify_dmc_attenuation(const DmcDesign& d, double inductance, double capacitance) {
    // Filter capacitance: explicit, else fc = fsw/10 sizing clamped to [1nF, 100µF].
    double filterCap = capacitance;
    if (filterCap <= 0) {
        const double fsw = d.switchingFrequency > 0 ? d.switchingFrequency : d.lineFrequency;
        const double cutoff = fsw / 10.0;
        filterCap = 1.0 / (4.0 * M_PI * M_PI * cutoff * cutoff * inductance);
        filterCap = std::min(std::max(filterCap, 1e-9), 100e-6);
    }
    const double loadImpedance = d.inputVoltage / d.operatingCurrent;

    // Test points: (frequency, required attenuation). From impedance points, else fsw harmonics.
    std::vector<std::pair<double, double>> testPoints;
    for (const auto& pt : d.impedancePoints) {
        const double z = pt.get_impedance().get_magnitude();
        testPoints.push_back({pt.get_frequency(), std::max(0.0, 20.0 * std::log10(z / loadImpedance))});
    }
    if (testPoints.empty()) {
        const double fsw = d.switchingFrequency > 0 ? d.switchingFrequency : d.lineFrequency;
        testPoints = {{fsw, 20.0}, {fsw * 2, 30.0}, {fsw * 5, 40.0}};
    }

    const double cutoffFrequency = 1.0 / (2.0 * M_PI * std::sqrt(inductance * filterCap));
    const bool ngspiceAvailable = ngspice_in_process_available();

    json results = json::array();
    for (const auto& [frequency, requiredAttenuation] : testPoints) {
        const double theoretical = (frequency > cutoffFrequency)
            ? 40.0 * std::log10(frequency / cutoffFrequency) : 0.0;
        double measured = theoretical;
        if (ngspiceAvailable) {
            NgspiceRunResult r = run_ngspice_in_process(
                dmc_deck(inductance, frequency, d.inputVoltage, d.operatingCurrent));
            if (r.success)
                measured = dm_attenuation(vec_of(r, "noise_src"), vec_of(r, "filter_out"));
        }
        const bool passed = measured >= requiredAttenuation * 0.9;   // 10% margin
        std::ostringstream msg;
        msg.setf(std::ios::fixed); msg.precision(1);
        msg << "At " << (frequency / 1e3) << " kHz: Required " << requiredAttenuation
            << " dB, Measured " << measured << " dB (Theoretical: " << theoretical << " dB) - "
            << (passed ? "PASS" : "FAIL");
        results.push_back(json{
            {"frequency", frequency},
            {"requiredAttenuation", requiredAttenuation},
            {"measuredAttenuation", measured},
            {"theoreticalAttenuation", theoretical},
            {"passed", passed},
            {"message", msg.str()},
        });
    }
    return results;
}

} // namespace Kirchhoff
