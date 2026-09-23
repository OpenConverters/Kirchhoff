#pragma once
// One canonical steady-state period out of a raw ngspice transient trace — shared by the component sims
// (CMC ideal deck, DMC LC deck) so they frame their waveforms by the SAME rule.
//
// WaveformProcessor takes a timed waveform's span as ONE period of the frequency it is given
// (calculate_sampled_waveform infers the period from time.back() - time.front()), so handing it a
// multi-period trace makes it read N cycles as one: every harmonic lands at N·f and a clean sine comes back
// with hundreds of percent THD (ABT #1356). The web wizards tile the plotted period for display (the
// Periods knob is display-only); the data they hand MKF must be exactly one period starting at t = 0.
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {

// The LAST `period` of (time, sig), time rebased to [0, period]. The window's first point is interpolated
// at exactly tEnd - period so the span is the period itself, not whatever ngspice timepoint happens to fall
// after it; the samples inside the window are kept as computed. Throws (naming `who`) when the trace is
// malformed or shorter than one period: there is no steady-state cycle to report.
inline void last_period(const std::vector<double>& time, const std::vector<double>& sig, double period,
                        std::vector<double>& outTime, std::vector<double>& outSig, const std::string& who) {
    if (time.size() < 2 || sig.size() != time.size())
        throw std::runtime_error(who + ": trace has " + std::to_string(time.size()) +
                                 " timepoints and " + std::to_string(sig.size()) + " samples");
    if (!(period > 0))
        throw std::invalid_argument(who + ": period must be > 0");
    const double tEnd = time.back();
    const double tBeg = tEnd - period;
    if (time.front() > tBeg)
        throw std::runtime_error(who + ": captured trace spans " + std::to_string(tEnd - time.front()) +
                                 " s, shorter than one period (" + std::to_string(period) + " s)");
    // First index strictly after tBeg; the sample before it brackets tBeg.
    size_t k = static_cast<size_t>(std::upper_bound(time.begin(), time.end(), tBeg) - time.begin());
    outTime.clear();
    outSig.clear();
    const double t0 = time[k - 1], t1 = time[k];
    const double f = (t1 > t0) ? (tBeg - t0) / (t1 - t0) : 0.0;
    outTime.push_back(0.0);
    outSig.push_back(sig[k - 1] + f * (sig[k] - sig[k - 1]));
    for (; k < time.size(); ++k) {
        outTime.push_back(time[k] - tBeg);
        outSig.push_back(sig[k]);
    }
}

} // namespace Kirchhoff
