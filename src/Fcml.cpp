#include "Fcml.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "processors/WaveformProcessor.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

int fcml_levels(const json& config, const char* who) {
    if (!config.is_object() || !config.contains("levels")) return 2;
    const json& v = config.at("levels");
    if (!v.is_number()) throw std::invalid_argument(std::string(who) + ": config.levels must be a number");
    const double x = v.get<double>();
    if (x != std::floor(x) || x < 2 || x > 6)
        throw std::invalid_argument(std::string(who) + ": config.levels must be a whole number in [2, 6], got " + std::to_string(x));
    return static_cast<int>(x);
}

namespace analytical {
namespace {

// The phase-shifted switching pattern over one period. Pair k (1-based) turns its main switch on at
// (k−1)·T/(N−1) for D·T; the main switch is the high one for a buck and the low one for a boost.
struct Pattern {
    int pairs;
    double D, T;
    bool boost;
    bool high_on(int k, double t) const {
        const double phi = (k - 1) * T / pairs;
        double x = std::fmod(t - phi, T);
        if (x < 0) x += T;
        const bool mainOn = x < D * T;
        return boost ? !mainOn : mainOn;
    }
    std::vector<double> edges() const {
        std::vector<double> e{0.0, T};
        for (int k = 1; k <= pairs; ++k) {
            const double phi = (k - 1) * T / pairs;
            for (double t : {phi, std::fmod(phi + D * T, T)}) if (t > 0 && t < T) e.push_back(t);
        }
        std::sort(e.begin(), e.end());
        e.erase(std::unique(e.begin(), e.end(), [&](double a, double b) { return std::fabs(a - b) < 1e-15 * T; }), e.end());
        return e;
    }
};

Pattern pattern(bool boost, int levels, double vin, double vo, double fs, const char* who) {
    if (levels < 2) throw std::invalid_argument(std::string(who) + ": levels must be >= 2");
    if (!(vin > 0) || !(vo > 0) || !(fs > 0)) throw std::invalid_argument(std::string(who) + ": Vin, Vo and fs must be > 0");
    if (boost ? !(vo > vin) : !(vo < vin))
        throw std::invalid_argument(std::string(who) + (boost ? ": a boost needs Vo > Vin" : ": a buck needs Vo < Vin"));
    return {levels - 1, boost ? 1.0 - vin / vo : vo / vin, 1.0 / fs, boost};
}

// Inductor voltage on each interval between edges (constant there).
std::vector<double> inductor_voltage(const Pattern& p, const std::vector<double>& e, double vin, double vo) {
    const double vtop = p.boost ? vo : vin, vs = vtop / p.pairs;
    std::vector<double> v;
    for (size_t i = 0; i + 1 < e.size(); ++i) {
        const double tm = 0.5 * (e[i] + e[i + 1]);
        int n = 0;
        for (int k = 1; k <= p.pairs; ++k) n += p.high_on(k, tm);
        const double vsw = vs * n;
        v.push_back(p.boost ? vin - vsw : vsw - vo);
    }
    return v;
}

} // namespace

double fcml_ripple_volt_seconds(bool boost, int levels, double vin, double vo, double fs) {
    const Pattern p = pattern(boost, levels, vin, vo, fs, "fcml_ripple_volt_seconds");
    const auto e = p.edges();
    const auto v = inductor_voltage(p, e, vin, vo);
    double lam = 0, lo = 0, hi = 0;
    for (size_t i = 0; i < v.size(); ++i) { lam += v[i] * (e[i + 1] - e[i]); lo = std::min(lo, lam); hi = std::max(hi, lam); }
    return hi - lo;
}

double fcml_flying_charge_per_amp(bool boost, int levels, double vin, double vo, double fs) {
    const Pattern p = pattern(boost, levels, vin, vo, fs, "fcml_flying_charge_per_amp");
    const auto e = p.edges();
    double worst = 0;
    for (int k = 1; k < p.pairs; ++k) {   // Cf_k sits between pair k and pair k+1
        double q = 0, lo = 0, hi = 0;
        for (size_t i = 0; i + 1 < e.size(); ++i) {
            const double tm = 0.5 * (e[i] + e[i + 1]);
            q += (static_cast<int>(p.high_on(k + 1, tm)) - static_cast<int>(p.high_on(k, tm))) * (e[i + 1] - e[i]);
            lo = std::min(lo, q); hi = std::max(hi, q);
        }
        worst = std::max(worst, hi - lo);
    }
    return worst;
}

MAS::OperatingPoint analytical_fcml(bool boost, int levels, double vin, double vo, double io, double fs, double L) {
    using Lbl = MAS::WaveformLabel;
    using WP = OpenMagnetics::WaveformProcessor;
    if (!(io > 0) || !(L > 0)) throw std::invalid_argument("analytical_fcml: Io and L must be > 0");
    const Pattern p = pattern(boost, levels, vin, vo, fs, "analytical_fcml");
    const auto e = p.edges();
    const auto v = inductor_voltage(p, e, vin, vo);
    // Piecewise-linear current from 0, then shifted to its mean.
    std::vector<double> t{e[0]}, i{0.0}, vt, vv;
    double area = 0;
    for (size_t k = 0; k < v.size(); ++k) {
        const double dt = e[k + 1] - e[k], i1 = i.back() + v[k] / L * dt;
        area += 0.5 * (i.back() + i1) * dt;
        t.push_back(e[k + 1]); i.push_back(i1);
        vt.push_back(e[k]); vv.push_back(v[k]); vt.push_back(e[k + 1]); vv.push_back(v[k]);
    }
    const double mean = boost ? io / (1.0 - p.D) : io;
    const double shift = mean - area / p.T;
    double lo = 1e300;
    for (double& x : i) { x += shift; lo = std::min(lo, x); }
    if (!(lo > 0))
        throw std::invalid_argument("analytical_fcml: the inductor runs discontinuous (valley " + std::to_string(lo) +
                                    " A); DCM is not modelled — raise L or the load");
    MAS::Waveform iw; iw.set_ancillary_label(Lbl::CUSTOM); iw.set_data(i); iw.set_time(t);
    MAS::Waveform vw; vw.set_ancillary_label(Lbl::CUSTOM); vw.set_data(vv); vw.set_time(vt);
    MAS::OperatingPoint op;
    op.get_mutable_excitations_per_winding().push_back(WP::complete_excitation(iw, vw, fs, "Primary"));
    return op;
}

FcmlStart fcml_periodic_start(bool boost, int levels, double vin, double vo, double io, double fs, double L,
                              double cf, double cout) {
    if (!(cf > 0) || !(cout > 0)) throw std::invalid_argument("fcml_periodic_start: the capacitances must be > 0");
    const Pattern p = pattern(boost, levels, vin, vo, fs, "fcml_periodic_start");
    const auto wave = winding_current_samples(analytical_fcml(boost, levels, vin, vo, io, fs, L), 0, p.T);
    const auto& tw = wave.first; const auto& iw = wave.second;
    // The flying-capacitor current is ±IL while its two pairs disagree: charging (+ on its high node) when the inner
    // pair is high and the outer low, for a boost's inductor current flowing into the switch node; a buck's flows out.
    // The boost's output takes IL while the outermost pair is high. Integrate each from t = 0 and take the charge
    // at t = 0 relative to its period mean.
    const int M = 20000;
    std::vector<double> q(p.pairs - 1, 0.0), qMean(p.pairs - 1, 0.0);
    double qo = 0, qoMean = 0;
    size_t j = 0;
    const double h = p.T / M;
    for (int s = 0; s < M; ++s) {
        const double t = (s + 0.5) * h;
        while (j + 1 < tw.size() && tw[j + 1] < t) ++j;
        const double il = (j + 1 < tw.size() && tw[j + 1] > tw[j])
            ? iw[j] + (t - tw[j]) / (tw[j + 1] - tw[j]) * (iw[j + 1] - iw[j]) : iw[j];
        for (int k = 1; k < p.pairs; ++k) {
            const double dir = static_cast<int>(p.high_on(k, t)) - static_cast<int>(p.high_on(k + 1, t));
            q[k - 1] += (boost ? 1.0 : -1.0) * dir * il * h;
            qMean[k - 1] += q[k - 1] / M;
        }
        if (boost) { qo += (il * p.high_on(p.pairs, t) - io) * h; qoMean += qo / M; }
    }
    FcmlStart st;
    st.inductorCurrent = iw.front();
    const double vtop = boost ? vo : vin;
    for (int k = 1; k < p.pairs; ++k) st.flyingCapacitorVoltages.push_back(k * vtop / p.pairs - qMean[k - 1] / cf);
    st.outputVoltage = boost ? vo - qoMean / cout : vo;
    return st;
}

} // namespace analytical

json build_fcml_tas(const FcmlCell& c) {
    auto port = [](const std::string& n){ json p; p["name"]=n; return p; };
    auto pin  = [](const std::string& cc, const char* p){ json e; e["component"]=cc; e["pin"]=p; return e; };
    auto prt  = [](const char* p){ json e; e["port"]=p; return e; };
    auto conn = [](const std::string& name, std::vector<json> eps){ json cn; cn["name"]=name; cn["endpoints"]=eps; return cn; };
    auto comp = [](const std::string& name, json data){ json cc; cc["name"]=name; cc["data"]=data; return cc; };
    auto bind = [](const char* p, const char* t){ json b; b["port"]=p; b["type"]=t; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb){
        json s; s["name"]=name; s["role"]=role; s["circuit"]=brick; s["inputPort"]=inb; s["outputPort"]=outb; return s; };
    auto sp = [](const char* st, const char* po){ json e; e["stage"]=st; e["port"]=po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps){
        json cn; cn["name"]=name; cn["kind"]=kind; if(dir[0]) cn["direction"]=dir; cn["endpoints"]=eps; return cn; };
    namespace AN = Kirchhoff::analytical;
    const char* who = c.boost ? "build_boost_tas (FCML)" : "build_buck_tas (FCML)";
    const int N = c.levels, pairs = N - 1;
    const double fs = c.switchingFrequency, Vo = c.outputVoltage, Io = c.outputPower / Vo;
    // The cell switches the efficiency-compensated duty: the lossless cell's output is Vo/η (see design_buck/boost).
    const double VoCell = Vo / c.efficiency;

    // Ratings from the worst corner (buck: Vin_max sets the step and the blocking; Vin_min the current is the same;
    // boost: Vin_min carries the largest inductor current), the magnetic's embedded point at nominal.
    const MAS::OperatingPoint opNom = AN::analytical_fcml(c.boost, N, c.inputVoltage, VoCell, Io, fs, c.inductance);
    double ipk = 0, irms = 0, ivalley = std::numeric_limits<double>::max();
    for (double vin : {c.inputVoltageMin, c.inputVoltage, c.inputVoltageMax}) {
        const MAS::OperatingPoint op = AN::analytical_fcml(c.boost, N, vin, VoCell, Io, fs, c.inductance);
        ipk = std::max(ipk, AN::winding_current(op, 0, "peak"));
        irms = std::max(irms, AN::winding_current(op, 0, "rms"));
        const auto samples = AN::winding_current_samples(op, 0, 1.0 / fs).second;
        ivalley = std::min(ivalley, *std::min_element(samples.begin(), samples.end()));
    }
    cfg::check_maximum_switch_current(c.config, ipk, who);
    const double vtopMax = c.boost ? Vo : c.inputVoltageMax, vStep = vtopMax / pairs;
    const double ratedVds = vStep / cfg::v_derate_mosfet(c.config);
    json mosfetMain; mosfetMain["semiconductor"]["mosfet"] = json::object();
    mosfetMain["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, ipk,
        cfg::rds_on_loss_fraction(c.config) * c.outputPower / (irms * irms), 125.0);
    json mosfetSync = mosfetMain;
    mosfetSync["inputs"]["designRequirements"]["role"] = "synchronousRectifier";
    auto body = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(vStep, ipk); return j; };
    json ind; ind["magnetic"] = json::object();
    ind["inputs"] = req::magnetic_inputs(c.inductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, AN::excitations_processed(opNom, "L1"));

    std::vector<json> comps{comp("L1", ind)}, conns;
    // Nodes: the high string runs top → nh{N−2} → … → nh1 → sw; the low string sw → nl1 → … → nl{N−2} → gnd.
    auto hnode = [&](int k) { return k == 0 ? std::string("sw_node") : k == pairs ? std::string("top") : "nh" + std::to_string(k); };
    auto lnode = [&](int k) { return k == 0 ? std::string("sw_node") : k == pairs ? std::string("gnd_net") : "nl" + std::to_string(k); };
    std::map<std::string, std::vector<json>> nets;
    for (int k = 1; k <= pairs; ++k) {
        const std::string qh = "QH" + std::to_string(k), ql = "QL" + std::to_string(k);
        const std::string dh = "DH" + std::to_string(k), dl = "DL" + std::to_string(k);
        comps.push_back(comp(qh, c.boost ? mosfetSync : mosfetMain));
        comps.push_back(comp(ql, c.boost ? mosfetMain : mosfetSync));
        comps.push_back(comp(dh, body()));
        comps.push_back(comp(dl, body()));
        // QH k: drain on the node above (k), source on the node below (k−1); QL k: drain on (k−1), source on (k).
        for (auto [n, p] : std::vector<std::pair<std::string, json>>{
                 {hnode(k), pin(qh, "drain")}, {hnode(k), pin(dh, "cathode")}, {hnode(k - 1), pin(qh, "source")}, {hnode(k - 1), pin(dh, "anode")},
                 {lnode(k - 1), pin(ql, "drain")}, {lnode(k - 1), pin(dl, "cathode")}, {lnode(k), pin(ql, "source")}, {lnode(k), pin(dl, "anode")}})
            nets[n].push_back(p);
        conns.push_back(conn("g" + std::to_string(k) + "h_net", {pin(qh, "gate"), prt(("gh" + std::to_string(k)).c_str())}));
        conns.push_back(conn("g" + std::to_string(k) + "l_net", {pin(ql, "gate"), prt(("gl" + std::to_string(k)).c_str())}));
    }
    for (int k = 1; k < pairs; ++k) {
        const std::string cf = "Cf" + std::to_string(k);
        const double vcf = k * vtopMax / pairs;
        json cap; cap["capacitor"] = json::object();
        // a flying capacitor's ripple current is the inductor current while its two pairs disagree
        cap["inputs"]["designRequirements"] = req::capacitor(c.flyingCapacitance, 1.1 * vcf / cfg::v_derate_capacitor(c.config),
                                                             irms, 0.05 * vStep / ipk, "dcLink");
        // the CAS role enum has no flying capacitor (dcLink would claim a bus it is not); role is optional
        cap["inputs"]["designRequirements"].erase("role");
        comps.push_back(comp(cf, cap));
        nets[hnode(k)].push_back(pin(cf, "1"));
        nets[lnode(k)].push_back(pin(cf, "2"));
    }
    // Numerical aids (ABT #96): a cap across every switch. With ideal capacitance-free body diodes, a pair whose diode
    // is freewheeling and whose switch then turns on hard leaves the inner nodes held by nothing but the flying
    // capacitor, and ngspice accepts a non-converged step there that dumps tens of volts off a flying capacitor in a
    // nanosecond. Each aid is sized so a commutation (two of them, swung by the valley current) completes in a tenth
    // of the dead time, so it does not shift the edges it smooths.
    const double tDead = c.deadFraction / fs;
    if (!(ivalley > 0) || !(tDead > 0))
        throw std::invalid_argument(std::string(who) + ": the switch aids need a positive valley current and dead time");
    const double cAid = 0.1 * ivalley * tDead / (2.0 * vStep);
    auto aid = [&](const std::string& name, const std::string& a, const std::string& b) {
        json csw; csw["capacitor"] = json::object();
        csw["inputs"]["designRequirements"]["capacitance"]["nominal"] = cAid;
        csw["inputs"]["designRequirements"]["ratedVoltage"] = 3.0 * vStep;
        cfg::mark_numerical_aid(csw);
        comps.push_back(comp(name, csw));
        nets[a].push_back(pin(name, "1")); nets[b].push_back(pin(name, "2")); };
    for (int k = 1; k <= pairs; ++k) {
        aid("CswH" + std::to_string(k), hnode(k), hnode(k - 1));
        aid("CswL" + std::to_string(k), lnode(k - 1), lnode(k));
    }
    // The inductor: buck sw → vout; boost vin → sw. The top rail is the input (buck) or the output (boost).
    nets["sw_node"].push_back(pin("L1", c.boost ? "primary_end" : "primary_start"));
    if (c.boost) {
        conns.push_back(conn("vin_net", {pin("L1", "primary_start"), prt("vin")}));
        nets["top"].push_back(prt("vout"));
    } else {
        conns.push_back(conn("vout_net", {pin("L1", "primary_end"), prt("vout")}));
        nets["top"].push_back(prt("vin"));
    }
    nets["gnd_net"].push_back(prt("gnd"));
    for (auto& [n, eps] : nets) conns.push_back(conn(n == "top" ? (c.boost ? "vout_net" : "vin_net") : n, eps));

    std::vector<json> cports{port("vin"), port("gnd"), port("vout")};
    for (int k = 1; k <= pairs; ++k) { cports.push_back(port("gh" + std::to_string(k))); cports.push_back(port("gl" + std::to_string(k))); }
    json cell; cell["name"] = c.boost ? "fcml-boost-cell" : "fcml-buck-cell";
    cell["ports"] = cports; cell["components"] = comps; cell["connections"] = conns;

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"] = req::capacitor(c.outputCapacitance, Vo / cfg::v_derate_capacitor(c.config),
        c.boost ? Io : AN::fcml_ripple_volt_seconds(false, N, c.inputVoltageMax, VoCell, fs) / c.inductance / (2.0 * std::sqrt(3.0)),
        cfg::get(c.config, "esrRippleFraction", req::ESR_RIPPLE_FRACTION) * Vo / ipk, "outputFilter");
    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capd)});
    filt["connections"] = json::array({conn("out", {pin("Cout", "1"), prt("in")}), conn("ret", {pin("Cout", "2"), prt("rtn")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = c.efficiency; dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", c.inputVoltageMin}, {"nominal", c.inputVoltage}, {"maximum", c.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = fs;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = Vo; o["regulation"] = "voltage"; dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = c.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = c.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }
    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("switchingCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("switchingCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("switchingCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("switchingCell", "vout"), sp("filter", "in")})});
    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(c.config, 0.004);
    an["maximumTimeStep"] = cfg::tran_max_timestep(c.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Phase-shifted PWM: pair k's main switch at (k−1)·360/(N−1) for D, its complement after a dead time.
    const double D = c.dutyCycle, dt = c.deadFraction;
    if (!(D > dt && 1.0 - D > 2.0 * dt))
        throw std::invalid_argument(std::string(who) + ": duty " + std::to_string(D) + " leaves no room for the dead times");
    tas["simulation"]["stimulus"] = json::array();
    for (int k = 1; k <= pairs; ++k) {
        const double phi = (k - 1) * 360.0 / pairs;
        const std::string mainSw = (c.boost ? "QL" : "QH") + std::to_string(k), compSw = (c.boost ? "QH" : "QL") + std::to_string(k);
        auto stim = [&](const std::string& sw, double duty, double phase) {
            json st; st["stage"] = "switchingCell"; st["component"] = sw; st["signal"] = "gate";
            st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = fs;
            st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = std::fmod(phase, 360.0); return st; };
        tas["simulation"]["stimulus"].push_back(stim(mainSw, D, phi));
        tas["simulation"]["stimulus"].push_back(stim(compSw, 1.0 - D - 2.0 * dt, phi + (D + dt) * 360.0));
    }
    // The boost's operating point has no DC solution with the flying capacitors at their levels (the inductor shorts
    // the switch node to Vin), and nothing would pull them there (see fcml_periodic_start): start the flying
    // capacitors, the output and the inductor on the periodic orbit.
    if (c.boost) {
        // The lossless deck settles at Vo/η into the Vo²/P load, i.e. Io/η.
        const AN::FcmlStart st = AN::fcml_periodic_start(true, N, c.inputVoltage, VoCell, Io / c.efficiency, fs, c.inductance,
                                                         c.flyingCapacitance, c.outputCapacitance);
        json ics = json::array();
        ics.push_back({{"node", "Vout"}, {"voltage", st.outputVoltage}});
        ics.push_back({{"node", "switchingCell.sw_node"}, {"voltage", 0.0}});
        for (int k = 1; k < pairs; ++k) {
            ics.push_back({{"node", "switchingCell.nh" + std::to_string(k)}, {"voltage", st.flyingCapacitorVoltages[k - 1]}});
            ics.push_back({{"node", "switchingCell.nl" + std::to_string(k)}, {"voltage", 0.0}});
        }
        ics.push_back({{"stage", "switchingCell"}, {"component", "L1"}, {"winding", 0}, {"current", st.inductorCurrent}});
        tas["simulation"]["initialConditions"] = ics;
    }
    req::finalize_control_seeds(tas, c.boost ? Topology::BOOST_CONVERTER : Topology::BUCK_CONVERTER);
    return tas;
}

} // namespace Kirchhoff
