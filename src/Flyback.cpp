#include "Flyback.hpp"
#include "DimensionJson.hpp"
#include "RasConverter.hpp"
#include "CasConverter.hpp"
#include "SasConverter.hpp"
#include "MasConverter.hpp"
#include "CiasConverter.hpp"
#include "CiasCircuitConverter.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_flyback + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"

#include <sstream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <limits>

namespace Kirchhoff {

using nlohmann::json;

namespace {
double nominal(const json& j, const std::string& what) {
    try { return PEAS::resolve_dimensional_values(j); }
    catch (const std::exception&) { throw std::runtime_error("flyback design: no nominal/min/max for " + what); }
}
} // namespace

FlybackDesign design_flyback(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    FlybackDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"), "outputVoltage");
    d.switchingFrequency = nominal(dr.at("switchingFrequency"), "switchingFrequency");
    d.efficiency = dr.value("efficiency", 0.88);

    // operating point (Vin, Pout): prefer operatingPoints[0], else designRequirements.
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"), "inputVoltage");
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"), "outputPower");
    }

    // Minimum input voltage drives the turns ratio + inductance sizing (MKF semantics).
    const json& iv = dr.at("inputVoltage");
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;
    d.isolationVoltage = dr.value("isolationVoltage", 0.0);

    // Total delivered power across all output rails (multi-output, ABT #86): the magnetizing inductance
    // and the primary current must store/carry the SUM of the rails, not just output 0. Single output ->
    // totalOutputPower == d.outputPower, so every value below stays byte-identical.
    auto output_power_i = [&](size_t i) -> double {
        if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
            return tasInputs.at("operatingPoints").at(0).at("outputs").at(i).at("power").get<double>();
        return nominal(dr.at("outputs").at(i).at("power"), "outputPower");
    };
    const size_t nOut = dr.at("outputs").size();
    double totalOutputPower = 0.0;
    for (size_t i = 0; i < nOut; ++i) totalOutputPower += output_power_i(i);

    // --- CCM design — faithful port of MKF Flyback::process_design_requirements()
    // (maximumDutyCycle branch, single output, no drain-source-voltage limit; Vd=0). ---
    const double rippleRatio  = 0.4;   // currentRippleRatio
    const double maxDutyCycle = 0.5;   // MKF Flyback default maximumDutyCycle

    const double Pin = d.outputPower / d.efficiency;
    const double maxEffectiveLoadCurrent = d.outputPower / d.outputVoltage;        // (η=1 numerator)
    const double averageInputCurrent = Pin / vinMin;
    const double maxEffectiveLoadCurrentReflected =
        averageInputCurrent * (1.0 - maxDutyCycle) / maxDutyCycle;
    double n = maxEffectiveLoadCurrent / maxEffectiveLoadCurrentReflected;
    n = std::round(n * 100.0) / 100.0;   // MKF roundFloat(turnsRatio, 2)
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(n);
    // Steady-state CCM duty at the nominal operating Vin (the open-loop PWM duty). The secondary must
    // produce Vout+Vd so the output AFTER the rectifier drop is the spec'd Vout:
    //   Vout+Vd = Vin·D / (n·(1-D))  ->  D = n·(Vout+Vd) / (Vin + n·(Vout+Vd)).
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    const double Vor = n * (d.outputVoltage + d.diodeDrop);
    d.dutyCycle = Vor / (d.inputVoltage + Vor);

    // Magnetizing inductance sized at the maximum duty / minimum Vin corner (MKF). The secondary ramp
    // (and hence Lm) is sized from the TOTAL rail power so the primary stores enough energy for every
    // output (single output -> totalOutputPower == d.outputPower, so this is byte-identical).
    const double centerSecondaryRamp = (totalOutputPower / d.outputVoltage) / (1.0 - maxDutyCycle);
    const double centerPrimaryRamp   = centerSecondaryRamp / n;
    const double tOn = maxDutyCycle / d.switchingFrequency;
    const double voltsSeconds = vinMin * tOn;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        voltsSeconds / rippleRatio / centerPrimaryRamp);

    // Conduction mode (ABT #80): CCM (default) sizes L for continuous magnetizing current; DCM/BCM size L
    // at/below the CCM–DCM boundary so the magnetizing current resets each cycle; QRM is boundary
    // conduction PLUS a first-valley idle (below). A pinned magnetic (della-Pollock Pass 2) always wins.
    // analytical_flyback already renders DCM vs CCM from L vs its own critical inductance, and the ngspice
    // deck uses d.dutyCycle + d.magnetizingInductance — so the mode flows to the design values, the
    // analytical waveforms AND the simulation from this one knob.
    const std::string mode = cfg::get_str(d.config, "mode", "ccm");
    if (mode != "ccm" && mode != "dcm" && mode != "bcm" && mode != "qrm")
        throw std::runtime_error("design_flyback: unknown conduction mode '" + mode +
                                 "' (flyback modes: ccm, dcm, bcm, qrm)");
    d.mode = mode;
    d.resonantCapacitance = 0.0;
    d.valleyDeadTime = 0.0;
    // CCM–DCM boundary (critical) inductance at the OPERATING point (Basso 2nd ed. p.747) — the same
    // formula analytical_flyback uses, evaluated at the operating Vin (not the min-Vin design corner).
    const double auxV = n * (d.outputVoltage + d.diodeDrop);
    const double Lcrit = (d.outputPower > 0.0)
        ? d.efficiency * d.inputVoltage * d.inputVoltage * auxV * auxV
            / (2.0 * d.outputPower * d.switchingFrequency
               * (d.inputVoltage + auxV) * (auxV + d.efficiency * d.inputVoltage))
        : 0.0;
    if (!req::provided_inductance(dr)) {
        if (mode == "ccm") {
            // keep the ripple-sized CCM inductance above
        } else if (mode == "dcm") {
            d.magnetizingInductance = 0.6 * Lcrit;                 // solidly discontinuous
        } else if (mode == "bcm") {
            d.magnetizingInductance = Lcrit;                        // critical: conduction fills the period
        } else { // qrm
            // Quasi-resonant (first-valley switching) at the declared fsw: boundary conduction plus a
            // half-resonant-period idle t_v = π·√(Lm·Cr) during which the drain rings on Lm·Cr and the
            // switch turns on at the first valley (Vds = Vin − Vor). Cycle timing at the operating point:
            //     T = t_on + t_reset + t_v,   t_reset = Vin·t_on/Vor      (volt-second balance)
            // with full energy transfer per cycle (DCM-family energy balance):
            //     Pin = ½·Lm·Ipk²·fsw,  Ipk = Vin·t_on/Lm   ⇒   Lm = Vin²·t_on²/(2·Pin·T).
            // Substituting Lm into t_v makes t_v ∝ t_on, so t_on solves in closed form:
            //     t_on = T / (1 + Vin/Vor + π·Vin·√(Cr/(2·Pin·T))).
            const double Cr  = cfg::qr_resonant_cap(d.config);
            const double T   = 1.0 / d.switchingFrequency;
            const double Pin = d.outputPower / d.efficiency;
            const double tOn = T / (1.0 + d.inputVoltage / auxV
                                        + M_PI * d.inputVoltage * std::sqrt(Cr / (2.0 * Pin * T)));
            d.magnetizingInductance = d.inputVoltage * d.inputVoltage * tOn * tOn / (2.0 * Pin * T);
            d.resonantCapacitance   = Cr;
            d.valleyDeadTime        = M_PI * std::sqrt(d.magnetizingInductance * Cr);
        }
    }
    if (mode == "qrm" && d.resonantCapacitance == 0.0) {
        // Pinned magnetic: the boundary solve above was skipped, but the valley timing is still defined
        // by the pinned Lm against the drain-node capacitance.
        d.resonantCapacitance = cfg::qr_resonant_cap(d.config);
        d.valleyDeadTime      = M_PI * std::sqrt(d.magnetizingInductance * d.resonantCapacitance);
    }
    // Operating-point duty: DCM/BCM/QRM follow the energy-balance duty D = sqrt(2·Pin·L·fsw)/Vin; CCM keeps
    // the voltage-ratio duty computed above. (For QRM this lands exactly on t_on/T from the solve above:
    // √(2·Pin·Lm·fsw)/Vin = t_on·fsw by construction.)
    if (mode != "ccm") {
        const double Pin = d.outputPower / d.efficiency;
        const double dEnergy = std::sqrt(2.0 * Pin * d.magnetizingInductance * d.switchingFrequency) / d.inputVoltage;
        if (!(dEnergy > 0.0 && dEnergy < 0.99))
            throw std::runtime_error("design_flyback: " + mode + " duty out of range (" +
                                     std::to_string(dEnergy) + ") at this operating point");
        d.dutyCycle = dEnergy;
    }

    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    const double iout = d.outputPower / d.outputVoltage;
    d.outputCapacitance = iout * d.dutyCycle / (d.switchingFrequency * 0.01 * d.outputVoltage);
    d.inputCapacitance = 10e-6;

    // Per-output legs (multi-output: N isolated secondaries, ABT #86). All secondaries see the same
    // magnetizing flux at the shared duty, so each rail's ratio n_i = Np/Ns_i is scaled so that the
    // reflected voltage n_i·(Vout_i+Vd_i) equals the main rail's Vor = n_0·(Vout_0+Vd_0). Higher-voltage
    // rails therefore get a SMALLER n_i. outputs[0] reproduces the scalars above byte-for-byte.
    const double Vor0 = d.turnsRatio * (d.outputVoltage + d.diodeDrop);
    for (size_t i = 0; i < nOut; ++i) {
        FlybackOutputLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(i).at("voltage"), "outputVoltage");
        leg.power   = output_power_i(i);
        const double iout_i = leg.power / leg.voltage;
        leg.diodeDrop = req::dideal_diode_drop(iout_i);
        if (i == 0) {
            leg.turnsRatio = d.turnsRatio;
            leg.diodeDrop  = d.diodeDrop;            // preserve the main rail's exact scalar value
            leg.outputCapacitance = d.outputCapacitance;
        } else {
            double ni = Vor0 / (leg.voltage + leg.diodeDrop);
            ni = std::round(ni * 100.0) / 100.0;
            leg.turnsRatio = req::provided_turns_ratio(dr, i).value_or(ni);
            leg.outputCapacitance = iout_i * d.dutyCycle / (d.switchingFrequency * 0.01 * leg.voltage);
        }
        leg.loadResistance = leg.voltage * leg.voltage / leg.power;
        d.outputs.push_back(leg);
    }
    return d;
}

namespace {
// Pull the single atom out of a to_cias leaf and rename it.
json atom(const json& leaf, const std::string& name) {
    json a = leaf.at("components").at(0);
    a["name"] = name;
    return a;
}
json pe(const std::string& c, const std::string& p) { json e; e["component"] = c; e["pin"] = p; return e; }
json po(const std::string& p) { json e; e["port"] = p; return e; }
json net(const std::string& name, std::vector<json> eps) {
    json n; n["name"] = name; n["endpoints"] = eps; return n;
}
} // namespace

json build_flyback_brick(const FlybackDesign& d, const PEAS::Fidelity& f) {
    // --- ideal/real atoms via the per-family to_cias generators ---
    json mosfetDoc; mosfetDoc["semiconductor"]["mosfet"] = json::object();
    json Q1 = atom(SAS::sas_to_cias(mosfetDoc, f), "Q1");

    json diodeDoc; diodeDoc["semiconductor"]["diode"] = json::object();
    json D1 = atom(SAS::sas_to_cias(diodeDoc, f), "D1");

    json magDoc; magDoc["magnetic"] = json::object();
    magDoc["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    json ratios = json::array(); { json r; r["nominal"] = d.turnsRatio; ratios.push_back(r); }
    magDoc["inputs"]["designRequirements"]["turnsRatios"] = ratios;
    json T1 = atom(MAS::mas_to_cias(magDoc, f), "T1");

    auto capDoc = [](double c, double v) {
        json j; j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = v; return j;
    };
    json Cin  = atom(CAS::cas_to_cias(capDoc(d.inputCapacitance,  d.inputVoltage * 2), f), "Cin");
    json Cout = atom(CAS::cas_to_cias(capDoc(d.outputCapacitance, d.outputVoltage * 2), f), "Cout");

    json rDoc;
    rDoc["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rDoc["inputs"]["designRequirements"]["resistance"]["nominal"] = d.loadResistance;
    rDoc["inputs"]["designRequirements"]["powerRating"] = d.outputPower;
    json Rload = atom(RAS::ras_to_cias(rDoc, f), "Rload");

    // --- wire the flyback (secondary return tied to node 0 for the sim; isolation = later phase) ---
    // Dot convention: L*_pri primary_start->primary_end, L*_sec secondary1_start->secondary1_end, K>0.
    // Flyback: diode conducts during OFF, so the anode is the non-dot end (secondary1_end).
    json components = json::array();
    for (auto& c : {Q1, T1, D1, Cin, Cout, Rload}) components.push_back(c);

    json connections = json::array();
    connections.push_back(net("VIN",   { pe("T1", "primary_start"), pe("Cin", "1"), po("VIN") }));
    connections.push_back(net("DRAIN", { pe("T1", "primary_end"),   pe("Q1", "drain") }));
    connections.push_back(net("GATE",  { pe("Q1", "gate"), po("GATE") }));
    connections.push_back(net("0",     { pe("Q1", "source"), pe("Cin", "2"),
                                         pe("T1", "secondary1_start"),
                                         pe("Cout", "2"), pe("Rload", "2") }));
    // QRM: real drain-source resonant capacitor so the deck's drain rings after diode cutoff and
    // turn-on lands in the first valley (see build_flyback_tas for the sizing rationale).
    if (d.resonantCapacitance > 0.0) {
        const double vdsStress = d.inputVoltageMax + d.turnsRatio * d.outputVoltage;
        components.push_back(atom(CAS::cas_to_cias(capDoc(d.resonantCapacitance,
                                                          vdsStress / cfg::v_derate_capacitor(d.config)), f),
                                  "Cres"));
        connections.at(1)["endpoints"].push_back(pe("Cres", "1"));   // DRAIN
        connections.at(3)["endpoints"].push_back(pe("Cres", "2"));   // 0
    }
    connections.push_back(net("SEC",   { pe("T1", "secondary1_end"), pe("D1", "anode") }));
    connections.push_back(net("VOUT",  { pe("D1", "cathode"), pe("Cout", "1"),
                                         pe("Rload", "1"), po("VOUT") }));

    json ports = json::array();
    for (auto p : {"VIN", "GATE", "VOUT"}) ports.push_back(json{{"name", p}});

    json brick;
    brick["name"] = "flyback";
    brick["ports"] = ports;
    brick["components"] = components;
    brick["connections"] = connections;
    return brick;
}

std::string emit_flyback_ngspice(const FlybackDesign& d, const PEAS::Fidelity& f) {
    json brick = build_flyback_brick(d, f);
    CIAS::CiasCircuit circ = CIAS::CiasCircuit::from_json(brick);
    std::string cards = CIAS::CiasToNgspiceConverter().to_cards(circ);

    const double period = 1.0 / d.switchingFrequency;
    const double ton = d.dutyCycle * period;
    const double tstep = period / 200.0;
    const double tstop = 600.0 * period;  // settle (RC ~ Rload*Cout)
    const double measFrom = tstop - 50.0 * period;

    std::ostringstream os;
    os.precision(10);
    os << "* Kirchhoff flyback (" << (f.is_ideal() ? "ideal" : "real") << ") "
       << d.inputVoltage << "V -> " << d.outputVoltage << "V / " << d.outputPower << "W\n";
    os << "* designed: n=" << d.turnsRatio << " D=" << d.dutyCycle
       << " Lp=" << d.magnetizingInductance << " Rload=" << d.loadResistance
       << " Cout=" << d.outputCapacitance << "\n";
    os << cards;
    os << "Vin VIN 0 DC " << d.inputVoltage << "\n";
    os << "Vgate GATE 0 PULSE(0 5 0 1n 1n " << ton << " " << period << ")\n";
    os << ".options reltol=1e-3 abstol=1e-9 vntol=1e-6 method=gear\n";
    os << ".tran " << tstep << " " << tstop << " 0 " << tstep << "\n";
    os << ".control\n";
    os << "run\n";
    os << "meas tran vout AVG v(VOUT) from=" << measFrom << " to=" << tstop << "\n";
    os << "meas tran vout_pp PP v(VOUT) from=" << measFrom << " to=" << tstop << "\n";
    os << "meas tran iin AVG i(Vin) from=" << measFrom << " to=" << tstop << "\n";
    os << "print vout vout_pp iin\n";
    os << ".endc\n";
    os << ".end\n";
    return os.str();
}

json build_flyback_tas(const FlybackDesign& d) {
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) {
        json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };

    // --- per-component stresses from the SINGLE FHA source (the SPICE-validated analytical solver) ---
    // Worst-case corner (Vin_min) drives the ratings; the declared nominal OP is what the TAS embeds
    // (which also fixes a latent inconsistency: the old code mixed worst-case currents with nominal
    // voltages in the embedded excitation). analytical_flyback carries the DCM secondary correction.
    namespace AN = Kirchhoff::analytical;
    const double n = d.turnsRatio, fsw = d.switchingFrequency, T = 1.0 / fsw, Lm = d.magnetizingInductance;
    const size_t nOut = d.outputs.size();

    // Per-output vectors for the multi-secondary FHA solver (ABT #86). Single output -> {Vout}/{Iout}/{n}
    // and isoSides {primary,secondary} exactly as before, so the single-output TAS stays byte-identical.
    std::vector<double> Vouts, iouts, ratios;
    std::vector<std::string> isoSides{"primary"};
    double totalOutputPower = 0.0;
    for (const auto& leg : d.outputs) {
        Vouts.push_back(leg.voltage);
        iouts.push_back(leg.power / leg.voltage);
        ratios.push_back(leg.turnsRatio);
        isoSides.push_back(req::isolation_side(isoSides.size()));   // sec0->secondary, sec1->tertiary, …
        totalOutputPower += leg.power;
    }
    const double Pin = totalOutputPower / d.efficiency;
    const double IinMin = Pin / d.inputVoltageMin;
    // The design's rectifier drop is passed through (NOT 0) so the solver's reflected voltage Vor —
    // and with it the reset/idle split (t_reset = Vin·t_on/Vor) — matches the deck's DIDEAL diode:
    // the QR valley arc then spans exactly the designed half resonant period t_v = π·√(Lm·Cres).
    // (QRM: d.resonantCapacitance > 0 renders the valley arc + magnetizing ring on the idle tail; at the
    // worst corner Vin_min the converter runs deeper into conduction, the idle vanishes and the ring with it.)
    const double rippleNaN = std::numeric_limits<double>::quiet_NaN();
    const MAS::OperatingPoint aopWorst = AN::analytical_flyback(d.inputVoltageMin, Vouts, iouts,
                                                               ratios, fsw, Lm, d.diodeDrop, d.efficiency,
                                                               rippleNaN, d.resonantCapacitance);
    const MAS::OperatingPoint aopNom   = AN::analytical_flyback(d.inputVoltage,    Vouts, iouts,
                                                               ratios, fsw, Lm, d.diodeDrop, d.efficiency,
                                                               rippleNaN, d.resonantCapacitance);
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");

    // Primary-switch stresses (the one switch carries every rail's reflected power). VOLTAGES at Vin_max;
    // the reflected voltage n_i·(Vout_i+Vd_i) is the shared Vor, so the main-rail term sizes the drain.
    const double maxTrr    = 0.05 * T;
    const double VdsStress = d.inputVoltageMax + n * d.outputVoltage;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn  = 0.01 * totalOutputPower / (IrmsPri * IrmsPri);   // <=1% of total Pout conduction
    const double IcinRms   = std::sqrt(std::max(0.0, IrmsPri * IrmsPri - IinMin * IinMin));

    // --- component PEAS docs: discriminator + detailed inputs.designRequirements ---
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0);

    std::optional<double> isoV = d.isolationVoltage > 0 ? std::optional<double>(d.isolationVoltage) : std::nullopt;

    json mag; mag["magnetic"] = json::object();
    mag["inputs"] = req::magnetic_inputs(Lm, 0.1, ratios, isoSides, isoV, 25.0, AN::excitations_processed(aopNom, "T1"));

    json capCin; capCin["capacitor"] = json::object();
    capCin["inputs"]["designRequirements"] = req::capacitor(
        d.inputCapacitance, d.inputVoltageMax / cfg::v_derate_capacitor(d.config), IcinRms,
        req::ESR_RIPPLE_FRACTION * d.inputVoltage / IpkPri, "inputFilter");
    // REAL RC clamp/snubber across the primary winding (dc+ ↔ sw = the primary, since primary_start=VIN and
    // primary_end=drain). The flyback is HARD-switched: at turn-off the leakage energy rings the drain up
    // past Vin+n·Vout, so a real RC clamp is a genuine board part here (an RCD clamp's damping network),
    // sourced + rendered — NOT a sim-only numerical aid. Sized from the ENERGY BUDGET (cfg::snubber_cap =
    // eps·P/(Vds²·fsw)) + cfg::snubber_res, rated to the drain stress. REAL refdes (Cclmp/Rclmp, role
    // "snubber") -> not matched by the numerical-aid strip (Csn*/Rsn*/Csw*).
    const double clampSnubC = cfg::snubber_cap(d.config, totalOutputPower, VdsStress, fsw);
    const auto clampSnub = req::snubber(clampSnubC, cfg::snubber_res(d.config), VdsStress, fsw);
    const json& clampCap = clampSnub.first;
    const json& clampRes = clampSnub.second;

    // --- stage bricks ---
    json inv; inv["name"] = "primary-switch";
    inv["ports"] = json::array({port("dc+"), port("sw"), port("gate"), port("dc-")});
    inv["components"] = json::array({comp("Q1", mosfet), comp("Cin", capCin),
                                     comp("Cclmp", clampCap), comp("Rclmp", clampRes)});
    inv["connections"] = json::array({
        conn("dc_pos", {pin("Cin", "1"), pin("Cclmp", "1"), prt("dc+")}),
        conn("sw",     {pin("Q1", "drain"), pin("Rclmp", "2"), prt("sw")}),
        conn("clmp_mid", {pin("Cclmp", "2"), pin("Rclmp", "1")}),
        conn("gate",   {pin("Q1", "gate"), prt("gate")}),
        conn("dc_neg", {pin("Q1", "source"), pin("Cin", "2"), prt("dc-")})});
    // QRM: REAL drain-source resonant capacitor (CAS role "resonant") — the valley-timing element the
    // design solves around (t_v = π·√(Lm·Cres)): physically the lumped switch Coss + transformer winding
    // capacitance (plus any added ZVS-extension C). Included as a sourced part so the simulated drain
    // actually rings after diode cutoff and turn-on lands in the first valley. NOT a numerical aid — never
    // stripped at real fidelity.
    if (d.resonantCapacitance > 0.0) {
        json capCres; capCres["capacitor"] = json::object();
        capCres["inputs"]["designRequirements"] = req::capacitor(
            d.resonantCapacitance, ratedVds,
            d.resonantCapacitance * VdsStress * fsw,           // ~charge·f_sw it cycles per period
            0.01 * std::sqrt(Lm / d.resonantCapacitance),      // ESR ≪ Z_r keeps the valley tank underdamped
            "resonant");
        inv["components"].push_back(comp("Cres", capCres));
        for (auto& c : inv["connections"]) {
            if (c["name"] == "sw")     c["endpoints"].push_back(pin("Cres", "1"));
            if (c["name"] == "dc_neg") c["endpoints"].push_back(pin("Cres", "2"));
        }
    }

    // typed terminal bindings (portType) + stage/ISC builders — a powerStage has one input + one output;
    // the isolation stage fans out (outputPorts[]). Defined before the per-output loop that uses them.
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const std::string& name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto istage = [](const char* name, json brick, json inb, std::vector<json> outbs) {
        json s; s["name"] = name; s["role"] = "isolation"; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPorts"] = outbs; return s; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const std::string& name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir;
        c["endpoints"] = eps; return c; };

    // --- isolation transformer: primary + N secondary windings, one (sec,sec_rtn) port pair per rail ---
    json xfmr; xfmr["name"] = "flyback-transformer";
    std::vector<json> xports{port("pri"), port("pri_rtn")};
    std::vector<json> xconns{
        conn("primary",     {pin("T1", "primary_start"), prt("pri")}),
        conn("primary_rtn", {pin("T1", "primary_end"),   prt("pri_rtn")})};
    std::vector<json> secBinds;

    // --- per-output rectifier + output-filter stages. Every secondary return + filter rtn ties to GND
    // (galvanic isolation between rails is a later phase; the sim references all rails to one ground). ---
    std::vector<json> rectFiltStages;
    std::vector<json> outIscs;                        // (sec wire, Vout external) per rail, in output order
    std::vector<json> gndEps{sp("inverter", "dc-")};  // GND ISC endpoints (primary return first)

    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        const std::string sfx = (i == 0) ? std::string() : std::to_string(i + 1);   // "", "2", "3", …
        const std::string wStart = "secondary" + std::to_string(i + 1) + "_start";
        const std::string wEnd   = "secondary" + std::to_string(i + 1) + "_end";
        const std::string secP = "sec" + sfx, secRtnP = "sec_rtn" + sfx;
        const std::string rectStage = "rectifier" + sfx, filtStage = "filter" + sfx;
        const std::string dName = (i == 0) ? "D1" : "D" + std::to_string(i + 1);
        const std::string coutName = "Cout" + sfx;
        const std::string voutPort = "Vout" + sfx;

        // rail ratings from its OWN secondary winding (worst corner). Rectifier blocks Vout_i + Vin_max/n_i.
        const double IpkSec  = AN::winding_current(aopWorst, i + 1, "peak");
        const double IrmsSec = AN::winding_current(aopWorst, i + 1, "rms");
        const double Iout_i  = leg.power / leg.voltage;
        const double VrStress = leg.voltage + d.inputVoltageMax / leg.turnsRatio;
        const double ratedVr  = VrStress / cfg::v_derate_diode(d.config);
        const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;               // Schottky-class if low V_R
        const double IcoutRms = std::sqrt(std::max(0.0, IrmsSec * IrmsSec - Iout_i * Iout_i));

        json diode; diode["semiconductor"]["diode"] = json::object();
        diode["inputs"]["designRequirements"] = req::diode(ratedVr, Iout_i / 0.7, maxVf, maxTrr);
        json capCout; capCout["capacitor"] = json::object();
        capCout["inputs"]["designRequirements"] = req::capacitor(
            leg.outputCapacitance, leg.voltage / cfg::v_derate_capacitor(d.config), IcoutRms,
            req::ESR_RIPPLE_FRACTION * leg.voltage / IpkSec, "outputFilter");

        // transformer secondary winding -> its (sec,sec_rtn) port pair (flyback diode conducts during OFF,
        // so the secondary_end/dot-opposite end feeds the rectifier anode; the dot end returns to ground).
        xports.push_back(port(secP.c_str()));
        xports.push_back(port(secRtnP.c_str()));
        xconns.push_back(conn(("secondary" + sfx).c_str(),     {pin("T1", wEnd.c_str()),   prt(secP.c_str())}));
        xconns.push_back(conn(("secondary_rtn" + sfx).c_str(), {pin("T1", wStart.c_str()), prt(secRtnP.c_str())}));
        secBinds.push_back(bind(secP.c_str(), "hfAc"));

        json rect; rect["name"] = "diode-rectifier" + sfx;
        rect["ports"] = json::array({port("ac_in"), port("dc_out")});
        rect["components"] = json::array({comp(dName.c_str(), diode)});
        rect["connections"] = json::array({
            conn("anode",   {pin(dName.c_str(), "anode"),   prt("ac_in")}),
            conn("cathode", {pin(dName.c_str(), "cathode"), prt("dc_out")})});

        // The output filter is part of the converter; the LOAD is not — it is synthesized from the
        // outputs requirement by the assembler (the dual of the input source). So this stage is Cout only.
        json filt; filt["name"] = "output-filter" + sfx;
        filt["ports"] = json::array({port("in"), port("rtn")});
        filt["components"] = json::array({comp(coutName.c_str(), capCout)});
        filt["connections"] = json::array({
            conn("out", {pin(coutName.c_str(), "1"), prt("in")}),
            conn("ret", {pin(coutName.c_str(), "2"), prt("rtn")})});

        rectFiltStages.push_back(pstage(rectStage, "outputRectifier", rect,
                                        bind("ac_in", "hfAc"), bind("dc_out", "pulsatingDc")));
        rectFiltStages.push_back(pstage(filtStage, "outputFilter", filt,
                                        bind("in", "pulsatingDc"), bind("in", "dcOutput")));

        outIscs.push_back(isc("sec" + sfx, "wire", "",
                              {sp("transformer", secP.c_str()), sp(rectStage.c_str(), "ac_in")}));
        outIscs.push_back(isc(voutPort, "externalPort", "output",
                              {sp(rectStage.c_str(), "dc_out"), sp(filtStage.c_str(), "in")}));
        gndEps.push_back(sp("transformer", secRtnP.c_str()));
        gndEps.push_back(sp(filtStage.c_str(), "rtn"));
    }

    xfmr["ports"] = xports;
    xfmr["components"] = json::array({comp("T1", mag)});
    xfmr["connections"] = xconns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    if (d.isolationVoltage > 0) dreq["isolationVoltage"] = d.isolationVoltage;
    dreq["outputs"] = json::array();
    json opDoc; opDoc["name"] = "full_load"; opDoc["inputVoltage"] = d.inputVoltage; opDoc["ambientTemperature"] = 25.0;
    opDoc["outputs"] = json::array();
    for (size_t i = 0; i < nOut; ++i) {
        const std::string oname = (i == 0) ? "out" : "out" + std::to_string(i + 1);
        json o; o["name"] = oname; o["voltage"]["nominal"] = d.outputs[i].voltage; o["regulation"] = "voltage";
        dreq["outputs"].push_back(o);
        json oo; oo["name"] = oname; oo["power"] = d.outputs[i].power; opDoc["outputs"].push_back(oo);
    }
    tas["inputs"]["operatingPoints"] = json::array({opDoc});

    std::vector<json> stages{
        req::control_stage("pwmController"),
        pstage("inverter", "inverter", inv, bind("dc+", "dcBus"), bind("sw", "hfAc")),
        istage("transformer", xfmr, bind("pri", "hfAc"), secBinds)};
    for (auto& s : rectFiltStages) stages.push_back(s);
    tas["topology"]["stages"] = stages;

    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("inverter", "dc+"), sp("transformer", "pri")}),
        isc("GND", "externalPort", "input", gndEps),
        isc("sw_node", "wire", "", {sp("inverter", "sw"), sp("transformer", "pri_rtn")})};
    for (auto& c : outIscs) iscs.push_back(c);
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.006); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "inverter"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    req::finalize_control_seeds(tas, Topology::FLYBACK_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
