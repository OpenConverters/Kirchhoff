#include "PushPull.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_push_pull + excitations_processed/winding_current
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kMaxDuty = 0.48;     // MKF PushPull default (D < 0.5 strictly)
constexpr double kRippleRatio = 0.4;  // output-inductor current ripple
} // namespace

PushPullDesign design_push_pull(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    PushPullDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    d.maxDutyCycle = cfg::get(d.config, "maxDutyCycle", kMaxDuty);
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double iout = d.outputPower / d.outputVoltage, fsw = d.switchingFrequency, T = 1.0 / fsw;
    // Turns ratio (MKF): N = D_max * 2 * Vin_min / (Vout + Vd). Rounded to 2 dp.
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    double N = d.maxDutyCycle * 2.0 * vinMin / (d.outputVoltage + d.diodeDrop);
    N = std::round(N * 100.0) / 100.0;
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value, so the rest of the stage is sized around the fixed transformer. The
    // primary:secondary step-down ratio is turnsRatios[1] — index 0 is the centre-tapped primary-half
    // ratio (1.0), so read index 1 (matches the order build_push_pull_tas emits to magnetic_inputs).
    N = req::provided_turns_ratio(dr, 1).value_or(N);
    d.turnsRatio = N;
    // Magnetizing inductance per half (MKF): Lm = Vin_min * tOn / Iprimary, tOn = D_max*T,
    // Iprimary = Pout / Vin_min / eff.
    const double tOn = d.maxDutyCycle * T;
    const double iPrimary = d.outputPower / vinMin / d.efficiency;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        vinMin * tOn / iPrimary);
    // Output inductor (MKF, worst case = max Vin): tOn_sec = (T/2)*(Vout+Vd)*N/Vin; ΔI = ripple*Iout;
    // Lout = (Vin/N - Vout) * tOn_sec / ΔI.
    const double tOnSec = (T / 2.0) * (d.outputVoltage + d.diodeDrop) * N / vinMax;
    const double dILout = cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * iout;
    d.outputInductance = (vinMax / N - d.outputVoltage) * tOnSec / dILout;
    // Operating per-switch duty at nominal Vin: Vout = 2*D*Vin/N  ->  D = N*Vout/(2*Vin).
    d.dutyCycle = N * (d.outputVoltage + d.diodeDrop) / (2.0 * d.inputVoltage);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);

    // Per-output legs (multi-output: N isolated center-tapped secondaries, ABT #86). Every rail sees the
    // SAME primary volt-seconds, so each turns ratio scales with its own (Vout_i+Vd_i):
    // N_i = D_max·2·Vin_min/(Vout_i+Vd_i) — each rail regulates to its own Vout at the shared max duty.
    // Rectifier/output-filter sizing repeats the main-output formulas per rail. outputs[0] reproduces the
    // scalars above byte-for-byte (a pinned turnsRatios[1+i] still overrides per rail).
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = dr.at("outputs").size();
    for (size_t i = 0; i < nOut; ++i) {
        PushPullOutputLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(i).at("voltage"));
        if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
            leg.power = tasInputs.at("operatingPoints").at(0).at("outputs").at(i).at("power").get<double>();
        else
            leg.power = nominal(dr.at("outputs").at(i).at("power"));
        const double iout_i = leg.power / leg.voltage;
        leg.diodeDrop = req::dideal_diode_drop(iout_i);
        if (i == 0) {
            leg.turnsRatio = d.turnsRatio;
            leg.diodeDrop = d.diodeDrop;      // preserve the main rail's exact scalar value
            leg.outputInductance = d.outputInductance;
            leg.outputCapacitance = d.outputCapacitance;
        } else {
            double ni = d.maxDutyCycle * 2.0 * vinMin / (leg.voltage + leg.diodeDrop);
            ni = std::round(ni * 100.0) / 100.0;
            leg.turnsRatio = req::provided_turns_ratio(dr, 1 + i).value_or(ni);
            const double tOnSec_i = (T / 2.0) * (leg.voltage + leg.diodeDrop) * leg.turnsRatio / vinMax;
            leg.outputInductance = (vinMax / leg.turnsRatio - leg.voltage) * tOnSec_i / (ripple * iout_i);
            leg.outputCapacitance = d.outputCapacitance;
        }
        leg.loadResistance = leg.voltage * leg.voltage / leg.power;
        d.outputs.push_back(leg);
    }
    return d;
}

json build_push_pull_tas(const PushPullDesign& d) {
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };
    auto mosfet = []() { json j; j["semiconductor"]["mosfet"] = json::object(); return j; };
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };

    const double Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency;
    const double Vin = d.inputVoltage, Dn = d.dutyCycle;

    // --- per-winding stresses from the SINGLE FHA source (the SPICE-validated analytical solver) ---
    // Two operating points: the WORST-CASE corner (Vin_min → higher per-switch duty → higher primary
    // conduction) drives the transformer/switch RATINGS; the DECLARED nominal operating point is what the
    // TAS embeds for the transformer. The output inductors Lout_i are SEPARATE magnetics (not
    // analytical_push_pull's transformer windings), so they keep their inline buck-derived stresses.
    //
    // Multi-output (ABT #86): every declared output is its OWN center-tapped secondary pair. The solver
    // takes the full {Vout_i}/{Iout_i}/{N_i} vectors (primary halves carry the summed reflected current);
    // the deck loops a rectifier + output filter per rail. Single-output reproduces the canonical deck.
    namespace AN = Kirchhoff::analytical;
    const double rippleRatio = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = d.outputs.size();
    std::vector<double> Vouts, iouts, turnsN;
    double totalOutputPower = 0.0;
    for (const auto& leg : d.outputs) {
        Vouts.push_back(leg.voltage);
        iouts.push_back(leg.power / leg.voltage);
        turnsN.push_back(leg.turnsRatio);
        totalOutputPower += leg.power;
    }
    const MAS::OperatingPoint aopWorst = AN::analytical_push_pull(d.inputVoltageMin, Vouts, iouts, turnsN, fsw,
                                            Lm, d.outputInductance, rippleRatio, d.diodeDrop);
    const MAS::OperatingPoint aopNom   = AN::analytical_push_pull(d.inputVoltage,    Vouts, iouts, turnsN, fsw,
                                            Lm, d.outputInductance, rippleRatio, d.diodeDrop);
    // Primary-half switch conduction (winding 0 = "Primary Half 1") from the worst-case corner.
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");
    // Per-switch operating duty is shared across rails (common primary); the output-inductor volt-seconds
    // (below, per rail) use it directly.
    const double D2 = std::min(1.0, 2.0 * Dn);

    // --- semiconductor stresses (push-pull, center-tapped) ---
    // Each primary switch blocks 2*Vin: when its half is off, the conducting half drives the center tap
    // such that the off switch drain sees the rail (Vin) plus the reflected opposite half (Vin) = 2*Vin.
    // Worst case at the max input corner: ratedVds = 2*Vin_max / V_DERATE. RdsOn budget spans the TOTAL
    // rail power (all outputs flow through the one primary pair).
    const double VdsStress = 2.0 * d.inputVoltageMax;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn  = (IrmsPri > 0.0)
        ? cfg::rds_on_loss_fraction(d.config) * totalOutputPower / (IrmsPri * IrmsPri)
        : cfg::rds_on_loss_fraction(d.config) * totalOutputPower;
    const double Tsw      = 1.0 / fsw;
    const double maxTrr   = 0.05 * Tsw;
    auto mosfetReq = [&]() { json j = mosfet();
        j["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0); return j; };
    // Center-tapped full-wave output rectifiers Dtop_i/Dbot_i are REAL rectifiers (not FET body diodes):
    // each reverse-blocks the FULL secondary swing 2*Vin/N_i (the non-conducting diode sees both half
    // windings in series, each at Vin/N_i) and carries the inductor current during its half-cycle (~Iout_i).
    auto rectDiode = [&](double N_i, double iout_i) { json j = diode();
        const double ratedVr_i = (2.0 * d.inputVoltageMax / N_i) / cfg::v_derate_diode(d.config);
        const double maxVf_i   = (ratedVr_i < 100.0) ? 0.6 : 1.2;
        j["inputs"]["designRequirements"] = req::diode(ratedVr_i, iout_i / 0.7, maxVf_i, maxTrr); return j; };

    // (2 + 2·N)-winding transformer: turnsRatios = [1 (2nd primary half), (N_i, N_i) per output].
    // Dot/terminal order mirrors MKF: Lpri_top pri_top->center_tap; Lpri_bot center_tap->pri_bot;
    // each Lsec_top_i sec_top_i->gnd; Lsec_bot_i gnd->sec_bot_i. isolationSides: both primary halves
    // "primary"; each output's pair shares its own isolation side (secondary, tertiary, …).
    // Sourced from analytical_push_pull at the nominal operating point — the FHA physics lives in ONE place.
    std::vector<double> xfmrRatios{1.0};                       // secondary1 = 2nd primary half (1:1)
    std::vector<std::string> xfmrIso{"primary", "primary"};    // PriHalf1 (implicit) + PriHalf2
    std::vector<bool> ceil{false};                             // 2nd primary half is structural 1:1 {nominal}
    for (size_t i = 0; i < nOut; ++i) {
        const std::string side = req::isolation_side(1 + i);   // out0->secondary, out1->tertiary, …
        xfmrRatios.push_back(d.outputs[i].turnsRatio);  xfmrRatios.push_back(d.outputs[i].turnsRatio);
        xfmrIso.push_back(side);                        xfmrIso.push_back(side);
        ceil.push_back(true);                           ceil.push_back(true);   // duty-derived {maximum}
    }
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, xfmrRatios, xfmrIso, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "T1"), ceil);
    json capd; capd["capacitor"] = json::object();      // main-rail output cap (output-filter stage)
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;
    // Snubber caps at the switching nodes. A push-pull's center-tapped transformer leaves the primary
    // magnetizing current without a path during the dead time between the two 180-deg phases; with an
    // IDEAL transformer (no parasitic C) the node voltage runs away and ngspice fails (timestep too
    // small). A small node-to-gnd snubber cap gives that current a finite-dV/dt path — physically real
    // in any push-pull. It is sized from the ENERGY BUDGET (snubber_cap = eps·P/(Vblock²·fsw), the
    // off-switch blocks ~2·Vin) rather than a fixed 2.2 nF: the fixed value over-sizes 7–39× at these
    // operating points and, ringing against the ideal lossless transformer, injects a load-independent
    // reactive charge that lifts Vout a few % (worst at the lowest current). The energy-budget cap is
    // still large enough to give the dead-time magnetizing current a finite-dV/dt path (decks converge),
    // while keeping the open-loop Vout on the analytical target. Overridable via config "snubberCap".
    auto snub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] =
            cfg::snubber_cap(d.config, totalOutputPower, 2.0 * d.inputVoltage, d.switchingFrequency);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        cfg::mark_numerical_aid(c);   // dV/dt convergence aid — tagged for the real-fidelity strip (ABT #96)
        return c; };

    // Series-RC leakage damper across the primary (drain-to-drain), mirroring the AHB Rdmp/Cdmp fix
    // (src/Ahb.cpp). The push-pull transformer is tightly coupled (K=0.9999), so its leakage (~Lm(1-K^2))
    // is tiny and rings (a few MHz) against the switch Coss / the node-snubber caps at every dead-time
    // commutation; with ideal zero-DCR windings nothing damps it, so ngspice keeps shrinking dt to track
    // the ring and the transient crawls / hangs ("Timestep too small") — worst under datasheet models and
    // many settle cycles (ABT #79). This RC damps that ring — R ~ tank Z0 = sqrt(Lleak/Coss) — while the
    // series C is high-impedance at fsw (~1.6 kohm at 100 kHz for 1 nF), so it barely loads the switching
    // fundamental (loss ~ mW) and leaves the open-loop Vout on the analytical target. NOT a dV/dt node
    // snubber -> the "dmp" name keeps it out of the fidelity snubber strip so it survives the real deck.
    auto dampR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"]; dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = cfg::get(d.config, "leakDampR", 10.0);
        dr["powerRating"] = 1.0; dr["role"] = "damping"; return c; };
    auto dampC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::get(d.config, "leakDampC", 1.0e-9);
        c["inputs"]["designRequirements"]["ratedVoltage"] = 2.0 * d.inputVoltage * 3; return c; };

    // Per-output filter inductor Lout_i (SEPARATE magnetic): DC-biased triangle, avg=Iout_i; sees
    // +(Vin/N_i - Vout_i) when either of the rail's diodes conducts (combined duty 2·Dn), -Vout_i otherwise.
    auto loutMag = [&](double N_i, double Vout_i, double iout_i, double Lo_i) {
        const double dILout = rippleRatio * iout_i;
        const double IpkLout = iout_i + dILout / 2.0;
        const double IrmsLout = std::sqrt(iout_i * iout_i + dILout * dILout / 12.0);
        const double vLoutOn = Vin / N_i - Vout_i;
        const double vLoutPk = std::max(std::fabs(vLoutOn), Vout_i), vLoutPkPk = Vin / N_i;
        const double vLoutRms = std::sqrt(D2 * vLoutOn * vLoutOn + (1.0 - D2) * Vout_i * Vout_i);
        json l; l["magnetic"] = json::object();
        l["inputs"] = req::magnetic_inputs(Lo_i, 0.2, /*single winding*/ {}, {"primary"}, std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpkLout, IrmsLout, iout_i, dILout, D2,
                                    vLoutPk, vLoutRms, 0.0, vLoutPkPk)});
        return l; };

    // Base components (primary side, shared): transformer + two low-side switches; per-rail rectifier +
    // output inductor (+ per-rail snubbers) are appended in the loop; primary snubbers + leakage damper last.
    std::vector<json> comps{comp("T1", xfmr), comp("Q1", mosfetReq()), comp("Q2", mosfetReq())};
    std::vector<json> cports{port("vin"), port("gnd"), port("vout"), port("gate1"), port("gate2")};
    std::vector<json> conns;
    conns.push_back(conn("vin_net", {pin("T1", "primary_end"), pin("T1", "secondary1_start"), prt("vin")}));
    conns.push_back(conn("pri_top", {pin("T1", "primary_start"), pin("Q1", "drain"), pin("Csn1", "1"), pin("Rdmp", "1")}));
    conns.push_back(conn("pri_bot", {pin("T1", "secondary1_end"), pin("Q2", "drain"), pin("Csn2", "1"), pin("Cdmp", "2")}));
    conns.push_back(conn("dmp_mid", {pin("Rdmp", "2"), pin("Cdmp", "1")}));
    std::vector<json> gndEps{pin("Q1", "source"), pin("Q2", "source")};

    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        const double iout_i = leg.power / leg.voltage;
        const std::string sfx = (i == 0) ? std::string() : std::to_string(i + 1);   // "", "2", "3", …
        const std::string dtop = "Dtop" + sfx, dbot = "Dbot" + sfx, loutN = "Lout" + sfx;
        const std::string csnT = "Csn" + std::to_string(3 + 2 * i), csnB = "Csn" + std::to_string(4 + 2 * i);
        const std::string half1 = "secondary" + std::to_string(2 + 2 * i);  // Sec i Half 1 winding
        const std::string half2 = "secondary" + std::to_string(3 + 2 * i);  // Sec i Half 2 winding

        comps.push_back(comp(dtop.c_str(), rectDiode(leg.turnsRatio, iout_i)));
        comps.push_back(comp(dbot.c_str(), rectDiode(leg.turnsRatio, iout_i)));
        comps.push_back(comp(loutN.c_str(), loutMag(leg.turnsRatio, leg.voltage, iout_i, leg.outputInductance)));

        // secondary halves -> full-wave rectifier diodes (rail center tap = gnd) (+ snubber cap to gnd)
        conns.push_back(conn(("sec_top" + sfx).c_str(),  {pin("T1", (half1 + "_start").c_str()), pin(dtop.c_str(), "anode"), pin(csnT.c_str(), "1")}));
        conns.push_back(conn(("sec_bot" + sfx).c_str(),  {pin("T1", (half2 + "_end").c_str()),   pin(dbot.c_str(), "anode"), pin(csnB.c_str(), "1")}));
        conns.push_back(conn(("sec_rect" + sfx).c_str(), {pin(dtop.c_str(), "cathode"), pin(dbot.c_str(), "cathode"), pin(loutN.c_str(), "primary_start")}));
        if (i == 0) {
            conns.push_back(conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}));   // main rail -> output-filter stage
        } else {
            json capi; capi["capacitor"] = json::object();
            capi["inputs"]["designRequirements"]["capacitance"]["nominal"] = leg.outputCapacitance;
            capi["inputs"]["designRequirements"]["ratedVoltage"] = leg.voltage * 2;
            const std::string coutN = "Cout" + sfx, voutP = "vout" + sfx;
            comps.push_back(comp(coutN.c_str(), capi));
            cports.push_back(port(voutP.c_str()));
            conns.push_back(conn((voutP + "_net").c_str(), {pin(loutN.c_str(), "primary_end"), pin(coutN.c_str(), "1"), prt(voutP.c_str())}));
            gndEps.push_back(pin(coutN.c_str(), "2"));
        }
        // rail center tap (= gnd) + this rail's secondary snubber returns
        gndEps.push_back(pin("T1", (half1 + "_end").c_str()));
        gndEps.push_back(pin("T1", (half2 + "_start").c_str()));
        comps.push_back(comp(csnT.c_str(), snub()));
        comps.push_back(comp(csnB.c_str(), snub()));
        gndEps.push_back(pin(csnT.c_str(), "2"));
        gndEps.push_back(pin(csnB.c_str(), "2"));
    }
    // primary snubbers + drain-to-drain leakage damper (appended after the per-rail block)
    comps.push_back(comp("Csn1", snub()));  comps.push_back(comp("Csn2", snub()));
    comps.push_back(comp("Rdmp", dampR())); comps.push_back(comp("Cdmp", dampC()));
    gndEps.push_back(pin("Csn1", "2"));  gndEps.push_back(pin("Csn2", "2"));  gndEps.push_back(prt("gnd"));
    conns.push_back(conn("gnd_net", gndEps));
    conns.push_back(conn("gate1_net", {pin("Q1", "gate"), prt("gate1")}));
    conns.push_back(conn("gate2_net", {pin("Q2", "gate"), prt("gate2")}));

    json cell; cell["name"] = "push-pull-cell";
    cell["ports"] = cports;
    cell["components"] = comps;
    cell["connections"] = conns;

    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capd)});
    filt["connections"] = json::array({
        conn("out", {pin("Cout", "1"), prt("in")}),
        conn("ret", {pin("Cout", "2"), prt("rtn")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
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

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("pushPullCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("pushPullCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("pushPullCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("pushPullCell", "vout"), sp("filter", "in")})};
    for (size_t i = 1; i < nOut; ++i) {
        const std::string g = "Vout" + std::to_string(i + 1), pt = "vout" + std::to_string(i + 1);
        iscs.push_back(isc(g.c_str(), "externalPort", "output", {sp("pushPullCell", pt.c_str())}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Two PWM drives 180 deg apart (the interleaved push-pull switching). The stimulus targets each
    // switch's "gate" pin (exposed on ports gate1/gate2); Q2 is phase-shifted half a period.
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "pushPullCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.dutyCycle; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    req::finalize_control_seeds(tas, Topology::PUSH_PULL_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
