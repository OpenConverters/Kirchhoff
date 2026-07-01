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

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency;
    const double Vin = d.inputVoltage, Vout = d.outputVoltage, Dn = d.dutyCycle;

    // --- per-winding stresses from the SINGLE FHA source (the SPICE-validated analytical solver) ---
    // Two operating points: the WORST-CASE corner (Vin_min → higher per-switch duty → higher primary
    // conduction) drives the transformer/switch RATINGS; the DECLARED nominal operating point is what the
    // TAS embeds for the transformer. The output inductor Lout is a SEPARATE magnetic (not one of
    // analytical_push_pull's four transformer windings), so it keeps its inline buck-derived stresses.
    namespace AN = Kirchhoff::analytical;
    const double rippleRatio = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const double Iout  = d.outputPower / Vout;
    const MAS::OperatingPoint aopWorst = AN::analytical_push_pull(d.inputVoltageMin, Vout, Iout, fsw,
                                            N, Lm, d.outputInductance, rippleRatio, d.diodeDrop);
    const MAS::OperatingPoint aopNom   = AN::analytical_push_pull(d.inputVoltage,    Vout, Iout, fsw,
                                            N, Lm, d.outputInductance, rippleRatio, d.diodeDrop);
    // Output inductor (SEPARATE magnetic): DC-biased triangle, avg=Iout, ripple from the inductor sizing.
    const double dILout = rippleRatio * Iout;
    const double IpkLout = Iout + dILout / 2.0;
    const double IrmsLout = std::sqrt(Iout * Iout + dILout * dILout / 12.0);
    // Primary-half switch conduction (winding 0 = "Primary Half 1") from the worst-case corner.
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");
    // VOLTAGES (output inductor only; the transformer volt-seconds come from the analytical excitation).
    // The output inductor sees +(Vin/N - Vout) when either diode conducts (combined duty 2·Dn), -Vout otherwise.
    const double D2 = std::min(1.0, 2.0 * Dn);
    const double vLoutOn = Vin / N - Vout;
    const double vLoutPk = std::max(std::fabs(vLoutOn), Vout), vLoutPkPk = Vin / N;
    const double vLoutRms = std::sqrt(D2 * vLoutOn * vLoutOn + (1.0 - D2) * Vout * Vout);

    // --- semiconductor stresses (push-pull, center-tapped) ---
    // Each primary switch blocks 2*Vin: when its half is off, the conducting half drives the center tap
    // such that the off switch drain sees the rail (Vin) plus the reflected opposite half (Vin) = 2*Vin.
    // Worst case at the max input corner: ratedVds = 2*Vin_max / V_DERATE.
    const double VdsStress = 2.0 * d.inputVoltageMax;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn  = (IrmsPri > 0.0)
        ? cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsPri * IrmsPri)
        : cfg::rds_on_loss_fraction(d.config) * d.outputPower;
    // Center-tapped full-wave output rectifiers Dtop/Dbot are REAL rectifiers (not FET body diodes):
    // each reverse-blocks the FULL secondary swing 2*N*Vin (the non-conducting diode sees both half
    // windings in series) and carries the inductor current during its half-cycle (~Iout).
    const double ratedVr  = (2.0 * N * d.inputVoltageMax) / cfg::v_derate_diode(d.config);
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;
    const double Tsw      = 1.0 / fsw;
    const double maxTrr   = 0.05 * Tsw;
    auto mosfetReq = [&]() { json j = mosfet();
        j["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0); return j; };
    auto rectDiode = [&]() { json j = diode();
        j["inputs"]["designRequirements"] = req::diode(ratedVr, Iout / 0.7, maxVf, maxTrr); return j; };

    // 4-winding transformer: turnsRatios = [1 (2nd primary half), N (sec top), N (sec bot)].
    // Dot/terminal order mirrors MKF: Lpri_top pri_top->center_tap; Lpri_bot center_tap->pri_bot;
    // Lsec_top sec_top->gnd; Lsec_bot gnd->sec_bot. Isolated -> isolationSides per winding
    // {primary, primary, secondary, secondary}. PushPullDesign carries no isolationVoltage -> nullopt.
    // Transformer (MAIN magnetic): four center-tapped windings [Primary Half 1/2, Secondary 0 Half 1/2]
    // sourced from analytical_push_pull at the nominal operating point — the FHA physics lives in ONE place.
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {1.0, N, N},
        {"primary", "primary", "secondary", "secondary"}, std::nullopt, 25.0,
        AN::excitations_processed(aopNom),
        // N (computed at maxDutyCycle, line ~42) is the duty CEILING -> emit the two secondary ratios as
        // {maximum}; the 1.0 second-primary half is a structural 1:1 and stays {nominal}.
        /*turnsRatioIsCeiling=*/{false, true, true});
    json lout; lout["magnetic"] = json::object();
    lout["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpkLout, IrmsLout, Iout, dILout, D2,
                                    vLoutPk, vLoutRms, 0.0, vLoutPkPk)});
    json capd; capd["capacitor"] = json::object();
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
            cfg::snubber_cap(d.config, d.outputPower, 2.0 * d.inputVoltage, d.switchingFrequency);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        return c; };

    json cell; cell["name"] = "push-pull-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate1"), port("gate2")});
    cell["components"] = json::array({comp("T1", xfmr), comp("Q1", mosfetReq()), comp("Q2", mosfetReq()),
                                      comp("Dtop", rectDiode()), comp("Dbot", rectDiode()), comp("Lout", lout),
                                      comp("Csn1", snub()), comp("Csn2", snub()),
                                      comp("Csn3", snub()), comp("Csn4", snub())});
    cell["connections"] = json::array({
        // center tap (= Vin) shared by both primary halves
        conn("vin_net",   {pin("T1", "primary_end"), pin("T1", "secondary1_start"), prt("vin")}),
        // primary half tops -> low-side switches (+ snubber cap to gnd)
        conn("pri_top",   {pin("T1", "primary_start"), pin("Q1", "drain"), pin("Csn1", "1")}),
        conn("pri_bot",   {pin("T1", "secondary1_end"), pin("Q2", "drain"), pin("Csn2", "1")}),
        // secondary halves -> full-wave rectifier diodes (center tap = gnd) (+ snubber cap to gnd)
        conn("sec_top",   {pin("T1", "secondary2_start"), pin("Dtop", "anode"), pin("Csn3", "1")}),
        conn("sec_bot",   {pin("T1", "secondary3_end"), pin("Dbot", "anode"), pin("Csn4", "1")}),
        conn("sec_rect",  {pin("Dtop", "cathode"), pin("Dbot", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net",  {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",   {pin("Q1", "source"), pin("Q2", "source"),
                           pin("T1", "secondary2_end"), pin("T1", "secondary3_start"),
                           pin("Csn1", "2"), pin("Csn2", "2"), pin("Csn3", "2"), pin("Csn4", "2"), prt("gnd")}),
        conn("gate1_net", {pin("Q1", "gate"), prt("gate1")}),
        conn("gate2_net", {pin("Q2", "gate"), prt("gate2")})});

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
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("pushPullCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("pushPullCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("pushPullCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("pushPullCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
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
