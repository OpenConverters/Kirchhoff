#include "Pfc.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include <cmath>
#include <vector>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kSenseResistance   = 0.1;     // input-current sense [Ω]
constexpr double kRippleFraction    = 0.30;    // hysteretic current ripple as a fraction of peak iL
constexpr double kOutputCapacitance = 220e-6;  // bus cap
constexpr double kPi                = 3.14159265358979323846;
} // namespace

PfcDesign design_pfc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    PfcDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.inputVoltageRms    = nominal(dr.at("inputVoltage"));
    d.lineFrequency      = nominal(dr.at("lineFrequency"));
    d.outputVoltage      = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency         = dr.value("efficiency", 1.0);
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
        d.outputPower = tasInputs.at("operatingPoints").at(0).at("outputs").at(0).at("power").get<double>();
    else
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));

    const double pin   = d.outputPower / std::max(d.efficiency, 1e-6);
    const double vpeak = d.inputVoltageRms * std::sqrt(2.0);
    const double rEmul = d.inputVoltageRms * d.inputVoltageRms / pin;  // emulated input resistance
    const double iPeak = vpeak / rEmul;                                // peak inductor/line current

    d.senseResistance = cfg::get(d.config, "senseResistance", kSenseResistance);
    // i_ref voltage = kref·V(busP) must equal the target sense voltage iL·Rsense = (V(busP)/rEmul)·Rsense.
    d.referenceGain = d.senseResistance / rEmul;
    // CCM ripple ΔiL = fraction·iPeak; size L for the target switching frequency at the line peak:
    //   f_peak = Vpk·(Vout−Vpk) / (ΔiL·L·Vout)  =>  L = Vpk·(Vout−Vpk) / (ΔiL·f·Vout).
    const double dIL = cfg::get(d.config, "currentRippleFraction", kRippleFraction) * iPeak;
    d.boostInductance = vpeak * (d.outputVoltage - vpeak)
                        / (dIL * d.switchingFrequency * d.outputVoltage);
    d.currentHysteresis = 0.5 * dIL * d.senseResistance;   // half-band on the i·Rsense signal
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", kOutputCapacitance);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;

    // ── Outer voltage loop: a DESIGNED PI compensator (derived from the plant, not hand-tuned). ──
    // The inner hysteretic current loop makes the stage emulate a conductance Ge = (1−gv)/Rsense, so the
    // small-signal plant from the control gv to the bus voltage is a SINGLE pole — the bus cap loaded by
    // Rload:   P(s) = −K0/(s+ωp),  K0 = Vrms²/(Rsense·C·Vbus),  ωp = 2/(Rload·C)  (the load pole).
    // A PI controller gv = g0 + kp·e + ki·∫e dt  (e = kv·(Vout−Vtarget)) places its zero ωz = ki/kp at the
    // load pole ωp, cancelling it and leaving a clean single-integrator loop → ~90° phase margin, robust
    // across line and load. Crossover ωc is set a DECADE BELOW the line frequency so the voltage loop does
    // not fight the inherent 2·fline bus ripple (which would distort the |Vac| current reference and spoil
    // the power factor). Unity loop gain at ωc fixes kp; ki = kp·ωp closes the pole/zero cancellation:
    //     kp = ωc·Rsense·C·Vbus / (kv·Vrms²) = ωc/(kv·K0),   ki = kp·ωp.
    d.outputDividerGain = 0.01;                                   // kv: V(voutScaled)=kv·Vout (~4 V at 400 V)
    const double kv = d.outputDividerGain;
    const double K0 = d.inputVoltageRms * d.inputVoltageRms
                      / (d.senseResistance * d.outputCapacitance * d.outputVoltage);
    const double wp = 2.0 / (d.loadResistance * d.outputCapacitance);   // load pole
    const double wc = 2.0 * kPi * d.lineFrequency / 10.0;               // crossover (ripple-safe)
    d.proportionalGain = wc / (kv * K0);
    d.integralGain     = d.proportionalGain * wp;
    return d;
}

json build_pfc_tas(const PfcDesign& d) {
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
    auto mosfet = [](const json& reqs) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto diode  = [](const json& reqs) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    auto indBrick = [&](double L, const json& excitation) { json j; j["magnetic"] = json::object();
        j["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"},
            std::nullopt, 25.0, {excitation}); return j; };
    auto resBrick = [&](double r) { json j; j["resistor"] = json::object();
        auto& dr = j["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = r;
        // Conservative requirement floor: a resistor dissipates I^2*R (series) or V^2/R (shunt);
        // the physical value is the smaller. Exact for load resistors (=Pout), safe for sense/divider.
        const double Iout_ = d.outputPower / d.outputVoltage, Vb_ = d.outputVoltage;
        const double i2r_ = Iout_*Iout_*r, v2r_ = Vb_*Vb_/r;
        dr["powerRating"] = (i2r_ < v2r_ ? i2r_ : v2r_);
        return j; };
    auto comparator = [&](double hyst) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = 5.0; e["outputLow"] = 0.0; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };
    auto multiplier = [&]() { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = 1.0; return j; };
    auto summer = [&](double gA, double gB) { json j; json& e = j["analog"]["summer"]["behavioral"];
        e["gainA"] = gA; e["gainB"] = gB; return j; };
    auto integrator = [&](double gain, double initial, double ref, double lo, double hi) {
        json j; json& e = j["analog"]["integrator"]["behavioral"];
        e["gain"] = gain; e["initial"] = initial; e["reference"] = ref;
        e["outputLow"] = lo; e["outputHigh"] = hi; return j; };

    // ───────────── POWER stage: diode bridge + boost + inductor-current sense ─────────────
    // Rsense is in SERIES with the inductor (busP -> Rsense -> nL -> L), so V(busP)-V(nL) = iL·Rsense is
    // the true input/inductor current (a return-side resistor would sense the load return, not iL). The
    // boost return is clean ground.
    // Boost-inductor excitation: the current is a rectified-sine ENVELOPE (peak iPeak at the line peak)
    // with HF switching ripple riding on it. Saturation is set by the line-peak instantaneous peak; heating
    // by the line-rms input current (plus the HF ripple); flux swing (core loss) by the HF voltage at the
    // boost rate. (No DC bias — the envelope returns to ~0 each line zero-crossing; offset = line average.)
    const double pinW  = d.outputPower / std::max(d.efficiency, 1e-6);
    const double vpeak = d.inputVoltageRms * std::sqrt(2.0);
    const double iLineRms = pinW / d.inputVoltageRms;       // input/inductor LF rms (boost emulates a resistor)
    const double iPeakEnv = std::sqrt(2.0) * iLineRms;       // line-peak of the current envelope
    const double dILpk = vpeak * (d.outputVoltage - vpeak)
                         / (d.boostInductance * d.switchingFrequency * d.outputVoltage);  // HF ripple at line peak
    const double IpkL  = iPeakEnv + dILpk / 2.0;
    const double IrmsL = std::sqrt(iLineRms * iLineRms + dILpk * dILpk / 12.0);
    const double IavgL = (2.0 / kPi) * iPeakEnv;             // mean of |line| over a half-cycle
    const double Dpk   = (d.outputVoltage - vpeak) / d.outputVoltage;  // boost duty at the line peak
    const double vIndPk   = std::max(vpeak, d.outputVoltage - vpeak), vIndPkPk = d.outputVoltage;
    const double vIndRms  = std::sqrt(Dpk * vpeak * vpeak + (1.0 - Dpk) * (d.outputVoltage - vpeak) * (d.outputVoltage - vpeak));
    const json indExc = req::winding_excitation("triangular", d.switchingFrequency,
        IpkL, IrmsL, IavgL, dILpk, Dpk, vIndPk, vIndRms, 0.0, vIndPkPk);

    // ── semiconductor requirements (worst-case corner) ──
    // The boost MOSFET SW and the boost diode D5 both block the DC bus Vout. The four bridge
    // diodes D1..D4 rectify the AC line; the off diode blocks ~Vout (the bus rail sits across the
    // off-side bridge legs). All carry the boost-inductor / line current envelope already computed.
    const double ratedVdsP = d.outputVoltage / cfg::v_derate_mosfet(d.config);   // SW blocks Vout
    const double ratedVrBoost = d.outputVoltage / cfg::v_derate_diode(d.config);// D5 blocks Vout
    const double ratedVrBridge = d.outputVoltage / cfg::v_derate_diode(d.config);// bridge diode blocks Vbus
    const double maxRdsOnP = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsL * IrmsL);
    const double maxVfBoost  = (ratedVrBoost  < 100.0) ? 0.6 : 1.2;
    const double maxVfBridge = (ratedVrBridge < 100.0) ? 0.6 : 1.2;
    // Bridge diodes carry the full rectified line current (avg of |i_line|); the boost diode carries
    // the output-side average current. Size both to the line current envelope (peak->continuous rating).
    const double IfBridge = IavgL / 0.7;   // line-current average with margin
    const double IfBoost  = IavgL / 0.7;   // boost-diode average current (= mean inductor current)
    const json reqSW = req::mosfet("mainSwitch", ratedVdsP, IpkL, maxRdsOnP, 125.0);
    const json reqD5 = req::diode(ratedVrBoost, IfBoost, maxVfBoost, 0.05 / d.switchingFrequency);
    const json reqBridge = req::diode(ratedVrBridge, IfBridge, maxVfBridge);  // line-freq rectifier (no trr spec)

    json pcell; pcell["name"] = "pfc-power";
    pcell["ports"] = json::array({port("acLine"), port("acNeutral"), port("gnd"), port("vout"),
                                  port("busP"), port("nL"), port("g")});
    pcell["components"] = json::array({
        comp("D1", diode(reqBridge)), comp("D2", diode(reqBridge)), comp("D3", diode(reqBridge)), comp("D4", diode(reqBridge)),
        comp("Rsense", resBrick(d.senseResistance)),   // inductor-current sense (in series with L)
        comp("L", indBrick(d.boostInductance, indExc)), comp("SW", mosfet(reqSW)), comp("D5", diode(reqD5)),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2)),
        comp("Rref", resBrick(10e3))});                 // floating-mains common-mode reference
    pcell["connections"] = json::array({
        // Full bridge: A=acLine, B=acNeutral, top rail busP, bottom rail = ground.
        conn("acLine",  {pin("D1","anode"), pin("D3","cathode"), prt("acLine")}),
        conn("acNeutral", {pin("D2","anode"), pin("D4","cathode"), pin("Rref","1"), prt("acNeutral")}),
        conn("busP",    {pin("D1","cathode"), pin("D2","cathode"), pin("Rsense","1"), prt("busP")}),
        conn("nL",      {pin("Rsense","2"), pin("L","primary_start"), prt("nL")}),
        conn("gnd_net", {pin("D3","anode"), pin("D4","anode"), pin("SW","source"),
                         pin("Cout","2"), pin("Rref","2"), prt("gnd")}),
        // Boost: nL -> L -> swNode -> D5 -> vout ; SW shunts swNode to ground.
        conn("swNode",  {pin("L","primary_end"), pin("SW","drain"), pin("D5","anode")}),
        conn("vout_net",{pin("D5","cathode"), pin("Cout","1"), prt("vout")}),
        conn("g_net",   {pin("SW","gate"), prt("g")})});

    // ──────────────── CONTROL stage (swappable): current loop + outer VOLTAGE loop ────────────────
    // INNER current loop: the comparator gates the switch on V(nL)−V(vth), where vth = V(busP)·gv. That
    // holds iL ≈ V(busP)·(1−gv)/Rsense ∝ |Vac| → unity PF (the differential sense cancels the busP common
    // mode). OUTER voltage loop: a DESIGNED PI compensator drives gv from the bus error (see design_pfc):
    //     gv = Iv + kp·voutScaled,   Iv = clamp(ivInit + ki·∫(voutScaled − vref)dt)
    // The integrator Iv is the integral path (ki, zero placed at the load pole); the summer Sgv adds the
    // proportional path kp·voutScaled. The constant kp·vref is folded into the integrator's initial so gv
    // starts at g0, and the integrator clamp is the anti-windup rail (gv held to ~0.1×–4× the design
    // conductance). A multiplier forms vth = busP·gv. Bus low → gv falls → more current; high → less.
    const double kref = d.referenceGain;
    const double g0   = 1.0 - kref;                                  // nominal current-loop gain
    const double kv   = d.outputDividerGain;
    const double kp   = d.proportionalGain;
    const double vref = kv * d.outputVoltage;                        // scaled bus setpoint
    const double ivInit = g0 - kp * vref;                            // so gv(0)=g0 at the precharge point
    const double gvLo = 1.0 - 4.0 * kref, gvHi = 1.0 - 0.1 * kref;   // gv draw-range rails (0.1×–4× design)
    const double ivLo = gvLo - kp * vref, ivHi = gvHi - kp * vref;   // integrator clamp = gv rail − offset
    const double rv2 = 1e3, rv1 = rv2 * (1.0 - kv) / kv;
    json ccell; ccell["name"] = "pfc-voltage-current-control";
    ccell["ports"] = json::array({port("busP"), port("nL"), port("vout"), port("gnd"), port("g")});
    ccell["components"] = json::array({
        comp("Rv1", resBrick(rv1)), comp("Rv2", resBrick(rv2)),
        comp("Iv", integrator(d.integralGain, ivInit, vref, ivLo, ivHi)),
        comp("Sgv", summer(1.0, kp)),     // gv = Iv + kp·voutScaled (PI: integral + proportional paths)
        comp("Mv", multiplier()),
        comp("Cmp", comparator(d.currentHysteresis))});
    ccell["connections"] = json::array({
        conn("vout",      {pin("Rv1","1"), prt("vout")}),
        conn("voutScaled",{pin("Rv1","2"), pin("Rv2","1"), pin("Iv","in"), pin("Sgv","inB")}),
        conn("gnd",       {pin("Rv2","2"), prt("gnd")}),
        conn("ivout",     {pin("Iv","out"), pin("Sgv","inA")}),
        conn("gv",        {pin("Sgv","out"), pin("Mv","inB")}),
        conn("busP",      {pin("Mv","inA"), prt("busP")}),
        conn("vth",       {pin("Mv","out"), pin("Cmp","inMinus")}),
        conn("nL",        {pin("Cmp","inPlus"), prt("nL")}),
        conn("g",         {pin("Cmp","out"), prt("g")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acSinglePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltageRms; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pfcController"),
        pstage("pfcPower", "switchingCell", pcell, bind("acLine", "acInput"), bind("vout", "dcOutput")),
        pstage("pfcControl", "control", ccell, bind("nL", "sense"), bind("g", "drive"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("AcLine", "externalPort", "input", {sp("pfcPower", "acLine")}),
        isc("AcNeutral", "externalPort", "input", {sp("pfcPower", "acNeutral")}),
        isc("GND", "externalPort", "input", {sp("pfcPower", "gnd"), sp("pfcControl", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout"), sp("pfcControl", "vout")}),
        isc("busP", "wire", "", {sp("pfcPower", "busP"), sp("pfcControl", "busP")}),
        isc("nL", "wire", "", {sp("pfcPower", "nL"), sp("pfcControl", "nL")}),
        isc("drive", "wire", "", {sp("pfcControl", "g"), sp("pfcPower", "g")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.06; an["maximumTimeStep"] = 2e-7;
    tas["simulation"]["analyses"] = json::array({an});
    // The controller drives the switch (closed loop) — no open-loop stimulus. Precharge the bus so steady
    // state is reached in a few line cycles (the bus-cap RC is far longer than the sim window).
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::POWER_FACTOR_CORRECTION);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
