#include "Vienna.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kBusCapacitance  = 470e-6;
constexpr double kSenseResistance = 0.1;
constexpr double kPi              = 3.14159265358979323846;
// Zero-sequence modulation factor α: the dimensionless transconductance (gm = α/Rsense) of the Vienna
// hysteretic rail-balancing path. The switched zero-sequence current injection has no clean closed form,
// so α is identified ONCE from the validated 400 Hz / 600 W operating point and held; the balancing gain
// then scales correctly with C, Rsense and line frequency for every other design (see design_vienna).
constexpr double kBalanceModulation = 4.0;
} // namespace

ViennaDesign design_vienna(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ViennaDesign d{};
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
    const double iPeak = pin * std::sqrt(2.0) / (3.0 * d.inputVoltageRms);  // per-phase peak (3φ, unity PF)
    const double dIL   = 0.3 * std::max(iPeak, 1e-3);
    d.senseResistance = cfg::get(d.config, "senseResistance", kSenseResistance);
    // i_ref voltage = kref·V(phase) must equal iL·Rsense for iL = V(phase)/rEmul; per phase Pin/3 into
    // rEmul = (Vrms²)/(Pin/3): kref = Rsense·Pin/(3·Vrms²).
    d.referenceGain = d.senseResistance * pin / (3.0 * d.inputVoltageRms * d.inputVoltageRms);
    d.boostInductance = (d.outputVoltage * 0.5) / (4.0 * d.switchingFrequency * dIL);
    // Hysteresis on m = V(phase)·(iref − iL·Rsense): a ±dIL/2 band at the line peak (m-band ∝ |v|).
    d.currentHysteresis = 0.5 * dIL * d.senseResistance * vpeak;
    d.busCapacitance = cfg::get(d.config, "busCapacitance", kBusCapacitance);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;

    // ── Active rail-balancing loop (DERIVED, not a fixed magic gain). The loop bal = −kbal·∫(busP+busN)dt
    // adds a common (zero-sequence) term to every phase current reference. Model: the common term bal [V]
    // offsets each phase's sensed-current reference by bal/Rsense [A]; its zero-sequence part charges the
    // rail-pair imbalance δ=(busP+busN)/2 across the rail caps C — a second-order (cap × integrator)
    // balance plant whose natural frequency is ω_bal = √(kbal·gm/C), gm = α/Rsense the zero-sequence
    // modulation transconductance. Placing ω_bal a few times BELOW the line frequency (so the slow
    // balancing does not distort the per-phase current shaping or fight the 2·fline ripple) gives
    //     kbal = ω_bal²·C·Rsense/α,   ω_bal = 2π·fline/6.
    // Everything scales with the design; only the dimensionless α is empirically identified (see above).
    // The common term is bounded to ±50% of the per-phase current-reference peak (anti-windup rail).
    const double wBal = 2.0 * kPi * d.lineFrequency / 6.0;
    d.balanceGain  = wBal * wBal * d.busCapacitance * d.senseResistance / cfg::get(d.config, "balanceModulation", kBalanceModulation);
    d.balanceClamp = 0.5 * d.referenceGain * vpeak;

    // ── Outer voltage loop: a DESIGNED PI compensator (mirrors design_pfc; derived from the plant). The
    // per-phase hysteretic current loop makes each leg emulate a conductance Ge = g/Rsense (g the dynamic
    // analog of the fixed kref), so the small-signal plant from the control g to the FULL bus (busP−busN)
    // is a single pole — the split bus (two rail caps C in series across the full bus → C/2) loaded by
    // Rload, fed by all THREE phases:  P(s) = K0/(s+ωp),  K0 = 3·Vrms²/(Rsense·(C/2)·Vbus) = 6·Vrms²/
    // (Rsense·C·Vbus),  ωp = 2/(Rload·(C/2)) = 4/(Rload·C) (load pole). A PI g = g0 − kp·busScaled − ki·∫
    // (busScaled − vref)dt places its zero ωz = ki/kp at the load pole ωp (pole/zero cancellation → clean
    // single-integrator loop, ~90° phase margin). Crossover ωc is a DECADE BELOW the line frequency so the
    // slow bus loop does not fight the per-phase current shaping. Unity loop gain at ωc fixes kp; ki = kp·ωp.
    // (Sign: bus high → g down → less current, i.e. NEGATIVE feedback — opposite sense to PFC's complement
    // gv, so the proportional/integral terms SUBTRACT here.)  kp = ωc/(kv·K0),  ki = kp·ωp.
    d.outputDividerGain = cfg::get(d.config, "outputDividerGain", 0.005);   // kv: ~4 V at 800 V
    const double kv  = d.outputDividerGain;
    const double Ceff = 0.5 * d.busCapacitance;                            // two rail caps in series
    const double K0v = 3.0 * d.inputVoltageRms * d.inputVoltageRms
                       / (d.senseResistance * Ceff * d.outputVoltage);
    const double wpv = 2.0 / (d.loadResistance * Ceff);                    // load pole
    // Crossover: a 3-phase bus has only a tiny 6·fline ripple (no 2·fline term to fight, unlike single-phase
    // PFC), so the bus loop can run a few times FASTER than fline without distorting the per-phase shaping —
    // and it must, to settle the precharged bus within the few-line-cycle sim window.
    const double wcv = 2.0 * kPi * d.lineFrequency / 3.0;
    d.proportionalGain = wcv / (kv * K0v);
    d.integralGain     = d.proportionalGain * wpv;
    return d;
}

json build_vienna_tas(const ViennaDesign& d) {
    auto port = [](const std::string& n) { json p; p["name"] = n; return p; };
    auto pin  = [](const std::string& c, const std::string& p) { json e; e["component"]=c; e["pin"]=p; return e; };
    auto prt  = [](const std::string& p) { json e; e["port"]=p; return e; };
    auto conn = [](const std::string& name, std::vector<json> eps) { json c; c["name"]=name; c["endpoints"]=eps; return c; };
    auto comp = [](const std::string& name, json data) { json c; c["name"]=name; c["data"]=data; return c; };
    auto bind = [](const char* p, const char* t) { json b; b["port"]=p; b["type"]=t; return b; };
    auto sp = [](const std::string& st, const std::string& po) { json e; e["stage"]=st; e["port"]=po; return e; };
    auto isc = [](const std::string& name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"]=name; c["kind"]=kind; if (dir[0]) c["direction"]=dir; c["endpoints"]=eps; return c; };
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

    // Per-phase boost-inductor excitation. Each Vienna leg boosts its phase to the HALF-bus (Vout/2, the
    // 3-level step). Current is a rectified-sine envelope (per-phase peak) with HF ripple; voltages are the
    // half-bus boost levels. (Mirrors the single-phase PFC inductor; per-phase current = total/3.)
    const double pinV   = d.outputPower / std::max(d.efficiency, 1e-6);
    const double vpeakV = d.inputVoltageRms * std::sqrt(2.0);
    const double Vhalf  = 0.5 * d.outputVoltage;
    const double iLineRmsV = pinV / (3.0 * d.inputVoltageRms);          // per-phase LF rms
    const double iPeakEnvV = std::sqrt(2.0) * iLineRmsV;                // per-phase line-current peak
    const double dILpkV = Vhalf / (4.0 * d.switchingFrequency * d.boostInductance);  // inverts the L sizing
    const double IpkLV  = iPeakEnvV + dILpkV / 2.0;
    const double IrmsLV = std::sqrt(iLineRmsV * iLineRmsV + dILpkV * dILpkV / 12.0);
    const double IavgLV = (2.0 / kPi) * iPeakEnvV;
    const double DpkV   = std::max(0.0, (Vhalf - vpeakV) / Vhalf);      // boost duty at line peak (to half-bus)
    const double vIndPkV   = std::max(vpeakV, Vhalf - vpeakV), vIndPkPkV = Vhalf;
    const double vIndRmsV  = std::sqrt(DpkV * vpeakV * vpeakV + (1.0 - DpkV) * (Vhalf - vpeakV) * (Vhalf - vpeakV));
    const json indExcV = req::winding_excitation("triangular", d.switchingFrequency,
        IpkLV, IrmsLV, IavgLV, dILpkV, DpkV, vIndPkV, vIndRmsV, 0.0, vIndPkPkV);
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

    // ── per-phase semiconductor requirements (worst-case corner) ──
    // Each Vienna leg's bidirectional switch SW* steers its phase to the midpoint; when off it blocks
    // the HALF bus (Vout/2, the 3-level step). The per-phase rectifier diodes Dp*/Dn* (Dp: X->busP,
    // Dn: busN->X — neither anti-parallel to SW*, which shunts X->gnd) block the FULL bus, NOT the half
    // bus: while one is conducting, node X sits at its rail, so the OTHER diode sees busP−busN = Vout
    // reverse (the canonical Vienna diode rating = the full DC link; only the switches see Vout/2). Rating
    // them for the half bus picks a part that AVALANCHES at ~0.6·Vout and clamps the bus there (it can
    // never boost to target). All carry the per-phase line-current envelope already computed.
    const double ratedVdsV = Vhalf / cfg::v_derate_mosfet(d.config);          // switch blocks the half-bus
    const double ratedVrV  = d.outputVoltage / cfg::v_derate_diode(d.config); // each diode blocks the FULL bus
    const double maxRdsOnV = cfg::rds_on_loss_fraction(d.config) * (d.outputPower / 3.0) / (IrmsLV * IrmsLV);
    const double maxVfV    = (ratedVrV < 100.0) ? 0.6 : 1.2;
    const double IfV       = IavgLV / 0.7;   // per-phase rectifier average current with margin
    const json reqSWV = req::mosfet("mainSwitch", ratedVdsV, IpkLV, maxRdsOnV, 125.0);
    const json reqDV  = req::diode(ratedVrV, IfV, maxVfV);   // phase rectifier (no trr spec)

    json pcell; pcell["name"] = "vienna-power";
    json ports = json::array({port("a"), port("b"), port("c"), port("gnd"), port("busP"), port("busN")});
    json comps = json::array({
        comp("Cp", capBrick(d.busCapacitance, d.outputVoltage)),   // +bus -> midpoint(gnd)
        comp("Cn", capBrick(d.busCapacitance, d.outputVoltage)),   // midpoint(gnd) -> -bus
        comp("Rload", resBrick(d.loadResistance))});               // across the full bus
    json conns = json::array({
        conn("busP_net", {pin("Cp","1"), prt("busP")}),
        conn("busN_net", {pin("Cn","2"), prt("busN")}),
        conn("mid",      {pin("Cp","2"), pin("Cn","1"), prt("gnd")})});
    // Rload across busP..busN
    conns.push_back(conn("rload_p", {pin("Rload","1"), prt("busP")}));
    conns.push_back(conn("rload_n", {pin("Rload","2"), prt("busN")}));

    // Per-phase Vienna leg: phase -> Rsense -> nL -> L -> X ; D+ (X->busP), D- (busN->X) ; bidirectional
    // switch X<->gnd. V(phase)-V(nL) = iL·Rsense is the inductor-current sense.
    const char* ph[3] = {"a", "b", "c"};
    for (int i = 0; i < 3; ++i) {
        const std::string p = ph[i];
        const std::string RS="Rs"+p, L="L"+p, SW="SW"+p, SW2="SQ"+p, DP="Dp"+p, DN="Dn"+p,
                          X="x"+p, G="g"+p, NL="nl"+p, CS="cs"+p;
        comps.push_back(comp(RS, resBrick(d.senseResistance)));
        comps.push_back(comp(L, indBrick(d.boostInductance, indExcV)));
        // Bidirectional midpoint switch = TWO MOSFETs in common-source back-to-back (drains at node X and at
        // the midpoint, sources tied together, BOTH gates = G). A SINGLE MOSFET cannot realise the Vienna
        // bidirectional steering switch: its intrinsic body diode (source→drain) clamps node X to the
        // midpoint on the half-cycle X swings negative, shunting the boost inductor straight to the midpoint
        // with no controllable off-state — the inductor sees the full phase voltage continuously and the
        // current (and so the input power) runs away while the rails collapse. Sharing a source puts the two
        // body diodes back-to-back so the pair BLOCKS both polarities when off and conducts both when on —
        // the true 3-level steering switch. (Ideal/REQUIREMENTS parts carry no body diode, so this reduces
        // to two series switches with unchanged closed-loop behaviour; the fix matters only for real
        // DATASHEET MOSFETs, which DO carry a body diode.)
        comps.push_back(comp(SW,  mosfet(reqSWV)));   // drain=X,        source=CS
        comps.push_back(comp(SW2, mosfet(reqSWV)));   // drain=midpoint, source=CS
        comps.push_back(comp(DP, diode(reqDV)));
        comps.push_back(comp(DN, diode(reqDV)));
        conns.push_back(conn("phin_"+p, {pin(RS,"1"), prt(p)}));                   // phase source -> Rsense
        conns.push_back(conn(NL, {pin(RS,"2"), pin(L,"primary_start"), prt(NL)})); // sense node -> L
        conns.push_back(conn(X, {pin(L,"primary_end"), pin(SW,"drain"),
                                 pin(DP,"anode"), pin(DN,"cathode")}));            // node X
        conns.push_back(conn(CS, {pin(SW,"source"), pin(SW2,"source")}));          // common source (floating)
        conns.push_back(conn("swret_"+p, {pin(SW2,"drain"), prt("gnd")}));         // switch pair -> midpoint
        conns.push_back(conn("dp_"+p, {pin(DP,"cathode"), prt("busP")}));          // X -> +bus
        conns.push_back(conn("dn_"+p, {pin(DN,"anode"), prt("busN")}));            // -bus -> X
        conns.push_back(conn(G+"_net", {pin(SW,"gate"), pin(SW2,"gate"), prt(G)}));
    }
    ports.push_back(port("nla")); ports.push_back(port("nlb")); ports.push_back(port("nlc"));
    ports.push_back(port("ga")); ports.push_back(port("gb")); ports.push_back(port("gc"));
    pcell["ports"] = ports; pcell["components"] = comps; pcell["connections"] = conns;

    // ──────────── CONTROL stage (swappable): per-phase current shaping + outer VOLTAGE loop ────────────
    // INNER per-phase current loop: gate the switch on V(phase)·(g·V(phase) − iL·Rsense) > 0 — same sign as
    // sign(v)·error, which handles the Vienna polarity flip. The error is
    //   err = V(nL) − (1−g)·V(phase) = g·V(phase) − iL·Rsense          (V(nL) = V(phase) − iL·Rsense),
    // so the leg emulates a conductance g/Rsense and holds iL ≈ g·V(phase)/Rsense → unity PF. It is formed
    // from g·V(phase) (a multiplier) summed with (V(nL) − V(phase)) (a difference); multiply the result by
    // V(phase) (a multiplier) and a hysteretic comparator gates the switch.
    // OUTER voltage loop: a DESIGNED PI (see design_vienna) drives the DYNAMIC conductance g from the
    // full-bus error so the bus REGULATES to target. A FIXED g cannot — with real-part losses the bus sags
    // below the boost-feasible region (half-bus < phase peak) and collapses to passive rectification
    // (~600 V, huge circulating current). g = vInt − kp·busScaled, vInt = clamp(gInit − ki·∫(busScaled −
    // vref)dt); bus low → g rises → more current → bus boosts back up. One loop, shared by the three phases.
    auto comparator = [&](double hyst) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = 5.0; e["outputLow"] = 0.0; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };
    auto multiplier = [&]() { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = 1.0; return j; };
    auto summer = [&](double gA, double gB) { json j; json& e = j["analog"]["summer"]["behavioral"];
        e["gainA"] = gA; e["gainB"] = gB; return j; };
    auto integrator = [&](double gain, double init, double ref, double lo, double hi) {
        json j; json& e = j["analog"]["integrator"]["behavioral"];
        e["gain"]=gain; e["initial"]=init; e["reference"]=ref; e["outputLow"]=lo; e["outputHigh"]=hi; return j; };
    json ccell; ccell["name"] = "vienna-control";
    json cports = json::array({port("gnd"), port("busP"), port("busN")});
    json ccomps = json::array();
    json cconns = json::array();
    json gndEps = json::array({prt("gnd")});

    // Outer bus-voltage PI → dynamic emulated conductance g (shared by all three phases). The integrator's
    // initial holds g(0)=g0 (the nominal conductance, = the old fixed kref) at the precharge (bus=target);
    // its clamp is the anti-windup rail (g held to 0.25×–4× the design conductance).
    const double kv     = d.outputDividerGain;
    const double vref   = kv * d.outputVoltage;                  // scaled full-bus setpoint
    const double g0     = d.referenceGain;                       // nominal conductance (old fixed kref)
    const double kp     = d.proportionalGain;
    const double gInit  = g0 + kp * vref;                        // so g(0)=g0 at the precharge point
    const double gLo = 0.25 * g0, gHi = 4.0 * g0;               // conductance draw range
    const double vIntLo = gLo + kp * vref, vIntHi = gHi + kp * vref;   // integrator clamp = g rail + offset
    ccomps.push_back(comp("Svs",   summer(kv, -kv)));            // busScaled = kv·(busP − busN)
    ccomps.push_back(comp("Ivolt", integrator(-d.integralGain, gInit, vref, vIntLo, vIntHi)));
    ccomps.push_back(comp("Sg",    summer(1.0, -kp)));           // g = vInt − kp·busScaled (PI: I + P paths)

    // ACTIVE midpoint/rail balancing: bal = ∫(−kbal·(busP+busN)) is added to EVERY phase current reference
    // (a common, zero-sequence term). When the top rail sags (busP+busN<0), bal rises → the positive half
    // of each phase draws more (charging C+) and the negative half less (sparing C−), restoring busP ≈ −busN
    // under an unbalanced half-load. (The grounded neutral fixes the midpoint; this balances the two RAILS.)
    ccomps.push_back(comp("Simb", summer(1.0, 1.0)));           // imb = busP + busN
    ccomps.push_back(comp("Ibal", integrator(-d.balanceGain, 0.0, 0.0, -d.balanceClamp, d.balanceClamp)));

    cconns.push_back(conn("busP_net",  {pin("Svs","inA"), pin("Simb","inA"), prt("busP")}));
    cconns.push_back(conn("busN_net",  {pin("Svs","inB"), pin("Simb","inB"), prt("busN")}));
    cconns.push_back(conn("busScaled", {pin("Svs","out"), pin("Ivolt","in"), pin("Sg","inB")}));
    cconns.push_back(conn("vInt",      {pin("Ivolt","out"), pin("Sg","inA")}));
    cconns.push_back(conn("imb",       {pin("Simb","out"), pin("Ibal","in")}));
    json gEps   = json::array({pin("Sg","out")});               // the dynamic conductance g, fanned to phases
    json balEps = json::array({pin("Ibal","out")});
    for (int i = 0; i < 3; ++i) {
        const std::string p = ph[i];
        const std::string GVM="Gvm"+p, SUB="Sub"+p, SUM="Sum"+p, ADD="Add"+p, MUL="Mul"+p, CMP="Cmp"+p,
                          NL="nl"+p, GVP="gvp"+p, NMP="nmp"+p, ERR="err"+p, ERP="errp"+p, M="m"+p, G="g"+p;
        ccomps.push_back(comp(GVM, multiplier()));                 // gvp = V(phase)·g   (dynamic iref·Rsense)
        ccomps.push_back(comp(SUB, summer(1.0, -1.0)));            // nmp = V(nL) − V(phase) = −iL·Rsense
        ccomps.push_back(comp(SUM, summer(1.0, 1.0)));             // err = nmp + gvp = g·V(phase) − iL·Rsense
        ccomps.push_back(comp(ADD, summer(1.0, 1.0)));             // err' = err + bal  (balancing)
        ccomps.push_back(comp(MUL, multiplier()));                 // m = V(phase)·err'
        ccomps.push_back(comp(CMP, comparator(d.currentHysteresis)));
        cports.push_back(port(p)); cports.push_back(port(NL)); cports.push_back(port(G));
        cconns.push_back(conn(p,   {pin(GVM,"inA"), pin(SUB,"inB"), pin(MUL,"inA"), prt(p)})); // V(phase)
        cconns.push_back(conn(NL,  {pin(SUB,"inA"), prt(NL)}));                                // V(nL)
        cconns.push_back(conn(GVP, {pin(GVM,"out"), pin(SUM,"inB")}));
        cconns.push_back(conn(NMP, {pin(SUB,"out"), pin(SUM,"inA")}));
        cconns.push_back(conn(ERR, {pin(SUM,"out"), pin(ADD,"inA")}));
        cconns.push_back(conn(ERP, {pin(ADD,"out"), pin(MUL,"inB")}));
        cconns.push_back(conn(M,   {pin(MUL,"out"), pin(CMP,"inPlus")}));
        cconns.push_back(conn(G,   {pin(CMP,"out"), prt(G)}));
        gndEps.push_back(pin(CMP,"inMinus"));
        balEps.push_back(pin(ADD,"inB"));
        gEps.push_back(pin(GVM,"inB"));
    }
    cconns.push_back(json{{"name","gcond"},{"endpoints",gEps}});
    cconns.push_back(json{{"name","bal"},{"endpoints",balEps}});
    cconns.push_back(json{{"name","gnd"},{"endpoints",gndEps}});
    ccell["ports"] = cports; ccell["components"] = ccomps; ccell["connections"] = cconns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acThreePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=d.outputVoltage; o["regulation"]="voltage";
      dreq["outputs"]=json::array({o}); }
    { json op; op["name"]="full_load"; op["inputVoltage"]=d.inputVoltageRms; op["ambientTemperature"]=25.0;
      json o; o["name"]="out"; o["power"]=d.outputPower; op["outputs"]=json::array({o});
      tas["inputs"]["operatingPoints"]=json::array({op}); }

    auto pstage = [&](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"]=name; s["role"]=role; s["circuit"]=brick; s["inputPort"]=inb; s["outputPort"]=outb; return s; };
    tas["topology"]["stages"] = json::array({
        req::control_stage("pfcController"),
        pstage("viennaPower", "switchingCell", pcell, bind("a","acInput"), bind("busP","dcOutput")),
        pstage("viennaControl", "control", ccell, bind("a","sense"), bind("ga","drive"))});
    // a/b/c are the three input phases (also tapped by the control); gnd = source neutral = midpoint.
    json interStage = json::array({
        isc("PhaseA","externalPort","input",{sp("viennaPower","a"), sp("viennaControl","a")}),
        isc("PhaseB","externalPort","input",{sp("viennaPower","b"), sp("viennaControl","b")}),
        isc("PhaseC","externalPort","input",{sp("viennaPower","c"), sp("viennaControl","c")}),
        isc("GND","externalPort","input",{sp("viennaPower","gnd"), sp("viennaControl","gnd")}),
        isc("busP","wire","",{sp("viennaPower","busP"), sp("viennaControl","busP")}),
        isc("busN","wire","",{sp("viennaPower","busN"), sp("viennaControl","busN")})});
    for (int i = 0; i < 3; ++i) {
        const std::string p = ph[i];
        interStage.push_back(isc("nl"+p,"wire","",{sp("viennaPower","nl"+p), sp("viennaControl","nl"+p)}));
        interStage.push_back(isc("drive"+p,"wire","",{sp("viennaControl","g"+p), sp("viennaPower","g"+p)}));
    }
    tas["topology"]["interStageConnections"] = interStage;

    json an; an["type"]="transient"; an["stopTime"]=0.04; an["maximumTimeStep"]=5e-7;
    tas["simulation"]["analyses"] = json::array({an});
    // Closed loop — the control stage drives the switches (no open-loop stimulus). Precharge each
    // half-bus to ±Vdc/2 about the grounded midpoint.
    json ics = json::array();
    { json ic; ic["node"]="busP"; ic["voltage"]= 0.5*d.outputVoltage; ics.push_back(ic); }
    { json ic; ic["node"]="busN"; ic["voltage"]=-0.5*d.outputVoltage; ics.push_back(ic); }
    tas["simulation"]["initialConditions"] = ics;
    req::finalize_control_seeds(tas, "viennaRectifierConverter");  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
