#pragma once

// Kirchhoff::RectifierType — the secondary-side rectifier topology variant shared by the isolated
// resonant/bridge converters (LLC, SRC, PSFB, PSHB, AHB). Mirrors MKF's per-topology rectifier enums
// (LlcRectifierType/BRectifierType/SrcRectifierType, all read from the MAS "rectifierType" key), but
// KH selects it from the optional `config` override object (cfg::get_str), so adding variants needs NO
// schema change. See docs/MKF_MIGRATION.md (rectifier-variant port).
//
//   centerTapped (CT) — 2 secondary half-windings + 2 diodes; CT node = output return. 1 diode in the
//                       conduction path; turns ratio per half = Vo_fha/(Vout+Vd).
//   fullBridge   (FB) — 1 secondary winding + 4-diode bridge. 2 diodes in the path → n=Vo_fha/(Vout+2Vd).
//   currentDoubler(CD)— 1 secondary winding + 2 diodes + 2 output inductors (each carries Iout/2, ripple
//                       cancels at 2·fs). 1 diode in the path.
//   voltageDoubler(VD)— 1 secondary winding + 2 diodes + 2 stacked output caps (load across the stack,
//                       each cap holds Vout/2). The cap stack delivers 2·Vsec_pk, so n is DOUBLED:
//                       n = 2·Vo_fha/(Vout+2Vd).

#include <string>
#include <cctype>

namespace Kirchhoff {

enum class RectifierType { CenterTapped, FullBridge, CurrentDoubler, VoltageDoubler };

// Case/space/underscore/hyphen-insensitive parse; falls back to `dflt` for an unknown/absent string so
// each topology keeps its own principled default (LLC/SRC → CT, PSFB/PSHB/AHB → FB).
inline RectifierType parse_rectifier_type(const std::string& s, RectifierType dflt) {
    std::string t;
    for (char c : s)
        if (!std::isspace(static_cast<unsigned char>(c)) && c != '_' && c != '-')
            t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (t == "centertapped"   || t == "ct") return RectifierType::CenterTapped;
    if (t == "fullbridge"     || t == "fb") return RectifierType::FullBridge;
    if (t == "currentdoubler" || t == "cd") return RectifierType::CurrentDoubler;
    if (t == "voltagedoubler" || t == "vd") return RectifierType::VoltageDoubler;
    return dflt;
}

inline const char* rectifier_name(RectifierType r) {
    switch (r) {
        case RectifierType::CenterTapped:   return "centerTapped";
        case RectifierType::FullBridge:     return "fullBridge";
        case RectifierType::CurrentDoubler: return "currentDoubler";
        case RectifierType::VoltageDoubler: return "voltageDoubler";
    }
    return "centerTapped";
}

// Secondary half-windings per output: 2 for center-tapped, 1 for the single-winding variants.
inline int rectifier_windings_per_output(RectifierType r) {
    return (r == RectifierType::CenterTapped) ? 2 : 1;
}

// Diodes in series in the conduction path (drives the Vd term in the turns-ratio sizing): the full
// bridge stacks two, every other variant conducts through one at a time. (VD has two drops in its
// OUTPUT equation — one per stacked cap — handled explicitly where n is doubled.)
inline int rectifier_path_diodes(RectifierType r) {
    return (r == RectifierType::FullBridge) ? 2 : 1;
}

} // namespace Kirchhoff
