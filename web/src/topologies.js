// Topology catalog: every converter the Kirchhoff engine implements (the
// dispatch table in src/KirchhoffApi.cpp), grouped by family, each with a
// working preset so SOLVE produces a design out of the box.
//
// Preset fields drive the spec form; buildSpec() assembles the TAS
// design-requirements JSON the engine expects.

// The converter families, named and ordered EXACTLY as the OpenMagnetics wizard picker groups them
// (the /wizards dropdown): Filters/PFC → Non-Isolated → Forward/Flyback → Bridge/Push-Pull → Resonant
// → Three-Phase PFC. Keeping the same taxonomy across the tools means a family means the same thing
// in Kirchhoff and OpenMagnetics.
export const FAMILIES = [
  'Filters / PFC',
  'Non-Isolated DC-DC',
  'Isolated Forward / Flyback',
  'Isolated Bridge / Push-Pull',
  'Resonant',
  'Three-Phase PFC',
]

// Labels for the rotary family dial (one detent per family). Real words, stacked on two lines where the
// family name has two parts — mirrors how the OpenMagnetics wizard picker names them.
export const FAMILY_SHORT = {
  'Filters / PFC': ['1-φ', 'PFC'],
  'Non-Isolated DC-DC': ['Non-isol.', 'DC-DC'],
  'Isolated Forward / Flyback': ['Forward', 'Flyback'],
  'Isolated Bridge / Push-Pull': ['Bridge', 'Push-pull'],
  'Resonant': ['Resonant'],
  'Three-Phase PFC': ['3-φ', 'PFC'],
}

const T = (id, name, family, desc, preset = {}) => ({
  id, name, family, desc,
  preset: {
    vinMin: null, vinNom: 48, vinMax: null,
    fs: 100e3, efficiency: 0.9, ambient: 25,
    isolation: null, lineFrequency: null, inputType: 'dc',
    outputs: [{ name: 'out', voltage: 12, power: 60 }],
    minOutputs: 1, maxOutputs: 4,
    ...preset,
  },
})

export const TOPOLOGIES = [
  // ── Non-Isolated DC-DC ──────────────────────────────────────────────────
  T('buck', 'Buck', 'Non-Isolated DC-DC',
    'Step-down switching cell: Q + D + L + Cout.'),
  T('boost', 'Boost', 'Non-Isolated DC-DC',
    'Step-up cell; requires Vout > Vin/η.',
    { vinNom: 12, outputs: [{ name: 'out', voltage: 48, power: 60 }] }),
  T('sepic', 'SEPIC', 'Non-Isolated DC-DC',
    'Step-up/down, non-inverting, coupled via Cs.'),
  T('cuk', 'Ćuk', 'Non-Isolated DC-DC',
    'Step-up/down, inverting; continuous input & output current.'),
  T('zeta', 'Zeta', 'Non-Isolated DC-DC',
    'Step-up/down, non-inverting; buck-like output.'),
  T('fsbb', '4-switch buck-boost', 'Non-Isolated DC-DC',
    'Full bridge over one inductor; seamless up/down.',
    { vinNom: 24, outputs: [{ name: 'out', voltage: 20, power: 60 }] }),

  // ── Isolated Forward / Flyback ────────────────────────────────────────────
  T('flyback', 'Flyback', 'Isolated Forward / Flyback',
    'Isolated single-switch; energy stored in the transformer (CCM).',
    { vinMin: 36, vinNom: 48, vinMax: 60, isolation: 1500,
      outputs: [{ name: 'out', voltage: 12, power: 24 }] }),
  T('isolated_buck', 'Isolated buck (Fly-Buck)', 'Isolated Forward / Flyback',
    'Buck with a coupled secondary; needs primary + isolated outputs.',
    { isolation: 1500, minOutputs: 2,
      outputs: [{ name: 'pri', voltage: 12, power: 30 }, { name: 'sec', voltage: 12, power: 30 }] }),
  T('isolated_buck_boost', 'Isolated buck-boost', 'Isolated Forward / Flyback',
    'Inverting Fly-Buck-Boost; primary + isolated outputs.',
    { isolation: 1500, minOutputs: 2,
      outputs: [{ name: 'pri', voltage: 12, power: 30 }, { name: 'sec', voltage: -12, power: 30 }] }),
  T('forward', 'Forward', 'Isolated Forward / Flyback',
    'Single-switch forward with demagnetization winding.',
    { isolation: 1500 }),
  T('two_switch_forward', 'Two-switch forward', 'Isolated Forward / Flyback',
    'Clamp diodes recycle magnetizing energy to the bus.',
    { isolation: 1500 }),
  T('acf', 'Active-clamp forward', 'Isolated Forward / Flyback',
    'Active clamp resets the core; synchronous rectifier.',
    { isolation: 1500 }),

  // ── Isolated Bridge / Push-Pull ──────────────────────────────────────────
  T('push_pull', 'Push-pull', 'Isolated Bridge / Push-Pull',
    'Center-tapped primary, two switches; current-doubling output.',
    { isolation: 1500 }),
  T('weinberg', 'Weinberg', 'Isolated Bridge / Push-Pull',
    'Current-fed push-pull with input choke.',
    { isolation: 1500 }),
  T('ahb', 'Asymmetric half-bridge', 'Isolated Bridge / Push-Pull',
    'Complementary duty half-bridge, ZVS capable.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 240 }] }),
  T('psfb', 'Phase-shifted full bridge', 'Isolated Bridge / Push-Pull',
    'ZVS full bridge; phase shift regulates the output.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 600 }] }),
  T('pshb', 'Phase-shifted half bridge', 'Isolated Bridge / Push-Pull',
    '3-level NPC phase-shift half bridge.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 300 }] }),
  T('dab', 'Dual active bridge', 'Isolated Bridge / Push-Pull',
    'Bidirectional; Vout floats to the power balance.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 48, power: 1000 }] }),

  // ── Resonant ──────────────────────────────────────────────────────────
  T('llc', 'LLC', 'Resonant',
    'Series-resonant with magnetizing inductance; ZVS across load.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 240 }] }),
  T('src', 'Series resonant (SRC)', 'Resonant',
    'Series L-C tank; buck-only resonant.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 240 }] }),
  T('cllc', 'CLLC', 'Resonant',
    'Symmetric bidirectional resonant tank.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 48, power: 1000 }] }),
  T('clllc', 'CLLLC', 'Resonant',
    'CLLC plus discrete secondary resonant inductor.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 48, power: 1000 }] }),

  // ── Filters / PFC (single-phase) ─────────────────────────────────────────────
  T('pfc', 'Boost PFC (1-φ)', 'Filters / PFC',
    'Single-phase boost power-factor correction.',
    { inputType: 'acSinglePhase', vinNom: 230, lineFrequency: 50,
      outputs: [{ name: 'out', voltage: 400, power: 300 }] }),

  // ── Three-Phase PFC ──────────────────────────────────────────────────────────
  T('vienna', 'Vienna (3-φ)', 'Three-Phase PFC',
    'Three-phase three-level Vienna rectifier.',
    { inputType: 'acThreePhase', vinNom: 230, lineFrequency: 50,
      outputs: [{ name: 'out', voltage: 700, power: 3000 }] }),
]

// Not yet in the engine (docs/TOPOLOGY_ROADMAP.md) — shown greyed out.
export const PLANNED = [
  { id: 'inverting_buck_boost', name: 'Inverting buck-boost', family: 'Non-Isolated DC-DC', desc: 'Planned.' },
  { id: 'half_bridge', name: 'Hard-switched half bridge', family: 'Isolated Bridge / Push-Pull', desc: 'Planned.' },
  { id: 'full_bridge', name: 'Hard-switched full bridge', family: 'Isolated Bridge / Push-Pull', desc: 'Planned.' },
  { id: 'acf_flyback', name: 'Active-clamp flyback', family: 'Isolated Forward / Flyback', desc: 'Planned.' },
  { id: 'totem_pole_pfc', name: 'Totem-pole PFC', family: 'Filters / PFC', desc: 'Planned.' },
]

// ── Variants ─────────────────────────────────────────────────────────────────
// Design knobs the engine reads from the spec's `config` object (KirchhoffConfig.hpp →
// cfg::get_str(d.config, key, default)). Each topology with real alternatives lists its
// variant axis; everything else offers a single "Standard" build. `key` is the config
// field the engine keys off; `default` mirrors the C++ builder's own default.
const RECTIFIER_3 = [
  { id: 'fullBridge', name: 'Full-bridge', desc: 'One secondary winding into a 4-diode bridge.' },
  { id: 'centerTapped', name: 'Center-tapped', desc: 'Two half-windings, 2-diode full-wave — low-V / high-I.' },
  { id: 'currentDoubler', name: 'Current-doubler', desc: 'Two output inductors — halves the rectifier RMS.' },
]
const SYNC_RECT = [
  { id: 'diode', name: 'Diode', desc: 'Schottky rectifier — simplest and most robust.' },
  { id: 'synchronous', name: 'Synchronous', desc: 'MOSFET rectifier — higher efficiency, esp. at low Vout.' },
]
const FLYBACK_MODE = [
  { id: 'ccm', name: 'CCM', desc: 'Continuous: magnetizing current never resets — lowest RMS, hard turn-on.' },
  { id: 'dcm', name: 'DCM', desc: 'Discontinuous: current resets each cycle — smaller core, low turn-on loss.' },
  { id: 'bcm', name: 'Boundary (BCM)', desc: 'Critical inductance — sits on the CCM/DCM boundary.' },
  { id: 'qrm', name: 'Quasi-resonant', desc: 'Boundary + valley switching: after diode cutoff the drain rings on Lm·Cres and the switch turns on at the first Vds valley (config.resonantCapacitance, default 220 pF).' },
]

export const VARIANTS = {
  buck: { key: 'rectifier', label: 'Rectifier', default: 'diode', options: SYNC_RECT },
  boost: { key: 'rectifier', label: 'Rectifier', default: 'diode', options: SYNC_RECT },
  flyback: { key: 'mode', label: 'Conduction mode', default: 'ccm', options: FLYBACK_MODE },
  ahb: { key: 'rectifierType', label: 'Secondary rectifier', default: 'fullBridge', options: RECTIFIER_3 },
  psfb: { key: 'rectifierType', label: 'Secondary rectifier', default: 'fullBridge', options: RECTIFIER_3 },
  pshb: { key: 'rectifierType', label: 'Secondary rectifier', default: 'fullBridge', options: RECTIFIER_3 },
  src: { key: 'rectifierType', label: 'Secondary rectifier', default: 'centerTapped', options: RECTIFIER_3 },
  llc: { key: 'rectifierType', label: 'Secondary rectifier', default: 'centerTapped', options: RECTIFIER_3 },
}

// The single fallback offered when a topology has no real variant axis.
const STANDARD = { key: null, label: 'Build', default: 'standard',
  options: [{ id: 'standard', name: 'Standard', desc: 'The canonical build for this topology.' }] }

export function variantAxis(topologyId) {
  return VARIANTS[topologyId] ?? STANDARD
}
export function defaultVariant(topologyId) {
  return variantAxis(topologyId).default
}

export function topologyById(id) {
  return TOPOLOGIES.find((t) => t.id === id)
}

// ── Advanced per-topology knobs ────────────────────────────────────────────────
// The expert-override surface: every design parameter the engine derives internally but
// an experienced user (or someone reproducing an existing converter) may want to pin.
//
// Two serialization channels, both already honoured by the C++ engine — no rebuild:
//   • CFG knobs  → spec.config[key]        read via cfg::get*(d.config, key, default)  (KirchhoffConfig.hpp)
//   • PIN knobs  → designRequirements.<f>  read via provided_*()  (ComponentRequirements.hpp:62-118)
// `def` mirrors the C++ builder's own default (shown as the greyed "auto" placeholder); a knob
// is serialized ONLY when the user turns its override on (matching the variant policy in buildSpec).
//
// Variant-axis keys (rectifier / mode / rectifierType — see VARIANTS) are intentionally NOT
// repeated here: they stay the Stage-2 card so exactly one control ever writes each config key.
//
// Turns-ratio pinning writes designRequirements.turnsRatios[0]; it is offered only for topologies
// whose primary:secondary ratio IS index 0 (forward / push-pull / weinberg read index 1 with a
// reserved index 0, so their turns-ratio pin waits for the P2 multi-winding "pin this design" table).

export const KNOB_TIERS = [
  { id: 1, label: 'Operating point' },
  { id: 2, label: 'Magnetics & tank' },
  { id: 3, label: 'Refinements' },
]

// tier: 1 operating-point | 2 magnetics/tank | 3 refinement. type: number | enum | bool | (pin implies number).
const num = (key, label, def, o = {}) =>
  ({ key, label, tier: o.tier ?? 3, type: 'number', def, sym: o.sym, unit: o.unit,
     min: o.min, max: o.max, step: o.step, int: o.int, pin: o.pin, tip: o.tip })
const enm = (key, label, options, def, tier = 1, tip = '') =>
  ({ key, label, tier, type: 'enum', options, def, tip })
const boo = (key, label, def = false, tier = 3, tip = '') =>
  ({ key, label, tier, type: 'bool', def, tip })

// Shared knob factories (defaults mirror the C++ builders).
const kPinN = (tip = 'Pin the primary:secondary turns ratio (Np/Ns); the stage is sized around it.') =>
  num('pinN', 'Turns ratio', null, { tier: 2, sym: 'n', min: 0, step: 0.1, pin: 'turnsRatio', tip })
const kPinL = (label = 'Magnetizing inductance', sym = 'Lm',
  tip = 'Pin the magnetizing (or main) inductance; the stage is sized around it.') =>
  num('pinL', label, null, { tier: 2, sym, unit: 'H', min: 0, pin: 'magnetizingInductance', tip })
const kRipple = (def = 0.4) => num('rippleRatio', 'Inductor ripple ΔIL/IL', def,
  { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05, tip: 'Peak-to-peak inductor current ripple / average. 0.2–0.4 typical.' })
const kMaxIsw = () => num('maximumSwitchCurrent', 'Max switch current', null,
  { sym: 'Isw', unit: 'A', min: 0, tip: 'Size the inductor to cap peak switch current instead of the ripple rule.' })
const kMaxDuty = (def = 0.95) => num('maximumDutyCycle', 'Max duty cycle', def,
  { sym: 'Dmax', min: 0.05, max: 1, step: 0.01, tip: 'Upper clamp on the operating duty cycle.' })
const kDead = (def = 0.01) => num('deadTimeFraction', 'Dead-time fraction', def,
  { sym: 't_d·fs', min: 0, max: 0.2, step: 0.005, tip: 'Dead time as a fraction of the switching period.' })
const kOutRipple = (def = 0.01) => num('outputRippleFraction', 'Output ripple ΔVo/Vo', def,
  { sym: 'ΔVo', min: 0.001, max: 0.2, step: 0.005, tip: 'Output-voltage ripple ratio; sizes the output capacitor.' })
const kOutCap = (def = 100e-6) => num('outputCapacitance', 'Output capacitance', def,
  { tier: 2, sym: 'Cout', unit: 'F', min: 0, tip: 'Explicit output capacitor (else sized from the ripple target).' })
const kVderate = () => num('vDerate', 'Voltage derating', 0.8,
  { sym: 'kV', min: 0.3, max: 1, step: 0.05, tip: 'Global device voltage-derating factor (IPC-9592 ≈ 0.8).' })

const POWER_FLOW = [{ id: 'forward', name: 'Forward' }, { id: 'reverse', name: 'Reverse' }]

// Non-isolated coupled-inductor family (SEPIC / Ćuk / Zeta) shares this refinement block.
const coupledBlock = () => [
  boo('synchronousRectifier', 'Synchronous rectifier', false, 1, 'Replace the catch diode with a driven low-side FET.'),
  boo('coupledInductor', 'Coupled inductor', false, 2, 'Wind L1 and L2 on one core (ripple steering).'),
  num('couplingCoefficient', 'Coupling k', 0.999, { tier: 2, sym: 'k', min: 0, max: 1, step: 0.001, tip: 'Magnetic coupling between L1 and L2 (coupled-inductor only).' }),
  num('l1RippleRatio', 'L1 ripple', 0.4, { sym: 'ΔIL1', min: 0.01, max: 2, step: 0.05, tip: 'Input-inductor current ripple ratio.' }),
  num('l2RippleRatio', 'L2 ripple', 0.3, { sym: 'ΔIL2', min: 0.01, max: 2, step: 0.05, tip: 'Second-inductor current ripple ratio.' }),
  num('couplingCapRipple', 'Coupling-cap ripple', 0.05, { min: 0.001, max: 0.5, step: 0.01, tip: 'Allowed ΔV on the energy-transfer capacitor.' }),
  num('outputCapRipple', 'Output-cap ripple', 0.01, { min: 0.001, max: 0.2, step: 0.005, tip: 'Allowed output-voltage ripple ratio.' }),
  kMaxDuty(0.95), kDead(0.01), kMaxIsw(), kVderate(),
]

export const KNOBS = {
  // ── Non-Isolated DC-DC ──────────────────────────────────────────────────────
  buck: [kPinL('Inductance', 'L', 'Pin the inductor value; the stage is sized around it.'),
    kRipple(0.4), kMaxIsw(), kMaxDuty(1.0), kDead(0.01), kOutRipple(0.01), kVderate()],
  boost: [kPinL('Inductance', 'L', 'Pin the inductor value; the stage is sized around it.'),
    kRipple(0.4), kMaxIsw(), kMaxDuty(1.0), kDead(0.01), kOutRipple(0.01), kVderate()],
  sepic: [kPinL('Inductance L1', 'L1'), ...coupledBlock()],
  zeta: [kPinL('Inductance L1', 'L1'), ...coupledBlock()],
  cuk: [
    boo('isolated', 'Isolated (transformer)', false, 1, 'Split the coupling cap and insert an isolation transformer.'),
    kPinN('Pin the turns ratio Np:Ns (isolated Ćuk only).'),
    enm('powerFlowDirection', 'Power flow', POWER_FLOW, 'forward', 1, 'Forward or reverse (bidirectional) power transfer.'),
    kPinL('Inductance L1', 'L1'),
    num('magnetizingCurrentFraction', 'Magnetizing-current fraction', 0.1, { tier: 2, min: 0.01, max: 1, step: 0.01, tip: 'Sizes Lm of the isolation transformer (isolated only).' }),
    ...coupledBlock(),
  ],
  fsbb: [
    enm('transitionMode', 'Transition mode', [{ id: 'splitPwm', name: 'Split PWM' }, { id: 'simultaneous', name: 'Simultaneous' }], 'splitPwm', 1, 'Buck-boost-region modulation strategy.'),
    num('fsbbSplitRatio', 'Split ratio', 0.5, { tier: 1, sym: 'κ', min: 0, max: 1, step: 0.05, tip: 'Duty split between the buck and boost legs in the transition band.' }),
    enm('powerFlowDirection', 'Power flow', POWER_FLOW, 'forward', 1),
    num('phaseCount', 'Interleaved phases', 1, { tier: 1, sym: 'N', min: 1, max: 6, step: 1, int: true, tip: 'Number of interleaved phases.' }),
    kPinL('Inductance', 'L'), kRipple(0.4),
    num('fsbbTransitionBand', 'Transition band', 0.1, { min: 0, max: 0.5, step: 0.01, tip: 'Width of the buck-boost band as |1 − Vo/Vin|.' }),
    kMaxDuty(0.95), kDead(0.01), kOutCap(100e-6), kMaxIsw(), kVderate(),
  ],

  // ── Isolated Forward / Flyback ────────────────────────────────────────────────
  flyback: [kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('resonantCapacitance', 'QR valley cap', 220e-12, { tier: 1, sym: 'Cres', unit: 'F', min: 0, tip: 'Quasi-resonant valley-switching capacitance (QRM mode only).' }),
    kVderate()],
  isolated_buck: [kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('inductorRippleRatio', 'Inductor ripple', 0.4, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }), kMaxIsw(), kVderate()],
  isolated_buck_boost: [kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('inductorRippleRatio', 'Inductor ripple', 0.4, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }), kMaxIsw(), kVderate()],
  forward: [kPinL('Magnetizing inductance', 'Lm'),
    num('maxDutyCycle', 'Max duty cycle', 0.5, { tier: 1, sym: 'Dmax', min: 0.05, max: 0.5, step: 0.01, tip: '≤ 0.5 with a 1:1 reset winding.' }),
    num('inductorRippleRatio', 'Output-inductor ripple', 0.4, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }), kVderate()],
  two_switch_forward: [kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('maxDutyCycle', 'Max duty cycle', 0.5, { tier: 1, sym: 'Dmax', min: 0.05, max: 0.5, step: 0.01 }),
    num('inductorRippleRatio', 'Output-inductor ripple', 0.4, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }), kVderate()],
  acf: [kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('operatingDutyCycle', 'Duty cycle', 0.45, { tier: 1, sym: 'D', min: 0.05, max: 0.75, step: 0.01 }),
    num('inductorRippleRatio', 'Output-inductor ripple', 0.4, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }), kDead(0.01), kVderate()],

  // ── Isolated Bridge / Push-Pull ──────────────────────────────────────────────
  push_pull: [kPinL('Magnetizing inductance', 'Lm'),
    num('maxDutyCycle', 'Max duty cycle', 0.48, { tier: 1, sym: 'Dmax', min: 0.05, max: 0.49, step: 0.01, tip: '< 0.5 per switch.' }),
    num('inductorRippleRatio', 'Output-inductor ripple', 0.4, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }),
    kOutCap(100e-6),
    num('leakDampR', 'Leak damper R', 10, { sym: 'R', unit: 'Ω', min: 0 }),
    num('leakDampC', 'Leak damper C', 1e-9, { sym: 'C', unit: 'F', min: 0 }), kMaxIsw(), kVderate()],
  weinberg: [
    enm('variant', 'Primary variant', [{ id: 'classic', name: 'Classic' }, { id: 'bridge', name: 'Bridge' }], 'classic', 1),
    num('boostDutyTarget', 'Boost-duty target', 0.55, { tier: 1, sym: 'D', min: 0.05, max: 0.95, step: 0.01, tip: 'Sizes the transformer turns ratio.' }),
    boo('synchronousRectifier', 'Synchronous rectifier', false, 1),
    boo('srPhaseSwap', 'SR phase swap', false, 3),
    kPinL('Magnetizing inductance', 'Lm'),
    num('l1RippleRatio', 'Input-inductor ripple', 0.3, { sym: 'ΔIL1', min: 0.01, max: 2, step: 0.05 }),
    num('bridgeTurnsScale', 'Bridge turns scale', 0.5, { tier: 2, min: 0.1, max: 2, step: 0.05 }),
    num('outputCapRipple', 'Output-cap ripple', 0.01, { min: 0.001, max: 0.2, step: 0.005 }),
    kMaxDuty(0.95), kDead(0.02),
    num('transformerCoupling', 'Transformer coupling k', 0.999, { tier: 2, sym: 'k', min: 0, max: 1, step: 0.001 }), kVderate()],
  ahb: [
    num('operatingDutyCycle', 'Duty cycle', 0.3, { tier: 1, sym: 'D', min: 0.05, max: 0.5, step: 0.01 }),
    kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('magnetizingRippleRatio', 'Magnetizing ripple', 0.4, { tier: 2, min: 0.01, max: 2, step: 0.05, tip: 'Sizes Lm.' }),
    num('inductorRippleRatio', 'Output-inductor ripple', 0.3, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }),
    kDead(0.01),
    num('cdOutputFactor', 'Current-doubler factor', 0.5, { min: 0, max: 1, step: 0.05 }),
    kOutCap(100e-6),
    num('leakDampR', 'Leak damper R', 10, { sym: 'R', unit: 'Ω', min: 0 }),
    num('leakDampC', 'Leak damper C', 1e-9, { sym: 'C', unit: 'F', min: 0 }), kVderate()],
  psfb: [
    num('commandedDuty', 'Commanded duty', 0.7, { tier: 1, sym: 'D', min: 0.05, max: 1, step: 0.01, tip: 'Effective duty = phase shift / 180°.' }),
    kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('switchDutyFraction', 'Switch duty fraction', 0.48, { min: 0.05, max: 0.5, step: 0.01 }),
    num('inductorRippleRatio', 'Output-inductor ripple', 0.3, { sym: 'ΔIL', min: 0.01, max: 2, step: 0.05 }),
    num('cdOutputFactor', 'Current-doubler factor', 0.5, { min: 0, max: 1, step: 0.05 }), kVderate()],
  pshb: [
    num('commandedDuty', 'Commanded duty', 0.7, { tier: 1, sym: 'D', min: 0.05, max: 1, step: 0.01 }),
    kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('magnetizingCurrentFraction', 'Magnetizing-current fraction', 0.3, { tier: 2, min: 0.01, max: 1, step: 0.01, tip: 'Sizes Lm.' }),
    kDead(0.01), kOutCap(100e-6),
    num('cdOutputFactor', 'Current-doubler factor', 0.5, { min: 0, max: 1, step: 0.05 }),
    num('outerTrim', 'Outer trim', 0.01, { min: 0, max: 0.2, step: 0.005 }),
    num('nodeShuntCap', 'Node shunt cap', 1e-9, { sym: 'C', unit: 'F', min: 0 }), kVderate()],
  dab: [
    num('dabPhaseShiftDeg', 'Phase shift', 25, { tier: 1, sym: 'φ', unit: '°', min: -90, max: 90, step: 1, tip: 'Outer phase shift D3 (the SPS control variable).' }),
    enm('dabModulationType', 'Modulation', [{ id: 'SPS', name: 'SPS' }, { id: 'EPS', name: 'EPS' }, { id: 'DPS', name: 'DPS' }, { id: 'TPS', name: 'TPS' }], 'SPS', 1),
    num('dabInnerPhaseShift1Deg', 'Inner shift D1', 0, { tier: 1, sym: 'D1', unit: '°', min: 0, max: 90, step: 1, tip: 'EPS/DPS/TPS only.' }),
    num('dabInnerPhaseShift2Deg', 'Inner shift D2', 0, { tier: 1, sym: 'D2', unit: '°', min: 0, max: 90, step: 1, tip: 'DPS/TPS only.' }),
    kPinN(),
    num('pinLr', 'Series inductance', null, { tier: 2, sym: 'Lr', unit: 'H', min: 0, pin: 'seriesInductance', tip: 'Pin the series/leakage inductance (the power-transfer element).' }),
    boo('useLeakageInductance', 'Use leakage as series L', false, 2),
    kPinL('Magnetizing inductance', 'Lm'),
    num('switchDutyFraction', 'Switch duty fraction', 0.499, { min: 0.05, max: 0.5, step: 0.001 }),
    kOutCap(100e-6), kVderate()],

  // ── Resonant ──────────────────────────────────────────────────────────────────
  llc: [
    num('qualityFactor', 'Quality factor', 0.4, { tier: 1, sym: 'Q', min: 0.05, max: 2, step: 0.05, tip: '0.3–0.5 typical at full load (TI SLUP263).' }),
    num('inductanceRatio', 'Inductance ratio', 5, { tier: 1, sym: 'Ln=Lm/Lr', min: 1, max: 20, step: 0.5, tip: '3–7 typical.' }),
    num('resonantBandMin', 'Freq band min', 80e3, { tier: 1, sym: 'fmin', unit: 'Hz', min: 0, tip: 'fr = √(fmin·fmax).' }),
    num('resonantBandMax', 'Freq band max', 200e3, { tier: 1, sym: 'fmax', unit: 'Hz', min: 0 }),
    boo('driveAtSwitchingFrequency', 'Drive at fsw', false, 1),
    kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('pinLr', 'Resonant inductance', null, { tier: 2, sym: 'Lr', unit: 'H', min: 0, pin: 'resonantInductance', tip: 'Pin Lr (overrides the Q/Ln/fr derivation).' }),
    num('pinCr', 'Resonant capacitance', null, { tier: 2, sym: 'Cr', unit: 'F', min: 0, pin: 'resonantCapacitance', tip: 'Pin Cr (overrides the Q/Ln/fr derivation).' }),
    num('bridgeFactor', 'Bridge factor', 0.5, { tier: 2, min: 0, max: 1, step: 0.05, tip: '0.5 half-bridge, 1.0 full-bridge.' }),
    num('switchDutyFraction', 'Switch duty fraction', 0.45, { min: 0.05, max: 0.5, step: 0.01 }),
    num('cdOutputFactor', 'Current-doubler factor', 0.465, { min: 0, max: 1, step: 0.05 }),
    num('rippleRatio', 'Output ripple', 0.3, { sym: 'ΔVo', min: 0.001, max: 1, step: 0.02 }),
    num('transformerCoupling', 'Transformer coupling k', 0.999, { tier: 2, sym: 'k', min: 0, max: 1, step: 0.001 }),
    num('busSplitCap', 'Bus split cap', 10e-6, { tier: 2, unit: 'F', min: 0 }), kVderate()],
  src: [
    num('qualityFactor', 'Quality factor', 0.8, { tier: 1, sym: 'Q', min: 0.05, max: 3, step: 0.05 }),
    num('inductanceRatio', 'Inductance ratio', 10, { tier: 1, sym: 'Lm/Lr', min: 1, max: 40, step: 1 }),
    num('gainHeadroom', 'Gain headroom', 1.08, { tier: 1, min: 1, max: 1.5, step: 0.01 }),
    kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('bridgeFactor', 'Bridge factor', 0.5, { tier: 2, min: 0, max: 1, step: 0.05 }),
    num('switchDutyFraction', 'Switch duty fraction', 0.45, { min: 0.05, max: 0.5, step: 0.01 }),
    num('cdOutputFactor', 'Current-doubler factor', 0.465, { min: 0, max: 1, step: 0.05 }),
    num('rippleRatio', 'Output ripple', 0.3, { sym: 'ΔVo', min: 0.001, max: 1, step: 0.02 }),
    num('transformerCoupling', 'Transformer coupling k', 0.999, { tier: 2, sym: 'k', min: 0, max: 1, step: 0.001 }),
    num('busSplitCap', 'Bus split cap', 10e-6, { tier: 2, unit: 'F', min: 0 }), kVderate()],
  cllc: [
    num('qualityFactor', 'Quality factor', 0.3, { tier: 1, sym: 'Q', min: 0.05, max: 2, step: 0.05 }),
    num('inductanceRatio', 'Inductance ratio', 4.45, { tier: 1, sym: 'Lm/Lr1', min: 1, max: 20, step: 0.5 }),
    num('gainHeadroom', 'Gain headroom', 1.08, { tier: 1, min: 1, max: 1.5, step: 0.01 }),
    enm('powerFlowDirection', 'Power flow', POWER_FLOW, 'forward', 1),
    kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('switchDutyFraction', 'Switch duty fraction', 0.47, { min: 0.05, max: 0.5, step: 0.01 }), kVderate()],
  clllc: [
    num('qualityFactor', 'Quality factor', 0.4, { tier: 1, sym: 'Q', min: 0.05, max: 2, step: 0.05 }),
    num('inductanceRatio', 'Inductance ratio', 6.0, { tier: 1, sym: 'Lm/Lr1', min: 1, max: 20, step: 0.5 }),
    enm('powerFlowDirection', 'Power flow', POWER_FLOW, 'forward', 1),
    kPinN(), kPinL('Magnetizing inductance', 'Lm'),
    num('switchDutyFraction', 'Switch duty fraction', 0.47, { min: 0.05, max: 0.5, step: 0.01 }),
    num('senseResistance', 'Sense resistance', 0.01, { sym: 'Rs', unit: 'Ω', min: 0 }),
    num('senseHysteresis', 'Sense hysteresis', 5e-3, { min: 0 }), kVderate()],

  // ── Filters / PFC ───────────────────────────────────────────────────────────
  pfc: [
    enm('mode', 'Conduction mode', [{ id: 'ccm', name: 'CCM' }, { id: 'crm', name: 'CrM / BCM' }, { id: 'dcm', name: 'DCM' }, { id: 'transition', name: 'Transition' }], 'ccm', 1),
    enm('topologyVariant', 'Variant', [{ id: 'boost', name: 'Boost' }, { id: 'totemPole', name: 'Totem-pole' }, { id: 'interleaved', name: 'Interleaved' }, { id: 'sepic', name: 'SEPIC' }, { id: 'cuk', name: 'Ćuk' }], 'boost', 1),
    num('numberOfPhases', 'Interleaved phases', 2, { tier: 1, sym: 'N', min: 1, max: 6, step: 1, int: true }),
    num('outputCapacitance', 'Bulk capacitance', 220e-6, { tier: 2, sym: 'Cbulk', unit: 'F', min: 0 }),
    num('currentRippleFraction', 'Current ripple', 0.3, { sym: 'ΔIL', min: 0.01, max: 1, step: 0.02 }),
    num('senseResistance', 'Sense resistance', 0.1, { sym: 'Rs', unit: 'Ω', min: 0 }), kVderate()],

  // ── Three-Phase PFC ───────────────────────────────────────────────────────────
  vienna: [
    num('phaseCount', 'Interleaved channels', 1, { tier: 1, sym: 'N', min: 1, max: 6, step: 1, int: true }),
    enm('samplingStrategy', 'Sampling', [{ id: 'peakOfLineOnly', name: 'Peak of line' }, { id: 'fullLineCycle', name: 'Full line cycle' }], 'fullLineCycle', 1),
    num('busCapacitance', 'Bus capacitance', 470e-6, { tier: 2, sym: 'Cbus', unit: 'F', min: 0 }),
    num('senseResistance', 'Sense resistance', 0.1, { sym: 'Rs', unit: 'Ω', min: 0 }),
    num('balanceModulation', 'Balance modulation', 4.0, { min: 0 }),
    num('outputDividerGain', 'Output divider gain', 0.005, { min: 0 }), kVderate()],
}

export function knobsFor(topologyId) {
  return KNOBS[topologyId] ?? []
}

// The knobs of a topology grouped into the tiers, dropping empty tiers.
export function knobGroups(topologyId) {
  const ks = knobsFor(topologyId)
  return KNOB_TIERS
    .map((t) => ({ ...t, knobs: ks.filter((k) => k.tier === t.id) }))
    .filter((g) => g.knobs.length)
}

// Assemble the design-requirements spec the engine consumes.
// `form` mirrors the preset shape (numbers already parsed); `topologyId` selects the
// variant axis so the chosen variant lands in spec.config under the engine's key.
export function buildSpec(form, topologyId) {
  const dr = {
    efficiency: form.efficiency,
    inputType: form.inputType,
    inputVoltage: dimensional(form.vinMin, form.vinNom, form.vinMax),
    switchingFrequency: { nominal: form.fs },
    outputs: form.outputs.map((o) => ({
      name: o.name,
      voltage: { nominal: o.voltage },
      regulation: 'voltage',
    })),
  }
  if (form.isolation) dr.isolationVoltage = form.isolation
  if (form.inputType !== 'dc') dr.lineFrequency = { nominal: form.lineFrequency }

  const ops = form.ops?.length
    ? form.ops
    : [{ name: 'full_load', vin: form.vinNom, ambient: form.ambient, powers: form.outputs.map((o) => o.power) }]

  const spec = {
    designRequirements: dr,
    operatingPoints: ops.map((op) => ({
      name: op.name,
      inputVoltage: op.vin,
      ambientTemperature: op.ambient,
      outputs: form.outputs.map((o, i) => ({ name: o.name, power: op.powers[i] })),
    })),
  }

  const config = {}
  // Variant knob → the engine's config key (rectifier / rectifierType). The "standard"
  // sentinel and default selections are left off so the builder keeps its own default.
  const axis = variantAxis(topologyId)
  if (axis.key && form.variant && form.variant !== axis.default) config[axis.key] = form.variant

  // Advanced per-topology knobs → config[key] (CFG) or designRequirements.<field> (PIN).
  // Only knobs the user explicitly turned on are serialized (same policy as the variant),
  // so an untouched knob leaves the C++ builder's own default in force.
  for (const k of knobsFor(topologyId)) {
    const st = form.knobs?.[k.key]
    if (!st?.on) continue
    const v = st.value
    if (k.type === 'number' && (v === null || v === undefined || v === '' || Number.isNaN(v))) continue
    switch (k.pin) {
      case 'turnsRatio': dr.turnsRatios = [v]; break
      case 'magnetizingInductance': dr.magnetizingInductance = v; break
      case 'resonantInductance': dr.desiredResonantInductance = v; break
      case 'resonantCapacitance': dr.desiredResonantCapacitance = v; break
      case 'seriesInductance': dr.desiredSeriesInductance = v; break
      default: config[k.key] = v
    }
  }
  // Transient length in switching periods (settling + shown), DC topologies only:
  // AC-input converters (PFC/Vienna) need line-cycle-scale stop times, which their
  // builders manage themselves.
  if (form.inputType === 'dc' && form.fs > 0 && form.settlePeriods > 0 && form.showPeriods > 0) {
    config.tranStopTime = (form.settlePeriods + form.showPeriods) / form.fs
  }
  if (Object.keys(config).length) spec.config = config
  return spec
}

function dimensional(min, nom, max) {
  const d = { nominal: nom }
  if (min !== null && min !== undefined && min !== '') d.minimum = min
  if (max !== null && max !== undefined && max !== '') d.maximum = max
  return d
}
