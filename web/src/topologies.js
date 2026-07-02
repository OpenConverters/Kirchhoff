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

export const VARIANTS = {
  buck: { key: 'rectifier', label: 'Rectifier', default: 'diode', options: SYNC_RECT },
  boost: { key: 'rectifier', label: 'Rectifier', default: 'diode', options: SYNC_RECT },
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
