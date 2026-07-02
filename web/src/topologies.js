// Topology catalog: every converter the Kirchhoff engine implements (the
// dispatch table in src/KirchhoffApi.cpp), grouped by family, each with a
// working preset so SOLVE produces a design out of the box.
//
// Preset fields drive the spec form; buildSpec() assembles the TAS
// design-requirements JSON the engine expects.

export const FAMILIES = [
  'Non-isolated DC-DC',
  'Flyback & isolated buck',
  'Forward family',
  'Bridge & phase-shift',
  'Resonant',
  'PFC (AC input)',
]

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
  // ── Non-isolated ──────────────────────────────────────────────────────
  T('buck', 'Buck', 'Non-isolated DC-DC',
    'Step-down switching cell: Q + D + L + Cout.'),
  T('boost', 'Boost', 'Non-isolated DC-DC',
    'Step-up cell; requires Vout > Vin/η.',
    { vinNom: 12, outputs: [{ name: 'out', voltage: 48, power: 60 }] }),
  T('sepic', 'SEPIC', 'Non-isolated DC-DC',
    'Step-up/down, non-inverting, coupled via Cs.'),
  T('cuk', 'Ćuk', 'Non-isolated DC-DC',
    'Step-up/down, inverting; continuous input & output current.'),
  T('zeta', 'Zeta', 'Non-isolated DC-DC',
    'Step-up/down, non-inverting; buck-like output.'),
  T('fsbb', '4-switch buck-boost', 'Non-isolated DC-DC',
    'Full bridge over one inductor; seamless up/down.',
    { vinNom: 24, outputs: [{ name: 'out', voltage: 20, power: 60 }] }),

  // ── Flyback & isolated buck ───────────────────────────────────────────
  T('flyback', 'Flyback', 'Flyback & isolated buck',
    'Isolated single-switch; energy stored in the transformer (CCM).',
    { vinMin: 36, vinNom: 48, vinMax: 60, isolation: 1500,
      outputs: [{ name: 'out', voltage: 12, power: 24 }] }),
  T('isolated_buck', 'Isolated buck (Fly-Buck)', 'Flyback & isolated buck',
    'Buck with a coupled secondary; needs primary + isolated outputs.',
    { isolation: 1500, minOutputs: 2,
      outputs: [{ name: 'pri', voltage: 12, power: 30 }, { name: 'sec', voltage: 12, power: 30 }] }),
  T('isolated_buck_boost', 'Isolated buck-boost', 'Flyback & isolated buck',
    'Inverting Fly-Buck-Boost; primary + isolated outputs.',
    { isolation: 1500, minOutputs: 2,
      outputs: [{ name: 'pri', voltage: 12, power: 30 }, { name: 'sec', voltage: -12, power: 30 }] }),

  // ── Forward family ────────────────────────────────────────────────────
  T('forward', 'Forward', 'Forward family',
    'Single-switch forward with demagnetization winding.',
    { isolation: 1500 }),
  T('two_switch_forward', 'Two-switch forward', 'Forward family',
    'Clamp diodes recycle magnetizing energy to the bus.',
    { isolation: 1500 }),
  T('acf', 'Active-clamp forward', 'Forward family',
    'Active clamp resets the core; synchronous rectifier.',
    { isolation: 1500 }),
  T('push_pull', 'Push-pull', 'Forward family',
    'Center-tapped primary, two switches; current-doubling output.',
    { isolation: 1500 }),
  T('weinberg', 'Weinberg', 'Forward family',
    'Current-fed push-pull with input choke.',
    { isolation: 1500 }),

  // ── Bridge & phase-shift ──────────────────────────────────────────────
  T('ahb', 'Asymmetric half-bridge', 'Bridge & phase-shift',
    'Complementary duty half-bridge, ZVS capable.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 240 }] }),
  T('psfb', 'Phase-shifted full bridge', 'Bridge & phase-shift',
    'ZVS full bridge; phase shift regulates the output.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 600 }] }),
  T('pshb', 'Phase-shifted half bridge', 'Bridge & phase-shift',
    '3-level NPC phase-shift half bridge.',
    { vinNom: 400, isolation: 3000, outputs: [{ name: 'out', voltage: 12, power: 300 }] }),
  T('dab', 'Dual active bridge', 'Bridge & phase-shift',
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

  // ── PFC ───────────────────────────────────────────────────────────────
  T('pfc', 'Boost PFC (1-φ)', 'PFC (AC input)',
    'Single-phase boost power-factor correction.',
    { inputType: 'acSinglePhase', vinNom: 230, lineFrequency: 50,
      outputs: [{ name: 'out', voltage: 400, power: 300 }] }),
  T('vienna', 'Vienna (3-φ)', 'PFC (AC input)',
    'Three-phase three-level Vienna rectifier.',
    { inputType: 'acThreePhase', vinNom: 230, lineFrequency: 50,
      outputs: [{ name: 'out', voltage: 700, power: 3000 }] }),
]

// Not yet in the engine (docs/TOPOLOGY_ROADMAP.md) — shown greyed out.
export const PLANNED = [
  { id: 'inverting_buck_boost', name: 'Inverting buck-boost', family: 'Non-isolated DC-DC', desc: 'Planned.' },
  { id: 'half_bridge', name: 'Hard-switched half bridge', family: 'Bridge & phase-shift', desc: 'Planned.' },
  { id: 'full_bridge', name: 'Hard-switched full bridge', family: 'Bridge & phase-shift', desc: 'Planned.' },
  { id: 'acf_flyback', name: 'Active-clamp flyback', family: 'Flyback & isolated buck', desc: 'Planned.' },
  { id: 'totem_pole_pfc', name: 'Totem-pole PFC', family: 'PFC (AC input)', desc: 'Planned.' },
]

export function topologyById(id) {
  return TOPOLOGIES.find((t) => t.id === id)
}

// Assemble the design-requirements spec the engine consumes.
// `form` mirrors the preset shape (numbers already parsed).
export function buildSpec(form) {
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

  // Transient length in switching periods (settling + shown), DC topologies only:
  // AC-input converters (PFC/Vienna) need line-cycle-scale stop times, which their
  // builders manage themselves.
  if (form.inputType === 'dc' && form.fs > 0 && form.settlePeriods > 0 && form.showPeriods > 0) {
    spec.config = { tranStopTime: (form.settlePeriods + form.showPeriods) / form.fs }
  }
  return spec
}

function dimensional(min, nom, max) {
  const d = { nominal: nom }
  if (min !== null && min !== undefined && min !== '') d.minimum = min
  if (max !== null && max !== undefined && max !== '') d.maximum = max
  return d
}
