// Waveform export — the design's signals as CSV, and as PEAS/MAS operating points.
//
// One rule runs through this file: a CSV column header is the JSON path of the SAME signal in
// the companion .json export, written in the PEAS/MAS vocabulary (the signalDescriptor keys
// `current` / `voltage` / `magneticFluxDensity`, the container keys `excitationsPerWinding` /
// `excitation` / `excitationsPerPort`). So `T1.excitationsPerWinding[0].current [A]` in the CSV
// and `magnetics[0].operatingPoint.excitationsPerWinding[0].current` in the JSON are the same
// signal, and neither file invents a private naming scheme.
//
// Schemas the JSON export's members validate against (each member, not the wrapper — there is no
// PEAS/MAS schema for "every waveform in a converter", so the wrapper claims to be none):
//   magnetics[].operatingPoint  → https://psma.com/mas/inputs/operatingPoint.json
//   components[].operatingPoint → https://psma.com/peas/inputs/twoTerminalOperatingPoint.json
//                              or https://psma.com/peas/inputs/multiPortOperatingPoint.json
import { synthesizeWaveform } from './synth.js'

// The PEAS/MAS signalDescriptor keys an excitation can carry, and their SI units.
export const QUANTITIES = {
  current: 'A',
  voltage: 'V',
  magneticFluxDensity: 'T',
  magneticFieldStrength: 'A/m',
  magnetizingCurrent: 'A',
}

export const SCHEMAS = {
  magnetics: 'https://psma.com/mas/inputs/operatingPoint.json',
  twoTerminalComponents: 'https://psma.com/peas/inputs/twoTerminalOperatingPoint.json',
  multiPortComponents: 'https://psma.com/peas/inputs/multiPortOperatingPoint.json',
}

// Component kinds the engine emits (ComponentWaveforms.cpp kind_of): everything with two terminals
// is a PEAS twoTerminalOperatingPoint; a MOSFET's headline pair (V_DS, I_D) is one port of a
// multiPortOperatingPoint. An unknown kind THROWS rather than being filed under a guess.
const TWO_TERMINAL = new Set(['capacitor', 'resistor', 'diode', 'inductor'])
const PORT_OF_KIND = { mosfet: 'drain' }

// ── waveform assembly ───────────────────────────────────────────────────────────────────────
// All sources carry exactly ONE steady-state switching cycle; the multi-period view repeats it
// (that is what periodic steady state means — no data invented). WavePane draws with this same
// function, so an export of N periods is sample-for-sample what the chart shows.
export function tile(data, time, n) {
  if (!(n > 1)) return { data, time }
  const T = time[time.length - 1] - time[0]
  const d = [], t = []
  for (let k = 0; k < n; ++k) {
    for (let i = 0; i < data.length; ++i) {
      d.push(data[i])
      t.push(time[i] + k * T)
    }
  }
  return { data: d, time: t }
}

// One signalDescriptor → the samples to export, with how they were obtained. Mirrors WavePane's
// signalTraces: real samples win; otherwise the closed-form reconstruction from `processed`.
export function traceOf(signal, frequency, periods = 1) {
  if (!signal) return null
  const wf = signal.waveform
  if (wf?.data?.length > 1 && wf.time?.length === wf.data.length) {
    const { data, time } = tile(wf.data, wf.time, periods)
    return { data, time, provenance: 'sampled' }
  }
  const syn = synthesizeWaveform(signal.processed, frequency)
  if (!syn) return null
  const { data, time } = tile(syn.data, syn.time, periods)
  return { data, time, provenance: 'synthesized' }
}

// The excitation-source hierarchy for one magnetic: simulated (ngspice) > full analytical capture >
// the stripped TAS excitations. The ngspice extraction rebuilds winding CURRENTS only, so the
// analytical voltage is spliced back in per winding and labelled — current measured, voltage
// predicted, never silently mixed. App.vue's waveSource and every export call THIS, so the pane and
// the file can never disagree about which source they are showing.
export function resolveExcitations(name, { analyticalWaveforms, ngspiceOps, operatingPoint } = {}) {
  if (!name) return { excitations: [], kind: 'none', voltageKind: null }
  const analyticalFull = analyticalWaveforms?.[name]?.excitationsPerWinding
  const sim = ngspiceOps?.[name]
  if (sim) {
    const excitations = (sim.excitationsPerWinding ?? []).map((e, i) => {
      const av = analyticalFull?.[i]?.voltage
      const hasSimV = e.voltage?.waveform?.data?.length > 1
      if (!hasSimV && av?.waveform?.data?.length > 1) return { ...e, voltage: av }
      return e
    })
    return { excitations, kind: 'ngspice', voltageKind: analyticalFull ? 'analytical' : null }
  }
  if (analyticalFull?.length)
    return { excitations: analyticalFull, kind: 'analytical (full waveforms)', voltageKind: null }
  return {
    excitations: operatingPoint?.excitationsPerWinding ?? [],
    kind: 'analytical (processed)',
    voltageKind: null,
  }
}

// ── signal collection ───────────────────────────────────────────────────────────────────────
// A signal record: { path, unit, provenance, data, time } — `path` doubles as the CSV column and
// the JSON pointer. `origin` says which pane produced it (for the header comment only).

function excitationSignals(excitations, pathOf, periods, origin) {
  const out = []
  excitations.forEach((exc, i) => {
    for (const [quantity, unit] of Object.entries(QUANTITIES)) {
      const t = traceOf(exc?.[quantity], exc?.frequency, periods)
      if (!t) continue
      out.push({ path: pathOf(i, quantity), unit, origin, ...t })
    }
  })
  return out
}

// Every winding of every magnetic.
export function magneticSignals({ magnetics = [], analyticalWaveforms, ngspiceOps, opIdx = 0, periods = 1 } = {}) {
  const out = []
  for (const m of magnetics) {
    const { excitations, kind } = resolveExcitations(m.name, {
      analyticalWaveforms, ngspiceOps,
      operatingPoint: m.inputs?.operatingPoints?.[opIdx],
    })
    out.push(...excitationSignals(excitations,
      (i, q) => `${m.name}.excitationsPerWinding[${i}].${q}`, periods, kind))
  }
  return out
}

// Every non-magnetic component an ngspice component-waveform run produced.
export function componentSignals({ componentWaves, periods = 1 } = {}) {
  const comps = componentWaves?.components ?? []
  const frequency = componentWaves?.referencePeriod ? 1 / componentWaves.referencePeriod : 0
  const out = []
  for (const c of comps) {
    const port = portOf(c)
    const pathOf = (_i, q) => (port === null
      ? `${c.ref}.excitation.${q}`
      : `${c.ref}.excitationsPerPort[${port}].${q}`)
    out.push(...excitationSignals([{ ...c, frequency }], pathOf, periods, 'ngspice'))
  }
  return out
}

// null → two-terminal (single `excitation`); a string → the port label of a multi-port component.
function portOf(component) {
  if (TWO_TERMINAL.has(component.kind)) return null
  const port = PORT_OF_KIND[component.kind]
  if (!port) {
    throw new Error(
      `waveform export: component ${component.ref} has kind '${component.kind}', which is neither a ` +
      'known two-terminal kind nor a mapped multi-port kind — add it to TWO_TERMINAL or PORT_OF_KIND ' +
      'in waveExport.js rather than filing it under a guess')
  }
  return port
}

export function designSignals(ctx) {
  return [...magneticSignals(ctx), ...componentSignals(ctx)]
}

// ── CSV ─────────────────────────────────────────────────────────────────────────────────────
// Wide layout: one `time [s]` column, one column per signal, and a `provenance` row under the
// header saying whether each column is `sampled` or `synthesized`.
//
// A wide CSV is only honest while every column shares one time base, and two kinds of signal
// disagree about the grid for two different reasons:
//
//   • a SAMPLED signal on another grid (a PFC line-cycle window beside a switching-cycle one) is
//     measured data. Resampling it would fabricate values between its samples, so it is NOT
//     resampled and NOT silently dropped — it is omitted and named in the header, and it is still
//     present in full in the JSON export.
//   • a SYNTHESIZED signal has no grid of its own to defend: it is a piecewise-linear closed form
//     (a vertex list), so evaluating it at this file's times is EXACT, not interpolation of
//     measurements. Those columns are evaluated onto the reference grid and still say
//     `synthesized` in the provenance row.
const TIME_TOL = 1e-9  // relative to the window length

// Value of a piecewise-linear vertex list at time t, extended periodically over its own span.
// Right-continuous at a discontinuity (duplicate times): at a vertical edge you get the value
// AFTER the edge, which is what the same instant means for a switching waveform.
export function evaluatePeriodic(data, time, t) {
  const n = time.length
  if (n === 0) return null
  if (n === 1) return data[0]
  const t0 = time[0], T = time[n - 1] - t0
  if (!(T > 0)) return data[n - 1]
  const u = t0 + (((t - t0) % T) + T) % T
  let i = 0
  while (i < n - 2 && time[i + 1] <= u) i++
  const a = time[i], b = time[i + 1]
  if (b === a) return data[i + 1]
  return data[i] + ((u - a) / (b - a)) * (data[i + 1] - data[i])
}

// Put every signal on one time grid: the first sampled signal's (measured data sets the grid), or
// the first signal's when nothing is sampled.
export function alignToGrid(signals) {
  if (!signals.length) return { kept: [], omitted: [], grid: [] }
  const ref = signals.find((s) => s.provenance === 'sampled') ?? signals[0]
  const span = Math.abs(ref.time[ref.time.length - 1] - ref.time[0]) || 1
  const sameGrid = (s) => s.time.length === ref.time.length &&
    s.time.every((t, i) => Math.abs(t - ref.time[i]) <= TIME_TOL * span)

  const kept = [], omitted = []
  for (const s of signals) {
    if (s === ref || sameGrid(s)) { kept.push(s); continue }
    if (s.provenance === 'synthesized') {
      kept.push({ ...s, data: ref.time.map((t) => evaluatePeriodic(s.data, s.time, t)), time: ref.time })
      continue
    }
    omitted.push(s)
  }
  return { kept, omitted, grid: ref.time }
}

// Full round-trip precision, no locale, and quoting for the one character that can break a cell.
function cell(v) {
  if (v === null || v === undefined) return ''
  const s = String(v)
  return /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s
}

export function toCsv(signals, meta = {}) {
  const { kept, omitted, grid } = alignToGrid(signals)
  const lines = []
  const comment = (k, v) => { if (v !== null && v !== undefined && v !== '') lines.push(`# ${k}: ${v}`) }

  lines.push('# Kirchhoff waveform export — kirchhoff.openconverters.com')
  comment('topology', meta.topology)
  comment('scope', meta.scope)
  comment('operatingPoint', meta.operatingPoint)
  comment('source', [...new Set(kept.map((s) => s.origin))].join(', '))
  comment('periods', meta.periods > 1
    ? `${meta.periods} (the steady-state cycle repeated; no data invented)` : meta.periods)
  comment('exported', meta.exported)
  lines.push('# column headers are the signal\'s path in the companion .json export ' +
             '(PEAS/MAS operatingPointExcitation.json signalDescriptor keys)')
  lines.push('# the row under the header says whether a column is `sampled` or `synthesized` ' +
             '(the closed form of the processed descriptor, evaluated on this file\'s time grid — ' +
             'exact for a piecewise-linear reconstruction, but not simulated)')
  if (omitted.length) {
    lines.push('# omitted — sampled on a different time base and not resampled; ' +
               'present in full in the .json export: ' + omitted.map((s) => s.path).join(', '))
  }

  if (!kept.length) {
    lines.push('# no exportable signal')
    return lines.join('\n') + '\n'
  }

  lines.push(['time [s]', ...kept.map((s) => `${s.path} [${s.unit}]`)].map(cell).join(','))
  lines.push(['provenance', ...kept.map((s) => s.provenance)].map(cell).join(','))
  for (let i = 0; i < grid.length; ++i) {
    lines.push([grid[i], ...kept.map((s) => s.data[i])].map(cell).join(','))
  }
  return lines.join('\n') + '\n'
}

// ── JSON (PEAS/MAS operating points) ────────────────────────────────────────────────────────
// The engine serialises unset optionals as explicit `null` (nlohmann/quicktype), and a null is a
// PRESENT property to a schema: `{"ancillaryLabel": null, "numberPeriods": null, data, time}`
// matches neither branch of the waveform `oneOf`, and every optional null fails its declared type.
// Dropping them is what makes the exported operating points validate.
export function stripNulls(value) {
  if (Array.isArray(value)) return value.map(stripNulls)
  if (value && typeof value === 'object') {
    const out = {}
    for (const [k, v] of Object.entries(value)) if (v !== null && v !== undefined) out[k] = stripNulls(v)
    return out
  }
  return value
}

// The design's operating conditions, taken from the MAS operating point that carries them. Required
// by every PEAS/MAS operating point — so when the design has none, this throws instead of inventing
// an ambient temperature.
export function conditionsOf({ magnetics = [], opIdx = 0 } = {}) {
  for (const m of magnetics) {
    const c = m.inputs?.operatingPoints?.[opIdx]?.conditions
    if (c && c.ambientTemperature !== null && c.ambientTemperature !== undefined) {
      return { conditions: stripNulls(c), from: m.name }
    }
  }
  throw new Error(
    'waveform export: no operating conditions in the design — a PEAS/MAS operating point requires ' +
    'conditions.ambientTemperature, and this export will not invent one')
}

export function designExcitationsJson(ctx) {
  const { magnetics = [], componentWaves, analyticalWaveforms, ngspiceOps, opIdx = 0 } = ctx
  const { conditions, from } = conditionsOf(ctx)
  const sources = {}

  const mag = magnetics.map((m) => {
    const { excitations, kind, voltageKind } = resolveExcitations(m.name, {
      analyticalWaveforms, ngspiceOps, operatingPoint: m.inputs?.operatingPoints?.[opIdx],
    })
    sources[m.name] = voltageKind ? `${kind} (voltage: ${voltageKind})` : kind
    const opName = m.inputs?.operatingPoints?.[opIdx]?.name
    return {
      name: m.name,
      isMain: !!m.isMain,
      operatingPoint: stripNulls({
        ...(opName ? { name: opName } : {}),
        conditions,
        excitationsPerWinding: excitations,
      }),
    }
  })

  const frequency = componentWaves?.referencePeriod ? 1 / componentWaves.referencePeriod : null
  const comps = (componentWaves?.components ?? []).map((c) => {
    const port = portOf(c)
    const excitation = stripNulls({
      name: c.ref,
      ...(frequency ? { frequency } : {}),
      ...(c.current ? { current: c.current } : {}),
      ...(c.voltage ? { voltage: c.voltage } : {}),
    })
    sources[c.ref] = 'ngspice'
    return {
      ref: c.ref,
      kind: c.kind,
      ...(c.stage ? { stage: c.stage } : {}),
      operatingPoint: port === null
        ? { conditions, excitation }
        : { conditions, excitationsPerPort: [{ port, excitation }] },
    }
  })

  return {
    kirchhoff: {
      topology: ctx.topology ?? null,
      exported: ctx.exported ?? new Date().toISOString(),
      conditionsFrom: `MAS operating point of ${from}`,
      waveformSource: sources,
      note: 'Each member of magnetics[]/components[] validates against the schema named in ' +
            '`schemas`; the wrapper object itself is not a schema type.',
    },
    schemas: SCHEMAS,
    magnetics: mag,
    components: comps,
  }
}
