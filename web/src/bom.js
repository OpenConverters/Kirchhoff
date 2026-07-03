// Extract a bill of materials from a TAS document: every component in every
// stage, with its kind, headline value and rating requirements. The TAS is a
// requirements BOM — parts carry the specs a real part must meet, which is
// exactly what a parts-database lookup (TAS DB, future) will consume.

import { si } from './units.js'

const KINDS = [
  ['semiconductor', (d) => (d.inputs?.designRequirements?.deviceType === 'diode' ? 'Diode' : 'MOSFET')],
  ['magnetic', (d) => ((d.inputs?.designRequirements?.turnsRatios?.length ?? 0) > 0 ? 'Transformer' : 'Inductor')],
  ['capacitor', () => 'Capacitor'],
  ['resistor', () => 'Resistor'],
  ['controller', () => 'Controller'],
]

export function extractBom(tas) {
  const rows = []
  for (const stage of tas?.topology?.stages ?? []) {
    for (const comp of stage.circuit?.components ?? []) {
      const d = comp.data ?? {}
      const req = d.inputs?.designRequirements ?? {}
      // FET body diodes (role:"bodyDiode") are intrinsic to their MOSFET, not a separate part — they
      // are drawn inside the switch symbol, so keep them out of the BOM (and thus the waveform list)
      // to match the schematic.
      if (req.role === 'bodyDiode') continue
      let kind = 'Component'
      for (const [key, name] of KINDS) {
        if (key in d) { kind = name(d); break }
      }
      rows.push({
        ref: comp.name,
        kind,
        stage: stage.name,
        role: stage.role ?? '',
        value: headlineValue(kind, req),
        rating: headlineRating(kind, req),
        requirements: req,
        windings: d.inputs?.operatingPoints?.[0]?.excitationsPerWinding ?? null,
      })
    }
  }
  return rows
}

function nomOf(x) {
  if (x === null || x === undefined) return null
  if (typeof x === 'number') return x
  return x.nominal ?? x.maximum ?? x.minimum ?? null
}

function headlineValue(kind, req) {
  switch (kind) {
    case 'Capacitor': return si(nomOf(req.capacitance), 'F')
    case 'Inductor': return si(nomOf(req.magnetizingInductance), 'H')
    case 'Transformer': {
      const lm = si(nomOf(req.magnetizingInductance), 'H')
      const n = [...new Set((req.turnsRatios ?? []).map((r) => trimNum(nomOf(r))))].join(' / ')
      return n ? `${lm} · n=${n}` : lm
    }
    case 'Resistor': return si(nomOf(req.resistance), 'Ω')
    case 'MOSFET': return req.maximumOnResistance ? `RDS(on) ≤ ${si(req.maximumOnResistance, 'Ω')}` : '—'
    case 'Diode': return req.maximumForwardVoltage ? `VF ≤ ${si(req.maximumForwardVoltage, 'V')}` : '—'
    case 'Controller': return req.category ?? '—'
    default: return '—'
  }
}

function headlineRating(kind, req) {
  switch (kind) {
    case 'Capacitor': return req.ratedVoltage ? si(req.ratedVoltage, 'V') : '—'
    case 'MOSFET': {
      const v = req.ratedDrainSourceVoltage ? si(req.ratedDrainSourceVoltage, 'V') : null
      const i = req.ratedContinuousDrainCurrent ? si(req.ratedContinuousDrainCurrent, 'A') : null
      return [v, i].filter(Boolean).join(' / ') || '—'
    }
    case 'Diode': {
      const v = req.ratedReverseVoltage ? si(req.ratedReverseVoltage, 'V') : null
      const i = req.ratedForwardCurrent ? si(req.ratedForwardCurrent, 'A') : null
      return [v, i].filter(Boolean).join(' / ') || '—'
    }
    case 'Resistor': return req.ratedPower ? si(req.ratedPower, 'W') : '—'
    default: return '—'
  }
}

function trimNum(x) {
  if (x === null || x === undefined) return '—'
  const s = x.toPrecision(4)
  return s.includes('.') ? s.replace(/\.?0+$/, '') : s
}

// Flatten a designRequirements object into printable key/value rows for the
// part drawer. Nested dimensional values are resolved to their spread.
export function requirementRows(req) {
  const rows = []
  for (const [key, val] of Object.entries(req ?? {})) {
    if (val === null || val === undefined) continue
    if (key === 'operatingPoints' || key === 'isolationSides') continue
    if (Array.isArray(val)) {
      if (key === 'turnsRatios' && val.length) {
        rows.push([label(key), val.map((r) => trimNum(nomOf(r))).join(' / ')])
      }
      continue
    }
    if (typeof val === 'object') {
      const parts = []
      if (val.minimum !== undefined) parts.push(`min ${fmtByKey(key, val.minimum)}`)
      if (val.nominal !== undefined) parts.push(`nom ${fmtByKey(key, val.nominal)}`)
      if (val.maximum !== undefined) parts.push(`max ${fmtByKey(key, val.maximum)}`)
      if (parts.length) rows.push([label(key), parts.join(' · ')])
      continue
    }
    if (typeof val === 'number') { rows.push([label(key), fmtByKey(key, val)]); continue }
    rows.push([label(key), String(val)])
  }
  return rows
}

const UNIT_BY_HINT = [
  [/voltage|vds|vgs/i, 'V'],
  [/current/i, 'A'],
  [/resistance|esr/i, 'Ω'],
  [/capacitance/i, 'F'],
  [/inductance/i, 'H'],
  [/frequency/i, 'Hz'],
  [/power/i, 'W'],
  [/time/i, 's'],
  [/temperature/i, '°C'],
]

function fmtByKey(key, num) {
  for (const [re, unit] of UNIT_BY_HINT) {
    if (re.test(key)) return unit === '°C' ? `${num} °C` : si(num, unit)
  }
  return si(num, '')
}

function label(key) {
  return key.replace(/([A-Z])/g, ' $1').replace(/^./, (c) => c.toUpperCase())
}
