// The export library, verified before any file it writes is trusted. Run: npm run test:unit
import { test } from 'node:test'
import assert from 'node:assert/strict'
import * as x from '../../src/waveExport.js'

const grid = (n, T) => Array.from({ length: n }, (_, i) => (i * T) / n)

// A sampled signalDescriptor with the engine's null-riddled shape (unset optionals serialise as
// explicit nulls — the export has to survive that, and strip it out of the JSON).
function sampled(values, T = 1e-5) {
  return {
    waveform: { data: values, time: grid(values.length, T), ancillaryLabel: null, numberPeriods: null },
    processed: { label: 'custom', peak: Math.max(...values), rms: null },
  }
}

const magnetic = (name, windings) => ({
  name,
  isMain: true,
  inputs: { operatingPoints: [{ name: 'full_load', conditions: { ambientTemperature: 25, cooling: null } }] },
  __w: windings,
})

function ctxOf(windings, extra = {}) {
  const m = magnetic('T1', windings)
  return {
    topology: 'flyback',
    magnetics: [m],
    analyticalWaveforms: { T1: { excitationsPerWinding: windings } },
    opIdx: 0,
    periods: 1,
    ...extra,
  }
}

test('tile repeats the cycle and advances time, and is identity for one period', () => {
  const { data, time } = x.tile([1, 2], [0, 1], 3)
  assert.deepEqual(data, [1, 2, 1, 2, 1, 2])
  assert.deepEqual(time, [0, 1, 1, 2, 2, 3])
  const one = x.tile([1, 2], [0, 1], 1)
  assert.deepEqual(one.data, [1, 2])
})

test('traceOf prefers real samples and marks them sampled', () => {
  const t = x.traceOf(sampled([0, 1, 2, 3]), 1e5, 1)
  assert.equal(t.provenance, 'sampled')
  assert.deepEqual(t.data, [0, 1, 2, 3])
})

test('traceOf falls back to the processed reconstruction and marks it synthesized', () => {
  const sig = { processed: { label: 'triangular', peakToPeak: 2, offset: 0, dutyCycle: 0.5 } }
  const t = x.traceOf(sig, 1e5, 1)
  assert.equal(t.provenance, 'synthesized')
  assert.ok(t.data.length > 1)
})

test('traceOf returns null when there is neither samples nor a closed form', () => {
  assert.equal(x.traceOf({ processed: { label: 'custom' } }, 1e5, 1), null)
  assert.equal(x.traceOf(null, 1e5, 1), null)
})

test('column paths are the PEAS/MAS path of the same signal', () => {
  const w = [{ frequency: 1e5, current: sampled([0, 1]), voltage: sampled([2, 3]) }]
  const sigs = x.magneticSignals(ctxOf(w))
  assert.deepEqual(sigs.map((s) => s.path), [
    'T1.excitationsPerWinding[0].current',
    'T1.excitationsPerWinding[0].voltage',
  ])
  assert.deepEqual(sigs.map((s) => s.unit), ['A', 'V'])
})

test('a two-terminal component is `excitation`, a MOSFET a named port', () => {
  const componentWaves = {
    referencePeriod: 1e-5,
    components: [
      { ref: 'Cout', kind: 'capacitor', current: sampled([0, 1]), voltage: sampled([2, 3]) },
      { ref: 'Q1', kind: 'mosfet', current: sampled([0, 1]), voltage: sampled([2, 3]) },
    ],
  }
  const paths = x.componentSignals({ componentWaves, periods: 1 }).map((s) => s.path)
  assert.deepEqual(paths, [
    'Cout.excitation.current', 'Cout.excitation.voltage',
    'Q1.excitationsPerPort[drain].current', 'Q1.excitationsPerPort[drain].voltage',
  ])
})

test('an unmapped component kind throws instead of being filed under a guess', () => {
  const componentWaves = { referencePeriod: 1e-5, components: [{ ref: 'U1', kind: 'igbt', voltage: sampled([1, 2]) }] }
  assert.throws(() => x.componentSignals({ componentWaves }), /kind 'igbt'/)
})

test('CSV is wide: a time column, a column per signal, and a provenance row', () => {
  const w = [{
    frequency: 1e5,
    current: sampled([0, 1, 2, 3]),
    voltage: { processed: { label: 'rectangular', peakToPeak: 2, offset: 0, dutyCycle: 0.5 } },
  }]
  const csv = x.toCsv(x.magneticSignals(ctxOf(w)), { topology: 'flyback', periods: 1 })
  const rows = csv.split('\n').filter((l) => l && !l.startsWith('#'))
  assert.equal(rows[0], 'time [s],T1.excitationsPerWinding[0].current [A],T1.excitationsPerWinding[0].voltage [V]')
  assert.equal(rows[1].split(',')[0], 'provenance')
  assert.deepEqual(rows[1].split(',').slice(1), ['sampled', 'synthesized'])
  // one data row per sample of the shared grid, and the first column is that grid
  assert.equal(rows.length - 2, 4)
  assert.equal(rows[2].split(',')[0], '0')
})

test('a signal on a different time base is omitted and NAMED, never resampled', () => {
  const w = [
    { frequency: 1e5, current: sampled([0, 1, 2, 3]) },
    { frequency: 1e5, current: sampled([0, 1, 2, 3, 4, 5], 2e-3) },  // line-cycle window
  ]
  const csv = x.toCsv(x.magneticSignals(ctxOf(w)), {})
  assert.match(csv, /# omitted — sampled on a different time base[^\n]*T1\.excitationsPerWinding\[1\]\.current/)
  const header = csv.split('\n').find((l) => l.startsWith('time [s]'))
  assert.ok(!header.includes('excitationsPerWinding[1]'))
})

test('CSV values round-trip exactly (no locale, no rounding)', () => {
  const w = [{ frequency: 1e5, current: sampled([1 / 3, 9.921875e-6]) }]
  const csv = x.toCsv(x.magneticSignals(ctxOf(w)), {})
  const first = csv.split('\n').filter((l) => l && !l.startsWith('#'))[2].split(',')
  assert.equal(Number(first[1]), 1 / 3)
})

test('stripNulls removes the nulls that make an engine waveform schema-invalid', () => {
  const cleaned = x.stripNulls(sampled([0, 1]).waveform)
  assert.deepEqual(Object.keys(cleaned).sort(), ['data', 'time'])
})

test('JSON export builds one MAS operating point per magnetic, nulls stripped', () => {
  const w = [{ frequency: 1e5, current: sampled([0, 1]), voltage: sampled([2, 3]) }]
  const j = x.designExcitationsJson(ctxOf(w))
  assert.equal(j.magnetics[0].name, 'T1')
  const op = j.magnetics[0].operatingPoint
  assert.equal(op.conditions.ambientTemperature, 25)
  assert.ok(!('cooling' in op.conditions))
  assert.equal(op.excitationsPerWinding.length, 1)
  assert.equal(j.schemas.magnetics, 'https://psma.com/mas/inputs/operatingPoint.json')
})

test('JSON export files components by terminal count', () => {
  const w = [{ frequency: 1e5, current: sampled([0, 1]), voltage: sampled([2, 3]) }]
  const componentWaves = {
    referencePeriod: 1e-5,
    components: [
      { ref: 'Cout', kind: 'capacitor', current: sampled([0, 1]), voltage: sampled([2, 3]) },
      { ref: 'Q1', kind: 'mosfet', current: sampled([0, 1]), voltage: sampled([2, 3]) },
    ],
  }
  const j = x.designExcitationsJson(ctxOf(w, { componentWaves }))
  assert.ok('excitation' in j.components[0].operatingPoint)
  assert.equal(j.components[1].operatingPoint.excitationsPerPort[0].port, 'drain')
})

test('an export with no operating conditions throws rather than inventing an ambient', () => {
  const w = [{ frequency: 1e5, current: sampled([0, 1]) }]
  const ctx = ctxOf(w)
  ctx.magnetics = [{ name: 'T1', inputs: { operatingPoints: [{ conditions: { ambientTemperature: null } }] } }]
  assert.throws(() => x.designExcitationsJson(ctx), /will not invent one/)
})

test('resolveExcitations splices the analytical voltage into an ngspice current-only extraction', () => {
  const analytical = { T1: { excitationsPerWinding: [{ frequency: 1e5, current: sampled([9, 9]), voltage: sampled([5, 6]) }] } }
  const ngspiceOps = { T1: { excitationsPerWinding: [{ frequency: 1e5, current: sampled([1, 2]), voltage: {} }] } }
  const r = x.resolveExcitations('T1', { analyticalWaveforms: analytical, ngspiceOps })
  assert.equal(r.kind, 'ngspice')
  assert.equal(r.voltageKind, 'analytical')
  assert.deepEqual(r.excitations[0].current.waveform.data, [1, 2])
  assert.deepEqual(r.excitations[0].voltage.waveform.data, [5, 6])
})
