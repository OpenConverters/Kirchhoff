// Meta-tests: the measurement library is verified against synthetic waveforms with
// closed-form answers BEFORE any physics assertion rests on it. Run: npm run test:unit
import { test } from 'node:test'
import assert from 'node:assert/strict'
import * as m from './measure.js'

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b} (tol ${tol})`)

// One period of a sine, amplitude A, offset O, given number of vertices.
function sine(A, O = 0, pts = 256, phase = 0) {
  const data = [], time = []
  for (let i = 0; i < pts; ++i) {
    const f = i / (pts - 1)
    time.push(f)
    data.push(O + A * Math.sin(2 * Math.PI * f + phase))
  }
  return { data, time }
}

test('sine RMS = A/sqrt(2), mean = offset, peak = A+|O|', () => {
  const s = sine(2, 0)
  near(m.rms(s), 2 / Math.SQRT2, 1e-3, 'rms')
  near(m.mean(s), 0, 1e-3, 'mean')
  near(m.max(s), 2, 1e-3, 'max')
  near(m.pkpk(s), 4, 2e-3, 'pkpk')
})

test('sine with offset: mean tracks offset, rms = sqrt(O^2 + A^2/2)', () => {
  const s = sine(1, 3)
  near(m.mean(s), 3, 2e-3, 'mean')
  near(m.rms(s), Math.sqrt(9 + 0.5), 2e-3, 'rms')
})

test('rectangular duty cycle', () => {
  // 30% high (value 1) then 70% low (value 0), sharp edges as coincident-time vertices
  const D = 0.3
  const sig = { data: [0, 1, 1, 0, 0], time: [0, 0, D, D, 1] }
  near(m.dutyCycle(sig), D, 0.01, 'duty')
  near(m.mean(sig), D, 0.01, 'mean of unit pulse = duty')
})

test('sawtooth 0..1 average = 0.5, rms = 1/sqrt(3)', () => {
  const saw = { data: [0, 1], time: [0, 1] }
  near(m.mean(saw), 0.5, 1e-3, 'mean')
  near(m.rms(saw), 1 / Math.sqrt(3), 2e-3, 'rms')
})

test('triangle inductor-current ripple ratio', () => {
  // symmetric triangle around offset 10, pk-pk 2 → ripple ratio 0.2
  const tri = { data: [9, 11, 9], time: [0, 0.5, 1] }
  near(m.rippleRatio(tri), 0.2, 1e-3, 'ripple')
})

test('phaseBetween: a 45-degree delay of b reads as -45 (signed φ_b - φ_a)', () => {
  const a = sine(1, 0, 512, 0)
  const b = sine(1, 0, 512, -Math.PI / 4) // b delayed by 45° → negative phase
  near(m.phaseBetween(a, b), -45, 1.5, 'phase')
  near(m.phaseBetween(b, a), 45, 1.5, 'phase is antisymmetric')
})

test('fundamental amplitude of a pure sine', () => {
  near(m.fundamental(sine(2.5, 0, 512)).amplitude, 2.5, 5e-3, 'A1')
})

test('nrmse: identical = 0, scaled differs', () => {
  const a = sine(1, 0, 256)
  near(m.nrmse(a, a), 0, 1e-9, 'self')
  assert.ok(m.nrmse(a, sine(1.5, 0, 256)) > 0.1, 'scaled differs')
})

test('frequency of a single emitted period = 1/T', () => {
  const s = sine(1, 0, 256)
  near(m.frequency(s), 1, 0.02, 'freq')
})
