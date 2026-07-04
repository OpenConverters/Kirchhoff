// Signal measurement over piecewise-linear waveforms {data:[], time:[]}.
//
// The engine emits one steady-state period as piecewise-linear vertices: data[i] is the
// value at time[i], linearly interpolated between vertices. Time steps are NON-uniform
// (ngspice) so every integral resamples onto a dense uniform grid first, then integrates
// there. This is generic signal processing — NOT magnetics math (which stays in MKF).
//
// A "signal" is { data:number[], time:number[] } with data.length === time.length >= 2,
// time strictly increasing; it represents exactly one period [time[0], time[last]].

const N_DENSE = 8192

export function period(sig) {
  return sig.time[sig.time.length - 1] - sig.time[0]
}

// Linear interpolation of the piecewise-linear signal at an absolute time t
// (t clamped to [t0, tLast]). Assumes time[] strictly increasing.
export function sampleAt(sig, t) {
  const { data, time } = sig
  if (t <= time[0]) return data[0]
  const last = time.length - 1
  if (t >= time[last]) return data[last]
  // binary search for the segment [time[j], time[j+1]] containing t
  let lo = 0, hi = last
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (time[mid] <= t) lo = mid; else hi = mid
  }
  const t0 = time[lo], t1 = time[hi]
  if (t1 === t0) return data[hi]
  const f = (t - t0) / (t1 - t0)
  return data[lo] + f * (data[hi] - data[lo])
}

// Resample one period onto a dense uniform grid of N points. Returns number[].
export function resample(sig, N = N_DENSE) {
  const t0 = sig.time[0]
  const T = period(sig)
  const y = new Array(N)
  for (let i = 0; i < N; ++i) y[i] = sampleAt(sig, t0 + (T * i) / (N - 1))
  return y
}

export function min(sig) { return Math.min(...sig.data) }
export function max(sig) { return Math.max(...sig.data) }
export function pkpk(sig) { return max(sig) - min(sig) }
export function peak(sig) { return Math.max(Math.abs(min(sig)), Math.abs(max(sig))) }

// Time-average (DC component) via trapezoidal integration on the dense grid.
export function mean(sig, N = N_DENSE) {
  const y = resample(sig, N)
  let s = 0
  for (let i = 1; i < N; ++i) s += 0.5 * (y[i] + y[i - 1])
  return s / (N - 1)
}

// True RMS over one period (dense-grid, so piecewise-linear ramps are captured).
export function rms(sig, N = N_DENSE) {
  const y = resample(sig, N)
  let s = 0
  for (let i = 1; i < N; ++i) {
    const a = y[i - 1] * y[i - 1], b = y[i] * y[i]
    s += 0.5 * (a + b)
  }
  return Math.sqrt(s / (N - 1))
}

// Peak-to-peak ripple as a fraction of |mean|. For an inductor current this is ΔIL/IL.
export function rippleRatio(sig) {
  const m = Math.abs(mean(sig))
  if (m === 0) return Infinity
  return pkpk(sig) / m
}

// Duty cycle: fraction of the period the signal sits above the mid-level
// (default = midpoint of min/max), with hysteresis to reject vertex noise.
// Meant for a rectangular-ish switch-node signal, not a triangle.
export function dutyCycle(sig, N = N_DENSE, level = null) {
  const lo = min(sig), hi = max(sig)
  const mid = level ?? (lo + hi) / 2
  const y = resample(sig, N)
  let above = 0
  for (let i = 0; i < N; ++i) if (y[i] > mid) above++
  return above / N
}

// Fundamental frequency by counting upward mid-level crossings. For a single emitted
// period this returns 1/period; for a tiled/ngspice multi-period trace it counts cycles.
export function frequency(sig, N = N_DENSE) {
  const lo = min(sig), hi = max(sig)
  const mid = (lo + hi) / 2
  const y = resample(sig, N)
  let crossings = 0
  for (let i = 1; i < N; ++i) if (y[i - 1] <= mid && y[i] > mid) crossings++
  const T = period(sig)
  return crossings > 0 ? crossings / T : 1 / T
}

// First-harmonic (fundamental) amplitude and phase (radians) over one period, via a
// single-bin DFT at the period frequency. amplitude = 2*|X1|/N; phase = atan2(-Im, Re).
export function fundamental(sig, N = N_DENSE) {
  const y = resample(sig, N)
  let re = 0, im = 0
  for (let i = 0; i < N; ++i) {
    const ang = (2 * Math.PI * i) / N
    re += y[i] * Math.cos(ang)
    im += y[i] * Math.sin(ang)
  }
  return { amplitude: (2 * Math.sqrt(re * re + im * im)) / N, phase: Math.atan2(-im, re) }
}

// Signed fundamental phase of b relative to a: φ_b − φ_a, in degrees in (-180, 180].
// A pure time-delay of b relative to a reads NEGATIVE (its peak arrives later).
// Both signals must share the same period.
export function phaseBetween(a, b, N = N_DENSE) {
  const pa = fundamental(a, N).phase
  const pb = fundamental(b, N).phase
  let d = ((pb - pa) * 180) / Math.PI
  while (d > 180) d -= 360
  while (d <= -180) d += 360
  return d
}

// Normalised RMS error between two signals, resampled to a common grid over one period,
// normalised by the peak-to-peak of the reference a. 0 == identical.
export function nrmse(a, b, N = N_DENSE) {
  const ya = resample(a, N), yb = resample(b, N)
  let s = 0
  for (let i = 0; i < N; ++i) { const e = ya[i] - yb[i]; s += e * e }
  const rmse = Math.sqrt(s / N)
  const span = Math.max(...ya) - Math.min(...ya)
  return span === 0 ? rmse : rmse / span
}
