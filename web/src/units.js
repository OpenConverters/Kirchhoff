// Engineering-notation formatting (SI prefixes), shared by every readout.

const PREFIXES = [
  [1e12, 'T'], [1e9, 'G'], [1e6, 'M'], [1e3, 'k'],
  [1, ''], [1e-3, 'm'], [1e-6, 'µ'], [1e-9, 'n'], [1e-12, 'p'],
]

export function si(value, unit = '', digits = 3) {
  if (value === null || value === undefined || Number.isNaN(value)) return '—'
  if (value === 0) return `0 ${unit}`.trim()
  const a = Math.abs(value)
  for (const [f, p] of PREFIXES) {
    if (a >= f * 0.9995) {
      return `${trim((value / f).toPrecision(digits))} ${p}${unit}`.trim()
    }
  }
  return `${trim(value.toPrecision(digits))} ${unit}`.trim()
}

function trim(s) {
  return s.includes('.') ? s.replace(/\.?0+$/, '') : s
}

export function pct(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) return '—'
  return `${(value * 100).toFixed(digits)} %`
}
