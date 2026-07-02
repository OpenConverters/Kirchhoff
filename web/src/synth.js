// Waveform synthesis from MAS "processed" descriptors.
//
// The analytical engine bakes each winding excitation into the TAS as the
// minimal schema-valid processed form (label + peak/offset/peakToPeak/duty),
// and downstream consumers (MKF's inputs.process()) re-derive the waveform
// from it. This is a 1:1 JS port of the label vertex constructions in
// MKF WaveformProcessor::create_waveform (deps/MKF/src/processors/
// WaveformProcessor.cpp:570) so what we plot is exactly what MKF would build.
//
// Labels with no closed form ("custom", i.e. resonant/phase-shift families)
// return null — the UI shows the processed stress readouts instead of
// inventing a shape.

const SINE_POINTS = 128

export function synthesizeWaveform(processed, frequency) {
  if (!processed || !frequency || !(frequency > 0)) return null
  const label = processed.label
  const peakToPeak = processed.peakToPeak
  if (peakToPeak === null || peakToPeak === undefined) return null
  const offset = processed.offset ?? 0
  const dutyCycle = processed.dutyCycle ?? 0.5
  const deadTime = processed.deadTime ?? 0
  const period = 1 / frequency

  let data, time
  switch (label) {
    case 'triangular': {
      const max = peakToPeak / 2 + offset
      const min = -peakToPeak / 2 + offset
      const dc = dutyCycle * period
      data = [min, max, min]
      time = [0, dc, period]
      break
    }
    case 'triangularWithDeadtime': {
      const max = peakToPeak / 2 + offset
      const min = -peakToPeak / 2 + offset
      const dc = dutyCycle * period
      data = [min, max, min, 0]
      time = [0, dc, period - deadTime, period]
      break
    }
    case 'unipolarTriangular': {
      const max = peakToPeak + offset
      const min = offset
      const dc = dutyCycle * period
      data = [min, max, min, min]
      time = [0, dc, dc, period]
      break
    }
    case 'rectangular': {
      const max = peakToPeak * (1 - dutyCycle) + offset
      const min = -peakToPeak * dutyCycle + offset
      const dc = dutyCycle * period
      data = [min, max, max, min, min]
      time = [0, 0, dc, dc, period]
      break
    }
    case 'rectangularWithDeadtime': {
      const max = peakToPeak * (1 - dutyCycle) + offset
      const min = -peakToPeak * dutyCycle + offset
      const dc = dutyCycle * period
      data = [0, max, max, min, min, 0, 0]
      time = [0, 0, dc, dc, period - deadTime, period - deadTime, period]
      break
    }
    case 'secondaryRectangular': {
      const max = -peakToPeak * (1 - dutyCycle) + offset
      const min = peakToPeak * dutyCycle + offset
      const dc = dutyCycle * period
      data = [min, max, max, min, min]
      time = [0, 0, dc, dc, period]
      break
    }
    case 'secondaryRectangularWithDeadtime': {
      const max = -peakToPeak * (1 - dutyCycle) + offset
      const min = peakToPeak * dutyCycle + offset
      const dc = dutyCycle * period
      data = [0, max, max, min, min, 0, 0]
      time = [0, 0, dc, dc, period - deadTime, period - deadTime, period]
      break
    }
    case 'unipolarRectangular': {
      const max = peakToPeak + offset
      const min = offset
      const dc = Math.min(0.5, dutyCycle) * period
      data = [min, max, max, min, min]
      time = [0, 0, dc, dc, period]
      break
    }
    case 'bipolarRectangular': {
      const max = +peakToPeak / 2
      const min = -peakToPeak / 2
      const dc = dutyCycle * period
      data = [0, 0, max, max, 0, 0, min, min, 0, 0]
      time = [
        0,
        0.25 * period - dc / 2, 0.25 * period - dc / 2,
        0.25 * period + dc / 2, 0.25 * period + dc / 2,
        0.75 * period - dc / 2, 0.75 * period - dc / 2,
        0.75 * period + dc / 2, 0.75 * period + dc / 2,
        period,
      ]
      break
    }
    case 'bipolarTriangular': {
      const max = +peakToPeak / 2
      const min = -peakToPeak / 2
      const dc = Math.min(0.5, dutyCycle) * period
      data = [min, min, max, max, min, min]
      time = [
        0,
        0.25 * period - dc / 2, 0.25 * period + dc / 2,
        0.75 * period - dc / 2, 0.75 * period + dc / 2,
        period,
      ]
      break
    }
    case 'flybackPrimary': {
      const max = peakToPeak + offset
      const min = offset
      const dc = dutyCycle * period
      data = [0, min, max, 0, 0]
      time = [0, 0, dc, dc, period]
      break
    }
    case 'flybackSecondary': {
      const max = peakToPeak + offset
      const min = offset
      const dc = dutyCycle * period
      data = [0, 0, max, min, 0]
      time = [0, dc, dc, period, period]
      break
    }
    case 'flybackSecondaryWithDeadtime': {
      const max = peakToPeak + offset
      const min = offset
      const dc = dutyCycle * period
      data = [0, 0, max, min, 0, 0]
      time = [0, dc, dc, period - deadTime, period - deadTime, period]
      break
    }
    case 'sinusoidal': {
      data = []
      time = []
      for (let i = 0; i < SINE_POINTS; ++i) {
        const angle = (i * 2 * Math.PI) / (SINE_POINTS - 1)
        time.push((i * period) / (SINE_POINTS - 1))
        data.push(Math.sin(angle) * (peakToPeak / 2) + offset)
      }
      break
    }
    default:
      return null // "custom" and anything unknown: no closed form
  }
  return { data, time, label }
}
