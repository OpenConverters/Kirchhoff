// Waveform export, driven through the real buttons and asserted on the bytes the browser actually
// downloads. The unit tests (tests/lib/waveExport.test.js) prove the builders; this proves the app
// wires them to a file the user receives.
import { test } from '@playwright/test'
import { boot, selectTopology, solve, expect } from './helpers.js'

// Click an export button and return what the browser saved, as text.
async function download(page, testid) {
  const btn = page.getByTestId(testid).first()
  await expect(btn, `${testid} is enabled`).toBeEnabled()
  const [dl] = await Promise.all([page.waitForEvent('download'), btn.click()])
  const stream = await dl.createReadStream()
  let text = ''
  for await (const chunk of stream) text += chunk
  return { name: dl.suggestedFilename(), text }
}

const rowsOf = (csv) => csv.split('\n').filter((l) => l && !l.startsWith('#'))

test.describe('waveform export', () => {
  test('the design CSV is wide, carries a provenance row, and names columns by PEAS/MAS path', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    const { name, text } = await download(page, 'export-design-csv')
    expect(name).toBe('flyback_waveforms.csv')
    expect(text, 'header names the companion JSON vocabulary').toContain('operatingPointExcitation.json')

    const rows = rowsOf(text)
    const header = rows[0].split(',')
    expect(header[0]).toBe('time [s]')
    expect(header.some((h) => /^T1\.excitationsPerWinding\[0\]\.current \[A\]$/.test(h)),
      `winding current column present in ${header.join(' | ')}`).toBe(true)

    const provenance = rows[1].split(',')
    expect(provenance[0]).toBe('provenance')
    expect(provenance.length, 'one provenance cell per column').toBe(header.length)
    for (const p of provenance.slice(1)) expect(['sampled', 'synthesized']).toContain(p)

    // Every data row is as wide as the header, and the time column is monotonic.
    const data = rows.slice(2)
    expect(data.length, 'the file has samples').toBeGreaterThan(10)
    let prev = -Infinity
    for (const r of data) {
      const cells = r.split(',')
      expect(cells.length).toBe(header.length)
      const t = Number(cells[0])
      expect(Number.isFinite(t)).toBe(true)
      expect(t).toBeGreaterThanOrEqual(prev)
      prev = t
    }
  })

  test('the selection CSV exports only the selected magnetic', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    const { name, text } = await download(page, 'export-selection-csv')
    expect(name).toBe('flyback_T1_waveforms.csv')
    const header = rowsOf(text)[0]
    expect(header).toContain('T1.excitationsPerWinding[0]')
    expect(header, 'no component columns in a magnetic selection').not.toContain('.excitation.')
  })

  test('the JSON export is a set of PEAS/MAS operating points with no nulls left in', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    const { name, text } = await download(page, 'export-design-json')
    expect(name).toBe('flyback_waveforms.json')
    const j = JSON.parse(text)

    expect(j.schemas.magnetics).toBe('https://psma.com/mas/inputs/operatingPoint.json')
    const op = j.magnetics.find((m) => m.isMain).operatingPoint
    expect(op.conditions.ambientTemperature, 'conditions come from the design').toEqual(expect.any(Number))
    expect(op.excitationsPerWinding.length).toBeGreaterThan(0)

    const wf = op.excitationsPerWinding[0].current.waveform
    expect(wf.data.length).toBeGreaterThan(2)
    expect(wf.time.length).toBe(wf.data.length)
    // The engine serialises unset optionals as nulls, and a null is a PRESENT property to a schema:
    // `{ancillaryLabel: null, numberPeriods: null, data, time}` matches NEITHER branch of the
    // waveform oneOf. Nothing exported may carry one.
    const nulls = []
    const walk = (v, path) => {
      if (v === null) { nulls.push(path); return }
      if (Array.isArray(v)) return v.forEach((x, i) => walk(x, `${path}[${i}]`))
      if (v && typeof v === 'object') for (const [k, x] of Object.entries(v)) walk(x, `${path}.${k}`)
    }
    walk(j.magnetics, 'magnetics')
    walk(j.components, 'components')
    expect(nulls, `nulls survive into the export at: ${nulls.slice(0, 5).join(', ')}`).toEqual([])
  })

  test('an ngspice run adds every component, MOSFETs as a named port', async ({ page }) => {
    test.setTimeout(240_000)
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'ngspice'), 'solve error').toBeNull()
    // The app kicks the component pass itself after an ngspice solve.
    await page.waitForFunction(() => (window.__bench.componentWaves?.components?.length ?? 0) > 0,
      null, { timeout: 180_000 })

    const { text } = await download(page, 'export-design-csv')
    const header = rowsOf(text)[0]
    expect(header, 'the MOSFET is a port of a multi-port component')
      .toContain('Q1.excitationsPerPort[drain].voltage [V]')
    expect(header, 'a capacitor is a two-terminal excitation').toContain('Cout.excitation.current [A]')

    const { text: jsonText } = await download(page, 'export-design-json')
    const j = JSON.parse(jsonText)
    const q1 = j.components.find((c) => c.ref === 'Q1')
    expect(q1.operatingPoint.excitationsPerPort[0].port).toBe('drain')
    const cout = j.components.find((c) => c.ref === 'Cout')
    expect('excitation' in cout.operatingPoint, 'two-terminal parts use `excitation`').toBe(true)
  })
})
