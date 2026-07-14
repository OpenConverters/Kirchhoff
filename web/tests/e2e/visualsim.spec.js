// Visual sim (CIAS-driven CircuitJS1 export): the solved flyback must yield a well-formed falstad
// circuit + URL. Asserted through the bench context (not by loading the iframe) so CI never talks
// to falstad.com. The exporter re-verifies the drawn wiring against the flattened CIAS nets on
// every call and throws on drift — so `url` existing means the netlist-consistency proof passed.
import { test } from '@playwright/test'
import { boot, selectTopology, solve, expect } from './helpers.js'

test('flyback solve produces a CIAS-consistent falstad export', async ({ page }) => {
  await boot(page)
  await selectTopology(page, 'flyback')
  const err = await solve(page, 'analytical')
  expect(err, `solve error: ${err}`).toBeNull()

  const vs = await page.evaluate(() => window.__bench.visualSim)
  expect(vs?.error, `export error: ${vs?.error}`).toBeUndefined()
  // self-hosted CircuitJS1 (no external host), circuit injected via ?cct=
  expect(vs.url).toContain('circuitjs/circuitjs.html?')
  expect(vs.url).toContain('cct=')
  // the three anchors of the drawing: transformer, switch, labeled output node
  expect(vs.text).toContain('T 320 128 400 128 4 ')
  expect(vs.text).toContain('f 256 240 320 240 32 ')
  expect(vs.text).toContain('207 640 128 688 128 0 vout')
  // similarity transform bookkeeping: scaled to the visual frequency
  expect(vs.fVis).toBe(500)
  expect(vs.scale).toBeCloseTo(vs.fsw / 500, 6)
})

for (const { id, sw } of [{ id: 'buck', sw: 'f 208 208 272 208 32 ' }, { id: 'boost', sw: 'f 256 288 320 288 32 ' }]) {
  test(`${id} solve produces a CIAS-consistent falstad export`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    const vs = await page.evaluate(() => window.__bench.visualSim)
    expect(vs?.error, `export error: ${vs?.error}`).toBeUndefined()
    expect(vs.url).toContain('circuitjs/circuitjs.html?')
    expect(vs.text).toContain(sw)                 // the main switch, at its exact declared coords
    expect(vs.text).toMatch(/\n207 .* vout$/m)    // a labeled output node
  })
}

test('a topology without a visual layout reports unsupported (no garbage export)', async ({ page }) => {
  await boot(page)
  await selectTopology(page, 'vienna')   // control-loop topology — no toy-sim layout (never will be)
  const err = await solve(page, 'analytical')
  expect(err, `solve error: ${err}`).toBeNull()
  const vs = await page.evaluate(() => window.__bench.visualSim)
  expect(vs?.unsupported).toBe(true)
})

// Every isolated/bridge topology that grew a visual layout must still produce a CIAS-consistent export
// (the exporter throws on any net drift, so a well-formed url IS the proof) with a labeled output node.
for (const id of ['fsbb', 'ahb', 'acf', 'psfb', 'pshb', 'llc', 'src', 'cllc', 'clllc', 'dab', 'forward', 'push_pull', 'weinberg']) {
  test(`${id} solve produces a CIAS-consistent falstad export`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    const vs = await page.evaluate(() => window.__bench.visualSim)
    expect(vs?.error, `export error: ${vs?.error}`).toBeUndefined()
    expect(vs.url).toContain('circuitjs/circuitjs.html?')
    expect(vs.text).toMatch(/\n207 .* vout$/m)     // a labeled output node
  })
}

// The 3+ winding isolated topologies are drawn with CircuitJS1's Custom Transformer (dump type 406).
// That element has a cold-parse bistability that latches Vout to a spurious 0; the self-hosted host
// page (web/public/circuitjs/circuitjs.html) self-heals by re-importing when it sees a `406`. Assert
// both halves of that contract stay in place: the export emits a 406, and the host page ships the heal.
for (const id of ['forward', 'push_pull', 'weinberg']) {
  test(`${id} draws a Custom Transformer (dump 406) for its multi-winding core`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    const vs = await page.evaluate(() => window.__bench.visualSim)
    expect(vs.text, `${id} needs a 406 Custom Transformer`).toMatch(/(^|\n)406 /)
  })
}

test('the CircuitJS1 host page ships the 406 cold-parse self-heal', async ({ page }) => {
  const res = await page.goto('/circuitjs/circuitjs.html')
  const html = await res.text()
  expect(html, 'self-heal must gate on getElements() to beat the cold-parse race').toContain('getElements().length')
  expect(html).toContain("importCircuit('$ 1 0.000005 10 50 5 50')")
})

// Regression guard for the synchronous-rectifier phase fix: the active-SR bridges (CLLC, CLLLC, DAB)
// drive their SECONDARY FETs π out of phase with the primary. In phase, the SR FETs short the tank and
// the live output collapses to ~0 (caught on CLLC: 0.35 V vs 48 V). The emitted CircuitJS1 text must
// therefore carry Math.PI as the phaseShift on the secondary gate drives — assert it can't silently
// regress to in-phase. (Passive-diode-secondary topologies deliberately have no such marker.)
for (const id of ['cllc', 'clllc', 'dab']) {
  test(`${id} drives its synchronous rectifier π out of phase (no tank short)`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    const vs = await page.evaluate(() => window.__bench.visualSim)
    // Math.PI stringified — the phaseShift field of the secondary SR gate-drive `v` elements.
    expect(vs.text, `${id} secondary SR drives must be π-shifted`).toContain(String(Math.PI))
  })
}

// The multi-variant rectifier topologies (RECTIFIER_3 axis: fullBridge / centerTapped / current-
// Doubler) must draw a CONVERGING circuit for EVERY variant, not just the default. The net-consistency
// checker validates the drawn nets but NOT where CircuitJS1 physically places the 406 Custom-Transformer
// posts, nor the winding phase — so this guard actually SIMULATES each exported circuit and asserts the
// output settles near the design Vout (a wrong 406 geometry or in-phase windings collapse it to ~0).
// Regression cover for the center-tapped + current-doubler layouts (LLC/SRC had NO center-tapped/doubler
// layout at all — the default threw 'no placement for D1').
const RECT3_CASES = [
  ...['llc', 'src', 'ahb'].flatMap((id) => ['fullBridge', 'centerTapped', 'currentDoubler'].map((v) => [id, v])),
]
for (const [id, variant] of RECT3_CASES) {
  test(`${id}/${variant} visual sim converges to Vout`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    await page.evaluate((v) => { window.__bench.form.variant = v }, variant)
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    const { url, vout, error } = await page.evaluate(() => {
      const vs = window.__bench.visualSim
      return { url: vs?.url, vout: vs?.vout, error: vs?.error || null }
    })
    expect(error, `export error: ${error}`).toBeNull()
    await page.goto(url)
    await page.waitForFunction(
      () => window.CircuitJS1?.getElements && window.CircuitJS1.getElements().length > 5,
      null, { timeout: 30000 })
    await page.waitForTimeout(6000)             // let the tank + output settle
    const vLoad = await page.evaluate(() => {
      try { const v = window.CircuitJS1.getNodeVoltage('vout'); if (v != null) return v } catch {}
      let best = 0
      for (const e of window.CircuitJS1.getElements()) { try { const v = e.getVoltageDiff(); if (Math.abs(v) < 100 && Math.abs(v) > best) best = Math.abs(v) } catch {} }
      return best
    })
    expect(vLoad, `${id}/${variant} Vout collapsed (${vLoad} V vs design ${vout} V) — check posts/phase`).toBeGreaterThan(vout * 0.7)
    expect(vLoad).toBeLessThan(vout * 1.3)
  })
}
