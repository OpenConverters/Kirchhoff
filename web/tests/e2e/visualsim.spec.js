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

// LLC/SRC default to the CENTER-TAPPED rectifier, drawn with a 3-winding Custom Transformer (406,
// one primary + two center-tapped secondaries) feeding a 2-diode full-wave rectifier D1/D2. The
// net-consistency checker validates the drawn nets but NOT where CircuitJS1 physically places the 406
// posts — so this guard actually SIMULATES the exported circuit and asserts the output converges to the
// design Vout (regression cover for the post geometry + winding phase). Was silently broken: LLC/SRC
// weren't in the export loop and their default variant had no layout ('no placement for D1').
for (const id of ['llc', 'src']) {
  test(`${id} center-tapped rectifier simulates to Vout`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)              // default variant is centerTapped
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    const { url, vout, text } = await page.evaluate(() => {
      const vs = window.__bench.visualSim
      return { url: vs?.url, vout: vs?.vout, text: vs?.text }
    })
    expect(text, 'center-tapped default must emit a 406 Custom Transformer').toMatch(/(^|\n)406 /)
    await page.goto(url)
    await page.waitForFunction(
      () => window.CircuitJS1?.getElements && window.CircuitJS1.getElements().length > 5,
      null, { timeout: 30000 })
    await page.waitForTimeout(6000)             // let the resonant tank + output settle
    const vLoad = await page.evaluate(() => {
      const els = window.CircuitJS1.getElements()
      // the output load is the highest-value resistor carrying the rectified rail; read 'vout' node.
      try { const v = window.CircuitJS1.getNodeVoltage('vout'); if (v != null) return v } catch {}
      let best = 0
      for (let i = 0; i < els.length; i++) { try { const v = els[i].getVoltageDiff(); if (Math.abs(v) < 100 && Math.abs(v) > best) best = Math.abs(v) } catch {} }
      return best
    })
    // The toy sim converges near the design Vout (measured 11.997 V for a 12 V design). Assert it did
    // NOT collapse to ~0 (the failure mode a wrong 406 geometry or in-phase windings would produce).
    expect(vLoad, `${id} Vout collapsed (${vLoad} V vs design ${vout} V) — check 406 posts/phase`).toBeGreaterThan(vout * 0.7)
    expect(vLoad).toBeLessThan(vout * 1.3)
  })
}
