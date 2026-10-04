// Visual sim (CIAS-driven CircuitJS1 export): the solved flyback must yield a well-formed falstad
// circuit + URL. Asserted through the bench context (not by loading the iframe) so CI never talks
// to falstad.com. The exporter re-verifies the drawn wiring against the flattened CIAS nets on
// every call and throws on drift — so `url` existing means the netlist-consistency proof passed.
import { test } from '@playwright/test'
import { boot, selectTopology, solve, setKnob, losslessTargetVout, expect } from './helpers.js'

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
for (const id of ['fsbb', 'ahb', 'acf', 'psfb', 'pshb', 'llc', 'src', 'cllc', 'clllc', 'dab', 'forward', 'push_pull', 'weinberg', 'lcc']) {
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

// The active-SR bridges (CLLC, CLLLC, DAB) have un-convergeable FET totem-poles on BOTH sides (ABT#262):
// CircuitJS1 can't solve the high-side N-channel source-followers, so the WHOLE sim froze at t=0 with
// "Convergence failed" (the pre-charged Cout=vout only made it LOOK alive). The exporter now drives each
// PRIMARY bridge with two ideal antiphase leg sources and draws the SECONDARY synchronous rectifier as its
// equivalent DIODE bridge (a SR FET conducts exactly on its body-diode half-cycle, so the diode is
// faithful and converges). Assert each actually RUNS (isRunning + time advancing — a frozen Cout would
// fool a bare Vout read) and settles near design Vout.
for (const id of ['cllc', 'clllc', 'dab']) {
  test(`${id} visual sim converges to Vout (ideal-driven primary + diode SR)`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
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
    const t0 = await page.evaluate(() => (window.CircuitJS1.getTime?.() ?? 0))
    await page.waitForTimeout(6000)
    const { running, dt } = await page.evaluate((t0) => ({
      running: !!window.CircuitJS1.isRunning?.(),
      dt: (window.CircuitJS1.getTime?.() ?? 0) - t0,
    }), t0)
    expect(running, `${id} sim not running (DC-singular / totem-pole freeze?)`).toBe(true)
    expect(dt, `${id} sim time not advancing (frozen at t=0)`).toBeGreaterThan(0)
    const vLoad = await page.evaluate(() => {
      try { const v = window.CircuitJS1.getNodeVoltage('vout'); if (v != null) return v } catch {}
      return 0
    })
    expect(vLoad, `${id} Vout collapsed (${vLoad} V vs design ${vout} V)`).toBeGreaterThan(vout * 0.7)
    expect(vLoad).toBeLessThan(vout * 1.3)
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
  ...['llc', 'src', 'ahb', 'psfb', 'pshb', 'half_bridge', 'full_bridge'].flatMap((id) => ['fullBridge', 'centerTapped', 'currentDoubler'].map((v) => [id, v])),
]
for (const [id, variant] of RECT3_CASES) {
  test(`${id}/${variant} visual sim converges to Vout`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    await page.evaluate((v) => { window.__bench.form.variant = v }, variant)
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    // The visual sim is lossless: judge it against the output the design sizes a lossless circuit
    // for (Vout, or Vout/efficiency where the turns ratio carries the efficiency), not Vout itself.
    const target = await losslessTargetVout(page)
    const { url, vout, error } = await page.evaluate(() => {
      const vs = window.__bench.visualSim
      return { url: vs?.url, vout: vs?.vout, error: vs?.error || null }
    })
    expect(error, `export error: ${error}`).toBeNull()
    await page.goto(url)
    await page.waitForFunction(
      () => window.CircuitJS1?.getElements && window.CircuitJS1.getElements().length > 5,
      null, { timeout: 30000 })
    // The DC-singular resonant freeze presents as isRunning=false / getTime() stuck at 0 while a
    // frozen Cout still reads =Vout (which fooled a pure getNodeVoltage check). Prove the solver is
    // actually ALIVE — running AND advancing time — before trusting the output voltage.
    const t0 = await page.evaluate(() => (window.CircuitJS1.getTime?.() ?? 0))
    await page.waitForTimeout(6000)             // let the tank + output settle
    const { running, dt } = await page.evaluate((t0) => ({
      running: !!window.CircuitJS1.isRunning?.(),
      dt: (window.CircuitJS1.getTime?.() ?? 0) - t0,
    }), t0)
    expect(running, `${id}/${variant} sim not running (DC-singular freeze?)`).toBe(true)
    expect(dt, `${id}/${variant} sim time not advancing (frozen at t=0)`).toBeGreaterThan(0)
    const vLoad = await page.evaluate(() => {
      try { const v = window.CircuitJS1.getNodeVoltage('vout'); if (v != null) return v } catch {}
      let best = 0
      for (const e of window.CircuitJS1.getElements()) { try { const v = e.getVoltageDiff(); if (Math.abs(v) < 100 && Math.abs(v) > best) best = Math.abs(v) } catch {} }
      return best
    })
    expect(vLoad, `${id}/${variant} Vout collapsed (${vLoad} V vs lossless target ${target} V, design ${vout} V) — check posts/phase`).toBeGreaterThan(target * 0.7)
    expect(vLoad, `${id}/${variant} Vout ${vLoad} V vs lossless target ${target} V (design ${vout} V)`).toBeLessThan(target * 1.3)
  })
}

// Flying-capacitor multilevel buck / boost (config.levels, ABT #1439): the stack is drawn with CircuitJS1
// analog switches (dump 159) on ground-referenced square drives, the flying capacitors pre-charged to
// k·Vtop/(N−1). A switch whose control post lands off its drive never closes and the output collapses, so
// each case SIMULATES the export and holds Vout to the deck's own lossless target: the ideal converter at
// the main switch's stimulus duty — D·Vin for the buck, Vin/(1−D) for the boost. (The preset designs for
// 90 % efficiency, so its duty is not the one that reaches nominal Vout losslessly.)
// At the export's own pace (speed 1.7, ~270 steps/s) the 6 s window other cases use covers ~4 switching
// periods while the output LC rings for tens of seconds, so no reading would be within a few % of anything.
// The test re-imports the SAME circuit text with only the header's speed field raised (50: ~30× more steps
// per second — the physics, timestep and every element are untouched), runs 20 s, and judges the MEAN of the
// last 10 s, which spans several output-ring cycles.
const FCML_CASES = [['buck', 3], ['buck', 4], ['boost', 3]]
for (const [id, levels] of FCML_CASES) {
  test(`fcml ${id} ${levels}-level visual sim lands on the lossless Vout of its duty`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    await page.evaluate(() => { window.__bench.form.variant = 'synchronous' })
    await setKnob(page, 'levels', levels)
    const err = await solve(page, 'analytical')
    expect(err, `solve error: ${err}`).toBeNull()
    const { url, text, vout, vin, error, duty, pairs } = await page.evaluate((boost) => {
      const vs = window.__bench.visualSim
      const tas = window.__bench.result.tas
      const main = boost ? 'QL1' : 'QH1'
      const s = (tas.simulation?.stimulus ?? []).find((x) => x.component === main)
      const cell = tas.topology.stages.find((st) => st.name === 'switchingCell')
      const pairs = (cell?.circuit?.components ?? []).filter((c) => /^QH\d+$/.test(c.name)).length
      return { url: vs?.url, text: vs?.text, vout: vs?.vout, vin: vs?.vin, error: vs?.error || null, duty: s?.waveform?.dutyCycle, pairs }
    }, id === 'boost')
    expect(error, `export error: ${error}`).toBeNull()
    expect(pairs, `${id} at ${levels} levels must carry ${levels - 1} switch pairs`).toBe(levels - 1)
    expect(typeof duty, 'main switch has no pwm duty').toBe('number')
    const target = id === 'boost' ? vin / (1 - duty) : duty * vin
    await page.goto(url)
    await page.waitForFunction(
      () => window.CircuitJS1?.getElements && window.CircuitJS1.getElements().length > 5,
      null, { timeout: 30000 })
    const fast = text.replace(/^\$ 1 (\S+) 1\.7 /, '$ 1 $1 50 ')
    expect(fast, 'export header no longer carries speed 1.7 — update the re-import').not.toBe(text)
    await page.evaluate((t) => window.CircuitJS1.importCircuit(t), fast)
    await page.waitForFunction(() => window.CircuitJS1.getElements().length > 5, null, { timeout: 30000 })
    const t0 = await page.evaluate(() => (window.CircuitJS1.getTime?.() ?? 0))
    const samples = []
    for (let i = 0; i < 20; i++) {
      await page.waitForTimeout(1000)
      samples.push(await page.evaluate(() => window.CircuitJS1.getNodeVoltage('vout')))
    }
    const { running, dt } = await page.evaluate((t0) => ({
      running: !!window.CircuitJS1.isRunning?.(),
      dt: (window.CircuitJS1.getTime?.() ?? 0) - t0,
    }), t0)
    expect(running, `${id} ${levels}L sim not running`).toBe(true)
    expect(dt, `${id} ${levels}L sim time not advancing (frozen at t=0)`).toBeGreaterThan(0)
    const tail = samples.slice(10)
    const vLoad = tail.reduce((a, v) => a + v, 0) / tail.length
    console.log(`fcml ${id} ${levels}L: mean Vout ${vLoad.toFixed(3)} V vs lossless target ${target.toFixed(3)} V (D ${duty.toFixed(4)}, Vin ${vin}, design ${vout})`)
    expect(Math.abs(vLoad - target) / target, `${id} ${levels}L Vout ${vLoad} V vs lossless target ${target} V (D ${duty}, design ${vout} V)`).toBeLessThan(0.03)
  })
}
