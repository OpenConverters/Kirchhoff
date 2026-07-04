// e2e drivers + extraction for the Kirchhoff topology bench.
//
// Philosophy (docs/TOPOLOGY_BENCH_PROPOSAL.md): DRIVE the app through the DOM (topology cards,
// knob checkboxes/inputs, solve button), READ rich data through window.__bench (getters keep it
// live). The only non-DOM affordances are family-dial rotation and engine selection — controls
// that are either deliberately hard to script or not part of the knob surface under test.
import { expect } from '@playwright/test'
import { TOPOLOGIES, knobsFor } from '../../src/topologies.js'

export { expect }

export const ALL_TOPOLOGIES = TOPOLOGIES.map((t) => ({ id: t.id, name: t.name, family: t.family }))
export const knobCatalog = knobsFor

export async function boot(page) {
  await page.goto('/')
  await page.waitForFunction(() => window.__bench?.engineState === 'ready', null, { timeout: 60_000 })
}

// Select a topology: rotate the family dial (hook), click its card (DOM), open the spec stage (DOM).
export async function selectTopology(page, id) {
  const t = TOPOLOGIES.find((x) => x.id === id)
  if (!t) throw new Error(`unknown topology ${id}`)
  await page.evaluate((f) => window.__bench.setFamily(f), t.family)
  // Open the topology stage so the card is laid out and its click handler reliably fires — selecting
  // (even re-selecting the same id) runs the app's selectTopology, which resets knob overrides.
  await page.getByTestId('stage-topology').click({ force: true })
  await page.getByTestId(`topo-${id}`).click({ force: true })
  await page.waitForFunction((tid) => window.__bench.topologyId === tid, id)
  // Confirm a clean knob slate (re-selection reset every override to auto).
  await page.waitForFunction(() =>
    window.__bench.form?.knobs && Object.values(window.__bench.form.knobs).every((k) => !k.on))
  // A topology with a variant lands on the variant stage; open the spec stage where knobs + solve live.
  await page.getByTestId('stage-spec').click({ force: true })
}

// Open the "Topology knobs" fold (native <details>) so its inputs become visible/interactive.
export async function openKnobs(page) {
  const fold = page.getByTestId('knobs-fold')
  if (await fold.count()) {
    await fold.evaluate((el) => { el.open = true })
  }
}

// Turn a knob's override on and set its value through the DOM. `value` semantics follow the knob
// type: number → filled; enum → selected option id; bool → checkbox state.
export async function setKnob(page, key, value) {
  const meta = currentKnob(page)
  await openKnobs(page)
  const toggle = page.getByTestId(`knob-${key}-auto`)
  await toggle.check()
  const input = page.getByTestId(`knob-${key}-input`)
  await expect(input).toBeVisible()
  const k = await meta
  const km = k.find((x) => x.key === key)
  if (!km) throw new Error(`knob ${key} not in catalog for current topology`)
  if (km.type === 'enum') await input.selectOption(String(value))
  else if (km.type === 'bool') { if (value) await input.check(); else await input.uncheck() }
  else await input.fill(String(value))
}

function currentKnob(page) {
  return page.evaluate(() => window.__bench.topologyId).then((id) => knobsFor(id))
}

// Turn a knob's override back off (deterministic isolation between knobs without re-selecting).
export async function clearKnob(page, key) {
  await openKnobs(page)
  const toggle = page.getByTestId(`knob-${key}-auto`)
  if (await toggle.isChecked()) await toggle.uncheck()
}

// Run a design. engine: 'analytical' (fast, deterministic) | 'ngspice' (real transient).
// Returns the runError string (null on success). Clicks the real solve button via a DOM dispatch
// (sidesteps the accordion fold-animation intercept without bypassing the handler).
export async function solve(page, engine = 'analytical') {
  await page.evaluate((e) => { window.__bench.form.engine = e }, engine)
  await page.evaluate(() => { window.__bench.form.result = null })
  await page.evaluate(() => document.querySelector('[data-testid=solve]').click())
  await page.waitForFunction(
    () => window.__bench.running === false && (window.__bench.result || window.__bench.runError),
    null, { timeout: 90_000 })
  return page.evaluate(() => window.__bench.runError)
}

export function readSpec(page) {
  return page.evaluate(() => window.__bench.lastSpec)
}

// The main magnetic's winding-`w` current or voltage as a {data,time} signal (or null).
export function windingWaveform(page, { winding = 0, side = 'current' } = {}) {
  return page.evaluate(({ winding, side }) => {
    const b = window.__bench
    const main = (b.waveMagnetics || []).find((m) => m.isMain) || b.waveMagnetics?.[0]
    if (!main) return null
    const awf = b.result?.analyticalWaveforms?.[main.name]
    const sig = awf?.excitationsPerWinding?.[winding]?.[side]?.waveform
    return sig && sig.data && sig.time ? { data: sig.data, time: sig.time } : null
  }, { winding, side })
}

// The engine's own processed stats for the main magnetic winding-`w` current/voltage.
export function windingProcessed(page, { winding = 0, side = 'current' } = {}) {
  return page.evaluate(({ winding, side }) => {
    const b = window.__bench
    const main = (b.waveMagnetics || []).find((m) => m.isMain) || b.waveMagnetics?.[0]
    if (!main) return null
    const awf = b.result?.analyticalWaveforms?.[main.name]
    return awf?.excitationsPerWinding?.[winding]?.[side]?.processed ?? null
  }, { winding, side })
}

// Every magnetic's winding-0 current processed block, keyed by magnetic name (for multi-winding checks).
export function allProcessed(page) {
  return page.evaluate(() => {
    const b = window.__bench
    const out = {}
    for (const m of b.waveMagnetics || []) {
      const awf = b.result?.analyticalWaveforms?.[m.name]
      out[m.name] = (awf?.excitationsPerWinding || []).map((w) => ({
        name: w.name, current: w.current?.processed ?? null, voltage: w.voltage?.processed ?? null,
      }))
    }
    return out
  })
}
