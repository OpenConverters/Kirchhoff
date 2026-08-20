// PARAMETER SWEEP (run: `node scripts/checkSweep.mjs [topology]`).
//
// Every other checker verifies ONE design point per topology — the card's own preset. But the drawing
// is generated from the solved BOM: change Vin/Vout/power/fsw and the component VALUES change, which
// changes the LENGTH of every label, which is exactly what the collision rules measure. "Clean at the
// preset" says nothing about "clean at 1.2 V / 40 A" or "at 1 kV". This sweeps a spread of physically
// sensible operating points through the same three rule sets:
//
//   NETS    checkSchematic rules A-F (netlist consistency, isolation, dangling ends, 2-terminal parts)
//   DRAW    auditSchematics    (phantom parts, overlaps, wires through bodies, junction dots, bounds)
//   LABEL   auditSchematicLabels (MEASURED text boxes vs wires / parts / each other)
//   VALUE   checkSchematicValues  (no drawn quantity may be zero, negative or non-finite)
//   GLYPH   checkSchematicGlyphs  (every character printed has a glyph in a face the app ships)
//   DECK    checkValueFidelity    (the number printed beside a part is the number the deck simulates)
//
// A design that cannot be solved at a point (the engine throws) is skipped and counted — that is a
// legitimate answer from the engine, not a drawing defect.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
// Renders through the SAME entry point the app uses (renderForAudit). Every topology is generated from
// CIAS now, but the rule stands: measure what the product draws, never a reconstruction of it.
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { checkSchematic } from '../src/schematicCheck.js'
import { auditDrawing } from './auditSchematics.mjs'
import { auditLabels, measure } from './auditSchematicLabels.mjs'
// The three rules that are pure functions of the SOLVED VALUES, and so belong here more than anywhere:
// the header above says clean-at-the-preset says nothing at 1.2 V / 40 A, and a value's magnitude is
// exactly what changes across these points — which SI prefix gets printed (and whether the app ships a
// glyph for it), whether any quantity collapses to zero, and whether the number beside a part is still
// the number the deck simulates.
import { valueProblems } from './checkSchematicValues.mjs'
import { uncoveredChars } from './checkSchematicGlyphs.mjs'
import { valueFidelity } from './checkValueFidelity.mjs'
import { chromium } from '@playwright/test'

// Points are multiplicative on the card's own preset so each stays in the topology's own domain:
// [label, Vin×, Vout×, power×, fsw]. The extremes are where long value strings appear (µ vs m vs k
// prefixes, 4-significant-digit values) and where the geometry has never been looked at.
const POINTS = [
  ['preset', 1, 1, 1, null],
  ['lowline-lowpower-50k', 0.5, 1, 0.1, 50e3],
  ['highline-highpower-500k', 2, 1, 5, 500e3],
  ['low-vout-high-current', 1, 0.1, 1, null],
  ['high-vout', 1, 5, 1, null],
  ['20kHz', 1, 1, 1, 20e3],
  ['1MHz', 1, 1, 1, 1e6],
  ['tiny-power-20k', 1, 1, 0.02, 20e3],
  ['big-power-1M', 1, 1, 20, 1e6],
]

const M = await init()
const browser = await chromium.launch()
// A page is opened PER RENDER, after the (sometimes multi-minute) design step has finished. Holding one
// page open across the whole sweep let Chromium close the target while PFC's line-cycle design churned,
// killing the run — a flaw in this tool that read as "PFC unverifiable".
const withPage = async (fn) => {
  const page = await browser.newPage()
  try { return await fn(page) } finally { await page.close() }
}
const only = process.argv[2]
const onlyPoint = process.env.KH_POINT   // run a single design point (line-frequency topologies are slow)

let combos = 0, skipped = 0, flagged = 0, total = 0, compared = 0
const skips = []   // every unsolvable point WITH the engine's reason — see the note at the end
for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  if (only && t.id !== only) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    for (const [pt, kVin, kVout, kP, fsw] of POINTS) {
      if (onlyPoint && pt !== onlyPoint) continue
      const preset = { ...t.preset, variant: opt ?? 'standard' }
      // Scale the WHOLE input range, not just the nominal. Scaling vinNom alone left the preset's own
      // vinMin/vinMax behind and built contradictory specs — flyback at the lowline point asked for a
      // 24 V nominal inside a 36–60 V range, and at the highline point a 96 V nominal in the same range.
      // The engine refuses those now (ABT #747), so the harness was quietly throwing away 8 of its 351
      // points and reporting them as "not solvable" — a harness bug wearing an engine's clothes.
      for (const k of ['vinMin', 'vinNom', 'vinMax'])
        if (typeof preset[k] === 'number') preset[k] = preset[k] * kVin
      preset.outputs = (preset.outputs ?? [{ name: 'out', voltage: 12, power: 60 }])
        .map((o) => ({ ...o, voltage: o.voltage * kVout, power: o.power * kP }))
      if (fsw) preset.fs = fsw          // buildSpec reads form.fs, not .fsw
      const spec = buildSpec(preset, t.id)
      if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
      const out = M.design_tas_full(t.id, JSON.stringify(spec))
      const name = `${t.id}${opt ? '/' + opt : ''} @ ${pt}`
      if (out.startsWith('Exception')) { skipped++; skips.push(`${t.id}${opt ? '/' + opt : ''} @${pt}: ${out.slice(10, 120).trim()}`); continue }
      const tas = JSON.parse(out).tas
      const { svg, pins } = renderForAudit(t.id, tas, opt ?? 'standard')
      const tasRefs = new Set((tas?.topology?.stages ?? []).flatMap((st) => (st.circuit?.components ?? []).map((c) => c.name)))
      const deck = M.generate_ngspice_circuit(JSON.stringify(tas), JSON.stringify({ origin: 'REQUIREMENTS' }))
      const fid = deck.startsWith('Exception')
        ? { problems: [`deck generation failed: ${deck.slice(0, 100)}`], compared: 0 }
        : valueFidelity(svg, deck)
      const problems = [
        ...checkSchematic({ svg, pins, tas }).map((x) => 'NETS  ' + x),
        ...auditDrawing(svg, tasRefs, pins).map((x) => 'DRAW  ' + x),
        ...auditLabels(await withPage((pg) => measure(pg, svg)), pins.filter((q) => q.pin === 'gate')).map((x) => 'LABEL ' + x),
        ...valueProblems(svg).problems.map((x) => 'VALUE ' + x),
        ...uncoveredChars(svg).map((c) => `GLYPH U+${c.codepoint.toString(16).toUpperCase().padStart(4, '0')} '${c.char}' is in no shipped face — "${c.text}"`),
        ...fid.problems.map((x) => 'DECK  ' + x),
      ]
      compared += fid.compared
      combos++
      total += problems.length
      if (problems.length) { flagged++; console.log(`\n✗ ${name}`); for (const p of problems) console.log('    ' + p) }
    }
  }
}
await browser.close()
// A skip count on its own is a shrug: it cannot be told apart from a harness that built an impossible
// spec (which is exactly what this sweep did for eight flyback points until the input range was scaled
// as a whole). Name every one, with the engine's own reason.
if (skips.length) {
  console.log(`\nnot solvable at these points — the engine's reason, verbatim:`)
  for (const s of skips) console.log('   ' + s)
}
console.log(`\n${combos} design points checked (${skipped} not solvable, skipped) — ${total} problem(s) in ${flagged}`)
// Say what the value rules actually covered. "0 problems" over 0 comparisons is the failure this suite
// keeps rediscovering, and it reads identically to real coverage unless the count is printed.
console.log(`${compared} printed R/L/C value(s) compared against the deck across those points`)
process.exit(flagged ? 1 : 0)
