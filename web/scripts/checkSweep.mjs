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

let combos = 0, skipped = 0, flagged = 0, total = 0
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
      if (out.startsWith('Exception')) { skipped++; continue }
      const tas = JSON.parse(out).tas
      const { svg, pins } = renderForAudit(t.id, tas, opt ?? 'standard')
      const tasRefs = new Set((tas?.topology?.stages ?? []).flatMap((st) => (st.circuit?.components ?? []).map((c) => c.name)))
      const problems = [
        ...checkSchematic({ svg, pins, tas }).map((x) => 'NETS  ' + x),
        ...auditDrawing(svg, tasRefs, pins).map((x) => 'DRAW  ' + x),
        ...auditLabels(await withPage((pg) => measure(pg, svg)), pins.filter((q) => q.pin === 'gate')).map((x) => 'LABEL ' + x),
      ]
      combos++
      total += problems.length
      if (problems.length) { flagged++; console.log(`\n✗ ${name}`); for (const p of problems) console.log('    ' + p) }
    }
  }
}
await browser.close()
console.log(`\n${combos} design points checked (${skipped} not solvable, skipped) — ${total} problem(s) in ${flagged}`)
process.exit(flagged ? 1 : 0)
