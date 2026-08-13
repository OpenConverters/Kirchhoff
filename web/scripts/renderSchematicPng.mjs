// Offline schematic renderer for eyeballing: node scripts/renderSchematicPng.mjs <outdir> [topo[/variant] ...]
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
// Draw what the PRODUCT draws: renderForAudit picks the CIAS layout where one exists (flyback), the
// hand-authored art otherwise — exactly as renderVerifiedSchematic does in the app.
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { chromium } from '@playwright/test'
import fs from 'node:fs'

// The app's own .sch-* rules, read out of src/style.css (harnessCss.mjs). The copy that used to live
// here carried NO font-size and fell back to `monospace`, so every schematic eyeballed through this
// tool wore 16 px system-mono labels instead of 11 px IBM Plex Mono — 45 % oversized, in the wrong
// typeface. Looking at the drawing is the check that catches what no rule can; it has to be the real
// drawing.
import { HARNESS_CSS as CSS } from './harnessCss.mjs'

const M = await init()
const outdir = process.argv[2]
const want = process.argv.slice(3)
fs.mkdirSync(outdir, { recursive: true })
const b = await chromium.launch()
for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = opt ? `${t.id}/${opt}` : t.id
    if (want.length && !want.includes(key) && !want.includes(t.id)) continue
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) throw new Error(`${key}: design failed: ${out.slice(0, 200)}`)
    const { svg } = renderForAudit(t.id, JSON.parse(out).tas, opt ?? 'standard')
    const [, , w, h] = svg.match(/viewBox="([-\d.]+) ([-\d.]+) ([\d.]+) ([\d.]+)"/).slice(1).map(Number)
    const p = await b.newPage({ viewport: { width: Math.ceil(w), height: Math.ceil(h) }, deviceScaleFactor: 4 })
    await p.setContent(`<style>${CSS}</style>${svg.replace('<svg ', `<svg width="${w}" height="${h}" `)}`)
    // Wait for the embedded IBM Plex Mono to load: screenshotting before it lands rasterises the
    // fallback face and every label comes out the wrong width.
    await p.evaluate(() => document.fonts.ready)
    await p.locator('svg').screenshot({ path: `${outdir}/${key.replace('/', '-')}.png` })
    await p.close()
    console.log(key, `${w}x${h}`)
  }
}
await b.close()
