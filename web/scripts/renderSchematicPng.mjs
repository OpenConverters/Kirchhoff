// Offline schematic renderer for eyeballing: node scripts/renderSchematicPng.mjs <outdir> [topo[/variant] ...]
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
import { renderSchematic, hasSchematic } from '../src/schematics.js'
import { chromium } from '@playwright/test'
import fs from 'node:fs'

const CSS = `
:root{--bg:#0e0b07;--ink:#e9dfc9;--ink-dim:#8f7f60;--amber:#ffb347;--amber-hi:#ffce85;
--amber-deep:#b87a1e;--cyan:#3ce0c8;--mono:monospace}
body{margin:0;background:#0b0906}
.sch-sym{stroke:var(--amber);stroke-width:1.6;fill:none;stroke-linecap:round}
.sch-wire{stroke:var(--amber-deep);stroke-width:1.3;fill:none}
.sch-fill{fill:var(--amber);stroke:none}
.sch-node{fill:var(--amber)}
.sch-ref{font-family:var(--mono);fill:var(--amber-hi)}
.sch-val{font-family:var(--mono);fill:var(--ink-dim)}
.sch-port{font-family:var(--mono);fill:var(--cyan)}
.sch-ctl{stroke:var(--cyan);stroke-width:1.1;fill:none;stroke-dasharray:4 3;opacity:.75}
.sch-sig{font-family:var(--mono);fill:var(--cyan)}
.sch-blk{font-family:var(--mono);fill:var(--ink-dim)}
.sch-hitbox{fill:transparent;stroke:none}
/* No font-size here: schematic text carries it as an SVG attribute in user units (schematics.js). */
svg{display:block}`

const M = await init()
const outdir = process.argv[2]
const want = process.argv.slice(3)
fs.mkdirSync(outdir, { recursive: true })
const b = await chromium.launch()
for (const t of TOPOLOGIES) {
  if (!hasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = opt ? `${t.id}/${opt}` : t.id
    if (want.length && !want.includes(key) && !want.includes(t.id)) continue
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) continue
    const svg = renderSchematic(t.id, extractBom(JSON.parse(out).tas), opt ?? 'standard')
    const [, , w, h] = svg.match(/viewBox="([-\d.]+) ([-\d.]+) ([\d.]+) ([\d.]+)"/).slice(1).map(Number)
    const p = await b.newPage({ viewport: { width: Math.ceil(w), height: Math.ceil(h) }, deviceScaleFactor: 4 })
    await p.setContent(`<style>${CSS}</style>${svg.replace('<svg ', `<svg width="${w}" height="${h}" `)}`)
    await p.locator('svg').screenshot({ path: `${outdir}/${key.replace('/', '-')}.png` })
    await p.close()
    console.log(key, `${w}x${h}`)
  }
}
await b.close()
