// Does the app SHIP a glyph for every character its schematics print? (run: `node scripts/checkSchematicGlyphs.mjs`)
//
// Ω and ≤ are in no IBM Plex Mono subset fontsource publishes — its latin face stops at U+00FF plus a
// handful of arrows, and there is no greek subset for the mono family at all. So every resistor value and
// every "RDS(on) ≤ …" fell through to whatever font the host happened to have: measured in the running
// app, Ω rendered at 0.743 em against the 0.6 em mono advance — a proportional glyph sitting among
// monospaced digits — and on a host without U+03A9 it is a tofu box. `document.fonts.check()` answers
// true throughout, and the drawing looks fine on any developer's machine, which is why it survived every
// other gate: the label rules measured it, the contrast gate coloured it, nobody asked where it came from.
//
// src/style.css now ships the missing glyphs (a DejaVu Sans Mono subset at 0.602 em, within 0.3% of Plex's
// 0.6 em advance). This gate keeps that true for whatever character a future value string introduces:
// every character drawn on any of the 39 schematics must fall inside the unicode-range of a face the app
// actually loads. The live half — that the browser really resolves them from those faces — is
// tests/e2e/hotspot.spec.js; this one covers every topology, which the e2e cannot afford to.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { renderForAudit } from '../src/ciasSchematic.js'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const read = (p) => fs.readFileSync(path.join(here, p), 'utf8')

// Every @font-face the app loads: the fontsource stylesheets main.js imports, plus src/style.css's own.
const mainJs = read('../src/main.js')
const sheets = [read('../src/style.css')]
for (const m of mainJs.matchAll(/import\s+'(@fontsource\/[^']+)'/g))
  sheets.push(read(path.join('../node_modules', m[1])))

// Only the families the schematic TEXT can actually use count. Every .sch-* text rule is font-family:
// var(--mono), so coverage means the faces in that stack — not any face the app happens to load. Ω, for
// instance, IS shipped in IBM Plex Sans's greek subset, which the drawing never uses: counting it would
// have declared this whole bug fixed while the schematic still rendered a fallback.
const styleCss = sheets[0]
const monoStack = styleCss.match(/--mono:\s*([^;]+);/)?.[1]
if (!monoStack) throw new Error('checkSchematicGlyphs: no --mono stack in src/style.css')
const families = [...monoStack.matchAll(/'([^']+)'/g)].map((m) => m[1])
if (!families.length) throw new Error(`checkSchematicGlyphs: no named family in --mono (${monoStack})`)
for (const sel of ['.sch-ref', '.sch-val', '.sch-port', '.sch-sig', '.sch-blk']) {
  const rule = styleCss.match(new RegExp(`\\${sel}\\s*(,[^{}]*)?\\{([^}]*)\\}`))?.[2] ?? ''
  if (!/font-family:\s*var\(--mono\)/.test(rule))
    throw new Error(`checkSchematicGlyphs: ${sel} no longer uses var(--mono) — this gate is checking the wrong stack`)
}

// A face with no unicode-range covers everything; fontsource always declares one.
const ranges = []
for (const css of sheets)
  for (const face of css.matchAll(/@font-face\s*\{([^}]*)\}/g)) {
    const family = face[1].match(/font-family:\s*['"]?([^;'"]+)/)?.[1]?.trim()
    if (!families.includes(family)) continue
    const decl = face[1].match(/unicode-range:\s*([^;]+)/)?.[1]
    if (!decl) { ranges.push({ family, lo: 0, hi: 0x10ffff }); continue }
    for (const part of decl.split(',')) {
      const t = part.trim().replace(/^U\+/i, '')
      if (t.includes('?')) { ranges.push({ family, lo: parseInt(t.replace(/\?/g, '0'), 16), hi: parseInt(t.replace(/\?/g, 'F'), 16) }); continue }
      const [a, b] = t.split('-')
      ranges.push({ family, lo: parseInt(a, 16), hi: b === undefined ? parseInt(a, 16) : parseInt(b, 16) })
    }
  }
if (ranges.length < 5) throw new Error(`checkSchematicGlyphs: only ${ranges.length} @font-face ranges found for ${families.join(', ')} — the stylesheets did not parse`)
const covered = (cp) => ranges.filter((r) => cp >= r.lo && cp <= r.hi)

const M = await init()
const where = new Map()   // character -> where it is drawn (first sighting)
for (const t of TOPOLOGIES) {
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${t.id}${opt ? '/' + opt : ''}: design failed: ${out.slice(0, 200)}`)
    const { svg } = renderForAudit(t.id, JSON.parse(out).tas, opt ?? 'standard')
    for (const m of svg.matchAll(/<text[^>]*>([^<]*)<\/text>/g))
      for (const c of m[1].replace(/&[a-z]+;/g, ' '))
        if (!where.has(c)) where.set(c, `${t.id}${opt ? '/' + opt : ''}  "${m[1]}"`)
  }
}
if (where.size < 20) throw new Error(`checkSchematicGlyphs: only ${where.size} characters found across every schematic — nothing was rendered`)

let bad = 0
for (const [c, seen] of [...where].sort()) {
  if (c === ' ' || c === ' ') continue
  const cp = c.codePointAt(0)
  if (!covered(cp).length) {
    bad++
    console.log(`U+${cp.toString(16).toUpperCase().padStart(4, '0')} '${c}' is in no shipped face — ${seen}`)
  }
}
console.log(bad
  ? `\n${bad} character(s) the app draws but does not ship a glyph for`
  : `Every one of the ${where.size} characters drawn across the 39 schematics is covered by a face the app ships`)
process.exit(bad ? 1 : 0)   // a gate that cannot fail is not a gate
