// Is the schematic READABLE? (run: `node scripts/checkSchematicContrast.mjs`)
//
// Every other schematic gate is geometric — does this label sit on that wire, does this junction carry
// a dot. All of them are proxies for one question they never actually ask: can a person read the ink?
// A value string can be perfectly placed and still be unreadable if its colour is too close to the
// background, and that regresses silently the first time someone tweaks a palette token.
//
// So: pull the REAL colours out of the app's stylesheet (harnessCss.mjs slices them from src/style.css,
// the same source the render harness uses) and measure WCAG 2.1 contrast against the frame background.
//   • text (refdes, value, port, signal, block caption) must clear 4.5:1 — AA for normal-size text,
//     and the value strings are 10 px, the smallest type in the product.
//   • line art (symbols, wires, junction dots) must clear 3:1 — AA for graphical objects.
// Reported with the measured ratio either way, because "passes" at 4.6 and "passes" at 13 are not the
// same drawing, and the first is one palette tweak from failing.
import { HARNESS_CSS } from './harnessCss.mjs'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const TEXT_MIN = 4.5, GRAPHIC_MIN = 3

// the schematic frame paints this behind everything (src/style.css .schematic-frame)
const FRAME_BG = '#0b0906'

const srgb = (hex) => {
  const h = hex.replace('#', '')
  const n = h.length === 3 ? [...h].map((c) => c + c) : h.match(/../g)
  return n.map((p) => parseInt(p, 16) / 255)
}
const lum = (hex) => {
  const [r, g, b] = srgb(hex).map((c) => (c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4))
  return 0.2126 * r + 0.7152 * g + 0.0722 * b
}
// what the eye actually sees when ink is painted at `alpha` over `bg`
const composite = (fg, bg, alpha) => {
  if (alpha >= 1) return fg
  const [f, b] = [srgb(fg), srgb(bg)]
  const hex = f.map((c, i) => Math.round((c * alpha + b[i] * (1 - alpha)) * 255).toString(16).padStart(2, '0'))
  return '#' + hex.join('')
}
const ratio = (a, b) => {
  const [l1, l2] = [lum(a), lum(b)].sort((x, y) => y - x)
  return (l1 + 0.05) / (l2 + 0.05)
}

// resolve `var(--token)` against the :root block the stylesheet declares
const vars = new Map([...HARNESS_CSS.matchAll(/--([\w-]+):\s*([^;]+);/g)].map((m) => [m[1], m[2].trim()]))
const resolve = (value) => {
  const v = value.trim()
  const m = v.match(/^var\(--([\w-]+)\)$/)
  return m ? resolve(vars.get(m[1]) ?? '') : v
}
// which .sch-* rule paints what, and how the result is judged
const SUBJECTS = [
  ['.sch-ref', 'fill', 'text', 'component refdes'],
  ['.sch-val', 'fill', 'text', 'component value (10 px — the smallest type in the app)'],
  ['.sch-port', 'fill', 'text', 'port name'],
  ['.sch-sig', 'fill', 'text', 'signal net label'],
  ['.sch-blk', 'fill', 'text', 'control-block caption'],
  ['.sch-sym', 'stroke', 'graphic', 'symbol line art'],
  ['.sch-wire', 'stroke', 'graphic', 'wire'],
  ['.sch-node', 'fill', 'graphic', 'junction dot'],
  ['.sch-ctl', 'stroke', 'graphic', 'control-signal stub'],
]

const ruleFor = (sel) => {
  const m = HARNESS_CSS.match(new RegExp(`(^|\\n)[^{}\\n]*\\${sel}\\s*[^{}]*\\{([^}]*)\\}`))
  return m ? m[2] : null
}

// The PRINT palette is the other half of the same question. A schematic is the one thing people take
// out of this app and there is no export, so Ctrl+P is the only route to paper — and the screen palette
// on white is amber at 1.8:1. src/style.css carries an @media print override; measure it against paper.
const rawCss = fs.readFileSync(path.join(path.dirname(fileURLToPath(import.meta.url)), '../src/style.css'), 'utf8')
  .replace(/\/\*[\s\S]*?\*\//g, '')
const printBlock = rawCss.match(/@media\s+print\s*\{((?:[^{}]*\{[^{}]*\})*)[^{}]*\}/)?.[1] ?? ''
const PAPER = printBlock.match(/\.schematic-frame\s*\{[^}]*background:\s*([^;]+)/)?.[1]?.trim() ?? '#ffffff'

let worst = Infinity, failed = 0
console.log('SCREEN — against the frame background ' + FRAME_BG)
for (const [sel, prop, kind, what] of SUBJECTS) {
  const rule = ruleFor(sel)
  if (!rule) { console.log(`${sel.padEnd(11)} NO RULE in src/style.css — the harness and the app disagree`); failed++; continue }
  const decl = rule.match(new RegExp(`(?:^|;)\\s*${prop}\\s*:\\s*([^;]+)`))
  if (!decl) { console.log(`${sel.padEnd(11)} no ${prop} declared`); failed++; continue }
  // A rule may paint at less than full opacity (the control stubs are drawn at 0.75), and what the eye
  // judges is the COMPOSITE over the background, not the declared colour — measuring the token alone
  // would pass ink that is a quarter fainter than measured.
  const alpha = Number(rule.match(/(?:^|;)\s*opacity\s*:\s*([\d.]+)/)?.[1] ?? 1)
  const colour = composite(resolve(decl[1]), FRAME_BG, alpha)
  const r = ratio(colour, FRAME_BG)
  const min = kind === 'text' ? TEXT_MIN : GRAPHIC_MIN
  worst = Math.min(worst, r / min)
  const verdict = r >= min ? 'ok  ' : 'FAIL'
  if (r < min) failed++
  console.log(`${verdict} ${sel.padEnd(11)} ${(colour + (alpha < 1 ? ` @${alpha}` : '')).padEnd(14)} ${r.toFixed(2)}:1  (needs ${min}:1 — ${what})`)
}

console.log(failed
  ? `\n${failed} schematic colour(s) below the readable minimum`
  : `\nEvery schematic colour clears its minimum; the tightest sits ${((worst - 1) * 100).toFixed(0)}% above it`)
console.log('\nPRINT — against paper ' + PAPER)
for (const [sel, prop, kind, what] of SUBJECTS) {
  const rule = printBlock.match(new RegExp(`\\.schematic-frame[^{}]*\\${sel}\\b[^{}]*\\{([^}]*)\\}`))?.[1]
  if (!rule) { console.log(`FAIL ${sel.padEnd(11)} no print override — it would print in the screen colour`); failed++; continue }
  const decl = rule.match(new RegExp(`(?:^|;)\\s*${prop}\\s*:\\s*([^;]+)`))
  if (!decl) { console.log(`FAIL ${sel.padEnd(11)} print rule declares no ${prop}`); failed++; continue }
  const alpha = Number(rule.match(/(?:^|;)\s*opacity\s*:\s*([\d.]+)/)?.[1] ?? 1)
  const colour = composite(resolve(decl[1]), PAPER, alpha)
  const r = ratio(colour, PAPER)
  const min = kind === 'text' ? TEXT_MIN : GRAPHIC_MIN
  worst = Math.min(worst, r / min)
  if (r < min) failed++
  console.log(`${r >= min ? 'ok  ' : 'FAIL'} ${sel.padEnd(11)} ${colour.padEnd(14)} ${r.toFixed(2)}:1  (needs ${min}:1 — ${what})`)
}

// The other half of "readable": a screen reader gets no purchase on line art at all. Two things have to
// hold, and they are checked here because both are generated per render:
//   • the drawing carries an accessible NAME (WCAG 1.1.1 / 4.1.2) — without it the one piece of content
//     in the pane announces as "graphic" and nothing else;
//   • every component that opens a drawer is exposed as a NAMED, FOCUSABLE button (WCAG 2.1.1 / 4.1.2).
//     The root is role="group" for exactly this reason: children of a role="img" are presentational, so
//     declaring the drawing an image would hide every one of those buttons.
const { TOPOLOGIES, VARIANTS, buildSpec } = await import('../src/topologies.js')
const { renderForAudit } = await import('../src/ciasSchematic.js')
const init = (await import('../../build-wasm-ng/kirchhoff.js')).default
const M = await init()
let unnamed = 0, probed = 0
for (const t of TOPOLOGIES) {
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) throw new Error(`${t.id}: design failed: ${out.slice(0, 200)}`)
    const key = `${t.id}${opt ? '/' + opt : ''}`
    const { svg } = renderForAudit(t.id, JSON.parse(out).tas, opt ?? 'standard')
    const root = svg.match(/<svg[^>]*>/)[0]
    const name = root.match(/aria-label="([^"]*)"/)?.[1]
    if (!name?.trim()) { unnamed++; console.log(`FAIL ${key} renders the drawing with no accessible name`) }
    if (/role="img"/.test(root)) { unnamed++; console.log(`FAIL ${key} root is role="img" — its component buttons are presentational and unreachable`) }
    for (const m of svg.matchAll(/<g class="sch-hot(?! sch-ann)[^"]*"[^>]*>/g)) {
      probed++
      const g = m[0]
      const ref = g.match(/data-ref="([^"]*)"/)?.[1]
      if (!/tabindex="0"/.test(g)) { unnamed++; console.log(`FAIL ${key}/${ref} is clickable but not focusable (no tab stop)`) }
      if (!/role="button"/.test(g)) { unnamed++; console.log(`FAIL ${key}/${ref} opens a drawer but is not exposed as a button`) }
      if (!/aria-label="[^"]+"/.test(g)) { unnamed++; console.log(`FAIL ${key}/${ref} is a button with no accessible name`) }
    }
  }
}
// A sweep that examined no component would report "all named" having looked at nothing.
if (!probed) throw new Error('checkSchematicContrast found no clickable components to check')
console.log(unnamed ? `${unnamed} accessibility problem(s) across ${probed} components` : `Every schematic is named and all ${probed} component buttons are focusable and named`)

process.exit(failed || unnamed ? 1 : 0)
