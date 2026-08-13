// The schematic stylesheet the offline harnesses render with — SLICED OUT OF THE APP'S OWN
// src/style.css, never hand-copied.
//
// Every offline tool that rasterises a schematic (the measured label audit, the PNG renderer) has to
// reproduce the app's CSS, because the SVG carries no styling of its own: font-size, font-family and
// letter-spacing all come from the stylesheet, and they decide where every glyph box lands. A hand
// copy drifts silently — renderSchematicPng.mjs carried a copy with NO font-size at all, so every
// schematic anyone "eyeballed" through it was drawn with 16 px browser-default labels in the wrong
// font, ~45 % oversized against the app's 11 px IBM Plex Mono. The pictures were of a drawing the
// product never renders.
//
// So: read the real rules at run time. If style.css changes a size, the harness follows.
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
// Comments are dropped first: a /* … */ sitting above a rule contains no braces, so it would
// otherwise be swallowed into that rule's selector and kill it.
const appCss = fs.readFileSync(path.join(here, '../src/style.css'), 'utf8').replace(/\/\*[\s\S]*?\*\//g, '')
const font = fs.readFileSync(
  path.join(here, '../node_modules/@fontsource/ibm-plex-mono/files/ibm-plex-mono-latin-400-normal.woff2'),
).toString('base64')

// `:root` holds the colour + --mono custom properties every .sch- rule dereferences.
const root = appCss.match(/:root\s*\{[^}]*\}/)
// Each top-level rule whose selector mentions .sch- (the schematic block, hover states and all).
// @media blocks are dropped first: the harness renders for SCREEN, and a print override lifted out of
// its media query would silently repaint every measurement.
const screenCss = appCss.replace(/@media[^{]*\{(?:[^{}]*\{[^{}]*\})*[^{}]*\}/g, '')
const rules = [...screenCss.matchAll(/([^{}]*\.sch-[^{}]*)\{([^}]*)\}/g)].map((m) => `${m[1].trim()}{${m[2]}}`)

// A silently-empty slice would give us back exactly the 16 px-default bug this module exists to kill,
// so prove the extraction worked instead of trusting the regex.
if (!root) throw new Error('harnessCss: no :root block in src/style.css')
const need = ['.sch-ref', '.sch-val', '.sch-port', '.sch-sig', '.sch-blk']
for (const sel of need) {
  const rule = rules.find((r) => r.split('{')[0].split(',').some((s) => s.trim() === sel))
  if (!rule) throw new Error(`harnessCss: no rule for ${sel} in src/style.css`)
  if (!/font-size\s*:/.test(rule)) throw new Error(`harnessCss: ${sel} has no font-size — text would render at the browser default`)
}

export const HARNESS_CSS = [
  `@font-face{font-family:'IBM Plex Mono';src:url(data:font/woff2;base64,${font}) format('woff2');font-weight:400;font-display:block}`,
  root[0],
  'body{margin:0;background:#0b0906}',
  'svg{display:block}',
  ...rules,
].join('\n')
