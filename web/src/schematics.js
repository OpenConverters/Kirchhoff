// Phosphor line-art schematic SYMBOLS. This file is the drawing alphabet — one function per part, each
// returning an SVG group and registering its own electrical terminals; the layouts that arrange them
// live in ciasSchematic.js and are generated from the CIAS netlist. Component groups carry
// data-ref="<refdes>" matching the TAS component names, so the view can make them clickable hotspots
// and annotate them with designed values from the BOM.
//
// It used to hold a hand-authored layout per topology as well. Those were ported to CIAS layouts (ABT
// #684) and deleted rather than left behind: once the app stopped rendering them they were a second,
// invisible copy of every drawing — edit one and nothing changes on screen, which is exactly the trap
// that let a fixed label sit unfixed in the product for a session.
//
// Symbol geometry is ported from chris-pikul/electronic-symbols (MIT,
// https://github.com/chris-pikul/electronic-symbols), scaled from its
// 150×150 tiles into this library's terminal contract (lead endpoints stay
// where the layouts expect them): MOSFET-N-Enhancement, Diode-COM-Standard,
// Inductor-COM-Air, Capacitor-IEC-NonPolarized, Resistor-IEEE-Standard,
// Ground-COM-General, Source-COM-DC/AC, Transformer-COM-Standard (coil
// style; orientation kept outward to preserve terminals).
//
// These are sketches of the power path — snubbers, balancing resistors and
// the controller live in the BOM table, not the drawing.

// ── primitives ─────────────────────────────────────────────────────────────

const P = (d, cls = 'sch-sym') => `<path class="${cls}" d="${d}"/>`
const wire = (...pts) => {
  let d = `M ${pts[0]} ${pts[1]}`
  for (let i = 2; i < pts.length; i += 2) d += ` L ${pts[i]} ${pts[i + 1]}`
  return P(d, 'sch-wire')
}
const dot = (x, y) => `<circle class="sch-node" cx="${x}" cy="${y}" r="3"/>`
// This SVG string is rendered via v-html (OutputPane), so every value that comes
// from the catalogue — a component ref-designator or a BOM part value — MUST be
// HTML-escaped before it enters the markup. Catalogue strings are ingested from
// vendor scrapes and are not guaranteed clean (fabricated/mis-mapped records
// have been found), so an unescaped value could break out of a <text> node or an
// attribute and inject script. esc() closes that; the CSP is the second layer.
const esc = (s) =>
  String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]))
// Schematic text size is set in CSS (style.css), which inside an SVG is already in USER UNITS — a
// CSS `px` here is a viewBox unit, not a device pixel, so labels DO scale with the drawing.
//
// Do not "fix" this by moving font-size to a presentation attribute: it is a no-op (measured — the
// rendered geometry is byte-identical either way). The ~14% variation in a label's measured user-unit
// width between a 1600 px and an 800 px window is Chromium quantising glyph advances to whole device
// pixels at different rasterisation scales. No choice of units removes it, which is why label
// clearances must not be tuned to 2 px and why the live-app gate runs at two viewport widths.
const txt = (x, y, s, cls = 'sch-val', anchor = 'middle') =>
  `<text class="${cls}" x="${x}" y="${y}" text-anchor="${anchor}">${esc(s)}</text>`

// Verification hook: symbols register their electrical terminals here (ref, pin-name, coord) ONLY while
// a connectivity check is running (`collectPins`). In normal rendering `_rec` is false, so this is a
// single boolean test per terminal — no output change, negligible cost. The netlist-vs-drawing checker
// (scripts/checkNets) reads these to confirm every net's drawn pins are actually wired together.
const _pins = []
let _rec = false
const regPin = (ref, pin, x, y) => { if (_rec) _pins.push({ ref, pin, x: Math.round(x), y: Math.round(y) }) }

// Invisible footprint marker for glyphs that are NOT BOM parts (source, ground, port, load). hot()
// gives every real part a hitbox the legibility audit can measure; these had none, so a label lying
// across the VAC source symbol was invisible to every rule. `owner` is the glyph's own caption text,
// which is never counted as a collision with itself.
const fp = (x, y, w, h, owner = '') =>
  // fill/stroke none INLINE, not via CSS: an SVG rect defaults to a solid black fill, and these markers
  // are emitted into every schematic — as a class alone they painted black boxes over the load, ground
  // and source glyphs in any consumer that does not ship the stylesheet.
  `<rect class="sch-fp" data-owner="${owner}" fill="none" stroke="none" x="${x}" y="${y}" width="${w}" height="${h}"/>`

function hot(ref, bom, box, body, labelPos) {
  const [bx, by, bw, bh] = box
  const row = bom?.get(ref)
  const [lx, ly, anchor = 'middle', noVal] = labelPos
  const val = !noVal && row?.value && row.value !== '—' ? row.value : ''
  // A part with no BOM row is drawn but NOT a part the drawer can open (App.openPart looks the ref up in
  // bomRows): a FET's intrinsic body diode is a real TAS component and deliberately not an orderable
  // one. Marked .sch-ann so the stylesheet withholds the cursor/hover that promise a drawer and the
  // click handler ignores it — clicking Q2's body diode used to highlight it and then do nothing.
  // Which refs may be annotations is not left to chance: checkSchematicHotspots.mjs pins the set to the
  // TAS components the BOM excludes on purpose, so a genuinely missing part cannot hide here.
  // KEYBOARD + SCREEN READER (ABT #693). A part that opens a drawer is a button, and it was reachable
  // only with a mouse: no tab stop, no key handler, nothing announced. tabindex makes it focusable in
  // document order (which is the order the layout places parts — the power path), role="button" says
  // what it does, and the name carries what a sighted reader gets from the two label lines: the refdes,
  // the kind and the value. Annotations get none of it — they do nothing when activated.
  const name = row ? `Component ${ref}${row.kind ? ', ' + row.kind : ''}${val ? ', ' + val : ''}` : null
  return `<g class="sch-hot${row ? '' : ' sch-ann'}" data-ref="${esc(ref)}"` +
    (name ? ` tabindex="0" role="button" aria-label="${esc(name)}"` : '') + `>
    <rect class="sch-hitbox" x="${bx}" y="${by}" width="${bw}" height="${bh}"/>
    ${body}
    ${txt(lx, ly, ref, 'sch-ref', anchor)}
    ${val ? txt(lx, ly + 12, val, 'sch-val', anchor) : ''}
  </g>`
}

// N-MOSFET (enhancement), drain at (x, y-26), source at (x, y+26), gate to the left.
// Geometry ported from chris-pikul/electronic-symbols Transistor-COM-MOSFET-N-Enhancement.svg
// (MIT), scaled from its 150×150 tile to this footprint: three-segment channel, source-side
// body tie with the N-channel arrow, and the classic enclosure circle.
// flip=true mirrors vertically so the SOURCE is on TOP (y-26) and DRAIN on the bottom — needed where
// the FET's body-diode / source node must face upward (e.g. Vienna's common-source bidirectional switch).
// labelDx nudges the ref/value block along x (same idea as xfmr's) for the case where the clear side
// still has a wire running down it a few px from the default anchor.
function mosfetV(ref, bom, x, y, labelSide = 'right', noVal = false, flip = false, labelDx = 0, labelDy = 0) {
  const inner =
    `<circle class="sch-sym" cx="${x - 8.7}" cy="${y}" r="17.3"/>` +
    // channel: three dashes
    P(`M ${x - 13} ${y - 11.9} L ${x - 13} ${y - 5.5}`) +
    P(`M ${x - 13} ${y - 3.2} L ${x - 13} ${y + 3.2}`) +
    P(`M ${x - 13} ${y + 5.5} L ${x - 13} ${y + 11.9}`) +
    // gate plate + lead
    P(`M ${x - 17.3} ${y - 8.7} L ${x - 17.3} ${y + 8.7}`) +
    P(`M ${x - 17.3} ${y} L ${x - 26} ${y}`) +
    // drain / source / body tie
    P(`M ${x} ${y - 26} L ${x} ${y - 8.7} L ${x - 13} ${y - 8.7}`) +
    P(`M ${x} ${y + 26} L ${x} ${y} L ${x - 13} ${y}`) +
    P(`M ${x} ${y + 8.7} L ${x - 13} ${y + 8.7}`) +
    `<polygon class="sch-fill" points="${x - 10.8},${y} ${x - 4.3},${y - 4.3} ${x - 4.3},${y + 4.3}"/>`
  const body = flip ? `<g transform="matrix(1,0,0,-1,0,${2 * y})">${inner}</g>` : inner
  const lab = labelSide === 'right' ? [x + 12 + labelDx, y - 2 + labelDy, 'start', noVal] : [x - 29 + labelDx, y - 2 + labelDy, 'end', noVal]
  regPin(ref, 'drain', x, flip ? y + 26 : y - 26); regPin(ref, 'source', x, flip ? y - 26 : y + 26); regPin(ref, 'gate', x - 26, y)
  return hot(ref, bom, [x - 27, y - 26, 46, 52], body, lab)
}

// N-MOSFET on a horizontal rail: drain (x-26, y), source (x+26, y), gate below.
// Same ported geometry, transposed (u,v) -> (v,-u) so the channel sits under the rail.
// flip=true mirrors the symbol horizontally so the SOURCE is on the LEFT (x-26) and DRAIN on the right —
// needed where a synchronous rectifier's body diode must face a specific way (source toward the sw node).
function mosfetH(ref, bom, x, y, noVal = false, flip = false, labelSide = 'above') {
  const inner =
    `<circle class="sch-sym" cx="${x}" cy="${y + 8.7}" r="17.3"/>` +
    P(`M ${x - 11.9} ${y + 13} L ${x - 5.5} ${y + 13}`) +
    P(`M ${x - 3.2} ${y + 13} L ${x + 3.2} ${y + 13}`) +
    P(`M ${x + 5.5} ${y + 13} L ${x + 11.9} ${y + 13}`) +
    P(`M ${x - 8.7} ${y + 17.3} L ${x + 8.7} ${y + 17.3}`) +
    P(`M ${x} ${y + 17.3} L ${x} ${y + 26}`) +
    P(`M ${x - 26} ${y} L ${x - 8.7} ${y} L ${x - 8.7} ${y + 13}`) +
    P(`M ${x + 26} ${y} L ${x} ${y} L ${x} ${y + 13}`) +
    P(`M ${x + 8.7} ${y} L ${x + 8.7} ${y + 13}`) +
    `<polygon class="sch-fill" points="${x},${y + 10.8} ${x - 4.3},${y + 4.3} ${x + 4.3},${y + 4.3}"/>`
  const body = flip ? `<g transform="matrix(-1,0,0,1,${2 * x},0)">${inner}</g>` : inner
  regPin(ref, 'drain', flip ? x + 26 : x - 26, y); regPin(ref, 'source', flip ? x - 26 : x + 26, y); regPin(ref, 'gate', x, y + 26)
  return hot(ref, bom, [x - 26, y - 12, 52, 40], body, [x, labelSide === 'below' ? y + 70 : y - 26, 'middle', noVal])
}

// Diode centered at (x, y), leads spanning 40 along `dir` (current flow direction).
// Outline triangle + bar, proportions from Diode-COM-Standard (×0.267).
function diode(ref, bom, x, y, dir, labelSide = 'above', noVal = false, labelDx = 0, labelDy = 0) {
  const t = 6.7, hh = 8.3, bh = 9.1
  let body
  if (dir === 'right' || dir === 'left') {
    const s = dir === 'right' ? 1 : -1
    body =
      P(`M ${x - 20 * s} ${y} L ${x - t * s} ${y}`) +
      P(`M ${x - t * s} ${y - hh} L ${x - t * s} ${y + hh} L ${x + t * s} ${y} Z`) +
      P(`M ${x + t * s} ${y - bh} L ${x + t * s} ${y + bh}`) +
      P(`M ${x + t * s} ${y} L ${x + 20 * s} ${y}`)
  } else {
    const s = dir === 'down' ? 1 : -1
    body =
      P(`M ${x} ${y - 20 * s} L ${x} ${y - t * s}`) +
      P(`M ${x - hh} ${y - t * s} L ${x + hh} ${y - t * s} L ${x} ${y + t * s} Z`) +
      P(`M ${x - bh} ${y + t * s} L ${x + bh} ${y + t * s}`) +
      P(`M ${x} ${y + t * s} L ${x} ${y + 20 * s}`)
  }
  const lab =
    dir === 'right' || dir === 'left'
      ? labelSide === 'above' ? [x + labelDx, y - 26 + labelDy, 'middle', noVal] : [x + labelDx, y + 24 + labelDy, 'middle', noVal]
      : labelSide === 'left' ? [x - 14 + labelDx, y - 2 + labelDy, 'end', noVal] : [x + 14 + labelDx, y - 2 + labelDy, 'start', noVal]
  // anode = triangle base (current-source end), cathode = bar end, per `dir` (anode→cathode flow)
  const ends = { right: [[-20, 0], [20, 0]], left: [[20, 0], [-20, 0]], down: [[0, -20], [0, 20]], up: [[0, 20], [0, -20]] }[dir]
  regPin(ref, 'anode', x + ends[0][0], y + ends[0][1]); regPin(ref, 'cathode', x + ends[1][0], y + ends[1][1])
  return hot(ref, bom, [x - 20, y - 20, 40, 40], body, lab)
}

// Coil bump (Inductor-COM-Air style): each hump is two smooth cubics —
// slightly taller than a semicircle, the vintage hand-drawn coil look.
const coilH = (x0, y, n = 4, w = 14, h = 8.4) => {
  let d = `M ${x0} ${y}`
  for (let i = 0; i < n; ++i) d += ` s 0 ${-h} ${w / 2} ${-h} s ${w / 2} ${h} ${w / 2} ${h}`
  return d
}
const coilV = (x, y0, n = 4, w = 14, h = 8.4, side = 1) => {
  let d = `M ${x} ${y0}`
  for (let i = 0; i < n; ++i) d += ` s ${h * side} 0 ${h * side} ${w / 2} s ${-h * side} ${w / 2} ${-h * side} ${w / 2}`
  return d
}

function indH(ref, bom, x, y, labelSide = 'above', labelDx = 0, labelDy = 0) {
  const lab = labelSide === 'above' ? [x + labelDx, y - 24 + labelDy, 'middle'] : [x + labelDx, y + 20 + labelDy, 'middle']
  regPin(ref, 'p0', x - 28, y); regPin(ref, 'p1', x + 28, y)
  return hot(ref, bom, [x - 28, y - 14, 56, 20], P(coilH(x - 28, y)), lab)
}

function indV(ref, bom, x, y, labelSide = 'right') {
  const lab = labelSide === 'right' ? [x + 13, y - 2, 'start'] : [x - 13, y - 2, 'end']
  regPin(ref, 'p0', x, y - 28); regPin(ref, 'p1', x, y + 28)
  return hot(ref, bom, [x - 12, y - 28, 26, 56], P(coilV(x, y - 28)), lab)
}

// IEC non-polarized: two straight plates (Capacitor-IEC-NonPolarized ×0.267).
function capV(ref, bom, x, y, labelSide = 'right', labelDx = 0, labelDy = 0) {
  regPin(ref, 'p0', x, y - 20); regPin(ref, 'p1', x, y + 20)
  const body =
    P(`M ${x} ${y - 20} L ${x} ${y - 2.5}`) +
    P(`M ${x - 8.3} ${y - 2.5} L ${x + 8.3} ${y - 2.5}`) +
    P(`M ${x - 8.3} ${y + 2.5} L ${x + 8.3} ${y + 2.5}`) +
    P(`M ${x} ${y + 2.5} L ${x} ${y + 20}`)
  const lab = labelSide === 'right' ? [x + 13 + labelDx, y - 2 + labelDy, 'start'] : [x - 13 + labelDx, y - 2 + labelDy, 'end']
  return hot(ref, bom, [x - 12, y - 20, 24, 40], body, lab)
}

function capH(ref, bom, x, y, labelSide = 'above', labelDx = 0, labelDy = 0) {
  regPin(ref, 'p0', x - 20, y); regPin(ref, 'p1', x + 20, y)
  const body =
    P(`M ${x - 20} ${y} L ${x - 2.5} ${y}`) +
    P(`M ${x - 2.5} ${y - 8.3} L ${x - 2.5} ${y + 8.3}`) +
    P(`M ${x + 2.5} ${y - 8.3} L ${x + 2.5} ${y + 8.3}`) +
    P(`M ${x + 2.5} ${y} L ${x + 20} ${y}`)
  const lab = labelSide === 'above' ? [x + labelDx, y - 26 + labelDy, 'middle'] : [x + labelDx, y + 24 + labelDy, 'middle']
  return hot(ref, bom, [x - 20, y - 12, 40, 24], body, lab)
}

// IEEE zigzag, 3 full cycles (Resistor-IEEE-Standard ×0.267).
const zigzagV = (x, y) =>
  `M ${x} ${y - 20} L ${x} ${y - 11.7} L ${x + 6.7} ${y - 8.9} L ${x - 6.7} ${y - 5.4}` +
  ` L ${x + 6.7} ${y - 1.8} L ${x - 6.7} ${y + 1.8} L ${x + 6.7} ${y + 5.4}` +
  ` L ${x - 6.7} ${y + 8.9} L ${x} ${y + 11.7} L ${x} ${y + 20}`

function resV(ref, bom, x, y, labelSide = 'right', labelDx = 0, labelDy = 0) {
  regPin(ref, 'p0', x, y - 20); regPin(ref, 'p1', x, y + 20)
  const lab = labelSide === 'right' ? [x + 13 + labelDx, y - 2 + labelDy, 'start'] : [x - 13 + labelDx, y - 2 + labelDy, 'end']
  return hot(ref, bom, [x - 8, y - 20, 16, 40], P(zigzagV(x, y)), lab)
}

const zigzagH = (x, y) =>
  `M ${x - 20} ${y} L ${x - 11.7} ${y} L ${x - 8.9} ${y + 6.7} L ${x - 5.4} ${y - 6.7}` +
  ` L ${x - 1.8} ${y + 6.7} L ${x + 1.8} ${y - 6.7} L ${x + 5.4} ${y + 6.7}` +
  ` L ${x + 8.9} ${y - 6.7} L ${x + 11.7} ${y} L ${x + 20} ${y}`

function resH(ref, bom, x, y, labelSide = 'above', labelDx = 0, labelDy = 0) {
  regPin(ref, 'p0', x - 20, y); regPin(ref, 'p1', x + 20, y)
  const lab = labelSide === 'above' ? [x + labelDx, y - 26 + labelDy, 'middle'] : [x + labelDx, y + 24 + labelDy, 'middle']
  return hot(ref, bom, [x - 20, y - 10, 40, 20], P(zigzagH(x, y)), lab)
}

// ── control-plane primitives ────────────────────────────────────────────────
// Signal net-label flag: a short dashed stub ending in a named label. Two flags carrying the same
// label are the same control net — standard schematic net-label practice; this is how the gate-drive
// wiring is drawn without routing dashed spaghetti across the power path.
function sig(x, y, label, dir = 'left', labelDx = 0, labelDy = 0) {
  const [dx, dy] = { left: [-13, 0], right: [13, 0], up: [0, -12], down: [0, 12] }[dir]
  const lab = {
    left: [x + dx - 3, y + 3, 'end'], right: [x + dx + 3, y + 3, 'start'],
    up: [x, y + dy - 4, 'middle'], down: [x, y + dy + 11, 'middle'],
  }[dir]
  return P(`M ${x} ${y} L ${x + dx} ${y + dy}`, 'sch-ctl') + txt(lab[0] + labelDx, lab[1] + labelDy, label, 'sch-sig', lab[2])
}

// Controller / control-block IC: a rectangle with named pin stubs (left/right sides, top-to-bottom).
// Each pin label is a signal net name matching a `sig` flag somewhere on the drawing.
function icBox(ref, bom, x, y, w, h, pinsL = [], pinsR = [], title = '') {
  const els = [`<rect class="sch-sym" x="${x - w / 2}" y="${y - h / 2}" width="${w}" height="${h}" rx="4"/>`]
  const place = (pins, side) =>
    pins.forEach((p, i) => {
      const py = y - h / 2 + (h * (i + 1)) / (pins.length + 1)
      const px = side === 'left' ? x - w / 2 : x + w / 2
      const dx = side === 'left' ? -12 : 12
      els.push(P(`M ${px} ${py} L ${px + dx} ${py}`, 'sch-ctl'))
      els.push(txt(px + dx + (side === 'left' ? -3 : 3), py + 3, p, 'sch-sig', side === 'left' ? 'end' : 'start'))
    })
  place(pinsL, 'left')
  place(pinsR, 'right')
  if (title) els.push(txt(x, y + 4, title, 'sch-val'))
  // noVal: the value (e.g. a controller category string) would land on the box border — the
  // in-box title carries the function; the full value lives in the BOM row / part drawer.
  return hot(ref, bom, [x - w / 2, y - h / 2, w, h], els.join(''), [x, y - h / 2 - 8, 'middle', true])
}

// The PWM controller IC (U1 / UDR): one gate-drive pin per driven switch, each matching a `sig`
// flag at that switch's gate.
const ctrlIC = (bom, x, y, gates, ref = 'U1', title = 'PWM') =>
  icBox(ref, bom, x, y, 64, Math.max(44, gates.length * 16 + 16), [], gates, title)

// Transformer at (x, y): primary terminals (x-10, y±h/2), secondary (x+10, y±h/2).
// opts.ct adds a secondary center tap stub ending at (x+24, y).
// opts.opp puts the secondary polarity dot at the bottom (flyback).
function xfmr(ref, bom, x, y, opts = {}) {
  const h = opts.h ?? 80
  const t = y - h / 2, b = y + h / 2
  // Transformer-COM-Standard coil style (two-cubic humps), windings facing
  // outward so the terminals stay at (x±10, y±h/2) for the layouts.
  // Polarity dots sit in the INNER gap between each winding line (x±10) and the
  // core (x±2): the humps only ever bulge OUTWARD past x±10, so this strip is
  // always clear of them (the outer side, where they used to be, is not).
  let body =
    P(coilV(x - 10, t, 4, h / 4, 8.4, -1)) + P(coilV(x + 10, t, 4, h / 4, 8.4, 1)) +
    // Magnetic core (two vertical bars) — NOT an electrical wire. Must be 'sch-sym', never 'sch-wire':
    // as a wire it sits ~8px from the winding terminals and masks a genuinely-disconnected winding from
    // the connectivity checker (this hid the LLC primary being unwired to the resonant tank).
    P(`M ${x - 2} ${t} L ${x - 2} ${b}`, 'sch-sym') +
    P(`M ${x + 2} ${t} L ${x + 2} ${b}`, 'sch-sym') +
    `<circle class="sch-fill" cx="${x - 7}" cy="${t + 5}" r="2.3"/>` +
    `<circle class="sch-fill" cx="${x + 7}" cy="${opts.opp ? b - 5 : t + 5}" r="2.3"/>`
  // center-tap stubs: wires must attach at the stub END, never bare mid-winding
  if (opts.ct === true || opts.ct === 'right' || opts.ct === 'both')
    body += P(`M ${x + 10} ${y} L ${x + 24} ${y}`, 'sch-wire')
  if (opts.ct === 'left' || opts.ct === 'both')
    body += P(`M ${x - 10} ${y} L ${x - 24} ${y}`, 'sch-wire')
  // labelDx / labelDy shift the ref/value block (end-anchored when shifted
  // sideways) so it clears whatever wires the layout routes through the
  // default top-center label zone.
  const ly = t - 24 + (opts.labelDy ?? 0)
  const lab = opts.labelDx ? [x + opts.labelDx, ly, 'end'] : [x, ly, 'middle']
  // winding terminals (for the magnetic connectivity check): primary L, secondary R, optional CT stubs
  regPin(ref, 'p0', x - 10, t); regPin(ref, 'p1', x - 10, b); regPin(ref, 's0', x + 10, t); regPin(ref, 's1', x + 10, b)
  if (opts.ct === true || opts.ct === 'right' || opts.ct === 'both') regPin(ref, 'sct', x + 24, y)
  if (opts.ct === 'left' || opts.ct === 'both') regPin(ref, 'pct', x - 24, y)
  return hot(ref, bom, [x - 22, t - 6, 44, h + 12], body, lab)
}

// Source-COM-DC: drawn +/− marks (plus toward the top terminal), ×0.3 of the tile.
function srcDC(x, y, label = 'VIN') {
  regPin('@src', 'p0', x, y - 15); regPin('@src', 'p1', x, y + 15)
  return (
    fp(x - 15, y - 15, 30, 30, label) +
    `<circle class="sch-sym" cx="${x}" cy="${y}" r="15"/>` +
    P(`M ${x - 3.75} ${y - 7.5} L ${x + 3.75} ${y - 7.5} M ${x} ${y - 11.25} L ${x} ${y - 3.75}`) +
    P(`M ${x - 3.75} ${y + 7.5} L ${x + 3.75} ${y + 7.5}`) +
    txt(x - 22, y + 3, label, 'sch-port', 'end')
  )
}

// Source-COM-AC: one full sine cycle across the circle.
function srcAC(x, y, label = 'VAC') {
  regPin('@src', 'p0', x, y - 15); regPin('@src', 'p1', x, y + 15)
  return (
    fp(x - 15, y - 15, 30, 30, label) +
    `<circle class="sch-sym" cx="${x}" cy="${y}" r="15"/>` +
    P(`M ${x - 11} ${y} q 5.5 -9 11 0 q 5.5 9 11 0`) +
    txt(x - 22, y + 3, label, 'sch-port', 'end')
  )
}

// Ground-COM-General: short stem + three bars (8:4:1), hanging BELOW the rail
// it connects to so the top bar never overlaps the rail line.
function gnd(x, y) {
  regPin('@gnd', 'gnd', x, y)   // primary-side ground; all @gnd symbols are the same net by convention
  return fp(x - 10, y, 20, 16) + P(
    `M ${x} ${y} L ${x} ${y + 6}` +
    ` M ${x - 10} ${y + 6} H ${x + 10} M ${x - 5} ${y + 11} H ${x + 5} M ${x - 1.25} ${y + 16} H ${x + 1.25}`
  )
}

// Isolated secondary return (signal-ground triangle): a DISTINCT reference from the primary earth `gnd`.
// On an isolated converter the output return is galvanically separate from primary ground — joined only
// THROUGH the transformer, never by a wire. Drawn distinct so a reviewer never mistakes the two sides for
// one node, and tagged @sgnd so the net checker can enforce that no wire bridges the isolation barrier.
function isoGnd(x, y) {
  regPin('@sgnd', 'sgnd', x, y)
  return fp(x - 9, y, 18, 16) + P(
    `M ${x} ${y} L ${x} ${y + 6}` +                                   // stem
    ` M ${x - 9} ${y + 6} L ${x + 9} ${y + 6} L ${x} ${y + 16} Z`     // downward hollow triangle
  )
}

// Isolation-barrier marker: a dashed vertical line drawn between a transformer's primary and secondary
// columns to make the galvanic boundary explicit on the schematic.
const isoBar = (x, y0, y1) => P(`M ${x} ${y0} L ${x} ${y1}`, 'sch-ctl')

function loadR(x, y, top, bot, labelDx = 0) {
  regPin('@load', 'p0', x, y - 20); regPin('@load', 'p1', x, y + 20)
  return (
    fp(x - 8, y - 20, 16, 40, 'LOAD') +
    wire(x, top, x, y - 20) + wire(x, y + 20, x, bot) +
    P(zigzagV(x, y)) +
    txt(x + 12 + labelDx, y + 2, 'LOAD', 'sch-val', 'start')
  )
}

function port(x, y, label, anchor = 'start') {
  regPin('@port', label, x, y)   // external port; label maps to a netlist @-node (VOUT→@vout, …)
  return fp(x - 3, y - 3, 6, 6, label) + `<circle class="sch-sym" cx="${x}" cy="${y}" r="3"/>` + txt(anchor === 'start' ? x + 8 : x - 8, y + 4, label, 'sch-port', anchor)
}

// A drawing WITHOUT an accessible name announces as "graphic" and nothing else, so the one piece of
// content in the pane is opaque to anyone not looking at it (WCAG 1.1.1 / 4.1.2). aria-label rather than
// <title> deliberately — <title> would also raise a browser tooltip over the whole drawing on hover,
// which the click-a-component interaction does not want.
//
// role="group", NOT role="img": every component in here is a button (see hot()), and the children of a
// role="img" are presentational — declaring the drawing an image would hide every one of those buttons
// from the assistive tech that needs them most. A group keeps the name and exposes what is inside it.
const svg = (w, h, inner, label = 'Power-path schematic') =>
  `<svg viewBox="0 0 ${w} ${h}" xmlns="http://www.w3.org/2000/svg" role="group" aria-label="${esc(label)}">${inner}</svg>`

// ── multi-winding transformer symbols ──────────────────────────────────────

// Four-winding transformer for the Weinberg / dual-inductor push-pull: two primary half-windings
// (left, each fed by its own coupled-inductor winding — NOT joined at a shared tap) and two secondary
// half-windings (right, center-tapped output). Returns terminal coords keyed by winding:
//   pA{top,bot} pB{top,bot} sC{top,bot} sD{top,bot}, plus the svg body under data-ref=ref.
function xfmr4(ref, bom, x, y, opts = {}) {
  const cL = x - 2, cR = x + 2, top = y - 100, bot = y + 100
  // winding vertical extents (2 stacked coils per side, gap at the core middle)
  const yA = [top + 10, top + 80], yB = [bot - 80, bot - 10]
  const body =
    P(coilV(x - 10, yA[0], 3, (yA[1] - yA[0]) / 3, 8.4, -1)) + P(coilV(x - 10, yB[0], 3, (yB[1] - yB[0]) / 3, 8.4, -1)) +
    P(coilV(x + 10, yA[0], 3, (yA[1] - yA[0]) / 3, 8.4, 1)) + P(coilV(x + 10, yB[0], 3, (yB[1] - yB[0]) / 3, 8.4, 1)) +
    // Core bars are 'sch-sym', never 'sch-wire' — same rule (and same reason) as the 2-winding xfmr:
    // as wires they are 200 px conductors in the middle of the symbol, drawn in the wire colour and
    // fed to the connectivity checker as real net pieces.
    P(`M ${cL} ${top} L ${cL} ${bot}`, 'sch-sym') + P(`M ${cR} ${top} L ${cR} ${bot}`, 'sch-sym') +
    `<circle class="sch-fill" cx="${x - 7}" cy="${yA[0] + 5}" r="2.3"/>` +
    `<circle class="sch-fill" cx="${x + 7}" cy="${yA[0] + 5}" r="2.3"/>`
  const el = hot(ref, bom, [x - 22, top - 6, 44, bot - top + 12], body, [x + (opts.labelDx ?? 0), top - 12 + (opts.labelDy ?? 0), 'middle'])
  regPin(ref, 'a0', x - 10, yA[0]); regPin(ref, 'a1', x - 10, yA[1]); regPin(ref, 'b0', x - 10, yB[0]); regPin(ref, 'b1', x - 10, yB[1])
  regPin(ref, 'c0', x + 10, yA[0]); regPin(ref, 'c1', x + 10, yA[1]); regPin(ref, 'd0', x + 10, yB[0]); regPin(ref, 'd1', x + 10, yB[1])
  return { el, pA: { top: [x - 10, yA[0]], bot: [x - 10, yA[1]] }, pB: { top: [x - 10, yB[0]], bot: [x - 10, yB[1]] },
           sC: { top: [x + 10, yA[0]], bot: [x + 10, yA[1]] }, sD: { top: [x + 10, yB[0]], bot: [x + 10, yB[1]] } }
}

// Three-winding transformer for the forward converter: primary (left, upper) + reset/demag tertiary
// (left, lower) + output secondary (right, upper), all on one core. Returns each terminal's coord.
// Terminals: p0/p1 primary, r0/r1 reset, s0/s1 secondary — regPin'd for the connectivity check.
function xfmr3(ref, bom, x, y, opts = {}) {
  const cL = x - 2, cR = x + 2, top = y - 80, bot = y + 80
  const yP = [top + 10, y - 10], yR = [y + 10, bot - 10], yS = [top + 10, y - 10]
  const coil = (cx, yy, dir) => P(coilV(cx, yy[0], 3, (yy[1] - yy[0]) / 3, 8.4, dir))
  const body =
    coil(x - 10, yP, -1) + coil(x - 10, yR, -1) + coil(x + 10, yS, 1) +
    P(`M ${cL} ${top} L ${cL} ${bot}`, 'sch-sym') + P(`M ${cR} ${top} L ${cR} ${bot}`, 'sch-sym') +   // core, not a conductor
    `<circle class="sch-fill" cx="${x - 7}" cy="${yP[0] + 5}" r="2.3"/>` +   // primary dot (top)
    `<circle class="sch-fill" cx="${x + 7}" cy="${yS[0] + 5}" r="2.3"/>` +   // secondary dot (top → same sense = forward transfer)
    `<circle class="sch-fill" cx="${x - 7}" cy="${yR[1] - 5}" r="2.3"/>`     // reset dot (bottom → opposite = magnetizing reset
  regPin(ref, 'p0', x - 10, yP[0]); regPin(ref, 'p1', x - 10, yP[1])
  regPin(ref, 'r0', x - 10, yR[0]); regPin(ref, 'r1', x - 10, yR[1])
  regPin(ref, 's0', x + 10, yS[0]); regPin(ref, 's1', x + 10, yS[1])
  const ly3 = top - 12 + (opts.labelDy ?? 0)
  const el = hot(ref, bom, [x - 22, top - 6, 44, bot - top + 12], body,
    opts.labelDx ? [x + opts.labelDx, ly3, 'end'] : [x, ly3, 'middle'])
  return { el, p0: [x - 10, yP[0]], p1: [x - 10, yP[1]], r0: [x - 10, yR[0]], r1: [x - 10, yR[1]], s0: [x + 10, yS[0]], s1: [x + 10, yS[1]] }
}

// ── Weinberg (dual-inductor / double-coupled current-fed push-pull): L1's TWO coupled windings each
//    feed a SEPARATE push-pull primary half through its own DCR loop-breaker (Rdcra/Rdcrb) — they are
//    NOT joined at a shared center tap. Each primary half is switched to ground by S1/S2. ────────────
// The phosphor symbol library, exposed so the CIAS-driven generator (ciasSchematic.js) draws the
// SAME line-art as the hand-authored layouts instead of re-inventing symbols. Each returns an SVG
// group; the `hot`-wrapped ones carry a data-ref hotspot + BOM value. Pure given their args (the
// regPin side effects are inert unless a collectPins recording is active).
export const symbols = {
  svg, wire, dot, mosfetV, mosfetH, diode, indH, indV, capV, capH, resV, resH,
  xfmr, xfmr3, xfmr4, srcDC, srcAC, gnd, isoGnd, loadR, port, sig, ctrlIC, icBox, txt,
}

// Verification hook: render once with terminal recording on, returning { svg, pins } where pins is a
// list of { ref, pin, x, y } for every registered electrical terminal (MOSFET drain/source/gate, diode
// anode/cathode, ground symbols, external ports). Used by the netlist-vs-drawing connectivity checker.
// Run an arbitrary drawing function with terminal recording on, returning { out, pins }. The CIAS
// generator uses this so rule E (no wire may end in mid-air) sees EVERY symbol's terminals — including
// the synthesized source/load glyphs, whose pins no layout metadata lists.
export function withPinRecording(fn) {
  _pins.length = 0
  _rec = true
  try { return { out: fn(), pins: _pins.slice() } } finally { _rec = false }
}
