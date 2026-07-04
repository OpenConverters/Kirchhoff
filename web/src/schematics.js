// Phosphor line-art schematics: a small SVG symbol library plus one
// hand-authored power-path layout per topology. Component groups carry
// data-ref="<refdes>" matching the TAS component names, so the view can make
// them clickable hotspots and annotate them with designed values from the BOM.
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
const txt = (x, y, s, cls = 'sch-val', anchor = 'middle') =>
  `<text class="${cls}" x="${x}" y="${y}" text-anchor="${anchor}">${s}</text>`

// Verification hook: symbols register their electrical terminals here (ref, pin-name, coord) ONLY while
// a connectivity check is running (`collectPins`). In normal rendering `_rec` is false, so this is a
// single boolean test per terminal — no output change, negligible cost. The netlist-vs-drawing checker
// (scripts/checkNets) reads these to confirm every net's drawn pins are actually wired together.
const _pins = []
let _rec = false
const regPin = (ref, pin, x, y) => { if (_rec) _pins.push({ ref, pin, x: Math.round(x), y: Math.round(y) }) }

function hot(ref, bom, box, body, labelPos) {
  const [bx, by, bw, bh] = box
  const row = bom?.get(ref)
  const [lx, ly, anchor = 'middle', noVal] = labelPos
  const val = !noVal && row?.value && row.value !== '—' ? row.value : ''
  return `<g class="sch-hot" data-ref="${ref}">
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
function mosfetV(ref, bom, x, y, labelSide = 'right', noVal = false) {
  const body =
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
  const lab = labelSide === 'right' ? [x + 12, y - 2, 'start', noVal] : [x - 29, y - 2, 'end', noVal]
  regPin(ref, 'drain', x, y - 26); regPin(ref, 'source', x, y + 26); regPin(ref, 'gate', x - 26, y)
  return hot(ref, bom, [x - 27, y - 26, 46, 52], body, lab)
}

// N-MOSFET on a horizontal rail: drain (x-26, y), source (x+26, y), gate below.
// Same ported geometry, transposed (u,v) -> (v,-u) so the channel sits under the rail.
function mosfetH(ref, bom, x, y, noVal = false) {
  const body =
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
  regPin(ref, 'drain', x - 26, y); regPin(ref, 'source', x + 26, y); regPin(ref, 'gate', x, y + 26)
  return hot(ref, bom, [x - 26, y - 12, 52, 40], body, [x, y - 26, 'middle', noVal])
}

// Diode centered at (x, y), leads spanning 40 along `dir` (current flow direction).
// Outline triangle + bar, proportions from Diode-COM-Standard (×0.267).
function diode(ref, bom, x, y, dir, labelSide = 'above', noVal = false) {
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
      ? labelSide === 'above' ? [x, y - 26, 'middle', noVal] : [x, y + 24, 'middle', noVal]
      : labelSide === 'left' ? [x - 14, y - 2, 'end', noVal] : [x + 14, y - 2, 'start', noVal]
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

function indH(ref, bom, x, y, labelSide = 'above') {
  const lab = labelSide === 'above' ? [x, y - 24, 'middle'] : [x, y + 20, 'middle']
  return hot(ref, bom, [x - 28, y - 14, 56, 20], P(coilH(x - 28, y)), lab)
}

function indV(ref, bom, x, y, labelSide = 'right') {
  const lab = labelSide === 'right' ? [x + 13, y - 2, 'start'] : [x - 13, y - 2, 'end']
  return hot(ref, bom, [x - 12, y - 28, 26, 56], P(coilV(x, y - 28)), lab)
}

// IEC non-polarized: two straight plates (Capacitor-IEC-NonPolarized ×0.267).
function capV(ref, bom, x, y, labelSide = 'right') {
  const body =
    P(`M ${x} ${y - 20} L ${x} ${y - 2.5}`) +
    P(`M ${x - 8.3} ${y - 2.5} L ${x + 8.3} ${y - 2.5}`) +
    P(`M ${x - 8.3} ${y + 2.5} L ${x + 8.3} ${y + 2.5}`) +
    P(`M ${x} ${y + 2.5} L ${x} ${y + 20}`)
  const lab = labelSide === 'right' ? [x + 13, y - 2, 'start'] : [x - 13, y - 2, 'end']
  return hot(ref, bom, [x - 12, y - 20, 24, 40], body, lab)
}

function capH(ref, bom, x, y, labelSide = 'above') {
  const body =
    P(`M ${x - 20} ${y} L ${x - 2.5} ${y}`) +
    P(`M ${x - 2.5} ${y - 8.3} L ${x - 2.5} ${y + 8.3}`) +
    P(`M ${x + 2.5} ${y - 8.3} L ${x + 2.5} ${y + 8.3}`) +
    P(`M ${x + 2.5} ${y} L ${x + 20} ${y}`)
  const lab = labelSide === 'above' ? [x, y - 26, 'middle'] : [x, y + 24, 'middle']
  return hot(ref, bom, [x - 20, y - 12, 40, 24], body, lab)
}

// IEEE zigzag, 3 full cycles (Resistor-IEEE-Standard ×0.267).
const zigzagV = (x, y) =>
  `M ${x} ${y - 20} L ${x} ${y - 11.7} L ${x + 6.7} ${y - 8.9} L ${x - 6.7} ${y - 5.4}` +
  ` L ${x + 6.7} ${y - 1.8} L ${x - 6.7} ${y + 1.8} L ${x + 6.7} ${y + 5.4}` +
  ` L ${x - 6.7} ${y + 8.9} L ${x} ${y + 11.7} L ${x} ${y + 20}`

function resV(ref, bom, x, y, labelSide = 'right') {
  const lab = labelSide === 'right' ? [x + 13, y - 2, 'start'] : [x - 13, y - 2, 'end']
  return hot(ref, bom, [x - 8, y - 20, 16, 40], P(zigzagV(x, y)), lab)
}

const zigzagH = (x, y) =>
  `M ${x - 20} ${y} L ${x - 11.7} ${y} L ${x - 8.9} ${y + 6.7} L ${x - 5.4} ${y - 6.7}` +
  ` L ${x - 1.8} ${y + 6.7} L ${x + 1.8} ${y - 6.7} L ${x + 5.4} ${y + 6.7}` +
  ` L ${x + 8.9} ${y - 6.7} L ${x + 11.7} ${y} L ${x + 20} ${y}`

function resH(ref, bom, x, y, labelSide = 'above') {
  const lab = labelSide === 'above' ? [x, y - 26, 'middle'] : [x, y + 24, 'middle']
  return hot(ref, bom, [x - 20, y - 10, 40, 20], P(zigzagH(x, y)), lab)
}

// ── control-plane primitives ────────────────────────────────────────────────
// Signal net-label flag: a short dashed stub ending in a named label. Two flags carrying the same
// label are the same control net — standard schematic net-label practice; this is how the gate-drive
// wiring is drawn without routing dashed spaghetti across the power path.
function sig(x, y, label, dir = 'left') {
  const [dx, dy] = { left: [-13, 0], right: [13, 0], up: [0, -12], down: [0, 12] }[dir]
  const lab = {
    left: [x + dx - 3, y + 3, 'end'], right: [x + dx + 3, y + 3, 'start'],
    up: [x, y + dy - 4, 'middle'], down: [x, y + dy + 11, 'middle'],
  }[dir]
  return P(`M ${x} ${y} L ${x + dx} ${y + dy}`, 'sch-ctl') + txt(lab[0], lab[1], label, 'sch-sig', lab[2])
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
    P(`M ${x - 2} ${t} L ${x - 2} ${b}`, 'sch-wire') +
    P(`M ${x + 2} ${t} L ${x + 2} ${b}`, 'sch-wire') +
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
  return hot(ref, bom, [x - 22, t - 6, 44, h + 12], body, lab)
}

// Source-COM-DC: drawn +/− marks (plus toward the top terminal), ×0.3 of the tile.
function srcDC(x, y, label = 'VIN') {
  return (
    `<circle class="sch-sym" cx="${x}" cy="${y}" r="15"/>` +
    P(`M ${x - 3.75} ${y - 7.5} L ${x + 3.75} ${y - 7.5} M ${x} ${y - 11.25} L ${x} ${y - 3.75}`) +
    P(`M ${x - 3.75} ${y + 7.5} L ${x + 3.75} ${y + 7.5}`) +
    txt(x - 22, y + 3, label, 'sch-port', 'end')
  )
}

// Source-COM-AC: one full sine cycle across the circle.
function srcAC(x, y, label = 'VAC') {
  return (
    `<circle class="sch-sym" cx="${x}" cy="${y}" r="15"/>` +
    P(`M ${x - 11} ${y} q 5.5 -9 11 0 q 5.5 9 11 0`) +
    txt(x - 22, y + 3, label, 'sch-port', 'end')
  )
}

// Ground-COM-General: short stem + three bars (8:4:1), hanging BELOW the rail
// it connects to so the top bar never overlaps the rail line.
function gnd(x, y) {
  regPin('@gnd', 'gnd', x, y)   // all ground symbols are the same net by convention
  return P(
    `M ${x} ${y} L ${x} ${y + 6}` +
    ` M ${x - 10} ${y + 6} H ${x + 10} M ${x - 5} ${y + 11} H ${x + 5} M ${x - 1.25} ${y + 16} H ${x + 1.25}`
  )
}

function loadR(x, y, top, bot) {
  return (
    wire(x, top, x, y - 20) + wire(x, y + 20, x, bot) +
    P(zigzagV(x, y)) +
    txt(x + 12, y + 2, 'LOAD', 'sch-val', 'start')
  )
}

function port(x, y, label, anchor = 'start') {
  regPin('@port', label, x, y)   // external port; label maps to a netlist @-node (VOUT→@vout, …)
  return `<circle class="sch-sym" cx="${x}" cy="${y}" r="3"/>` + txt(anchor === 'start' ? x + 8 : x - 8, y + 4, label, 'sch-port', anchor)
}

const svg = (w, h, inner) =>
  `<svg viewBox="0 0 ${w} ${h}" xmlns="http://www.w3.org/2000/svg" role="img">${inner}</svg>`

// ── topology layouts ───────────────────────────────────────────────────────

// Freewheel rectifier: DEFAULT = diode D1 (gnd → sw_node); SYNCHRONOUS (abt #67/#81) = low-side MOSFET Q2
// with its anti-parallel body diode D2, both wired into the power path (matching Buck.cpp's brick).
function buck(bom, variant) {
  const rect =
    variant === 'synchronous'
      ? [
          mosfetV('Q2', bom, 240, 150, 'right', true), wire(240, 80, 240, 124), wire(240, 176, 240, 220),
          diode('D2', bom, 290, 150, 'up', 'right', true), wire(290, 80, 290, 130), wire(290, 170, 290, 220),
          dot(290, 80), dot(290, 220),
        ]
      : [diode('D1', bom, 240, 150, 'up', 'right'), wire(240, 80, 240, 130), wire(240, 170, 240, 220)]
  return svg(720, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 154, 80), wire(70, 165, 70, 220, 520, 220),
    mosfetH('Q1', bom, 180, 80),
    wire(206, 80, 320, 80), dot(240, 80),
    ...rect, dot(240, 220),
    indH('L1', bom, 348, 80), wire(376, 80, 560, 80), dot(430, 80),
    capV('Cout', bom, 430, 150), wire(430, 80, 430, 130), wire(430, 170, 430, 220), dot(430, 220),
    loadR(520, 150, 80, 220), dot(520, 80), dot(520, 220),
    gnd(300, 220), port(600, 80, 'VOUT'), wire(560, 80, 600, 80),
    sig(180, 106, 'g1', 'down'),
    ...(variant === 'synchronous' ? [sig(214, 150, 'g2')] : []),
    ctrlIC(bom, 640, 220, variant === 'synchronous' ? ['g1', 'g2'] : ['g1']),
  ].join(''))
}

// Output rectifier: DEFAULT = diode D1 (sw_node → vout); SYNCHRONOUS (abt #67/#81) = high-side MOSFET Q2
// with its anti-parallel body diode D2 (drawn above the FET), both wired into the power path (Boost.cpp).
function boost(bom, variant) {
  const rect =
    variant === 'synchronous'
      ? [
          wire(240, 80, 284, 80), mosfetH('Q2', bom, 310, 80, true), wire(336, 80, 560, 80),
          diode('D2', bom, 310, 40, 'right', 'above', true), wire(284, 80, 284, 40, 290, 40), wire(330, 40, 336, 40, 336, 80),
        ]
      : [diode('D1', bom, 310, 80, 'right'), wire(240, 80, 290, 80), wire(330, 80, 560, 80)]
  return svg(720, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 122, 80), wire(70, 165, 70, 220, 520, 220),
    indH('L1', bom, 150, 80), wire(178, 80, 240, 80), dot(240, 80),
    mosfetV('Q1', bom, 240, 150), wire(240, 80, 240, 124), wire(240, 176, 240, 220), dot(240, 220),
    ...rect, dot(430, 80),
    capV('Cout', bom, 430, 150), wire(430, 80, 430, 130), wire(430, 170, 430, 220), dot(430, 220),
    loadR(520, 150, 80, 220), dot(520, 80), dot(520, 220),
    gnd(300, 220), port(600, 80, 'VOUT'), wire(560, 80, 600, 80),
    sig(214, 150, 'g1'),
    ...(variant === 'synchronous' ? [sig(310, 106, 'g2', 'down')] : []),
    ctrlIC(bom, 640, 240, variant === 'synchronous' ? ['g1', 'g2'] : ['g1']),
  ].join(''))
}

function sepic(bom) {
  return svg(760, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 122, 80), wire(70, 165, 70, 220, 580, 220),
    indH('L1', bom, 150, 80), wire(178, 80, 230, 80), dot(230, 80),
    mosfetV('Q1', bom, 230, 150), wire(230, 80, 230, 124), wire(230, 176, 230, 220), dot(230, 220),
    capH('Cs', bom, 290, 80), wire(230, 80, 270, 80), wire(310, 80, 360, 80), dot(360, 80),
    indV('L2', bom, 360, 150), wire(360, 80, 360, 122), wire(360, 178, 360, 220), dot(360, 220),
    diode('D1', bom, 430, 80, 'right'), wire(360, 80, 410, 80), wire(450, 80, 620, 80), dot(500, 80),
    capV('Cout', bom, 500, 150), wire(500, 80, 500, 130), wire(500, 170, 500, 220), dot(500, 220),
    loadR(580, 150, 80, 220), dot(580, 80), dot(580, 220),
    gnd(300, 220), port(660, 80, 'VOUT'), wire(620, 80, 660, 80),
    sig(204, 150, 'g1'), ctrlIC(bom, 680, 240, ['g1']),
  ].join(''))
}

function cuk(bom) {
  return svg(760, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 122, 80), wire(70, 165, 70, 220, 580, 220),
    indH('L1', bom, 150, 80), wire(178, 80, 230, 80), dot(230, 80),
    mosfetV('Q1', bom, 230, 150), wire(230, 80, 230, 124), wire(230, 176, 230, 220), dot(230, 220),
    // real RC snubber across Q1 (sw node → Crc_sw → Rrc_sw → gnd), per the cukCell nets
    wire(195, 80, 230, 80), capV('Crc_sw', bom, 195, 105, 'left'), wire(195, 80, 195, 85),
    resV('Rrc_sw', bom, 195, 150, 'left'), wire(195, 125, 195, 130),
    wire(195, 170, 195, 220), dot(195, 220),
    capH('C1', bom, 290, 80), wire(230, 80, 270, 80), wire(310, 80, 360, 80), dot(360, 80),
    diode('D1', bom, 360, 150, 'down', 'right'), wire(360, 80, 360, 130), wire(360, 170, 360, 220), dot(360, 220),
    indH('L2', bom, 428, 80), wire(360, 80, 400, 80), wire(456, 80, 620, 80), dot(500, 80),
    capV('Cout', bom, 500, 150), wire(500, 80, 500, 130), wire(500, 170, 500, 220), dot(500, 220),
    loadR(580, 150, 80, 220), dot(580, 80), dot(580, 220),
    gnd(300, 220), port(660, 80, 'VOUT (−)'), wire(620, 80, 660, 80),
    sig(204, 150, 'g1', 'down'), ctrlIC(bom, 680, 250, ['g1']),
  ].join(''))
}

function zeta(bom) {
  return svg(760, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 154, 80), wire(70, 165, 70, 220, 600, 220),
    mosfetH('Q1', bom, 180, 80), wire(206, 80, 260, 80), dot(260, 80),
    indV('L1', bom, 260, 150), wire(260, 80, 260, 122), wire(260, 178, 260, 220), dot(260, 220),
    capH('Cc', bom, 320, 80), wire(260, 80, 300, 80), wire(340, 80, 390, 80), dot(390, 80),
    diode('D1', bom, 390, 150, 'up', 'right'), wire(390, 80, 390, 130), wire(390, 170, 390, 220), dot(390, 220),
    indH('L2', bom, 458, 80), wire(390, 80, 430, 80), wire(486, 80, 620, 80), dot(530, 80),
    capV('Cout', bom, 530, 150), wire(530, 80, 530, 130), wire(530, 170, 530, 220), dot(530, 220),
    loadR(600, 150, 80, 220), dot(600, 80), dot(600, 220),
    gnd(300, 220), port(660, 80, 'VOUT'), wire(620, 80, 660, 80),
    sig(180, 106, 'g1', 'down'), ctrlIC(bom, 680, 250, ['g1']),
  ].join(''))
}

function fsbb(bom) {
  const top = 70, gy = 320, mid = 190
  return svg(860, 450, [
    srcDC(60, 195), wire(60, 180, 60, top, 220, top), wire(60, 210, 60, gy, 680, gy), // continuous ground rail
    // left leg Q1/Q2 over switch node sw1 (Q1.drain=VIN, mid=sw1, Q2.source=gnd)
    mosfetV('Q1', bom, 220, 132, 'right', true), wire(220, top, 220, 106), wire(220, 158, 220, mid), dot(220, mid),
    mosfetV('Q2', bom, 220, 248, 'right', true), wire(220, mid, 220, 222), wire(220, 274, 220, gy), dot(220, gy),
    // RC snubber on sw1 — own column left of the leg, labels clear
    wire(150, mid, 220, mid), capV('Crc_sw1', bom, 150, mid + 32, 'left'), wire(150, mid, 150, mid + 12),
    resV('Rrc_sw1', bom, 150, mid + 84, 'left'), wire(150, mid + 52, 150, mid + 64), wire(150, mid + 104, 150, gy), dot(150, gy),
    // inductor L bridges the two switch nodes
    indH('L', bom, 340, mid), wire(220, mid, 312, mid), wire(368, mid, 440, mid), dot(440, mid),
    // right leg Q3/Q4 over sw2 (Q3.drain=VOUT, mid=sw2, Q4.source=gnd)
    mosfetV('Q3', bom, 440, 132, 'right', true), wire(440, top, 440, 106), wire(440, 158, 440, mid),
    mosfetV('Q4', bom, 440, 248, 'right', true), wire(440, mid, 440, 222), wire(440, 274, 440, gy), dot(440, gy),
    // RC snubber on sw2 — own column right of the leg
    wire(440, mid, 510, mid), capV('Crc_sw2', bom, 510, mid + 32, 'right'), wire(510, mid, 510, mid + 12),
    resV('Rrc_sw2', bom, 510, mid + 84, 'right'), wire(510, mid + 52, 510, mid + 64), wire(510, mid + 104, 510, gy), dot(510, gy),
    // VOUT rail off Q3 drain
    wire(440, top, 720, top), dot(620, top),
    capV('Cout', bom, 620, 195), wire(620, top, 620, 175), wire(620, 215, 620, gy), dot(620, gy),
    loadR(680, 195, top, gy), dot(680, top),
    gnd(330, gy), port(770, top, 'VOUT'), wire(720, top, 770, top),
    sig(194, 132, 'g1'), sig(194, 248, 'g2'), sig(414, 132, 'g3'), sig(414, 248, 'g4'),
    ctrlIC(bom, 500, 390, ['g1', 'g2', 'g3', 'g4']),
  ].join(''))
}

function flyback(bom) {
  // Quasi-resonant variant: the drain-node resonant capacitor (valley-timing element) is a real
  // power-path part, so it IS drawn — across Q1 drain-source — whenever the design carries it.
  const qr = bom?.get('Cres')
  return svg(760, 350, [
    srcDC(70, 160), wire(70, 145, 70, 70, 240, 70), wire(70, 175, 70, 260, 250, 260),
    capV('Cin', bom, 150, 165, 'left'), wire(150, 70, 150, 145), wire(150, 185, 150, 260), dot(150, 70), dot(150, 260),
    xfmr('T1', bom, 260, 140, { opp: true, labelDy: -30 }), wire(240, 70, 250, 70, 250, 100), wire(250, 180, 250, 202),
    mosfetV('Q1', bom, 250, 228), wire(250, 254, 250, 260),
    // RC clamp across the primary (dc+ → Cclmp → Rclmp → drain): the real leakage-ring damper,
    // wired exactly as the TAS primary-switch stage (dc_pos / clmp_mid / sw).
    dot(205, 70), capV('Cclmp', bom, 205, 90, 'left'), resV('Rclmp', bom, 205, 130, 'left'),
    wire(205, 150, 205, 195, 250, 195), dot(250, 195),
    ...(qr ? [
      dot(205, 195),
      capV('Cres', bom, 205, 215, 'left'), wire(205, 235, 205, 260), dot(205, 260),
    ] : []),
    gnd(180, 260),
    // secondary — flyback polarity: dot at the bottom, diode from the top end
    wire(270, 100, 270, 90, 340, 90),
    diode('D1', bom, 360, 90, 'right'), wire(380, 90, 620, 90), dot(470, 90),
    wire(270, 180, 270, 268, 620, 268), dot(470, 268),
    capV('Cout', bom, 470, 180), wire(470, 90, 470, 160), wire(470, 200, 470, 268),
    loadR(560, 180, 90, 268), dot(560, 90), dot(560, 268),
    port(660, 90, 'VOUT'), wire(620, 90, 660, 90), port(660, 268, 'RTN'), wire(620, 268, 660, 268),
    sig(224, 228, 'g1', 'down'), ctrlIC(bom, 400, 315, ['g1']),
  ].join(''))
}

function forward(bom) {
  return svg(800, 350, [
    srcDC(60, 160), wire(60, 145, 60, 70, 250, 70), wire(60, 175, 60, 260, 260, 260),
    // demagnetization winding back to the bus
    diode('Ddemag', bom, 160, 120, 'up', 'right'), wire(160, 70, 160, 100), dot(160, 70),
    wire(160, 140, 160, 190, 250, 190),
    xfmr('T1', bom, 270, 140, { labelDy: -30 }), wire(250, 70, 260, 70, 260, 100), wire(250, 190, 260, 190, 260, 180),
    mosfetV('Q1', bom, 260, 216), wire(260, 180, 260, 190), wire(260, 242, 260, 260),
    gnd(200, 260),
    // secondary: forward diode + freewheel + output LC
    wire(280, 100, 280, 90, 360, 90),
    diode('Dfwd', bom, 380, 90, 'right'), wire(400, 90, 450, 90), dot(450, 90),
    wire(280, 180, 280, 268, 640, 268), dot(450, 268),
    diode('Dfw', bom, 450, 180, 'up', 'right'), wire(450, 90, 450, 160), wire(450, 200, 450, 268),
    indH('Lout', bom, 510, 90), wire(450, 90, 482, 90), wire(538, 90, 660, 90), dot(580, 90),
    capV('Cout', bom, 580, 180), wire(580, 90, 580, 160), wire(580, 200, 580, 268), dot(580, 268),
    loadR(640, 180, 90, 268), dot(640, 90), dot(640, 268),
    port(690, 90, 'VOUT'), wire(660, 90, 690, 90),
    sig(234, 216, 'g1'), ctrlIC(bom, 450, 320, ['g1']),
  ].join(''))
}

function pushPull(bom) {
  return svg(880, 430, [
    srcDC(60, 170), wire(60, 155, 60, 90, 175, 90, 175, 170, 256, 170), // VIN+ → primary center tap (clear of the winding-end rails)
    wire(60, 185, 60, 320, 200, 320),
    // center-tapped primary: each half drives its own switch; Q1 source routes LEFT to gnd (NOT through Q2)
    xfmr('T1', bom, 280, 170, { h: 130, ct: 'both', labelDy: -14 }),
    wire(270, 105, 200, 105, 200, 104), // winding top end → Q1 drain (pri_top rail at y=105)
    mosfetV('Q1', bom, 200, 130, 'right', true),
    wire(200, 156, 200, 185, 150, 185, 150, 320), dot(150, 320), // Q1 source → left → gnd
    wire(270, 235, 200, 235, 200, 240), // winding bottom end → Q2 drain (pri_bot rail at y=235)
    mosfetV('Q2', bom, 200, 266, 'right', true),
    wire(200, 292, 200, 320), dot(200, 320), gnd(130, 320),
    sig(174, 130, 'g1'), sig(174, 266, 'g2'),
    // real RC damper across the primary (pri_top → Rdmp → Cdmp → pri_bot) — loop on the left, clear of VIN+
    dot(250, 105), wire(250, 105, 250, 55, 105, 55, 105, 100),
    resV('Rdmp', bom, 105, 130, 'left'), capV('Cdmp', bom, 105, 200, 'left'), wire(105, 150, 105, 180),
    wire(105, 220, 105, 235, 200, 235), dot(200, 235),
    // secondary full-wave rectifier: both winding ends → Dtop / Dbot → output rail
    wire(290, 105, 290, 90, 380, 90), diode('Dtop', bom, 400, 90, 'right'), wire(420, 90, 480, 90), dot(480, 90),
    wire(290, 235, 290, 250, 380, 250), diode('Dbot', bom, 400, 250, 'right', 'below'), wire(420, 250, 480, 250), wire(480, 250, 480, 90),
    // secondary center tap = output return
    wire(304, 170, 340, 170, 340, 330, 700, 330),
    indH('Lout', bom, 540, 90), wire(480, 90, 512, 90), wire(568, 90, 700, 90), dot(620, 90),
    capV('Cout', bom, 620, 190), wire(620, 90, 620, 170), wire(620, 210, 620, 330), dot(620, 330),
    loadR(700, 190, 90, 330), dot(700, 90), dot(700, 330),
    port(760, 90, 'VOUT'), wire(700, 90, 760, 90),
    ctrlIC(bom, 400, 390, ['g1', 'g2']),
  ].join(''))
}

// ── secondary rectifier variants (rectifierType: centerTapped | fullBridge | currentDoubler) ──
// Each draws a complete secondary — rectifier + output cap + load + VOUT — to the RIGHT of a
// transformer at (tx, ty, h). They emit the exact refdes the TAS builders use per variant, so the
// hotspots map to BOM rows. The host also sets the transformer's winding shape to match (a
// center-tapped winding for CT, a single winding for FB / CD).

// Output-filter tail shared by every secondary: an optional series output inductor `lout` on the
// positive rail, then Cout ∥ LOAD → VOUT. Positive rail enters at (x0, yTop); the return rail sits at
// retY. Returns the svg elements plus retX (where the return rail should reach the load bottom).
function outTail(bom, x0, yTop, retY, lout) {
  const els = []
  let px = x0
  if (lout) { els.push(wire(x0, yTop, x0 + 12, yTop), indH(lout, bom, x0 + 40, yTop), wire(x0 + 68, yTop, x0 + 80, yTop)); px = x0 + 80 }
  const cX = px + 44, lX = cX + 58, pX = cX + 112, cY = (yTop + retY) / 2
  els.push(
    wire(px, yTop, cX + 88, yTop), dot(cX, yTop),
    capV('Cout', bom, cX, cY), wire(cX, yTop, cX, cY - 10), wire(cX, cY + 10, cX, retY), dot(cX, retY),
    loadR(lX, cY, yTop, retY), dot(lX, yTop), dot(lX, retY),
    port(pX, yTop, 'VOUT'), wire(cX + 88, yTop, pX, yTop),
  )
  return { els, retX: lX }
}

// Center-tapped full-wave: two half-windings (transformer drawn ct:'right'), diodes o.d1/o.d2, tap →
// return. o.lout (name) inserts a series output inductor (bridge families have one; resonant doesn't).
function secCT(bom, tx, ty, h, o = {}) {
  const d1 = o.d1 || 'D1', d2 = o.d2 || 'D2'
  const yT = ty - h / 2, yB = ty + h / 2, aT = yT - 18, aB = yB + 18, retY = yB + 90
  const dX = tx + 100, jX = tx + 180
  const t = outTail(bom, jX, aT, retY, o.lout)
  return [
    wire(tx + 10, yT, tx + 10, aT, tx + 80, aT),
    diode(d1, bom, dX, aT, 'right'), wire(dX + 20, aT, jX, aT), dot(jX, aT),
    wire(tx + 10, yB, tx + 10, aB, tx + 80, aB),
    diode(d2, bom, dX, aB, 'right', 'below'), wire(dX + 20, aB, jX, aB), wire(jX, aB, jX, aT),
    wire(tx + 24, ty, tx + 35, ty, tx + 35, retY, t.retX, retY),
    ...t.els,
  ]
}

// Single-winding full-bridge (4 diodes o.diodes). The winding's two ends drive the two bridge leg-mids.
function secFB(bom, tx, ty, h, o = {}) {
  const ds = o.diodes || ['DH1', 'DL1', 'DH2', 'DL2']
  const yM = ty, yP = ty - h / 2 - 30, yN = ty + h / 2 + 90
  const xA = tx + 90, xB = tx + 160
  const t = outTail(bom, xB, yP, yN, o.lout)
  return [
    wire(tx + 10, ty - h / 2, tx + 10, yM, xA, yM), dot(xA, yM),
    wire(tx + 10, ty + h / 2, tx + 40, ty + h / 2, tx + 40, yN + 34, xB + 44, yN + 34, xB + 44, yM, xB, yM), dot(xB, yM),
    // ref-only labels (4 packed diodes) — VF ratings live in the BOM / part drawer
    diode(ds[0], bom, xA, (yP + yM) / 2, 'up', 'left', true), wire(xA, (yP + yM) / 2 - 20, xA, yP), wire(xA, (yP + yM) / 2 + 20, xA, yM),
    diode(ds[1], bom, xA, (yN + yM) / 2, 'up', 'left', true), wire(xA, (yN + yM) / 2 - 20, xA, yM), wire(xA, (yN + yM) / 2 + 20, xA, yN),
    diode(ds[2], bom, xB, (yP + yM) / 2, 'up', 'right', true), wire(xB, (yP + yM) / 2 - 20, xB, yP), wire(xB, (yP + yM) / 2 + 20, xB, yM),
    diode(ds[3], bom, xB, (yN + yM) / 2, 'up', 'right', true), wire(xB, (yN + yM) / 2 - 20, xB, yM), wire(xB, (yN + yM) / 2 + 20, xB, yN),
    dot(xA, yP), dot(xB, yP), wire(xA, yP, xB, yP),
    wire(xA, yN, t.retX, yN), gnd(xA + 20, yN),
    ...t.els,
  ]
}

// Current-doubler: single winding, two output inductors o.lo1/o.lo2, two catch diodes o.d1/o.d2.
function secCD(bom, tx, ty, h, o = {}) {
  const lo1 = o.lo1 || 'Lo1', lo2 = o.lo2 || 'Lo2', d1 = o.d1 || 'D1', d2 = o.d2 || 'D2'
  const yT = ty - h / 2, yB = ty + h / 2, retY = yB + 90, voX = tx + 150
  const t = outTail(bom, voX, yT, retY, null)
  return [
    indH(lo1, bom, tx + 90, yT), wire(tx + 10, yT, tx + 62, yT), wire(tx + 118, yT, voX, yT), dot(voX, yT),
    indH(lo2, bom, tx + 90, yB), wire(tx + 10, yB, tx + 62, yB), wire(tx + 118, yB, voX, yB),
    // Rlb: series loop-breaker between Lo2's output and the vout node (functional aid that
    // survives to the real deck — a real BOM row, so it is drawn in its electrical position)
    resV('Rlb', bom, voX, ty, 'right'), wire(voX, yT, voX, ty - 20), wire(voX, ty + 20, voX, yB),
    diode(d1, bom, tx + 40, (yT + retY) / 2, 'up', 'left'), wire(tx + 40, (yT + retY) / 2 - 20, tx + 40, yT), wire(tx + 40, (yT + retY) / 2 + 20, tx + 40, retY),
    diode(d2, bom, tx + 90, (yB + retY) / 2, 'up'), wire(tx + 90, (yB + retY) / 2 - 20, tx + 90, yB), wire(tx + 90, (yB + retY) / 2 + 20, tx + 90, retY),
    wire(tx + 40, retY, t.retX, retY), gnd(tx + 60, retY),
    ...t.els,
  ]
}

// Dispatch to the secondary drawer for a rectifier variant, forwarding refdes/inductor options.
function secondaryFor(bom, tx, ty, h, variant, o = {}) {
  if (variant === 'fullBridge') return secFB(bom, tx, ty, h, o)
  if (variant === 'currentDoubler') return secCD(bom, tx, ty, h, o)
  return secCT(bom, tx, ty, h, o)
}
const resonantSecondary = (bom, tx, ty, h, variant) => secondaryFor(bom, tx, ty, h, variant)
const ctOpt = (variant) => (variant === 'centerTapped' ? 'right' : undefined)
// Bridge families (ahb/psfb/pshb) name the rectifier Dr*/Lout and carry an output inductor.
function bridgeRefs(variant) {
  // secFB draws diodes at [legA-top, legA-bot, legB-top, legB-bot]; the netlist pairs the sec_a leg as
  // Dr1(top)/Dr3(bot) and the sec_b leg as Dr2(top)/Dr4(bot), so the refdes order must be Dr1,Dr3,Dr2,Dr4.
  if (variant === 'fullBridge') return { diodes: ['Dr1', 'Dr3', 'Dr2', 'Dr4'], lout: 'Lout' }
  if (variant === 'currentDoubler') return { d1: 'Dr1', d2: 'Dr2', lo1: 'Lout', lo2: 'Lo2' }
  return { d1: 'Dr1', d2: 'Dr2', lout: 'Lout' }
}

// Half-bridge + split-bus front end shared by LLC and SRC: VIN, split caps Chi/Clo with balancing
// bleeders Rbal_hi/Rbal_lo, and the Q1/Q2 half bridge. Returns { els, sw, msplit, gy, top } so the
// per-topology tank can attach. tankReturnX is where the primary return re-enters the msplit rail.
function halfBridgeSplitBus(bom, top, gy) {
  const mid = (top + gy) / 2
  const els = [
    srcDC(50, mid + 5), wire(50, mid - 10, 50, top, 120, top), wire(50, mid + 20, 50, gy, 120, gy),
    gnd(50, gy),
    // split bus Chi / Clo (msplit = cap midpoint) + balancing bleeders Rbal_hi / Rbal_lo
    dot(120, top), dot(120, gy),
    capV('Chi', bom, 120, (top + mid) / 2, 'left'), wire(120, top, 120, (top + mid) / 2 - 20), wire(120, (top + mid) / 2 + 20, 120, mid), dot(120, mid),
    capV('Clo', bom, 120, (mid + gy) / 2, 'left'), wire(120, mid, 120, (mid + gy) / 2 - 20), wire(120, (mid + gy) / 2 + 20, 120, gy),
    resV('Rbal_hi', bom, 170, (top + mid) / 2, 'right'), wire(170, top, 170, (top + mid) / 2 - 20), dot(170, top), wire(170, (top + mid) / 2 + 20, 170, mid), dot(170, mid),
    resV('Rbal_lo', bom, 170, (mid + gy) / 2, 'right'), wire(170, mid, 170, (mid + gy) / 2 - 20), wire(170, (mid + gy) / 2 + 20, 170, gy), dot(170, gy),
    wire(120, top, 170, top), wire(120, gy, 170, gy), wire(120, mid, 170, mid),
    // Q1/Q2 half bridge, switch node = mid
    mosfetV('Q1', bom, 300, mid - 65, 'right', true), wire(300, top, 300, mid - 91), dot(300, top), wire(300, mid - 39, 300, mid), dot(300, mid),
    mosfetV('Q2', bom, 300, mid + 55, 'right', true), wire(300, mid, 300, mid + 29), wire(300, mid + 81, 300, gy), dot(300, gy),
    wire(170, top, 300, top), wire(170, gy, 300, gy),
    sig(274, mid - 65, 'g1'), sig(274, mid + 55, 'g2'),
  ]
  return { els, sw: [300, mid], msplit: 170, mid, top, gy }
}

function llc(bom, variant = 'centerTapped') {
  const top = 70, gy = 300, tx = 620, ty = 185, h = 80
  const hb = halfBridgeSplitBus(bom, top, gy)
  const [swx, swy] = hb.sw
  return svg(1060, 380, [
    ...hb.els,
    // resonant tank sw → Cr → Lr → T1 primary top (clear horizontal lane, well-spaced labels)
    capH('Cr', bom, 380, swy), wire(swx, swy, 360, swy),
    indH('Lr', bom, 480, swy), wire(400, swy, 452, swy), wire(508, swy, 540, swy, 540, ty - h / 2),
    xfmr('T1', bom, tx, ty, { h, ct: ctOpt(variant), labelDx: -44 }),
    // primary return: T1 pri bottom → back to the split-bus midpoint (msplit rail)
    wire(tx - 10, ty + h / 2, tx - 10, 335, 220, 335, 220, swy, hb.msplit, swy),
    ...resonantSecondary(bom, tx, ty, h, variant),
    ctrlIC(bom, 130, 350, ['g1', 'g2']),
  ].join(''))
}

function dab(bom) {
  const top = 90, gy = 300, mid = 195
  const gateOf = (ref) => 'g' + ref.slice(-1)   // QA→gA … QH→gH
  // one bridge leg: hi FET (rail→mid) + lo FET (mid→rail return) at column x, with a parallel
  // balancing divider (bias<hi>/bias<lo>) in a clear lane at bx. `flip` mirrors the gate flag/bias
  // lane to the right for the second leg of a bridge so labels never collide with the neighbour.
  const leg = (qh, ql, rh, rl, x, bx) => {
    const bside = bx < x ? 'left' : 'right'
    return [
      mosfetV(qh, bom, x, 140, 'right', true), wire(x, top, x, 114), wire(x, 166, x, mid), dot(x, mid), dot(x, top),
      mosfetV(ql, bom, x, 250, 'right', true), wire(x, mid, x, 224), wire(x, 276, x, gy), dot(x, gy),
      sig(x - 26, 140, gateOf(qh)), sig(x - 26, 250, gateOf(ql)),
      // balancing divider in its own lane
      resV(rh, bom, bx, 140, bside), wire(bx, top, bx, 120), dot(bx, top), wire(bx, 160, bx, mid), dot(bx, mid),
      resV(rl, bom, bx, 250, bside), wire(bx, mid, bx, 230), wire(bx, 270, bx, gy), dot(bx, gy),
      wire(Math.min(bx, x), mid, Math.max(bx, x), mid),
    ]
  }
  return svg(1300, 470, [
    srcDC(50, 190), wire(50, 175, 50, top, 450, top), wire(50, 205, 50, gy, 450, gy), // rails span both primary legs + bias lanes
    gnd(50, gy),
    // ---- primary full bridge: leg A (QA/QB) + leg C (QC/QD) ----
    ...leg('QA', 'QB', 'RbiasA_hi', 'RbiasA_lo', 230, 130),
    ...leg('QC', 'QD', 'RbiasC_hi', 'RbiasC_lo', 350, 450),
    // series Lr: midA → (down, across) → Lr → T1 pri-top; T1 pri-bot → midC
    wire(230, mid, 230, 380, 500, 380), indH('Lr', bom, 528, 380, 'below'), wire(556, 380, 560, 380, 560, 155),
    xfmr('T1', bom, 570, mid, { h: 80, labelDy: -26 }),
    wire(560, 235, 350, 235, 350, mid),
    // RC snubber across the primary (midC → Crc_pri → Rrc_pri → midA lane), clear lane below
    dot(350, mid), wire(350, 330, 380, 330), capH('Crc_pri', bom, 400, 330), wire(420, 330, 440, 330),
    resH('Rrc_pri', bom, 460, 330, 'below'), wire(350, mid, 350, 330), wire(480, 330, 500, 330, 500, mid), dot(500, mid),
    wire(500, mid, 500, 380),
    // ---- secondary full bridge: leg E (QE/QF) + leg G (QG/QH) ----
    ...leg('QE', 'QF', 'RbiasE_hi', 'RbiasE_lo', 770, 660),
    ...leg('QG', 'QH', 'RbiasG_hi', 'RbiasG_lo', 890, 990),
    // T1 sec-top → midE ; T1 sec-bot → midG
    wire(580, 155, 580, 110, 770, 110, 770, mid),
    wire(580, 235, 620, 235, 620, 420, 890, 420, 890, mid),
    // output: bridge legs' rails (spanning bias E lane at 660 → bias G at 990) → Cout ∥ load → VOUT
    dot(770, top), dot(890, top), wire(660, top, 1090, top), dot(1010, top),
    dot(770, gy), dot(890, gy), wire(660, gy, 1050, gy), gnd(960, gy), // isolated secondary return
    capV('Cout', bom, 1010, mid), wire(1010, top, 1010, mid - 20), wire(1010, mid + 20, 1010, gy), dot(1010, gy),
    loadR(1090, mid, top, gy), dot(1090, top),
    port(1150, top, 'VOUT'), wire(1090, top, 1150, top),
    // controllers: primary PWM + secondary SR driver, gate nets by label
    ctrlIC(bom, 300, 415, ['gA', 'gB', 'gC', 'gD']),
    icBox('UDR', bom, 830, 415, 70, 76, [], ['gE', 'gF', 'gG', 'gH'], 'SR DRV'),
  ].join(''))
}

// ── two-switch forward: Q1/Q2 sandwich the primary, D1/D2 clamp the reset to the bus ─────
function twoSwitchForward(bom) {
  return svg(880, 400, [
    srcDC(60, 200), wire(60, 185, 60, 70, 320, 70), wire(60, 215, 60, 320, 760, 320),
    // Q1 (high) and Q2 (low) sandwich the primary; the T1 primary sits to their right (in series)
    mosfetV('Q1', bom, 320, 120, 'right', true), wire(320, 70, 320, 94), dot(320, 70), wire(320, 146, 320, 165), dot(320, 165),
    mosfetV('Q2', bom, 320, 290, 'right', true), wire(320, 245, 320, 264), dot(320, 245), wire(320, 316, 320, 320),
    sig(294, 120, 'g1'), sig(294, 290, 'g2'),
    xfmr('T1', bom, 430, 205, { h: 80, labelDy: -24 }),
    wire(320, 165, 420, 165), wire(320, 245, 420, 245), // Q1 source → pri top ; Q2 drain → pri bottom
    // reset/clamp diodes: D1 (gnd → primary top), D2 (primary bottom → Vin) — own column, ref-only
    diode('D1', bom, 230, 210, 'up', 'left', true), wire(230, 190, 230, 165, 320, 165), wire(230, 230, 230, 320), dot(230, 320),
    diode('D2', bom, 270, 110, 'up', 'left', true), wire(270, 90, 270, 70), dot(270, 70), wire(270, 130, 270, 245, 320, 245),
    // secondary: forward diode + freewheel into Lout / Cout
    wire(440, 165, 480, 165, 480, 130, 520, 130),
    diode('Dfwd', bom, 540, 130, 'right'), wire(560, 130, 600, 130), dot(600, 130),
    diode('Dfw', bom, 600, 200, 'up', 'right'), wire(600, 130, 600, 180), wire(600, 220, 600, 320), dot(600, 320),
    wire(440, 245, 480, 245, 480, 320), dot(480, 320),
    indH('Lout', bom, 670, 130), wire(600, 130, 642, 130), wire(698, 130, 760, 130), dot(720, 130),
    capV('Cout', bom, 720, 220), wire(720, 130, 720, 200), wire(720, 240, 720, 320), dot(720, 320),
    loadR(800, 220, 130, 320), dot(800, 130), dot(800, 320),
    port(850, 130, 'VOUT'), wire(800, 130, 850, 130),
    ctrlIC(bom, 470, 360, ['g1', 'g2']),
  ].join(''))
}

// ── active-clamp forward: clamp leg Sc+Cc resets the core; synchronous rectifiers ────────
function acf(bom) {
  return svg(920, 400, [
    srcDC(60, 190), wire(60, 175, 60, 70, 340, 70), wire(60, 205, 60, 300, 760, 300),
    // main switch Q1 in series with the primary (VIN → Q1 → sw → T1 pri → gnd)
    mosfetV('Q1', bom, 320, 145, 'right', true), wire(320, 70, 320, 119), dot(320, 70), wire(320, 171, 320, 185),
    xfmr('T1', bom, 380, 195, { h: 90, labelDx: -18, labelDy: -22 }), wire(320, 185, 370, 185), dot(320, 185),
    wire(370, 235, 320, 235, 320, 300), dot(320, 300),
    // active-clamp leg: Sc (Vin → clamp node) in series with Cc (clamp node → switch node)
    mosfetV('Sc', bom, 220, 110, 'right', true), wire(220, 70, 220, 84), dot(220, 70),
    capV('Cc', bom, 220, 170, 'left'), wire(220, 136, 220, 150),
    wire(220, 190, 260, 190, 260, 185, 320, 185),
    sig(294, 145, 'g1'), sig(194, 110, 'gc'),
    // secondary synchronous rectifiers SRfwd (series) + SRfw (freewheel)
    wire(390, 155, 440, 155, 440, 120, 474, 120), mosfetH('SRfwd', bom, 500, 120, true), wire(526, 120, 570, 120), dot(570, 120),
    mosfetV('SRfw', bom, 570, 195, 'right', true), wire(570, 120, 570, 169), wire(570, 221, 570, 300), dot(570, 300),
    wire(390, 235, 440, 235, 440, 300), dot(440, 300),
    sig(500, 146, 'sr1', 'down'), sig(544, 195, 'sr2'),
    indH('Lout', bom, 640, 120), wire(570, 120, 612, 120), wire(668, 120, 750, 120), dot(700, 120),
    capV('Cout', bom, 700, 210), wire(700, 120, 700, 190), wire(700, 230, 700, 300), dot(700, 300),
    loadR(770, 210, 120, 300), dot(770, 120), dot(770, 300),
    port(830, 120, 'VOUT'), wire(770, 120, 830, 120),
    ctrlIC(bom, 160, 355, ['g1', 'gc', 'sr1', 'sr2']),
  ].join(''))
}

// ── isolated buck (Fly-Buck): sync buck whose inductor is the transformer primary ────────
function isolatedBuck(bom) {
  return svg(900, 400, [
    srcDC(60, 180), wire(60, 165, 60, 70, 230, 70), wire(60, 195, 60, 300, 420, 300),
    // sync half bridge QS1/QS2 (refs right, gate flags left)
    mosfetV('QS1', bom, 230, 110, 'right', true), wire(230, 70, 230, 84), dot(230, 70), wire(230, 136, 230, 180),
    mosfetV('QS2', bom, 230, 250, 'right', true), wire(230, 180, 230, 224), wire(230, 276, 230, 300), dot(230, 300),
    dot(230, 180), sig(204, 110, 'g1'), sig(204, 250, 'g2'),
    // discrete antiparallel diodes across the sync FETs (DS1: sw→vin, DS2: gnd→sw) — own column left of QS
    wire(230, 70, 160, 70), diode('DS1', bom, 160, 125, 'up', 'left'), wire(160, 70, 160, 105),
    wire(160, 145, 160, 180, 230, 180),
    diode('DS2', bom, 160, 235, 'up', 'left'), wire(160, 180, 160, 215), dot(160, 180),
    wire(160, 255, 160, 300), dot(160, 300),
    xfmr('T1', bom, 360, 185, { h: 90, labelDy: -24 }),
    wire(230, 180, 300, 180, 300, 140, 350, 140), // sw node → primary top
    // primary buck rail = main output (Vpri)
    wire(350, 230, 420, 230), dot(420, 230),
    capV('Cpri', bom, 420, 265, 'left'), wire(420, 230, 420, 245), wire(420, 285, 420, 300), dot(420, 300),
    port(490, 230, 'VOUT'), wire(420, 230, 490, 230),
    // isolated secondary rail: own return, single flyback rectifier + preload/bleed Rsec
    wire(370, 140, 370, 118, 760, 118),
    wire(370, 230, 370, 258, 600, 258), diode('Dsec', bom, 620, 258, 'right'), wire(640, 258, 680, 258), dot(680, 258),
    resV('Rsec', bom, 645, 188, 'left'), wire(645, 118, 645, 168), dot(645, 118), wire(645, 208, 645, 258), dot(645, 258),
    capV('Csec', bom, 680, 188), wire(680, 168, 680, 118), dot(680, 118), wire(680, 208, 680, 258),
    port(760, 258, 'VISO'), wire(680, 258, 760, 258), port(760, 118, 'ISO-RTN', 'end'),
    ctrlIC(bom, 140, 350, ['g1', 'g2']),
  ].join(''))
}

// ── isolated buck-boost: single-switch inverting flyback with a second isolated rail ─────
function isolatedBuckBoost(bom) {
  return svg(860, 360, [
    srcDC(60, 180), wire(60, 165, 60, 70, 280, 70), wire(60, 195, 60, 300, 700, 300),
    mosfetV('QS1', bom, 210, 100, 'right', true), wire(210, 70, 210, 74), dot(210, 70), wire(210, 126, 210, 140, 280, 140),
    xfmr('T1', bom, 290, 180, { h: 80, opp: true, labelDy: -24 }), dot(280, 140),
    wire(280, 220, 280, 300), dot(280, 300),
    // inverting primary rail via Dpri
    diode('Dpri', bom, 170, 180, 'up', 'left'), wire(170, 160, 170, 140, 280, 140), wire(170, 200, 170, 240, 150, 240),
    capV('Cpri', bom, 150, 270, 'left'), wire(150, 240, 150, 250), dot(150, 240), wire(150, 290, 150, 300), dot(150, 300),
    port(120, 240, 'VOUT(−)', 'end'), wire(150, 240, 120, 240),
    // isolated secondary rail (shares primary ground per the model) + preload/bleed Rsec
    wire(300, 140, 300, 110, 660, 110, 660, 300), dot(660, 300),
    wire(300, 220, 300, 250, 520, 250), diode('Dsec', bom, 540, 250, 'right'), wire(560, 250, 600, 250), dot(600, 250),
    resV('Rsec', bom, 565, 178, 'left'), wire(565, 110, 565, 158), dot(565, 110), wire(565, 198, 565, 250), dot(565, 250),
    capV('Csec', bom, 600, 178), wire(600, 158, 600, 110), dot(600, 110), wire(600, 198, 600, 250),
    port(690, 250, 'VISO'), wire(600, 250, 690, 250),
    sig(184, 100, 'g1'), ctrlIC(bom, 140, 330, ['g1']),
  ].join(''))
}

// Four-winding transformer for the Weinberg / dual-inductor push-pull: two primary half-windings
// (left, each fed by its own coupled-inductor winding — NOT joined at a shared tap) and two secondary
// half-windings (right, center-tapped output). Returns terminal coords keyed by winding:
//   pA{top,bot} pB{top,bot} sC{top,bot} sD{top,bot}, plus the svg body under data-ref=ref.
function xfmr4(ref, bom, x, y) {
  const cL = x - 2, cR = x + 2, top = y - 100, bot = y + 100
  // winding vertical extents (2 stacked coils per side, gap at the core middle)
  const yA = [top + 10, top + 80], yB = [bot - 80, bot - 10]
  const body =
    P(coilV(x - 10, yA[0], 3, (yA[1] - yA[0]) / 3, 8.4, -1)) + P(coilV(x - 10, yB[0], 3, (yB[1] - yB[0]) / 3, 8.4, -1)) +
    P(coilV(x + 10, yA[0], 3, (yA[1] - yA[0]) / 3, 8.4, 1)) + P(coilV(x + 10, yB[0], 3, (yB[1] - yB[0]) / 3, 8.4, 1)) +
    P(`M ${cL} ${top} L ${cL} ${bot}`, 'sch-wire') + P(`M ${cR} ${top} L ${cR} ${bot}`, 'sch-wire') +
    `<circle class="sch-fill" cx="${x - 7}" cy="${yA[0] + 5}" r="2.3"/>` +
    `<circle class="sch-fill" cx="${x + 7}" cy="${yA[0] + 5}" r="2.3"/>`
  const el = hot(ref, bom, [x - 22, top - 6, 44, bot - top + 12], body, [x, top - 12, 'middle'])
  return { el, pA: { top: [x - 10, yA[0]], bot: [x - 10, yA[1]] }, pB: { top: [x - 10, yB[0]], bot: [x - 10, yB[1]] },
           sC: { top: [x + 10, yA[0]], bot: [x + 10, yA[1]] }, sD: { top: [x + 10, yB[0]], bot: [x + 10, yB[1]] } }
}

// ── Weinberg (dual-inductor / double-coupled current-fed push-pull): L1's TWO coupled windings each
//    feed a SEPARATE push-pull primary half through its own DCR loop-breaker (Rdcra/Rdcrb) — they are
//    NOT joined at a shared center tap. Each primary half is switched to ground by S1/S2. ────────────
function weinberg(bom) {
  const gy = 380
  const T = xfmr4('T1', bom, 560, 230)
  const [pAt, pAb] = [T.pA.top, T.pA.bot], [pBt, pBb] = [T.pB.top, T.pB.bot]
  const [sCt, sCb] = [T.sC.top, T.sC.bot], [sDt, sDb] = [T.sD.top, T.sD.bot]
  return svg(1000, 440, [
    srcDC(60, 210), wire(60, 195, 60, 90, 190, 90), wire(60, 225, 60, gy, 900, gy), // continuous ground/output-return rail
    // coupled input choke L1 (two windings), both fed from VIN+; each winding returns through its DCR
    // loop-breaker to a DIFFERENT primary half of T1 (the defining dual-inductor structure)
    xfmr('L1', bom, 210, 170, { h: 80, labelDx: -18, labelDy: -20 }),
    wire(200, 130, 200, 90), dot(200, 90), wire(220, 130, 220, 90), dot(220, 90),
    wire(200, 210, 200, 250, 250, 250), resH('Rdcra', bom, 280, 250, 'above'), wire(310, 250, pAb[0], 250, pAb[0], pAb[1]), // L1a → Rdcra → pri A bottom
    wire(220, 210, 220, 320, 250, 320), resH('Rdcrb', bom, 280, 320, 'below'), wire(310, 320, pBt[0], 320, pBt[0], pBt[1]), // L1b → Rdcrb → pri B top
    T.el,
    // each primary half's outer end → its own switch → ground
    mosfetV('S1', bom, 470, 180, 'right', true), wire(pAt[0], pAt[1], 470, pAt[1], 470, 154), wire(470, 206, 470, gy), dot(470, gy),
    mosfetV('S2', bom, 470, 330, 'right', true), wire(pBb[0], pBb[1], 470, pBb[1], 470, 304), wire(470, 356, 470, gy),
    gnd(430, gy), sig(444, 180, 'g1'), sig(444, 330, 'g2'),
    // center-tapped full-wave secondary: outer ends → Dpos / Dneg, inner ends → CT (ground) → Cout ∥ load
    wire(sCt[0], sCt[1], 600, sCt[1], 600, 110, 640, 110), diode('Dpos', bom, 660, 110, 'right'), wire(680, 110, 740, 110), dot(740, 110),
    wire(sDb[0], sDb[1], 600, sDb[1], 600, 350, 640, 350), diode('Dneg', bom, 660, 350, 'right', 'below'), wire(680, 350, 740, 350), wire(740, 350, 740, 110),
    wire(sCb[0], sCb[1], 610, sCb[1], 610, sDt[1], sDt[0], sDt[1]), wire(610, (sCb[1] + sDt[1]) / 2, 610, gy), dot(610, gy), // CT → gnd
    wire(740, 110, 900, 110), dot(820, 110),
    capV('Cout', bom, 820, 245), wire(820, 110, 820, 225), wire(820, 265, 820, gy), dot(820, gy),
    loadR(900, 245, 110, gy), dot(900, 110), dot(900, gy),
    port(960, 110, 'VOUT'), wire(900, 110, 960, 110),
    ctrlIC(bom, 620, 60, ['g1', 'g2']),
  ].join(''))
}

// ── asymmetric half-bridge: Q1/Q2 half bridge + DC-blocking cap Cb feed the primary ──────
function ahb(bom, variant = 'fullBridge') {
  const tx = 540, ty = 200, h = 90
  return svg(1120, 420, [
    srcDC(60, 190), wire(60, 175, 60, 80, 300, 80), wire(60, 205, 60, 330, 200, 330),
    // Q1/Q2 half bridge — ref right, gate flag left (no collision)
    mosfetV('Q1', bom, 180, 128, 'right', true), wire(180, 102, 180, 80), dot(180, 80), wire(180, 154, 180, 200), dot(180, 200),
    mosfetV('Q2', bom, 180, 250, 'right', true), wire(180, 200, 180, 224), wire(180, 276, 180, 330), dot(180, 330), gnd(120, 330),
    sig(154, 128, 'g1'), sig(154, 250, 'g2'),
    // DC-blocking cap Cb from the Vin rail into the primary (cb_mid) — long lead runs to T1
    capV('Cb', bom, 300, 122, 'left'), wire(300, 80, 300, 102), dot(300, 80), wire(300, 142, 300, 155, tx - 10, 155),
    xfmr('T1', bom, tx, ty, { h, ct: ctOpt(variant), labelDy: -24 }),
    wire(tx - 10, 245, tx - 10, 300, 220, 300, 220, 200, 180, 200), // primary return (sw_net) → sw node
    // real RC damper across the primary (cb_mid → Rdmp → Cdmp → sw_net) — own column between Cb and T1
    dot(430, 155), resV('Rdmp', bom, 430, 190, 'right'), wire(430, 155, 430, 170),
    capV('Cdmp', bom, 430, 258, 'right'), wire(430, 210, 430, 238), wire(430, 278, 430, 300), dot(430, 300),
    ...secondaryFor(bom, tx, ty, h, variant, bridgeRefs(variant)),
    ctrlIC(bom, 120, 45, ['g1', 'g2']),
    icBox('UDR', bom, 960, 360, 64, 44, [], ['sr'], 'SR DRV'),
  ].join(''))
}

// ── phase-shifted full bridge: four switches, series Lr, full-bridge secondary ───────────
function psfb(bom, variant = 'fullBridge') {
  const tx = 330, ty = 195, h = 90
  return svg(1040, 430, [
    srcDC(60, 175), wire(60, 160, 60, 80, 320, 80), wire(60, 190, 60, 320, 280, 320), // ground rail spans both legs
    mosfetV('QA', bom, 150, 128, 'right', true), wire(150, 102, 150, 80), dot(150, 80), wire(150, 154, 150, 200), dot(150, 200),
    mosfetV('QB', bom, 150, 246, 'right', true), wire(150, 200, 150, 220), wire(150, 272, 150, 320), dot(150, 320),
    mosfetV('QC', bom, 280, 128, 'right', true), wire(280, 102, 280, 80), dot(280, 80), wire(280, 154, 280, 195),
    mosfetV('QD', bom, 280, 246, 'right', true), wire(280, 195, 280, 220), wire(280, 272, 280, 320), dot(280, 320), gnd(110, 320),
    // leg-A mid → series Lr → transformer primary; primary return → leg-C mid
    wire(150, 200, 200, 200), indH('Lr', bom, 228, 200), wire(256, 200, 300, 200, 300, 150),
    xfmr('T1', bom, tx, ty, { h, ct: ctOpt(variant), labelDy: -24 }),
    wire(tx - 10, 240, tx - 10, 300, 280, 300, 280, 195), dot(280, 195),
    // real RC snubber between the two leg midpoints (midA → Crc_pri → Rrc_pri → midC), per psfbCell
    dot(178, 200), wire(178, 200, 178, 240), capV('Crc_pri', bom, 178, 260, 'right'),
    wire(178, 280, 178, 300, 200, 300), resH('Rrc_pri', bom, 220, 300, 'below'),
    wire(240, 300, 280, 300), dot(280, 300),
    // secondary bleed/balance resistors from each winding end to the secondary return (Rbsa/Rbsb) —
    // dropped low so labels clear the transformer winding
    dot(390, 195), wire(390, 195, 390, 285), resV('Rbsa', bom, 390, 305, 'left'),
    wire(390, 325, 420, 325),
    dot(534, 240), wire(534, 240, 590, 240, 590, 285), resV('Rbsb', bom, 590, 305, 'right'),
    wire(590, 325, 560, 325), dot(560, 325),
    ...secondaryFor(bom, tx, ty, h, variant, bridgeRefs(variant)),
    sig(124, 128, 'gA'), sig(124, 246, 'gB'), sig(254, 128, 'gC'), sig(254, 246, 'gD'),
    ctrlIC(bom, 90, 385, ['gA', 'gB', 'gC', 'gD']),
    icBox('UDR', bom, 900, 390, 64, 44, [], ['sr'], 'SR DRV'),
  ].join(''))
}

// ── phase-shifted half bridge (3-level NPC): split caps + clamp diodes + series Lr ───────
function pshb(bom, variant = 'fullBridge') {
  const tx = 540, ty = 240, h = 90
  return svg(1140, 460, [
    srcDC(60, 180), wire(60, 165, 60, 70, 150, 70), wire(60, 195, 60, 340, 200, 340), gnd(110, 340), // primary return
    // split input caps CsHi / CsLo about the neutral (mid)
    capV('CsHi', bom, 150, 100, 'left'), wire(150, 70, 150, 80), dot(150, 70), wire(150, 120, 150, 170), dot(150, 170),
    capV('CsLo', bom, 150, 260, 'left'), wire(150, 170, 150, 240), wire(150, 280, 150, 340), dot(150, 340),
    // NPC stack S1..S4
    mosfetV('S1', bom, 250, 100, 'right', true), wire(250, 74, 250, 70, 150, 70), wire(250, 126, 250, 140), dot(250, 140),
    mosfetV('S2', bom, 250, 180, 'right', true), wire(250, 154, 250, 140), wire(250, 206, 250, 225), dot(250, 225),
    mosfetV('S3', bom, 250, 270, 'right', true), wire(250, 244, 250, 225), wire(250, 296, 250, 310), dot(250, 310),
    mosfetV('S4', bom, 250, 350, 'right', true), wire(250, 324, 250, 310), wire(250, 376, 250, 340, 200, 340),
    // clamp diodes tie inner nodes to the neutral (mid)
    diode('DC1', bom, 200, 140, 'left'), wire(180, 140, 150, 140, 150, 170), wire(220, 140, 250, 140),
    diode('DC2', bom, 200, 310, 'left', 'below'), wire(180, 310, 150, 310, 150, 170), wire(220, 310, 250, 310),
    // stack output (bridge_a) → series Lr → primary; primary return → neutral (mid)
    wire(250, 225, 300, 225), indH('Lr', bom, 340, 225), wire(368, 225, 440, 225, 440, 195, tx - 10, 195),
    xfmr('T1', bom, tx, ty, { h, ct: ctOpt(variant), labelDy: -24 }),
    wire(tx - 10, 285, tx - 10, 300, 150, 300, 150, 170),
    // real RC snubber across the primary (pri_x → Crc_pri → Rrc_pri → neutral) — own column, labels
    // toward the clear gap before T1
    dot(450, 195), wire(450, 195, 450, 215), capV('Crc_pri', bom, 450, 235, 'right'), wire(450, 255, 450, 262),
    resV('Rrc_pri', bom, 450, 282, 'right'), wire(450, 302, 450, 300), dot(450, 300),
    ...secondaryFor(bom, tx, ty, h, variant, bridgeRefs(variant)),
    sig(224, 100, 'g1'), sig(224, 180, 'g2'), sig(224, 270, 'g3'), sig(224, 350, 'g4'),
    ctrlIC(bom, 90, 400, ['g1', 'g2', 'g3', 'g4']),
    icBox('UDR', bom, 1000, 410, 64, 44, [], ['sr'], 'SR DRV'),
  ].join(''))
}

// ── series-resonant (SRC): half bridge + split bus + series Cr–Lr tank + CT secondary ───
// Same half-bridge + split-bus front end as LLC; the only difference is a bare series Cr–Lr tank.
function src(bom, variant = 'centerTapped') {
  const top = 70, gy = 300, tx = 620, ty = 185, h = 80
  const hb = halfBridgeSplitBus(bom, top, gy)
  const [swx, swy] = hb.sw
  return svg(1060, 380, [
    ...hb.els,
    capH('Cr', bom, 380, swy), wire(swx, swy, 360, swy),
    indH('Lr', bom, 480, swy), wire(400, swy, 452, swy), wire(508, swy, 540, swy, 540, ty - h / 2),
    xfmr('T1', bom, tx, ty, { h, ct: ctOpt(variant), labelDx: -44 }),
    wire(tx - 10, ty + h / 2, tx - 10, 335, 220, 335, 220, swy, hb.msplit, swy),
    ...resonantSecondary(bom, tx, ty, h, variant),
    ctrlIC(bom, 130, 350, ['g1', 'g2']),
  ].join(''))
}

// Secondary synchronous-rectifier full bridge for CLLC / CLLLC (drawn as MOSFETs). The two
// tank output nodes enter at (nx, ny) [node_c] and (nx, ny2) [node_d]; refs name the SR FETs.
function srBridgeOut(bom, nx, ny, ny2, refs) {
  const yP = 80, yN = 300, cx = nx + 300, lx = cx + 100
  const xA = nx + 80, xB = nx + 190
  return [
    wire(nx, ny, xA, ny), dot(xA, ny),
    wire(nx, ny2, nx + 30, ny2, nx + 30, 330, xB + 48, 330, xB + 48, ny2, xB, ny2), dot(xB, ny2),
    mosfetV(refs[0], bom, xA, 130, 'right', true), wire(xA, 104, xA, yP), wire(xA, 156, xA, ny),
    mosfetV(refs[1], bom, xA, 250, 'right', true), wire(xA, 224, xA, ny), wire(xA, 276, xA, yN),
    mosfetV(refs[2], bom, xB, 130, 'right', true), wire(xB, 104, xB, yP), wire(xB, 156, xB, ny2),
    mosfetV(refs[3], bom, xB, 250, 'right', true), wire(xB, 224, xB, ny2), wire(xB, 276, xB, yN),
    dot(xA, yP), dot(xB, yP), wire(xA, yP, lx, yP), // VOUT+ rail spans the bridge → Cout → load
    wire(xA, yN, lx, yN),
    dot(cx, yP), capV('Cout', bom, cx, 190), wire(cx, yP, cx, 170), wire(cx, 210, cx, yN), dot(cx, yN),
    loadR(lx, 190, yP, yN), dot(lx, yP), dot(lx, yN),
    port(lx + 60, yP, 'VOUT'), wire(lx, yP, lx + 60, yP),
    gnd(nx + 140, yN),   // isolated secondary return reference
  ]
}

// ── CLLC: full bridge + primary tank Cr1–Lr1, symmetric secondary tank Lr2–Cr2, SR bridge ─
function cllc(bom) {
  return svg(1040, 430, [
    srcDC(60, 175), wire(60, 160, 60, 80, 300, 80), wire(60, 190, 60, 320, 280, 320), // ground rail spans both legs
    mosfetV('Q1', bom, 150, 128, 'right', true), wire(150, 102, 150, 80), dot(150, 80), wire(150, 154, 150, 205), dot(150, 205),
    mosfetV('Q2', bom, 150, 250, 'right', true), wire(150, 205, 150, 224), wire(150, 276, 150, 320), dot(150, 320),
    mosfetV('Q3', bom, 280, 128, 'right', true), wire(280, 102, 280, 80), dot(280, 80), wire(280, 154, 280, 205), // Q3.source → node_b (leg mid / T1.primary_end)
    mosfetV('Q4', bom, 280, 250, 'right', true), wire(280, 205, 280, 224), wire(280, 276, 280, 320), dot(280, 320), gnd(110, 320),
    // node_a → Cr1 → Lr1 → primary; primary return (node_b) → leg 3/4 mid
    wire(150, 205, 175, 205), capH('Cr1', bom, 200, 205), indH('Lr1', bom, 268, 205), wire(225, 205, 240, 205),
    wire(296, 205, 310, 205, 310, 150),
    xfmr('T1', bom, 340, 195, { h: 90, labelDy: -24 }),
    wire(330, 240, 330, 300, 280, 300, 280, 205), dot(280, 205),
    // secondary tank Lr2 → Cr2 into the SR bridge
    wire(350, 150, 380, 150), indH('Lr2', bom, 408, 150), wire(436, 150, 460, 150), capH('Cr2', bom, 486, 150), wire(508, 150, 540, 150),
    wire(350, 240, 540, 240),
    ...srBridgeOut(bom, 540, 150, 240, ['Qa', 'Qb', 'Qc', 'Qd']),
    sig(124, 128, 'g1'), sig(124, 250, 'g2'), sig(254, 128, 'g3'), sig(254, 250, 'g4'),
    sig(594, 130, 'sa'), sig(594, 250, 'sb'), sig(704, 130, 'sc'), sig(704, 250, 'sd'),
    icBox('U1', bom, 120, 380, 64, 80, ['g1', 'g2', 'g3', 'g4'], ['sa', 'sb', 'sc', 'sd'], 'PWM'),
  ].join(''))
}

// ── CLLLC: like CLLC but with a discrete secondary resonant inductor path (QE..QH SR) ────
function clllc(bom) {
  return svg(1040, 430, [
    srcDC(60, 175), wire(60, 160, 60, 80, 300, 80), wire(60, 190, 60, 320, 280, 320), // ground rail spans both legs
    mosfetV('Q1', bom, 150, 128, 'right', true), wire(150, 102, 150, 80), dot(150, 80), wire(150, 154, 150, 205), dot(150, 205),
    mosfetV('Q2', bom, 150, 250, 'right', true), wire(150, 205, 150, 224), wire(150, 276, 150, 320), dot(150, 320),
    mosfetV('Q3', bom, 280, 128, 'right', true), wire(280, 102, 280, 80), dot(280, 80), wire(280, 154, 280, 205), // Q3.source → node_b (leg mid / T1.primary_end)
    mosfetV('Q4', bom, 280, 250, 'right', true), wire(280, 205, 280, 224), wire(280, 276, 280, 320), dot(280, 320), gnd(110, 320),
    wire(150, 205, 175, 205), capH('Cr1', bom, 200, 205), indH('Lr1', bom, 268, 205), wire(225, 205, 240, 205),
    wire(296, 205, 310, 205, 310, 150),
    xfmr('T1', bom, 340, 195, { h: 90, labelDy: -24 }),
    wire(330, 240, 330, 300, 280, 300, 280, 205), dot(280, 205),
    // secondary tank Lr2 → Cr2, then the SR current-sense shunt Rsense into the bridge (senseP/senseM
    // feed the SR controller, per the clllcPower/srControl nets)
    wire(350, 150, 380, 150), indH('Lr2', bom, 408, 150), wire(436, 150, 450, 150),
    capH('Cr2', bom, 470, 150), resH('Rsense', bom, 510, 150, 'above'),
    wire(530, 150, 540, 150),
    sig(490, 150, 'sP', 'down'), sig(530, 150, 'sM', 'down'),
    wire(350, 240, 540, 240),
    ...srBridgeOut(bom, 540, 150, 240, ['QE', 'QF', 'QG', 'QH']),
    sig(124, 128, 'g1'), sig(124, 250, 'g2'), sig(254, 128, 'g3'), sig(254, 250, 'g4'),
    sig(594, 130, 'gA'), sig(594, 250, 'gB'), sig(704, 130, 'gB'), sig(704, 250, 'gA'),
    ctrlIC(bom, 120, 380, ['g1', 'g2', 'g3', 'g4']),
    icBox('SR', bom, 450, 390, 70, 64, ['sP', 'sM'], ['gA', 'gB'], 'SR CTRL'),
  ].join(''))
}

// ── boost PFC: full-bridge line rectifier feeding a boost cell (L, SW, D5) ────────────────
function pfc(bom) {
  return svg(880, 520, [
    srcAC(70, 200, 'VAC'),
    // full-bridge line rectifier: acLine (top) / acNeutral (bottom); busP=150, gnd=320, mid=235
    wire(70, 185, 120, 185, 120, 235, 180, 235),
    wire(70, 215, 95, 215, 95, 360, 290, 360, 290, 235, 250, 235), // acNeutral wraps under the bridge
    dot(180, 235), dot(250, 235),
    diode('D1', bom, 180, 197, 'up'), wire(180, 177, 180, 150), wire(180, 217, 180, 235),
    diode('D3', bom, 180, 273, 'up', 'left'), wire(180, 253, 180, 235), wire(180, 293, 180, 320),
    diode('D2', bom, 250, 197, 'up'), wire(250, 177, 250, 150), wire(250, 217, 250, 235),
    diode('D4', bom, 250, 273, 'up'), wire(250, 253, 250, 235), wire(250, 293, 250, 320),
    dot(180, 150), dot(250, 150), wire(180, 150, 250, 150),
    wire(180, 320, 660, 320), gnd(180, 320),
    // Rref bleeds the acNeutral reference to ground (per pfcPower nets)
    dot(130, 360), resV('Rref', bom, 130, 340, 'left'), wire(130, 320, 180, 320), dot(130, 320),
    // boost cell: rectified bus → Rsense (current shunt) → L → switch node; SW shunt, D5 into the bus cap
    wire(250, 150, 260, 150), resH('Rsense', bom, 280, 150, 'above'),
    sig(255, 150, 'busP', 'down'), sig(303, 150, 'nL', 'down'),
    indH('L', bom, 340, 150), wire(300, 150, 312, 150), wire(368, 150, 410, 150), dot(410, 150),
    mosfetV('SW', bom, 410, 225), wire(410, 150, 410, 199), wire(410, 251, 410, 320), dot(410, 320),
    diode('D5', bom, 480, 150, 'right'), wire(410, 150, 460, 150), wire(500, 150, 600, 150), dot(560, 150),
    capV('Cout', bom, 560, 235), wire(560, 150, 560, 215), wire(560, 255, 560, 320), dot(560, 320),
    loadR(630, 235, 150, 320), dot(630, 150), dot(630, 320),
    port(690, 150, 'VBUS'), wire(630, 150, 690, 150),
    // output-voltage divider feeding the control law (vout → Rv1 → vs → Rv2 → gnd)
    dot(660, 150), wire(660, 150, 660, 170), resV('Rv1', bom, 660, 190, 'left'),
    sig(660, 226, 'vs', 'right'), wire(660, 210, 660, 242), resV('Rv2', bom, 660, 262, 'left'),
    wire(660, 282, 660, 320), dot(660, 320),
    sig(384, 225, 'g'),
    // ── control law (average-current-mode PFC), signal nets by label: vs → ∫/EA → ×busP → PWM vs nL ──
    txt(60, 405, 'CONTROL — average-current-mode PFC law', 'sch-blk', 'start'),
    icBox('Iv', bom, 160, 460, 60, 56, ['vs'], ['iv'], '∫'),
    icBox('Sgv', bom, 300, 460, 60, 56, ['iv', 'vs'], ['gv'], 'EA'),
    icBox('Mv', bom, 440, 460, 60, 56, ['busP', 'gv'], ['vth'], '×'),
    icBox('Cmp', bom, 580, 460, 60, 56, ['nL', 'vth'], ['g'], 'PWM'),
    ctrlIC(bom, 740, 460, ['g'], 'U1', 'CTRL'),
  ].join(''))
}

// ── Vienna: 3-phase 3-level rectifier — three boost legs into a split DC bus ─────────────
function viennaLeg(bom, x, ph) {
  // vertical leg: phase-in → Rs (current shunt) → L → node X, with Dp↑ to busP and Dn↑ from busN,
  // plus the bidirectional midpoint switch (SW+SQ, common node) clamping X to the neutral rail.
  const busP = 60, neu = 270, busN = 340, X = 130, sx = x + 48
  return [
    port(x - 48, 462, ph, 'start'), wire(x - 48, 462, x - 48, 455),
    resV(`Rs${ph}`, bom, x - 48, 435, 'left'), wire(x - 48, 415, x - 48, 403),
    sig(x - 48, 410, `nl${ph}`, 'right'),
    indV(`L${ph}`, bom, x - 48, 375, 'left'), wire(x - 48, 347, x - 48, X, x, X), dot(x, X),
    diode(`Dp${ph}`, bom, x, 95, 'up'), wire(x, 75, x, busP), wire(x, 115, x, X), dot(x, busP),
    diode(`Dn${ph}`, bom, x, 300, 'up'), wire(x, 280, x, X), wire(x, 320, x, busN), dot(x, busN),
    wire(x, X, sx, X),
    mosfetV(`SW${ph}`, bom, sx, 160, 'right', true), wire(sx, 134, sx, X), wire(sx, 186, sx, 194),
    mosfetV(`SQ${ph}`, bom, sx, 220, 'right', true), wire(sx, 246, sx, neu), dot(sx, neu),
    sig(sx - 26, 160, `g${ph}`), sig(sx - 26, 220, `g${ph}`),
  ]
}
function vienna(bom) {
  const cols = [{ x: 155, ph: 'a' }, { x: 365, ph: 'b' }, { x: 575, ph: 'c' }]
  // per-phase current-loop block chain (Gvm/Sub/Sum/Add/Mul/Cmp), nets by label per viennaControl
  const phaseRow = (ph, y) => [
    icBox(`Gvm${ph}`, bom, 150, y, 60, 56, [ph, 'gcond'], [`gvp${ph}`], '×'),
    icBox(`Sub${ph}`, bom, 280, y, 60, 56, [`nl${ph}`, ph], [`nmp${ph}`], '−'),
    icBox(`Sum${ph}`, bom, 410, y, 60, 56, [`nmp${ph}`, `gvp${ph}`], [`err${ph}`], 'Σ'),
    icBox(`Add${ph}`, bom, 540, y, 60, 56, [`err${ph}`, 'bal'], [`errp${ph}`], '+'),
    icBox(`Mul${ph}`, bom, 670, y, 60, 56, [ph, `errp${ph}`], [`m${ph}`], '×'),
    icBox(`Cmp${ph}`, bom, 800, y, 60, 56, [`m${ph}`, 'gnd'], [`g${ph}`], 'PWM'),
  ].join('')
  return svg(940, 830, [
    wire(95, 60, 780, 60), wire(95, 270, 780, 270), wire(95, 340, 780, 340), // busP / neutral / busN
    ...cols.flatMap((c) => viennaLeg(bom, c.x, c.ph)),
    txt(110, 264, 'N', 'sch-port', 'end'),
    sig(770, 60, 'busP', 'up'), sig(770, 340, 'busN', 'down'),
    // split caps Cp (busP→neutral) and Cn (neutral→busN) + load across the full bus
    capV('Cp', bom, 720, 165, 'right'), wire(720, 60, 720, 145), dot(720, 60), wire(720, 185, 720, 270), dot(720, 270),
    capV('Cn', bom, 720, 305, 'right'), wire(720, 270, 720, 285), wire(720, 325, 720, 340), dot(720, 340),
    resV('Rload', bom, 790, 200, 'right'), wire(790, 60, 790, 180), wire(790, 220, 790, 340),
    dot(790, 60), dot(790, 340),
    port(850, 60, 'BUS+'), wire(790, 60, 850, 60), port(850, 340, 'BUS−'), wire(790, 340, 850, 340),
    // ── control law: bus-voltage loop + imbalance loop (shared), then one current loop per phase ──
    txt(60, 505, 'CONTROL — bus-voltage + balance loops, per-phase current loops', 'sch-blk', 'start'),
    icBox('Svs', bom, 150, 560, 60, 56, ['busP', 'busN'], ['vbus'], 'G'),
    icBox('Ivolt', bom, 280, 560, 60, 56, ['vbus'], ['vint'], '∫'),
    icBox('Sg', bom, 410, 560, 60, 56, ['vint', 'vbus'], ['gcond'], 'EA'),
    icBox('Simb', bom, 540, 560, 60, 56, ['busP', 'busN'], ['imb'], '−'),
    icBox('Ibal', bom, 670, 560, 60, 56, ['imb'], ['bal'], 'G'),
    ctrlIC(bom, 810, 560, ['ga', 'gb', 'gc'], 'U1', 'CTRL'),
    phaseRow('a', 650), phaseRow('b', 720), phaseRow('c', 790),
  ].join(''))
}

const LAYOUTS = {
  buck, boost, sepic, cuk, zeta, fsbb, flyback, forward,
  push_pull: pushPull, llc, dab,
  two_switch_forward: twoSwitchForward, acf, isolated_buck: isolatedBuck,
  isolated_buck_boost: isolatedBuckBoost, weinberg, ahb, psfb, pshb,
  src, cllc, clllc, pfc, vienna,
}

export function hasSchematic(topologyId) {
  return topologyId in LAYOUTS
}

export function renderSchematic(topologyId, bomRows, variant) {
  const fn = LAYOUTS[topologyId]
  if (!fn) return null
  const rows = bomRows ?? []
  const bom = new Map(rows.map((r) => [r.ref, r]))
  const main = fn(bom, variant)
  // Parity guard: every BOM component must be drawn (no hidden parts, no auxiliary strip).
  // A miss is a layout bug — surface it loudly in the console so it gets fixed, never swallowed.
  const drawn = new Set([...main.matchAll(/data-ref="([^"]+)"/g)].map((mm) => mm[1]))
  const missing = rows.filter((r) => !drawn.has(r.ref)).map((r) => r.ref)
  if (missing.length)
    console.warn(`schematic '${topologyId}': BOM components not drawn: ${missing.join(', ')}`)
  return main
}

// Verification hook: render once with terminal recording on, returning { svg, pins } where pins is a
// list of { ref, pin, x, y } for every registered electrical terminal (MOSFET drain/source/gate, diode
// anode/cathode, ground symbols, external ports). Used by the netlist-vs-drawing connectivity checker.
export function collectPins(topologyId, bomRows, variant) {
  _pins.length = 0
  _rec = true
  let svg
  try { svg = renderSchematic(topologyId, bomRows, variant) } finally { _rec = false }
  return { svg, pins: _pins.slice() }
}
