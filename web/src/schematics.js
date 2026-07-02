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
  return hot(ref, bom, [x - 27, y - 26, 46, 52], body, lab)
}

// N-MOSFET on a horizontal rail: drain (x-26, y), source (x+26, y), gate below.
// Same ported geometry, transposed (u,v) -> (v,-u) so the channel sits under the rail.
function mosfetH(ref, bom, x, y) {
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
  return hot(ref, bom, [x - 26, y - 12, 52, 40], body, [x, y - 26, 'middle'])
}

// Diode centered at (x, y), leads spanning 40 along `dir` (current flow direction).
// Outline triangle + bar, proportions from Diode-COM-Standard (×0.267).
function diode(ref, bom, x, y, dir, labelSide = 'above') {
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
      ? labelSide === 'above' ? [x, y - 26, 'middle'] : [x, y + 24, 'middle']
      : labelSide === 'left' ? [x - 14, y - 2, 'end'] : [x + 14, y - 2, 'start']
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
  return `<circle class="sch-sym" cx="${x}" cy="${y}" r="3"/>` + txt(anchor === 'start' ? x + 8 : x - 8, y + 4, label, 'sch-port', anchor)
}

const svg = (w, h, inner) =>
  `<svg viewBox="0 0 ${w} ${h}" xmlns="http://www.w3.org/2000/svg" role="img">${inner}</svg>`

// ── topology layouts ───────────────────────────────────────────────────────

function buck(bom) {
  return svg(720, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 154, 80), wire(70, 165, 70, 220, 520, 220),
    mosfetH('Q1', bom, 180, 80),
    wire(206, 80, 320, 80), dot(240, 80),
    diode('D1', bom, 240, 150, 'up', 'right'), wire(240, 80, 240, 130), wire(240, 170, 240, 220), dot(240, 220),
    indH('L1', bom, 348, 80), wire(376, 80, 560, 80), dot(430, 80),
    capV('Cout', bom, 430, 150), wire(430, 80, 430, 130), wire(430, 170, 430, 220), dot(430, 220),
    loadR(520, 150, 80, 220), dot(520, 80), dot(520, 220),
    gnd(300, 220), port(600, 80, 'VOUT'), wire(560, 80, 600, 80),
  ].join(''))
}

function boost(bom) {
  return svg(720, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 122, 80), wire(70, 165, 70, 220, 520, 220),
    indH('L1', bom, 150, 80), wire(178, 80, 240, 80), dot(240, 80),
    mosfetV('Q1', bom, 240, 150), wire(240, 80, 240, 124), wire(240, 176, 240, 220), dot(240, 220),
    diode('D1', bom, 310, 80, 'right'), wire(240, 80, 290, 80), wire(330, 80, 560, 80), dot(430, 80),
    capV('Cout', bom, 430, 150), wire(430, 80, 430, 130), wire(430, 170, 430, 220), dot(430, 220),
    loadR(520, 150, 80, 220), dot(520, 80), dot(520, 220),
    gnd(300, 220), port(600, 80, 'VOUT'), wire(560, 80, 600, 80),
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
  ].join(''))
}

function cuk(bom) {
  return svg(760, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 122, 80), wire(70, 165, 70, 220, 580, 220),
    indH('L1', bom, 150, 80), wire(178, 80, 230, 80), dot(230, 80),
    mosfetV('Q1', bom, 230, 150), wire(230, 80, 230, 124), wire(230, 176, 230, 220), dot(230, 220),
    capH('C1', bom, 290, 80), wire(230, 80, 270, 80), wire(310, 80, 360, 80), dot(360, 80),
    diode('D1', bom, 360, 150, 'down', 'right'), wire(360, 80, 360, 130), wire(360, 170, 360, 220), dot(360, 220),
    indH('L2', bom, 428, 80), wire(360, 80, 400, 80), wire(456, 80, 620, 80), dot(500, 80),
    capV('Cout', bom, 500, 150), wire(500, 80, 500, 130), wire(500, 170, 500, 220), dot(500, 220),
    loadR(580, 150, 80, 220), dot(580, 80), dot(580, 220),
    gnd(300, 220), port(660, 80, 'VOUT (−)'), wire(620, 80, 660, 80),
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
  ].join(''))
}

function fsbb(bom) {
  return svg(760, 300, [
    srcDC(70, 150), wire(70, 135, 70, 80, 190, 80), wire(70, 165, 70, 220, 500, 220),
    mosfetV('Q1', bom, 190, 115, 'left'), wire(190, 80, 190, 89), wire(190, 141, 190, 150), dot(190, 150),
    mosfetV('Q2', bom, 190, 185, 'left'), wire(190, 150, 190, 159), wire(190, 211, 190, 220), dot(190, 220),
    indH('L', bom, 260, 150), wire(190, 150, 232, 150), wire(288, 150, 330, 150), dot(330, 150),
    mosfetV('Q3', bom, 330, 115), wire(330, 80, 330, 89), wire(330, 141, 330, 150),
    mosfetV('Q4', bom, 330, 185), wire(330, 150, 330, 159), wire(330, 211, 330, 220), dot(330, 220),
    wire(330, 80, 620, 80), dot(500, 80),
    capV('Cout', bom, 500, 150), wire(500, 80, 500, 130), wire(500, 170, 500, 220), dot(500, 220),
    wire(330, 220, 580, 220),
    loadR(580, 150, 80, 220), dot(580, 80),
    gnd(260, 220), port(660, 80, 'VOUT'), wire(620, 80, 660, 80),
  ].join(''))
}

function flyback(bom) {
  return svg(760, 320, [
    srcDC(70, 160), wire(70, 145, 70, 70, 240, 70), wire(70, 175, 70, 260, 250, 260),
    capV('Cin', bom, 150, 165, 'left'), wire(150, 70, 150, 145), wire(150, 185, 150, 260), dot(150, 70), dot(150, 260),
    xfmr('T1', bom, 260, 140, { opp: true, labelDy: -30 }), wire(240, 70, 250, 70, 250, 100), wire(250, 180, 250, 202),
    mosfetV('Q1', bom, 250, 228), wire(250, 254, 250, 260),
    gnd(180, 260),
    // secondary — flyback polarity: dot at the bottom, diode from the top end
    wire(270, 100, 270, 90, 340, 90),
    diode('D1', bom, 360, 90, 'right'), wire(380, 90, 620, 90), dot(470, 90),
    wire(270, 180, 270, 268, 620, 268), dot(470, 268),
    capV('Cout', bom, 470, 180), wire(470, 90, 470, 160), wire(470, 200, 470, 268),
    loadR(560, 180, 90, 268), dot(560, 90), dot(560, 268),
    port(660, 90, 'VOUT'), wire(620, 90, 660, 90), port(660, 268, 'RTN'), wire(620, 268, 660, 268),
  ].join(''))
}

function forward(bom) {
  return svg(800, 320, [
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
  ].join(''))
}

function pushPull(bom) {
  return svg(800, 340, [
    srcDC(60, 160), wire(60, 145, 60, 90, 230, 90, 230, 160, 246, 160), wire(60, 175, 60, 300, 200, 300),
    // center-tapped primary: each half drives its own switch, BOTH sources to the
    // primary ground rail (the previous sketch mis-chained Q1 source into Q2 drain).
    // Winding ends are left VERTICALLY; taps attach at the ct stubs.
    xfmr('T1', bom, 270, 160, { h: 120, ct: 'both', labelDy: -12 }),
    wire(260, 100, 260, 86, 200, 86, 200, 94),
    mosfetV('Q1', bom, 200, 120, 'left'),
    wire(200, 146, 200, 157, 140, 157, 140, 300), dot(140, 300),
    wire(260, 220, 260, 224, 200, 224),
    mosfetV('Q2', bom, 200, 250, 'right'),
    wire(200, 276, 200, 300), dot(200, 300),
    gnd(120, 300),
    // secondary full-wave rectifier: both windings' outer ends through diodes
    wire(280, 100, 280, 84, 360, 84),
    diode('Dtop', bom, 380, 84, 'right'), wire(400, 84, 460, 84), dot(460, 84),
    wire(280, 220, 280, 236, 360, 236),
    diode('Dbot', bom, 380, 236, 'right', 'below'), wire(400, 236, 460, 236), wire(460, 236, 460, 84),
    // center tap is the secondary return (isolated — kept off the primary rail's level)
    wire(294, 160, 310, 160, 310, 310, 650, 310),
    indH('Lout', bom, 510, 84), wire(460, 84, 482, 84), wire(538, 84, 680, 84), dot(590, 84),
    capV('Cout', bom, 590, 190), wire(590, 84, 590, 170), wire(590, 210, 590, 310), dot(590, 310),
    loadR(650, 190, 84, 310), dot(650, 84), dot(650, 310),
    port(710, 84, 'VOUT'), wire(680, 84, 710, 84),
  ].join(''))
}

function llc(bom) {
  return svg(800, 340, [
    srcDC(60, 165), wire(60, 150, 60, 80, 170, 80), wire(60, 180, 60, 280, 345, 280),
    mosfetV('Q1', bom, 170, 118, 'left'), wire(170, 80, 170, 92), wire(170, 144, 170, 165), dot(170, 165),
    mosfetV('Q2', bom, 170, 212, 'left'), wire(170, 165, 170, 186), wire(170, 238, 170, 280), dot(170, 280),
    gnd(120, 280),
    // resonant tank into the primary — winding terminals are entered/left VERTICALLY
    capH('Cr', bom, 225, 165), wire(170, 165, 205, 165),
    indH('Lr', bom, 300, 165), wire(245, 165, 272, 165), wire(328, 165, 345, 165, 345, 105, 370, 105, 370, 130),
    xfmr('T1', bom, 380, 170, { h: 80, ct: 'right', labelDx: -40 }),
    wire(370, 210, 370, 235, 345, 235, 345, 280),
    // center-tapped secondary, full-wave
    wire(390, 130, 390, 112, 460, 112),
    diode('D1', bom, 480, 112, 'right'), wire(500, 112, 560, 112), dot(560, 112),
    wire(390, 210, 390, 228, 460, 228),
    diode('D2', bom, 480, 228, 'right', 'below'), wire(500, 228, 560, 228), wire(560, 228, 560, 112),
    wire(404, 170, 415, 170, 415, 300, 668, 300),
    wire(560, 112, 700, 112), dot(610, 112),
    capV('Cout', bom, 610, 200), wire(610, 112, 610, 180), wire(610, 220, 610, 300), dot(610, 300),
    loadR(668, 200, 112, 300), dot(668, 112), dot(668, 300),
    port(724, 112, 'VOUT'), wire(700, 112, 724, 112),
  ].join(''))
}

function dab(bom) {
  return svg(840, 345, [
    srcDC(52, 160), wire(52, 145, 52, 78, 250, 78), wire(52, 175, 52, 262, 250, 262),
    // bridge A/B leg
    mosfetV('QA', bom, 140, 112, 'left', true), wire(140, 78, 140, 86), wire(140, 138, 140, 160), dot(140, 160), dot(140, 78),
    mosfetV('QB', bom, 140, 208, 'left', true), wire(140, 160, 140, 182), wire(140, 234, 140, 262), dot(140, 262),
    // bridge C/D leg
    mosfetV('QC', bom, 250, 112, 'right', true), wire(250, 78, 250, 86), wire(250, 138, 250, 160), dot(250, 160),
    mosfetV('QD', bom, 250, 208, 'right', true), wire(250, 160, 250, 182), wire(250, 234, 250, 262),
    gnd(90, 262),
    // series Lr from leg A midpoint into the primary; return to leg C midpoint
    wire(140, 160, 152, 160, 152, 300, 305, 300), indH('Lr', bom, 333, 300, 'below'),
    wire(361, 300, 380, 300, 380, 210),
    xfmr('T1', bom, 390, 170, { h: 80, labelDy: -24 }),
    wire(380, 130, 380, 100, 262, 100, 262, 148),
    wire(262, 148, 262, 160, 250, 160),
    // secondary bridge
    wire(400, 130, 400, 100, 530, 100, 530, 148), wire(530, 148, 530, 160, 542, 160),
    wire(400, 210, 400, 300, 528, 300, 528, 172),
    mosfetV('QE', bom, 542, 112, 'left', true), wire(542, 78, 542, 86), wire(542, 138, 542, 160), dot(542, 160), dot(542, 78),
    mosfetV('QF', bom, 542, 208, 'left', true), wire(542, 160, 542, 182), wire(542, 234, 542, 262),
    mosfetV('QG', bom, 640, 112, 'right', true), wire(640, 78, 640, 86), wire(640, 138, 640, 160), dot(640, 160),
    mosfetV('QH', bom, 640, 208, 'right', true), wire(640, 160, 640, 182), wire(640, 234, 640, 262), dot(640, 262),
    wire(528, 172, 528, 166, 628, 166)/* to leg G mid */, wire(628, 166, 640, 166, 640, 160),
    wire(542, 78, 640, 78), wire(542, 262, 640, 262),
    wire(640, 78, 760, 78), dot(700, 78), wire(640, 262, 760, 262), dot(700, 262),
    capV('Cout', bom, 700, 170), wire(700, 78, 700, 150), wire(700, 190, 700, 262),
    loadR(760, 170, 78, 262),
    port(788, 78, 'VOUT'), wire(760, 78, 782, 78),
  ].join(''))
}

// Secondary full-bridge diode rectifier + output LC + load, shared by AHB / PSFB / PSHB
// (all default to a 4-diode full-bridge secondary off a single transformer winding). The
// transformer's secondary terminals must sit at (sx, 150) top and (sx, 270) bottom. Emits
// Dr1..Dr4 and Lout/Cout with the exact refdes the TAS builders use.
function fwBridgeOut(bom, sx) {
  const topY = 150, botY = 270, yP = 120, yN = 300, yM = 210
  const xA = sx + 95, xB = sx + 165
  const cx = xB + 150, lx = xB + 220
  return [
    // AC leg-mid entries — secB wraps under the gnd rail (clean no-connect crossing) into leg B
    wire(sx, topY, sx + 40, topY, sx + 40, yM, xA, yM),
    wire(sx, botY, sx + 62, botY, sx + 62, 342, xB + 48, 342, xB + 48, yM, xB, yM),
    dot(xA, yM), dot(xB, yM),
    diode('Dr1', bom, xA, 165, 'up'), wire(xA, 145, xA, yP), wire(xA, 185, xA, yM),
    diode('Dr3', bom, xA, 255, 'up', 'left'), wire(xA, 235, xA, yM), wire(xA, 275, xA, yN),
    diode('Dr2', bom, xB, 165, 'up'), wire(xB, 145, xB, yP), wire(xB, 185, xB, yM),
    diode('Dr4', bom, xB, 255, 'up'), wire(xB, 235, xB, yM), wire(xB, 275, xB, yN),
    dot(xA, yP), dot(xB, yP), wire(xA, yP, xB, yP),
    wire(xA, yN, lx, yN), gnd(xA + 20, yN),
    wire(xB, yP, cx - 52, yP), indH('Lout', bom, cx - 24, yP), wire(cx + 4, yP, cx + 90, yP), dot(cx, yP),
    capV('Cout', bom, cx, 210), wire(cx, yP, cx, 190), wire(cx, 230, cx, yN), dot(cx, yN),
    loadR(lx, 210, yP, yN), dot(lx, yP), dot(lx, yN),
    port(lx + 60, yP, 'VOUT'), wire(lx, yP, lx + 60, yP),
  ]
}

// ── two-switch forward: Q1/Q2 sandwich the primary, D1/D2 clamp the reset to the bus ─────
function twoSwitchForward(bom) {
  return svg(820, 360, [
    srcDC(60, 190), wire(60, 175, 60, 70, 300, 70), wire(60, 205, 60, 300, 700, 300),
    mosfetV('Q1', bom, 280, 110), wire(280, 70, 280, 84), dot(280, 70), wire(280, 136, 280, 150),
    xfmr('T1', bom, 290, 190, { h: 80, labelDx: -18, labelDy: -22 }),
    dot(280, 150), dot(280, 230),
    mosfetV('Q2', bom, 280, 270), wire(280, 230, 280, 244), wire(280, 296, 280, 300),
    // reset/clamp diodes: D1 (gnd → primary top), D2 (primary bottom → Vin)
    diode('D1', bom, 200, 225, 'up', 'left'), wire(200, 205, 200, 150, 280, 150), wire(200, 245, 200, 300), dot(200, 300),
    diode('D2', bom, 235, 105, 'up', 'left'), wire(235, 85, 235, 70), dot(235, 70), wire(235, 125, 235, 230, 280, 230),
    // secondary: forward diode + freewheel into Lout / Cout
    wire(300, 150, 300, 120, 360, 120),
    diode('Dfwd', bom, 380, 120, 'right'), wire(400, 120, 440, 120), dot(440, 120),
    diode('Dfw', bom, 440, 190, 'up', 'right'), wire(440, 120, 440, 170), wire(440, 210, 440, 300), dot(440, 300),
    wire(300, 230, 300, 300), dot(300, 300),
    indH('Lout', bom, 510, 120), wire(440, 120, 482, 120), wire(538, 120, 620, 120), dot(590, 120),
    capV('Cout', bom, 590, 210), wire(590, 120, 590, 190), wire(590, 230, 590, 300), dot(590, 300),
    loadR(660, 210, 120, 300), dot(660, 120), dot(660, 300),
    port(720, 120, 'VOUT'), wire(660, 120, 720, 120),
  ].join(''))
}

// ── active-clamp forward: clamp leg Sc+Cc resets the core; synchronous rectifiers ────────
function acf(bom) {
  return svg(860, 360, [
    srcDC(60, 190), wire(60, 175, 60, 70, 330, 70), wire(60, 205, 60, 300, 720, 300),
    mosfetV('Q1', bom, 300, 145), wire(300, 70, 300, 119), dot(300, 70), wire(300, 171, 300, 185),
    xfmr('T1', bom, 310, 195, { h: 90, labelDx: -18, labelDy: -22 }), dot(300, 185),
    wire(300, 240, 300, 300), dot(300, 300),
    // clamp leg: Sc (Vin → clamp node) in series with Cc (clamp node → switch node)
    mosfetV('Sc', bom, 220, 110, 'left'), wire(220, 70, 220, 84), dot(220, 70),
    capV('Cc', bom, 220, 168), wire(220, 136, 220, 148),
    wire(220, 188, 260, 188, 260, 185, 300, 185),
    // secondary synchronous rectifiers SRfwd (series) + SRfw (freewheel)
    wire(320, 185, 320, 120, 374, 120), mosfetH('SRfwd', bom, 400, 120), wire(426, 120, 470, 120), dot(470, 120),
    mosfetV('SRfw', bom, 470, 195, 'right'), wire(470, 120, 470, 169), wire(470, 221, 470, 300), dot(470, 300),
    wire(320, 240, 320, 300), dot(320, 300),
    indH('Lout', bom, 540, 120), wire(470, 120, 512, 120), wire(568, 120, 650, 120), dot(620, 120),
    capV('Cout', bom, 620, 210), wire(620, 120, 620, 190), wire(620, 230, 620, 300), dot(620, 300),
    loadR(690, 210, 120, 300), dot(690, 120), dot(690, 300),
    port(750, 120, 'VOUT'), wire(690, 120, 750, 120),
  ].join(''))
}

// ── isolated buck (Fly-Buck): sync buck whose inductor is the transformer primary ────────
function isolatedBuck(bom) {
  return svg(860, 360, [
    srcDC(60, 180), wire(60, 165, 60, 70, 230, 70), wire(60, 195, 60, 300, 420, 300),
    mosfetV('QS1', bom, 230, 110), wire(230, 70, 230, 84), dot(230, 70), wire(230, 136, 230, 180),
    mosfetV('QS2', bom, 230, 250), wire(230, 180, 230, 224), wire(230, 276, 230, 300), dot(230, 300),
    dot(230, 180),
    xfmr('T1', bom, 310, 185, { h: 90, labelDy: -24 }),
    wire(230, 180, 265, 180, 265, 140, 300, 140), // sw node → primary top
    // primary buck rail = main output
    wire(300, 230, 360, 230), dot(360, 230),
    capV('Cpri', bom, 360, 265, 'left'), wire(360, 230, 360, 245), wire(360, 285, 360, 300), dot(360, 300),
    port(440, 230, 'VOUT'), wire(360, 230, 440, 230),
    // isolated secondary rail: own return, single flyback rectifier
    wire(320, 140, 320, 118, 700, 118),
    wire(320, 230, 320, 258, 540, 258), diode('Dsec', bom, 560, 258, 'right'), wire(580, 258, 620, 258), dot(620, 258),
    capV('Csec', bom, 620, 188), wire(620, 168, 620, 118), dot(620, 118), wire(620, 208, 620, 258),
    port(700, 258, 'VISO'), wire(620, 258, 700, 258), port(700, 118, 'ISO-RTN', 'end'),
  ].join(''))
}

// ── isolated buck-boost: single-switch inverting flyback with a second isolated rail ─────
function isolatedBuckBoost(bom) {
  return svg(860, 360, [
    srcDC(60, 180), wire(60, 165, 60, 70, 280, 70), wire(60, 195, 60, 300, 700, 300),
    mosfetV('QS1', bom, 280, 100), wire(280, 70, 280, 74), dot(280, 70), wire(280, 126, 280, 140),
    xfmr('T1', bom, 290, 180, { h: 80, opp: true, labelDx: 46, labelDy: -24 }), dot(280, 140),
    wire(280, 220, 280, 300), dot(280, 300),
    // inverting primary rail via Dpri
    diode('Dpri', bom, 200, 180, 'up', 'left'), wire(200, 160, 200, 140, 280, 140), wire(200, 200, 200, 240, 150, 240),
    capV('Cpri', bom, 150, 270, 'left'), wire(150, 240, 150, 250), dot(150, 240), wire(150, 290, 150, 300), dot(150, 300),
    port(120, 240, 'VOUT(−)', 'end'), wire(150, 240, 120, 240),
    // isolated secondary rail (shares primary ground per the model)
    wire(300, 140, 300, 110, 660, 110, 660, 300), dot(660, 300),
    wire(300, 220, 300, 250, 520, 250), diode('Dsec', bom, 540, 250, 'right'), wire(560, 250, 600, 250), dot(600, 250),
    capV('Csec', bom, 600, 178), wire(600, 158, 600, 110), dot(600, 110), wire(600, 198, 600, 250),
    port(690, 250, 'VISO'), wire(600, 250, 690, 250),
  ].join(''))
}

// ── Weinberg: current-fed push-pull with a coupled input choke feeding the primary tap ───
function weinberg(bom) {
  return svg(820, 360, [
    srcDC(60, 190),
    wire(60, 175, 60, 100, 122, 100), indH('L1', bom, 150, 100), wire(178, 100, 246, 100, 246, 190),
    wire(60, 205, 60, 320, 200, 320),
    xfmr('T1', bom, 300, 190, { h: 130, ct: 'both', labelDy: -12 }),
    // push-pull switches on the primary outer ends
    wire(290, 125, 290, 110, 200, 110, 200, 118), mosfetV('S1', bom, 200, 144, 'left'),
    wire(200, 170, 200, 182, 140, 182, 140, 320), dot(140, 320),
    wire(290, 255, 290, 260, 200, 260), mosfetV('S2', bom, 200, 286, 'right'),
    wire(200, 312, 200, 320), dot(200, 320), gnd(120, 320),
    // center-tapped full-wave secondary → Dpos / Dneg
    wire(310, 125, 310, 110, 390, 110), diode('Dpos', bom, 410, 110, 'right'), wire(430, 110, 490, 110), dot(490, 110),
    wire(310, 255, 310, 268, 390, 268), diode('Dneg', bom, 410, 268, 'right', 'below'), wire(430, 268, 490, 268), wire(490, 268, 490, 110),
    wire(324, 190, 340, 190, 340, 330, 690, 330),
    wire(490, 110, 720, 110), dot(600, 110),
    capV('Cout', bom, 600, 220), wire(600, 110, 600, 200), wire(600, 240, 600, 330), dot(600, 330),
    loadR(690, 220, 110, 330), dot(690, 110), dot(690, 330),
    port(750, 110, 'VOUT'), wire(720, 110, 750, 110),
  ].join(''))
}

// ── asymmetric half-bridge: Q1/Q2 half bridge + DC-blocking cap Cb feed the primary ──────
function ahb(bom) {
  return svg(920, 360, [
    srcDC(60, 175), wire(60, 160, 60, 80, 300, 80), wire(60, 190, 60, 320, 200, 320),
    mosfetV('Q1', bom, 170, 128, 'left'), wire(170, 102, 170, 80), dot(170, 80), wire(170, 154, 170, 190), dot(170, 190),
    mosfetV('Q2', bom, 170, 240, 'left'), wire(170, 190, 170, 214), wire(170, 266, 170, 320), dot(170, 320), gnd(120, 320),
    // DC-blocking cap Cb from the Vin rail down into the primary
    capV('Cb', bom, 270, 122, 'left'), wire(270, 80, 270, 102), dot(270, 80), wire(270, 142, 310, 142, 310, 150),
    xfmr('T1', bom, 320, 190, { h: 90, labelDy: -24 }),
    wire(310, 235, 310, 260, 210, 260, 210, 190, 170, 190), // primary return → sw node
    ...fwBridgeOut(bom, 330),
  ].join(''))
}

// ── phase-shifted full bridge: four switches, series Lr, full-bridge secondary ───────────
function psfb(bom) {
  return svg(980, 360, [
    srcDC(60, 175), wire(60, 160, 60, 80, 320, 80), wire(60, 190, 60, 320, 200, 320),
    mosfetV('QA', bom, 150, 128, 'left', true), wire(150, 102, 150, 80), dot(150, 80), wire(150, 154, 150, 200), dot(150, 200),
    mosfetV('QB', bom, 150, 246, 'left', true), wire(150, 200, 150, 220), wire(150, 272, 150, 320), dot(150, 320),
    mosfetV('QC', bom, 280, 128, 'right', true), wire(280, 102, 280, 80), dot(280, 80), wire(280, 154, 280, 195),
    mosfetV('QD', bom, 280, 246, 'right', true), wire(280, 195, 280, 220), wire(280, 272, 280, 320), dot(280, 320), gnd(110, 320),
    // leg-A mid → series Lr → transformer primary; primary return → leg-C mid
    wire(150, 200, 200, 200), indH('Lr', bom, 228, 200), wire(256, 200, 300, 200, 300, 150),
    xfmr('T1', bom, 330, 195, { h: 90, labelDy: -24 }),
    wire(320, 240, 320, 300, 280, 300, 280, 195), dot(280, 195),
    ...fwBridgeOut(bom, 340),
  ].join(''))
}

// ── phase-shifted half bridge (3-level NPC): split caps + clamp diodes + series Lr ───────
function pshb(bom) {
  return svg(980, 380, [
    srcDC(60, 180), wire(60, 165, 60, 70, 150, 70), wire(60, 195, 60, 340, 200, 340),
    // split input caps CsHi / CsLo about the neutral (mid)
    capV('CsHi', bom, 150, 100, 'left'), wire(150, 70, 150, 80), dot(150, 70), wire(150, 120, 150, 170), dot(150, 170),
    capV('CsLo', bom, 150, 260, 'left'), wire(150, 170, 150, 240), wire(150, 280, 150, 340), dot(150, 340),
    // NPC stack S1..S4
    mosfetV('S1', bom, 250, 100, 'left', true), wire(250, 74, 250, 70, 150, 70), wire(250, 126, 250, 140), dot(250, 140),
    mosfetV('S2', bom, 250, 180, 'left', true), wire(250, 154, 250, 140), wire(250, 206, 250, 225), dot(250, 225),
    mosfetV('S3', bom, 250, 270, 'left', true), wire(250, 244, 250, 225), wire(250, 296, 250, 310), dot(250, 310),
    mosfetV('S4', bom, 250, 350, 'left', true), wire(250, 324, 250, 310), wire(250, 376, 250, 340, 200, 340),
    // clamp diodes tie inner nodes to the neutral (mid)
    diode('DC1', bom, 200, 140, 'left'), wire(180, 140, 150, 140, 150, 170), wire(220, 140, 250, 140),
    diode('DC2', bom, 200, 310, 'left', 'below'), wire(180, 310, 150, 310, 150, 170), wire(220, 310, 250, 310),
    // stack output (bridge_a) → series Lr → primary; primary return → neutral (mid)
    wire(250, 225, 300, 225), indH('Lr', bom, 328, 225), wire(356, 225, 400, 225, 400, 195),
    xfmr('T1', bom, 430, 240, { h: 90, labelDy: -24 }),
    wire(420, 285, 420, 300, 150, 300, 150, 170),
    ...fwBridgeOut(bom, 440),
  ].join(''))
}

// ── series-resonant (SRC): half bridge + split bus + series Cr–Lr tank + CT secondary ───
function src(bom) {
  return svg(840, 360, [
    srcDC(60, 175), wire(60, 160, 60, 80, 170, 80), wire(60, 190, 60, 300, 170, 300),
    mosfetV('Q1', bom, 170, 118, 'left'), wire(170, 92, 170, 80), dot(170, 80), wire(170, 144, 170, 165), dot(170, 165),
    mosfetV('Q2', bom, 170, 212, 'left'), wire(170, 186, 170, 165), wire(170, 238, 170, 300), dot(170, 300),
    // split bus Chi / Clo — tank returns to the mid-bus
    capV('Chi', bom, 240, 118, 'right'), wire(240, 80, 240, 98), dot(240, 80), wire(240, 138, 240, 190), dot(240, 190),
    capV('Clo', bom, 240, 250, 'right'), wire(240, 190, 240, 230), wire(240, 270, 240, 300), dot(240, 300), gnd(200, 300),
    // series tank Cr → Lr off the switch node into the primary
    capH('Cr', bom, 210, 165), wire(170, 165, 190, 165), indH('Lr', bom, 300, 165), wire(230, 165, 272, 165),
    wire(328, 165, 355, 165, 355, 110, 380, 110, 380, 130),
    xfmr('T1', bom, 390, 170, { h: 80, ct: 'right', labelDx: -40 }),
    wire(380, 210, 380, 235, 355, 235, 355, 190, 240, 190), // primary return → mid bus
    // center-tapped full-wave secondary
    wire(400, 130, 400, 112, 470, 112), diode('D1', bom, 490, 112, 'right'), wire(510, 112, 570, 112), dot(570, 112),
    wire(400, 210, 400, 228, 470, 228), diode('D2', bom, 490, 228, 'right', 'below'), wire(510, 228, 570, 228), wire(570, 228, 570, 112),
    wire(414, 170, 425, 170, 425, 300, 700, 300),
    wire(570, 112, 720, 112), dot(620, 112),
    capV('Cout', bom, 620, 205), wire(620, 112, 620, 185), wire(620, 225, 620, 300), dot(620, 300),
    loadR(700, 205, 112, 300), dot(700, 112), dot(700, 300),
    port(756, 112, 'VOUT'), wire(720, 112, 756, 112),
  ].join(''))
}

// Secondary synchronous-rectifier full bridge for CLLC / CLLLC (drawn as MOSFETs). The two
// tank output nodes enter at (nx, ny) [node_c] and (nx, ny2) [node_d]; refs name the SR FETs.
function srBridgeOut(bom, nx, ny, ny2, refs) {
  const yP = 80, yN = 300, cx = nx + 210, lx = cx + 90
  const xA = nx + 70, xB = nx + 140
  return [
    wire(nx, ny, xA, ny), dot(xA, ny),
    wire(nx, ny2, nx + 30, ny2, nx + 30, 330, xB + 48, 330, xB + 48, ny2, xB, ny2), dot(xB, ny2),
    mosfetV(refs[0], bom, xA, 130, 'left', true), wire(xA, 104, xA, yP), wire(xA, 156, xA, ny),
    mosfetV(refs[1], bom, xA, 250, 'left', true), wire(xA, 224, xA, ny), wire(xA, 276, xA, yN),
    mosfetV(refs[2], bom, xB, 130, 'right', true), wire(xB, 104, xB, yP), wire(xB, 156, xB, ny2),
    mosfetV(refs[3], bom, xB, 250, 'right', true), wire(xB, 224, xB, ny2), wire(xB, 276, xB, yN),
    dot(xA, yP), dot(xB, yP), wire(xA, yP, cx + 60, yP),
    wire(xA, yN, lx, yN),
    dot(cx, yP), capV('Cout', bom, cx, 190), wire(cx, yP, cx, 170), wire(cx, 210, cx, yN), dot(cx, yN),
    loadR(lx, 190, yP, yN), dot(lx, yP), dot(lx, yN),
    port(lx + 60, yP, 'VOUT'), wire(lx, yP, lx + 60, yP),
  ]
}

// ── CLLC: full bridge + primary tank Cr1–Lr1, symmetric secondary tank Lr2–Cr2, SR bridge ─
function cllc(bom) {
  return svg(1040, 360, [
    srcDC(60, 175), wire(60, 160, 60, 80, 300, 80), wire(60, 190, 60, 320, 110, 320),
    mosfetV('Q1', bom, 150, 128, 'left', true), wire(150, 102, 150, 80), dot(150, 80), wire(150, 154, 150, 205), dot(150, 205),
    mosfetV('Q2', bom, 150, 250, 'left', true), wire(150, 205, 150, 224), wire(150, 276, 150, 320), dot(150, 320),
    mosfetV('Q3', bom, 280, 128, 'right', true), wire(280, 102, 280, 80), dot(280, 80), wire(280, 154, 280, 150),
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
  ].join(''))
}

// ── CLLLC: like CLLC but with a discrete secondary resonant inductor path (QE..QH SR) ────
function clllc(bom) {
  return svg(1040, 360, [
    srcDC(60, 175), wire(60, 160, 60, 80, 300, 80), wire(60, 190, 60, 320, 110, 320),
    mosfetV('Q1', bom, 150, 128, 'left', true), wire(150, 102, 150, 80), dot(150, 80), wire(150, 154, 150, 205), dot(150, 205),
    mosfetV('Q2', bom, 150, 250, 'left', true), wire(150, 205, 150, 224), wire(150, 276, 150, 320), dot(150, 320),
    mosfetV('Q3', bom, 280, 128, 'right', true), wire(280, 102, 280, 80), dot(280, 80), wire(280, 154, 280, 150),
    mosfetV('Q4', bom, 280, 250, 'right', true), wire(280, 205, 280, 224), wire(280, 276, 280, 320), dot(280, 320), gnd(110, 320),
    wire(150, 205, 175, 205), capH('Cr1', bom, 200, 205), indH('Lr1', bom, 268, 205), wire(225, 205, 240, 205),
    wire(296, 205, 310, 205, 310, 150),
    xfmr('T1', bom, 340, 195, { h: 90, labelDy: -24 }),
    wire(330, 240, 330, 300, 280, 300, 280, 205), dot(280, 205),
    wire(350, 150, 380, 150), indH('Lr2', bom, 408, 150), wire(436, 150, 460, 150), capH('Cr2', bom, 486, 150), wire(508, 150, 540, 150),
    wire(350, 240, 540, 240),
    ...srBridgeOut(bom, 540, 150, 240, ['QE', 'QF', 'QG', 'QH']),
  ].join(''))
}

// ── boost PFC: full-bridge line rectifier feeding a boost cell (L, SW, D5) ────────────────
function pfc(bom) {
  return svg(880, 380, [
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
    // boost cell: rectified bus → L → switch node; SW shunt, D5 into the bus cap
    indH('L', bom, 340, 150), wire(250, 150, 312, 150), wire(368, 150, 410, 150), dot(410, 150),
    mosfetV('SW', bom, 410, 225), wire(410, 150, 410, 199), wire(410, 251, 410, 320), dot(410, 320),
    diode('D5', bom, 480, 150, 'right'), wire(410, 150, 460, 150), wire(500, 150, 600, 150), dot(560, 150),
    capV('Cout', bom, 560, 235), wire(560, 150, 560, 215), wire(560, 255, 560, 320), dot(560, 320),
    loadR(630, 235, 150, 320), dot(630, 150), dot(630, 320),
    port(690, 150, 'VBUS'), wire(630, 150, 690, 150),
  ].join(''))
}

// ── Vienna: 3-phase 3-level rectifier — three boost legs into a split DC bus ─────────────
function viennaLeg(bom, x, ph) {
  // vertical leg: phase-in → L → node X, with Dp↑ to busP and Dn↑ from busN, plus the
  // bidirectional midpoint switch (SW+SQ, common node) clamping X to the neutral rail.
  const busP = 60, neu = 270, busN = 340, X = 130, sx = x + 48
  return [
    port(x - 48, 410, ph, 'start'), wire(x - 48, 410, x - 48, 403),
    indV(`L${ph}`, bom, x - 48, 375, 'left'), wire(x - 48, 347, x - 48, X, x, X), dot(x, X),
    diode(`Dp${ph}`, bom, x, 95, 'up'), wire(x, 75, x, busP), wire(x, 115, x, X), dot(x, busP),
    diode(`Dn${ph}`, bom, x, 300, 'up'), wire(x, 280, x, X), wire(x, 320, x, busN), dot(x, busN),
    wire(x, X, sx, X),
    mosfetV(`SW${ph}`, bom, sx, 160, 'right', true), wire(sx, 134, sx, X), wire(sx, 186, sx, 194),
    mosfetV(`SQ${ph}`, bom, sx, 220, 'right', true), wire(sx, 246, sx, neu), dot(sx, neu),
  ]
}
function vienna(bom) {
  const cols = [{ x: 155, ph: 'a' }, { x: 365, ph: 'b' }, { x: 575, ph: 'c' }]
  return svg(940, 440, [
    wire(95, 60, 780, 60), wire(95, 270, 780, 270), wire(95, 340, 780, 340), // busP / neutral / busN
    ...cols.flatMap((c) => viennaLeg(bom, c.x, c.ph)),
    txt(110, 264, 'N', 'sch-port', 'end'),
    // split caps Cp (busP→neutral) and Cn (neutral→busN) + load across the full bus
    capV('Cp', bom, 720, 165, 'right'), wire(720, 60, 720, 145), dot(720, 60), wire(720, 185, 720, 270), dot(720, 270),
    capV('Cn', bom, 720, 305, 'right'), wire(720, 270, 720, 285), wire(720, 325, 720, 340), dot(720, 340),
    loadR(790, 200, 60, 340), dot(790, 60), dot(790, 340),
    port(850, 60, 'BUS+'), wire(790, 60, 850, 60), port(850, 340, 'BUS−'), wire(790, 340, 850, 340),
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

export function renderSchematic(topologyId, bomRows) {
  const fn = LAYOUTS[topologyId]
  if (!fn) return null
  const bom = new Map((bomRows ?? []).map((r) => [r.ref, r]))
  return fn(bom)
}
