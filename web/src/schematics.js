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

const LAYOUTS = {
  buck, boost, sepic, cuk, zeta, fsbb, flyback, forward,
  push_pull: pushPull, llc, dab,
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
