// CIAS-driven schematic generator — the ONLY schematic generator. The power-path drawing is assembled
// from the TAS's inline CIAS bricks, the same structure the ngspice deck and the falstad visual sim are
// generated from, so all three views share one source of truth. WHICH components appear (e.g. the QRM
// resonant cap), their VALUES/labels and the net TRUTH come from CIAS; only the GEOMETRY (grid
// coordinates — CIAS carries none) is declared per topology, below.
//
// Every render is verified against the flattened CIAS netlist by schematicCheck.js and THROWS on drift,
// never drawing something unverified. That now includes rule G, which identifies the passive-only nodes
// (a snubber midpoint, a tank's Cr–Lr join) that no MOSFET/diode/port anchors — the blind spot this file
// used to warn about.
//
// Authoring a layout:
//   • `place[ref].draw(bom)` positions each CIAS component; the symbol registers its own terminals.
//   • a wire is `{ from: 'Q1.drain', to: 'T1.p1', via: [[x, y]] }` — ENDS NAME TERMINALS, so a wire
//     cannot land short of the part it connects to; a bare [x, y] is a rail corner. Diagonals throw.
//   • `needs: [refs]` gates a wire on the components it belongs to, so an absent part takes its wiring
//     with it; a whole layout may be a FUNCTION of the present refs when the drawing changes shape.
//   • junction dots are DERIVED from the finished wiring, never listed.
//   • `synth(bom, present)` draws the non-CIAS glyphs the TAS assembler implies: source, load, earth,
//     ports, gate-drive flags and the controller blocks.
import { symbols as S, withPinRecording } from './schematics.js'
import { ciasComponents } from './cias.js'
import { extractBom } from './bom.js'
import { checkSchematic } from './schematicCheck.js'

const { svg, wire, dot, mosfetV, mosfetH, diode, indH, indV, capV, capH, resV, resH, xfmr, xfmr3, xfmr4, srcDC, srcAC, gnd, isoGnd, loadR, port, sig, ctrlIC, icBox, txt } = S

// wire(...pts) helper takes a flat point list; our layout stores polylines as flat arrays.
const poly = (pts) => wire(...pts)

// ── layout primitives ───────────────────────────────────────────────────────
// A wire is authored as `{ from: 'Q1.drain', to: 'T1.p1', via: [[x, y], …] }`: its ENDS are terminal
// names, resolved against the coordinates the symbols themselves registered while drawing, and only the
// corners in between are typed. That removes the defect class hand-typed endpoints kept producing —
// acf's transformer secondary sat 5 px from the wire that was supposed to reach it, and passed every
// check for as long as the tolerance was 8 px. Here the wire cannot miss: it IS the terminal.
// `{ pts: [...] }` is still accepted for a run between two bare coordinates (a rail corner, a port).
const resolveRoute = (topologyId, w, at) => {
  const raw = w.pts ? [...w.pts] : [...at(w.from), ...(w.via ?? []).flat(), ...at(w.to)]
  // A via that lands exactly on the terminal it leads to would leave a zero-length segment behind.
  const pts = []
  for (let i = 0; i < raw.length; i += 2)
    if (pts.length < 2 || raw[i] !== pts[pts.length - 2] || raw[i + 1] !== pts[pts.length - 1]) pts.push(raw[i], raw[i + 1])
  for (let i = 2; i < pts.length; i += 2) {
    // Every segment must be axis-aligned: the netlist checker, the label rules and the visual-sim
    // exporter all assume orthogonal routing, and a stray diagonal would be invisible to all three.
    if (pts[i] !== pts[i - 2] && pts[i + 1] !== pts[i - 1])
      throw new Error(`ciasSchematic '${topologyId}': wire ${w.from ?? '?'}→${w.to ?? '?'} has a diagonal segment ` +
                      `(${pts[i - 2]},${pts[i - 1]})→(${pts[i]},${pts[i + 1]}) — add a via to turn the corner`)
  }
  // Two terminals that coincide (parts butting together) leave a single point behind: that is not a
  // wire, and emitting it drew a path with no segments — which then attracted a junction dot.
  return pts.length >= 4 ? pts : null
}

// Junction dots are DERIVED, not listed. A dot means "three or more conductors meet here", so it is a
// property of the finished wiring rather than something to remember to add: the T-DOT rule in
// auditSchematics.mjs looks for exactly this, and the FALSE-DOT rule rejects one at a pure crossing.
// Counted per point: a polyline END contributes one conductor, an interior CORNER two (its two
// segments), a segment passing THROUGH two, and a component terminal one — its own lead.
// Every drawn wire is counted, not just the routed ones, because a symbol may draw its own run to a
// rail (loadR does), and that run joins the node exactly like any other.
const deriveDots = (svgParts, terminals) => {
  const key = (x, y) => `${x},${y}`
  const count = new Map()
  const bump = (k, n) => count.set(k, (count.get(k) ?? 0) + n)
  const polys = [...svgParts.matchAll(/<path class="sch-wire" d="([^"]+)"/g)]
    .map((m) => [...m[1].matchAll(/[ML]\s*(-?[\d.]+)\s+(-?[\d.]+)/g)].map((z) => [+z[1], +z[2]]))
  const segs = []
  for (const n of polys) {
    for (let i = 1; i < n.length; i++) segs.push([n[i - 1], n[i]])
    n.forEach((pt, i) => bump(key(pt[0], pt[1]), i === 0 || i === n.length - 1 ? 1 : 2))
  }
  for (const t of terminals) bump(key(t.x, t.y), 1)
  for (const k of [...count.keys()]) {
    const [x, y] = k.split(',').map(Number)
    for (const [a, b] of segs) {
      const inside = (a[0] === b[0] && x === a[0] && y > Math.min(a[1], b[1]) && y < Math.max(a[1], b[1])) ||
                     (a[1] === b[1] && y === a[1] && x > Math.min(a[0], b[0]) && x < Math.max(a[0], b[0]))
      if (inside) bump(k, 2)
    }
  }
  return [...count].filter(([, n]) => n >= 3).map(([k]) => k.split(',').map(Number))
}

// ── composable layout fragments ─────────────────────────────────────────────
// The bridge families share their secondaries (full-bridge, centre-tapped, current-doubler) and their
// output tail. Expressed as fragments — { place, wires, synth } merged into a parent — one description
// serves every topology that uses it, and a layout may be a FUNCTION of the present CIAS refs, so the
// drawing follows the netlist rather than a variant string: if the design carries Dr3 it is a full
// bridge, if it carries a second choke it is a current doubler.
const merge = (...frags) => ({
  place: Object.assign({}, ...frags.map((f) => f.place ?? {})),
  wires: frags.flatMap((f) => f.wires ?? []),
  // Lazy: a fragment's glyphs must be DRAWN inside the recording pass, or the terminals they register
  // (a VOUT port, the isolated return) are recorded before recording starts and no wire can name them.
  synthEls: (b) => frags.flatMap((f) => (f.synthEls ? f.synthEls(b) : [])),
})

// Output tail: optional choke, Cout, the load, the VOUT port and the isolated return.
const outTailFrag = (x0, yTop, retY, lout) => {
  const px = lout ? x0 + 80 : x0
  const cX = px + 44, lX = cX + 58, pX = cX + 112, cY = (yTop + retY) / 2
  return {
    retX: lX,
    coutX: cX,
    place: {
      ...(lout ? { [lout]: { draw: (b) => indH(lout, b, x0 + 40, yTop) } } : {}),
      Cout: { draw: (b) => capV('Cout', b, cX, cY) },
    },
    wires: [
      ...(lout ? [{ from: [x0, yTop], to: `${lout}.p0` }, { from: `${lout}.p1`, to: '@port.VOUT', via: [[pX, yTop]] }]
               : [{ from: [x0, yTop], to: '@port.VOUT', via: [[pX, yTop]] }]),
      { from: 'Cout.p0', to: [cX, yTop] },
      { from: 'Cout.p1', to: [cX, retY] },
    ],
    synthEls: () => [loadR(lX, cY, yTop, retY), port(pX, yTop, 'VOUT'), isoGnd(cX, retY)],
  }
}

// Centre-tapped secondary: one diode per winding half into a shared output node, centre tap = return.
const secCTFrag = (tx, ty, h, o) => {
  const yT = ty - h / 2, yB = ty + h / 2, aT = yT - 18, aB = yB + 18, retY = yB + 90
  const dX = tx + 100, jX = tx + 180
  const tail = outTailFrag(jX, aT, retY, o.lout)
  return merge({
    place: {
      [o.d1]: { draw: (b) => diode(o.d1, b, dX, aT, 'right') },
      [o.d2]: { draw: (b) => diode(o.d2, b, dX, aB, 'right', 'below') },
    },
    wires: [
      { from: 'T1.s0', to: `${o.d1}.anode`, via: [[tx + 10, aT]] },
      { from: `${o.d1}.cathode`, to: [jX, aT] },
      { from: 'T1.s1', to: `${o.d2}.anode`, via: [[tx + 10, aB]] },
      { from: `${o.d2}.cathode`, to: [jX, aT], via: [[jX, aB]] },
      { from: 'T1.sct', to: [tail.retX, retY], via: [[tx + 35, ty], [tx + 35, retY]] },
    ],
  }, tail)
}

// Full-bridge secondary: four diodes in two legs; sec_b wraps under the bridge to the far leg.
const secFBFrag = (tx, ty, h, o) => {
  const ds = o.diodes
  const yM = ty, yP = ty - h / 2 - 30, yN = ty + h / 2 + 90
  const xA = tx + 90, xB = tx + 160
  const tail = outTailFrag(xB, yP, yN, o.lout)
  const wrapX = Math.min(xB + 44, tail.coutX - 22)
  return merge({
    place: {
      [ds[0]]: { draw: (b) => diode(ds[0], b, xA, (yP + yM) / 2, 'up', 'left', true) },
      [ds[1]]: { draw: (b) => diode(ds[1], b, xA, (yN + yM) / 2, 'up', 'left', true) },
      [ds[2]]: { draw: (b) => diode(ds[2], b, xB, (yP + yM) / 2, 'up', 'right', true) },
      // ds[3] labels LEFT: on the right it lands on the sec_b wrap lane wherever that lane falls (llc/src
      // put it at 802, straight through "DL2"), and nudging it back off the lane printed it on its own body.
      [ds[3]]: { draw: (b) => diode(ds[3], b, xB, (yN + yM) / 2, 'up', 'left', true) },
    },
    wires: [
      { from: 'T1.s0', to: [xA, yM], via: [[tx + 10, yM]] },
      // sec_b wraps under the bridge and climbs a lane that must stay CLEAR of Cout's column: with no
      // output inductor in the tail (llc/src) xB+44 lands exactly on it.
      { from: 'T1.s1', to: [xB, yM], via: [[tx + 40, ty + h / 2], [tx + 40, yN + 34], [wrapX, yN + 34], [wrapX, yM]] },
      { from: `${ds[0]}.anode`, to: [xA, yM] },
      { from: `${ds[0]}.cathode`, to: [xA, yP] },
      { from: `${ds[1]}.anode`, to: [xA, yN] },
      { from: `${ds[1]}.cathode`, to: [xA, yM] },
      { from: `${ds[2]}.anode`, to: [xB, yM] },
      { from: `${ds[2]}.cathode`, to: [xB, yP] },
      { from: `${ds[3]}.anode`, to: [xB, yN] },
      { from: `${ds[3]}.cathode`, to: [xB, yM] },
      { from: [xA, yP], to: [xB, yP] },
      { from: [xA, yN], to: [tail.retX, yN] },
    ],
  }, tail)
}

// Current doubler: one winding, two chokes, two catch diodes into a shared return.
const secCDFrag = (tx, ty, h, o) => {
  const yT = ty - h / 2, yB = ty + h / 2, retY = yB + 90, voX = tx + 190
  const tail = outTailFrag(voX, yT, retY, null)
  // The two catch diodes get their OWN columns between the winding terminals and the chokes, 60 px
  // apart so a two-character refdes fits between them, with their bodies parked in the lane below yB.
  const dX1 = tx + 35, dX2 = tx + 95, dY = (yB + retY) / 2, loX = tx + 135
  return merge({
    place: {
      [o.lo1]: { draw: (b) => indH(o.lo1, b, loX, yT) },
      [o.lo2]: { draw: (b) => indH(o.lo2, b, loX, yB) },
      // 'left': right of it only ~31 px separate the label from Cout's column, so wider values crossed it
      Rlb: { draw: (b) => resV('Rlb', b, voX, ty, 'left') },
      [o.d1]: { draw: (b) => diode(o.d1, b, dX1, dY, 'up', 'right', true) },
      [o.d2]: { draw: (b) => diode(o.d2, b, dX2, dY, 'up', 'right', true) },
    },
    wires: [
      { from: 'T1.s0', to: `${o.lo1}.p0`, via: [[tx + 10, yT]] },
      { from: `${o.lo1}.p1`, to: [voX, yT] },
      { from: 'T1.s1', to: `${o.lo2}.p0`, via: [[tx + 10, yB]] },
      { from: `${o.lo2}.p1`, to: [voX, yB] },
      // Rlb: series loop-breaker between Lo2's output and the vout node — a real BOM row, so it is
      // drawn in its electrical position
      { from: 'Rlb.p0', to: [voX, yT] },
      { from: 'Rlb.p1', to: [voX, yB] },
      { from: `${o.d1}.cathode`, to: [dX1, yT] },
      { from: `${o.d1}.anode`, to: [dX1, retY] },
      { from: `${o.d2}.cathode`, to: [dX2, yB] },
      { from: `${o.d2}.anode`, to: [dX2, retY] },
      { from: [dX1, retY], to: [tail.retX, retY] },
    ],
  }, tail)
}

// Which secondary a design HAS, read off its CIAS refs rather than a variant string.
const secondaryFrag = (present, tx, ty, h, refs) =>
  present.has(refs.fb?.[1] ?? '') ? secFBFrag(tx, ty, h, { diodes: refs.fb, lout: refs.lout })
  : present.has(refs.lo2 ?? '') ? secCDFrag(tx, ty, h, { d1: refs.d1, d2: refs.d2, lo1: refs.lo1 ?? refs.lout, lo2: refs.lo2 })
  : secCTFrag(tx, ty, h, { d1: refs.d1, d2: refs.d2, lout: refs.lout })

// Half-bridge + split-bus front end shared by LLC and SRC: VIN, the split caps with their balancing
// bleeders, and the Q1/Q2 half bridge whose midpoint is the switch node.
const halfBridgeSplitBusFrag = (top, gy) => {
  const mid = (top + gy) / 2, hiY = (top + mid) / 2, loY = (mid + gy) / 2
  return {
    sw: [300, mid], msplit: 170, mid,
    place: {
      Chi: { draw: (b) => capV('Chi', b, 120, hiY, 'left') },
      Clo: { draw: (b) => capV('Clo', b, 120, loY, 'left') },
      Rbal_hi: { draw: (b) => resV('Rbal_hi', b, 170, hiY, 'right') },
      // Labelled on the right like Rbal_hi. It used to be nudged 26 px LEFT to dodge the tank lane,
      // which printed the name and the value through its own zigzag; the lane moved instead.
      Rbal_lo: { draw: (b) => resV('Rbal_lo', b, 170, loY, 'right') },
      Q1: { draw: (b) => mosfetV('Q1', b, 300, mid - 65, 'right', true) },
      Q2: { draw: (b) => mosfetV('Q2', b, 300, mid + 55, 'right', true) },
    },
    wires: [
      { from: '@src.p0', to: 'Q1.drain', via: [[50, top], [300, top]] },     // top rail (caps + bleeders tap it)
      { from: '@src.p1', to: 'Q2.source', via: [[50, gy], [300, gy]] },      // bottom rail
      { from: 'Chi.p0', to: [120, top] },
      { from: 'Chi.p1', to: [120, mid] },
      { from: 'Clo.p0', to: [120, mid] },
      { from: 'Clo.p1', to: [120, gy] },
      { from: 'Rbal_hi.p0', to: [170, top] },
      { from: 'Rbal_hi.p1', to: [170, mid] },
      { from: 'Rbal_lo.p0', to: [170, mid] },
      { from: 'Rbal_lo.p1', to: [170, gy] },
      { from: [120, mid], to: [170, mid] },                                  // split-bus midpoint rail
      { from: 'Q1.source', to: 'Q2.drain' },                                 // switch node
    ],
    synthEls: () => [srcDC(50, mid + 5), gnd(50, gy), sig(274, mid - 65, 'g1'), sig(274, mid + 55, 'g2')],
  }
}

// LLC and SRC differ only in the tank they carry (LLC's magnetising inductance vs SRC's bare series
// pair), which is a VALUE difference, not a drawing one — so they share this layout.
const resonantHalfBridge = (present) => {
  const top = 70, gy = 300, tx = 620, ty = 185, h = 80
  const hb = halfBridgeSplitBusFrag(top, gy)
  const [swx, swy] = hb.sw
  const sec = secondaryFrag(present, tx, ty, h,
    { fb: ['DH1', 'DL1', 'DH2', 'DL2'], d1: 'D1', d2: 'D2', lo1: 'Lo1', lo2: 'Lo2' })
  const frag = merge(hb, {
    place: {
      Cr: { draw: (b) => capH('Cr', b, 380, swy) },
      Lr: { draw: (b) => indH('Lr', b, 480, swy) },
      T1: { draw: (b) => xfmr('T1', b, tx, ty, { h, ct: present.has('DL1') || present.has('Lo2') ? undefined : 'right', labelDx: -44 }) },
    },
    wires: [
      { from: [swx, swy], to: 'Cr.p0' },                                     // resonant tank, one clear lane
      { from: 'Cr.p1', to: 'Lr.p0' },
      { from: 'Lr.p1', to: 'T1.p0', via: [[540, swy], [540, ty - h / 2]] },
      // The primary return climbs at x=240, not 220: at 220 it ran up the only clear lane Rbal_lo's
      // label had, which is what pushed that label back onto its own resistor.
      { from: 'T1.p1', to: [hb.msplit, swy], via: [[tx - 10, 335], [240, 335], [240, swy]] },
    ],
  }, sec)
  return {
    size: [1060, 380],
    place: frag.place,
    wires: frag.wires,
    synth: (b) => [...frag.synthEls(b), ctrlIC(b, 130, 350, ['g1', 'g2'])],
  }
}



// Synchronous-rectifier output bridge shared by CLLC and CLLLC: four FETs, Cout, load, VOUT port and
// the isolated return.
const srBridgeOutFrag = (nx, ny, ny2, refs) => {
  const yP = 80, yN = 300, cx = nx + 300, lx = cx + 100
  const xA = nx + 80, xB = nx + 190
  return {
    place: {
      [refs[0]]: { draw: (b) => mosfetV(refs[0], b, xA, 130, 'right', true) },
      [refs[1]]: { draw: (b) => mosfetV(refs[1], b, xA, 250, 'right', true) },
      [refs[2]]: { draw: (b) => mosfetV(refs[2], b, xB, 130, 'right', true) },
      [refs[3]]: { draw: (b) => mosfetV(refs[3], b, xB, 250, 'right', true, false, 0, 8) },   // +8y: the ref sat on the ny2 return
      Cout: { draw: (b) => capV('Cout', b, cx, 190) },
    },
    wires: [
      { from: [nx, ny], to: [xA, ny] },
      { from: [nx, ny2], to: [xB, ny2], via: [[nx + 30, ny2], [nx + 30, 330], [xB + 48, 330], [xB + 48, ny2]] },
      { from: `${refs[0]}.drain`, to: [xA, yP] },
      { from: `${refs[0]}.source`, to: [xA, ny] },                          // the tank feeds this leg mid
      { from: `${refs[1]}.drain`, to: `${refs[0]}.source` },
      { from: `${refs[1]}.source`, to: [xA, yN] },
      { from: `${refs[2]}.drain`, to: [xB, yP] },
      { from: `${refs[2]}.source`, to: `${refs[3]}.drain` },
      { from: `${refs[3]}.drain`, to: [xB, ny2] },
      { from: `${refs[3]}.source`, to: [xB, yN] },
      { from: [xA, yP], to: '@port.VOUT', via: [[lx + 60, yP]] },            // VOUT+ rail spans the bridge
      { from: [xA, yN], to: [lx, yN] },
      { from: 'Cout.p0', to: [cx, yP] },
      { from: 'Cout.p1', to: [cx, yN] },
    ],
    synthEls: () => [loadR(lx, 190, yP, yN), port(lx + 60, yP, 'VOUT'), isoGnd(nx + 140, yN)],
  }
}

// CLLC and CLLLC share every coordinate; CLLLC only adds the SR current-sense shunt in the secondary
// tank (and with it a separate SR controller block).
const cllcLike = (present, hasSense) => {
  const sr = srBridgeOutFrag(540, 150, 240, hasSense ? ['QE', 'QF', 'QG', 'QH'] : ['Qa', 'Qb', 'Qc', 'Qd'])
  const flags = hasSense ? ['gA', 'gB'] : ['sa', 'sb', 'sc', 'sd']
  const frag = merge({
    place: {
      Q1: { draw: (b) => mosfetV('Q1', b, 150, 128, 'right', true) },
      Q2: { draw: (b) => mosfetV('Q2', b, 150, 250, 'right', true) },
      Q3: { draw: (b) => mosfetV('Q3', b, 280, 128, 'right', true) },
      Q4: { draw: (b) => mosfetV('Q4', b, 280, 250, 'right', true) },
      Cr1: { draw: (b) => capH('Cr1', b, 190, 205) },
      Lr1: { draw: (b) => indH('Lr1', b, 238, 205) },
      // centred and raised: offset right, T1's own label sat nearer Lr2 than T1
      T1: { draw: (b) => xfmr('T1', b, 340, 195, { h: 90, labelDy: -46 }) },
      Lr2: { draw: (b) => indH('Lr2', b, 408, 150) },
      ...(hasSense
        // The two parts butt together, so their label blocks did too: "Cr2 Rsense" over one value pair
        // read as one part. Pushed apart by 26 px, each value sits under its own name.
        ? { Cr2: { draw: (b) => capH('Cr2', b, 470, 150, 'above', -12) },
            Rsense: { draw: (b) => resH('Rsense', b, 510, 150, 'above', 14) } }
        : { Cr2: { draw: (b) => capH('Cr2', b, 486, 150) } }),
    },
    wires: [
      { from: '@src.p0', to: [280, 80], via: [[60, 80]] },                   // rails span exactly both legs
      { from: '@src.p1', to: [280, 320], via: [[60, 320]] },
      { from: 'Q1.drain', to: [150, 80] },
      { from: 'Q1.source', to: 'Q2.drain' },                                 // node_a
      { from: 'Q2.source', to: [150, 320] },
      { from: 'Q3.drain', to: [280, 80] },
      // ONE unbroken leg-mid column (Q3.source → Q4.drain = node_b): drawn as two segments meeting at
      // y=205 it put a vertex exactly where the tank lane crosses, which reads as a junction.
      { from: 'Q3.source', to: 'Q4.drain' },
      { from: 'Q4.source', to: [280, 320] },
      // node_a → Cr1 → Lr1 → primary; the tank steps up to cross the leg column as a plain crossing
      { from: [150, 205], to: 'Cr1.p0' },
      // Cr1's right terminal and Lr1's left one are the SAME point (the parts butt together), so the
      // short stub gives the winding terminal a wire to touch — rule C requires one.
      { from: 'Lr1.p0', to: [202, 205] },
      { from: 'Lr1.p1', to: 'T1.p0', via: [[270, 205], [270, 175], [310, 175], [310, 150]] },
      { from: 'T1.p1', to: [280, 215], via: [[330, 265], [305, 265], [305, 215]] },
      // secondary tank into the SR bridge
      { from: 'T1.s0', to: 'Lr2.p0' },
      ...(hasSense
        ? [{ from: 'Lr2.p1', to: 'Cr2.p0' }, { from: 'Cr2.p1', to: 'Rsense.p0' }, { from: 'Rsense.p1', to: [540, 150] }]
        : [{ from: 'Lr2.p1', to: 'Cr2.p0' }, { from: 'Cr2.p1', to: [540, 150] }]),
      { from: 'T1.s1', to: [540, 240] },
    ],
    synthEls: (b) => [
      srcDC(60, 175), gnd(110, 320),
      sig(124, 128, 'g1'), sig(124, 250, 'g2'), sig(254, 128, 'g3'), sig(254, 250, 'g4'),
      sig(594, 130, flags[0]), sig(594, 250, flags[1], 'left', -12),
      ...(hasSense ? [sig(704, 130, 'gB'), sig(704, 250, 'gA'), sig(490, 150, 'sP', 'down'), sig(530, 150, 'sM', 'down'),
                      icBox('SR', b, 450, 390, 70, 64, ['sP', 'sM'], ['gA', 'gB'], 'SR CTRL'),
                      ctrlIC(b, 220, 380, ['g1', 'g2', 'g3', 'g4'])]
                   : [sig(704, 130, flags[2]), sig(704, 250, flags[3]),
                      icBox('U1', b, 220, 380, 64, 80, ['g1', 'g2', 'g3', 'g4'], flags, 'PWM')]),
    ],
  }, sr)
  return { size: [1040, 430], place: frag.place, wires: frag.wires, synth: (b) => frag.synthEls(b) }
}

const LAYOUTS = {
  // ── non-isolated: boost ───────────────────────────────────────────────────
  boost: {
    size: [720, 300],
    place: {
      L1:   { draw: (b) => indH('L1', b, 150, 80) },
      Q1:   { draw: (b) => mosfetV('Q1', b, 240, 150, 'left') },
      D1:   { draw: (b) => diode('D1', b, 310, 80, 'right') },                        // diode variant
      Q2:   { draw: (b) => mosfetH('Q2', b, 310, 80, true, true, 'below') },          // synchronous variant
      Cout: { draw: (b) => capV('Cout', b, 430, 150) },
    },
    wires: [
      { from: '@src.p0', to: 'L1.p0', via: [[70, 80]] },
      { from: '@src.p1', to: [520, 220], via: [[70, 220]] },
      // switch node: one horizontal run from L1 to the rectifier, tapped from below by Q1
      { from: 'L1.p1', to: 'D1.anode', needs: ['D1'] },
      { from: 'L1.p1', to: 'Q2.source', needs: ['Q2'] },
      { from: 'Q1.drain', to: [240, 80] },
      { from: 'Q1.source', to: [240, 220] },
      { from: 'Cout.p0', to: [430, 80] },
      { from: 'Cout.p1', to: [430, 220] },
      // rectifier: a diode, or the high-side FET with its body diode drawn above it
      { from: 'D1.cathode', to: '@port.VOUT', via: [[600, 80]], needs: ['D1'] },
      { from: 'Q2.drain', to: '@port.VOUT', via: [[600, 80]], needs: ['Q2'] },
      { from: 'D2.anode', to: [284, 80], via: [[284, 40]], needs: ['Q2'] },
      { from: 'D2.cathode', to: [336, 80], via: [[336, 40]], needs: ['Q2'] },
    ],
    synth: (b, present) => [
      srcDC(70, 150), loadR(520, 150, 80, 220), gnd(300, 220), port(600, 80, 'VOUT'),
      ...(present.has('Q2') ? [diode('D2', b, 310, 40, 'right', 'above', true), sig(310, 106, 'g2', 'down')] : []),
      sig(214, 150, 'g1', 'down'),
      ctrlIC(b, 640, 240, present.has('Q2') ? ['g1', 'g2'] : ['g1']),
    ],
  },
  // ── non-isolated: SEPIC ───────────────────────────────────────────────────
  sepic: {
    size: [760, 300],
    place: {
      L1:   { draw: (b) => indH('L1', b, 150, 80) },
      Q1:   { draw: (b) => mosfetV('Q1', b, 230, 150) },
      Cs:   { draw: (b) => capH('Cs', b, 290, 80) },
      L2:   { draw: (b) => indV('L2', b, 360, 150) },
      D1:   { draw: (b) => diode('D1', b, 430, 80, 'right') },
      Cout: { draw: (b) => capV('Cout', b, 500, 150) },
    },
    wires: [
      { from: '@src.p0', to: 'L1.p0', via: [[70, 80]] },
      { from: '@src.p1', to: [580, 220], via: [[70, 220]] },
      { from: 'L1.p1', to: 'Cs.p0' },                                                 // switch node run (Q1 taps it)
      { from: 'Q1.drain', to: [230, 80] },
      { from: 'Q1.source', to: [230, 220] },
      { from: 'Cs.p1', to: 'D1.anode' },                                              // L2 taps this run
      { from: 'L2.p0', to: [360, 80] },
      { from: 'L2.p1', to: [360, 220] },
      { from: 'D1.cathode', to: '@port.VOUT', via: [[660, 80]] },
      { from: 'Cout.p0', to: [500, 80] },
      { from: 'Cout.p1', to: [500, 220] },
    ],
    synth: (b) => [
      srcDC(70, 150), loadR(580, 150, 80, 220), gnd(300, 220), port(660, 80, 'VOUT'),
      sig(204, 150, 'g1'), ctrlIC(b, 680, 240, ['g1']),
    ],
  },
  // ── non-isolated: Ćuk ─────────────────────────────────────────────────────
  cuk: {
    size: [760, 300],
    place: {
      L1:      { draw: (b) => indH('L1', b, 150, 80) },
      Q1:      { draw: (b) => mosfetV('Q1', b, 230, 150, 'left') },
      C1:      { draw: (b) => capH('C1', b, 290, 80) },
      Rrc_sw:  { draw: (b) => resV('Rrc_sw', b, 320, 130, 'left') },   // snubber: node → R → C → gnd
      Crc_sw:  { draw: (b) => capV('Crc_sw', b, 320, 185, 'left') },
      D1:      { draw: (b) => diode('D1', b, 360, 150, 'down', 'right') },
      L2:      { draw: (b) => indH('L2', b, 428, 80) },
      Cout:    { draw: (b) => capV('Cout', b, 500, 150) },
    },
    wires: [
      { from: '@src.p0', to: 'L1.p0', via: [[70, 80]] },
      { from: '@src.p1', to: [580, 220], via: [[70, 220]] },
      { from: 'L1.p1', to: 'C1.p0' },                                  // switch node run (Q1 taps it)
      { from: 'Q1.drain', to: [230, 80] },
      { from: 'Q1.source', to: [230, 220] },
      { from: 'C1.p1', to: 'L2.p0' },                                  // C1/D1/L2 node (snubber + D1 tap it)
      { from: 'Rrc_sw.p0', to: [320, 80] },
      { from: 'Rrc_sw.p1', to: 'Crc_sw.p0' },
      { from: 'Crc_sw.p1', to: [320, 220] },
      { from: 'D1.anode', to: [360, 80] },       // Ćuk's catch diode conducts node → ground: anode up
      { from: 'D1.cathode', to: [360, 220] },
      { from: 'L2.p1', to: '@port.VOUT (−)', via: [[660, 80]] },   // the Ćuk output is inverting, and the port says so
      { from: 'Cout.p0', to: [500, 80] },
      { from: 'Cout.p1', to: [500, 220] },
    ],
    synth: (b) => [
      srcDC(70, 150), loadR(580, 150, 80, 220), gnd(300, 220), port(660, 80, 'VOUT (−)'),
      sig(204, 150, 'g1', 'down'), ctrlIC(b, 680, 250, ['g1']),
    ],
  },
  // ── non-isolated: Zeta ────────────────────────────────────────────────────
  zeta: {
    size: [760, 300],
    place: {
      Q1:   { draw: (b) => mosfetH('Q1', b, 180, 80) },
      L1:   { draw: (b) => indV('L1', b, 260, 150) },
      Cc:   { draw: (b) => capH('Cc', b, 320, 80) },
      D1:   { draw: (b) => diode('D1', b, 390, 150, 'up', 'right') },
      L2:   { draw: (b) => indH('L2', b, 458, 80) },
      Cout: { draw: (b) => capV('Cout', b, 530, 150) },
    },
    wires: [
      { from: '@src.p0', to: 'Q1.drain', via: [[70, 80]] },
      { from: '@src.p1', to: [600, 220], via: [[70, 220]] },
      { from: 'Q1.source', to: 'Cc.p0' },                              // switch node run (L1 taps it)
      { from: 'L1.p0', to: [260, 80] },
      { from: 'L1.p1', to: [260, 220] },
      { from: 'Cc.p1', to: 'L2.p0' },                                  // Cc/D1/L2 node
      { from: 'D1.cathode', to: [390, 80] },
      { from: 'D1.anode', to: [390, 220] },
      { from: 'L2.p1', to: '@port.VOUT', via: [[660, 80]] },
      { from: 'Cout.p0', to: [530, 80] },
      { from: 'Cout.p1', to: [530, 220] },
    ],
    synth: (b) => [
      srcDC(70, 150), loadR(600, 150, 80, 220), gnd(300, 220), port(660, 80, 'VOUT'),
      sig(180, 106, 'g1', 'down'), ctrlIC(b, 680, 250, ['g1']),
    ],
  },
  // ── non-isolated: four-switch buck-boost ──────────────────────────────────
  fsbb: {
    size: [860, 450],
    place: {
      Q1:       { draw: (b) => mosfetV('Q1', b, 220, 132, 'right', true) },
      Q2:       { draw: (b) => mosfetV('Q2', b, 220, 248, 'right', true) },
      Crc_sw1:  { draw: (b) => capV('Crc_sw1', b, 150, 222, 'left') },
      Rrc_sw1:  { draw: (b) => resV('Rrc_sw1', b, 150, 274, 'left') },
      L:        { draw: (b) => indH('L', b, 340, 190) },
      Q3:       { draw: (b) => mosfetV('Q3', b, 440, 132, 'right', true) },
      Q4:       { draw: (b) => mosfetV('Q4', b, 440, 248, 'right', true) },
      Crc_sw2:  { draw: (b) => capV('Crc_sw2', b, 510, 222, 'right') },
      Rrc_sw2:  { draw: (b) => resV('Rrc_sw2', b, 510, 274, 'right') },
      Cout:     { draw: (b) => capV('Cout', b, 620, 195) },
    },
    wires: [
      { from: '@src.p0', to: 'Q1.drain', via: [[60, 70], [220, 70]] },       // Vin+ rail → left leg
      { from: '@src.p1', to: [680, 320], via: [[60, 320]] },                 // continuous ground rail
      // left leg: switch node sw1 between Q1 and Q2, with its RC snubber in a column of its own
      { from: 'Q1.source', to: 'Q2.drain' },
      { from: 'Q2.source', to: [220, 320] },
      { from: 'Crc_sw1.p0', to: [220, 190], via: [[150, 190]] },
      { from: 'Crc_sw1.p1', to: 'Rrc_sw1.p0' },
      { from: 'Rrc_sw1.p1', to: [150, 320] },
      // the inductor bridges the two switch nodes
      { from: 'L.p0', to: [220, 190] },
      { from: 'L.p1', to: [440, 190] },
      // right leg: sw2 between Q3 and Q4, its own snubber column, Vout off Q3's drain
      { from: 'Q3.source', to: 'Q4.drain' },
      { from: 'Q4.source', to: [440, 320] },
      { from: 'Crc_sw2.p0', to: [440, 190], via: [[510, 190]] },
      { from: 'Crc_sw2.p1', to: 'Rrc_sw2.p0' },
      { from: 'Rrc_sw2.p1', to: [510, 320] },
      { from: 'Q3.drain', to: '@port.VOUT', via: [[440, 70]] },              // up out of the leg, then the Vout rail
      { from: 'Cout.p0', to: [620, 70] },
      { from: 'Cout.p1', to: [620, 320] },
    ],
    synth: (b) => [
      srcDC(60, 195), loadR(680, 195, 70, 320), gnd(330, 320), port(770, 70, 'VOUT'),
      sig(194, 132, 'g1'), sig(194, 248, 'g2'), sig(414, 132, 'g3'), sig(414, 248, 'g4'),
      ctrlIC(b, 500, 390, ['g1', 'g2', 'g3', 'g4']),
    ],
  },
  // ── isolated: buck with an auxiliary isolated rail (flybuck) ──────────────
  isolated_buck: {
    size: [900, 400],
    place: {
      QS1:  { draw: (b) => mosfetV('QS1', b, 230, 110, 'right', true) },
      QS2:  { draw: (b) => mosfetV('QS2', b, 230, 250, 'right', true) },
      T1:   { draw: (b) => xfmr('T1', b, 360, 185, { h: 90, labelDy: -24 }) },
      Cpri: { draw: (b) => capV('Cpri', b, 420, 265, 'left') },
      Dsec: { draw: (b) => diode('Dsec', b, 620, 258, 'right', 'below') },
      Rsec: { draw: (b) => resV('Rsec', b, 645, 188, 'left') },
      Csec: { draw: (b) => capV('Csec', b, 680, 188) },
    },
    wires: [
      { from: '@src.p0', to: 'QS1.drain', via: [[60, 70], [230, 70]] },
      { from: '@src.p1', to: [420, 300], via: [[60, 300]] },                    // primary (buck) earth rail
      { from: 'QS1.source', to: 'QS2.drain' },                                  // sw node
      { from: 'QS2.source', to: [230, 300] },
      // discrete antiparallel diodes across the sync FETs, in a column of their own
      { from: 'DS1.cathode', to: [160, 70] },
      { from: 'DS1.anode', to: [230, 180], via: [[160, 180]] },
      { from: 'DS2.cathode', to: [160, 180] },
      { from: 'DS2.anode', to: [160, 300] },
      { from: [230, 180], to: 'T1.p0', via: [[300, 180], [300, 140]] },         // sw node → primary top
      // The primary buck rail leaves T1's primary terminal DOWNWARD first: run straight right at y=230
      // and it passes through T1's SECONDARY terminal — a wire across the isolation barrier.
      { from: 'T1.p1', to: 'Cpri.p0', via: [[350, 245]] },
      { from: 'Cpri.p0', to: '@port.VOUT', via: [[420, 230]] },
      { from: 'Cpri.p1', to: [420, 300] },
      // isolated secondary: its own floating return, one rectifier, preload Rsec
      { from: 'T1.s0', to: '@sgnd.sgnd', via: [[370, 118]] },
      { from: 'T1.s1', to: 'Dsec.anode', via: [[370, 215], [600, 215]] },
      { from: 'Dsec.cathode', to: '@port.VISO' },
      { from: 'Rsec.p0', to: [645, 118] },
      { from: 'Rsec.p1', to: [645, 258] },
      { from: 'Csec.p0', to: [680, 118] },
      { from: 'Csec.p1', to: [680, 258] },
    ],
    synth: (b) => [
      srcDC(60, 180), gnd(110, 300), isoGnd(720, 118), port(490, 230, 'VOUT'), port(760, 258, 'VISO'),
      // DS1/DS2 are the sync FETs' antiparallel diodes: TAS components (role bodyDiode), filtered out
      // of the BOM because they are intrinsic, so they are drawn here rather than placed as bricks.
      diode('DS1', b, 160, 125, 'up', 'left'), diode('DS2', b, 160, 235, 'up', 'left'),
      sig(204, 110, 'g1', 'down'), sig(204, 250, 'g2', 'down'),
      ctrlIC(b, 140, 350, ['g1', 'g2']),
    ],
  },
  // ── isolated: buck-boost with an auxiliary isolated rail ──────────────────
  isolated_buck_boost: {
    size: [860, 360],
    place: {
      QS1:  { draw: (b) => mosfetV('QS1', b, 210, 100, 'right', true) },
      T1:   { draw: (b) => xfmr('T1', b, 290, 180, { h: 80, opp: true, labelDy: -24 }) },
      Dpri: { draw: (b) => diode('Dpri', b, 170, 180, 'up', 'left') },
      Cpri: { draw: (b) => capV('Cpri', b, 150, 270, 'left') },
      Dsec: { draw: (b) => diode('Dsec', b, 540, 250, 'right', 'below') },
      Rsec: { draw: (b) => resV('Rsec', b, 565, 178, 'left') },
      Csec: { draw: (b) => capV('Csec', b, 600, 178) },
    },
    wires: [
      { from: '@src.p0', to: 'QS1.drain', via: [[60, 70], [210, 70]] },
      { from: '@src.p1', to: [280, 300], via: [[60, 300]] },
      { from: 'QS1.source', to: 'T1.p0', via: [[210, 140]] },                   // sw node → primary top
      { from: 'T1.p1', to: [280, 300] },                                        // primary bottom → return rail
      // inverting primary rail via Dpri: switch node → diode → VOUT(−) with its own cap
      { from: 'Dpri.cathode', to: [210, 140], via: [[170, 140]] },
      { from: 'Dpri.anode', to: [150, 240], via: [[170, 240]] },
      { from: 'Cpri.p0', to: [150, 240] },
      { from: [150, 240], to: '@port.VOUT(−)' },
      { from: 'Cpri.p1', to: [150, 300] },
      // isolated secondary with its own floating return, joined to the primary only through T1
      { from: 'T1.s0', to: '@sgnd.sgnd', via: [[300, 110]] },
      { from: 'T1.s1', to: 'Dsec.anode', via: [[300, 250]] },
      { from: 'Dsec.cathode', to: '@port.VISO' },
      { from: 'Rsec.p0', to: [565, 110] },
      { from: 'Rsec.p1', to: [565, 250] },
      { from: 'Csec.p0', to: [600, 110] },
      { from: 'Csec.p1', to: [600, 250] },
    ],
    synth: (b) => [
      srcDC(60, 180), gnd(100, 300), isoGnd(690, 110),
      // Port at 135, not 120: end-anchored, its label grows LEFT, and in a narrow window (where labels
      // are ~14 % wider relative to the drawing) it reached the Vin− riser at x=60.
      port(135, 240, 'VOUT(−)', 'end'), port(760, 250, 'VISO'),
      sig(184, 100, 'g1'), ctrlIC(b, 340, 330, ['g1']),   // x=340: at 140 the 'U1' ref sat on the return rail
    ],
  },
  // ── isolated: single-switch forward ───────────────────────────────────────
  forward: {
    size: [820, 400],
    place: {
      // Label right and lifted 12 px: the left lane (Vin riser .. Ddemag column) is too narrow for the
      // RDS(on) string, and at its default height the right-hand label clips T1's footprint.
      Q1:     { draw: (b) => mosfetV('Q1', b, 220, 90, 'right', false, false, 0, -12) },
      Ddemag: { draw: (b) => diode('Ddemag', b, 165, 140, 'up', 'left') },
      T1:     { draw: (b) => xfmr3('T1', b, 330, 180, { labelDy: -40 }).el },
      Dfwd:   { draw: (b) => diode('Dfwd', b, 420, 90, 'right') },
      Dfw:    { draw: (b) => diode('Dfw', b, 470, 180, 'up', 'right') },
      Lout:   { draw: (b) => indH('Lout', b, 530, 90) },
      Cout:   { draw: (b) => capV('Cout', b, 580, 180) },
    },
    wires: [
      { from: '@src.p0', to: 'Q1.drain', via: [[60, 60], [220, 60]] },       // Vin+ rail (Ddemag taps it)
      { from: '@src.p1', to: [320, 320], via: [[60, 320]] },                 // primary return rail
      { from: 'Q1.source', to: 'T1.p0', via: [[220, 110]] },
      // the reset winding returns the magnetising energy to Vin through Ddemag
      { from: 'Ddemag.cathode', to: [165, 60] },
      { from: 'Ddemag.anode', to: 'T1.r0', via: [[165, 190]] },
      { from: 'T1.p1', to: [270, 320], via: [[270, 170]] },
      { from: 'T1.r1', to: [320, 320] },
      // secondary: forward diode, freewheel, output LC on their own isolated return
      { from: 'T1.s0', to: 'Dfwd.anode', via: [[340, 90]] },
      { from: 'T1.s1', to: [640, 320], via: [[340, 320]] },
      { from: 'Dfwd.cathode', to: [470, 90] },
      { from: 'Dfw.cathode', to: [470, 90] },
      { from: 'Dfw.anode', to: [470, 320] },
      { from: 'Lout.p0', to: [470, 90] },
      { from: 'Lout.p1', to: '@port.VOUT', via: [[690, 90]] },
      { from: 'Cout.p0', to: [580, 90] },
      { from: 'Cout.p1', to: [580, 320] },
    ],
    synth: (b) => [
      srcDC(60, 180), gnd(120, 320), isoGnd(430, 320), loadR(640, 180, 90, 320), port(690, 90, 'VOUT'),
      sig(194, 90, 'g1'), ctrlIC(b, 250, 360, ['g1']),   // 'left' (house style for a vertical FET): 'down' ran the stub along the device outline
    ],
  },
  // ── isolated: two-switch forward ──────────────────────────────────────────
  two_switch_forward: {
    size: [910, 400],   // 910 wide: at 880 the VOUT port label ran off the edge
    place: {
      Q1:   { draw: (b) => mosfetV('Q1', b, 320, 120, 'right', true) },
      Q2:   { draw: (b) => mosfetV('Q2', b, 320, 290, 'right', true) },
      D1:   { draw: (b) => diode('D1', b, 230, 210, 'up', 'left', true) },   // clamp: gnd → primary top
      D2:   { draw: (b) => diode('D2', b, 270, 110, 'up', 'left', true) },   // clamp: primary bottom → Vin
      T1:   { draw: (b) => xfmr('T1', b, 430, 205, { h: 80, labelDy: -24 }) },
      Dfwd: { draw: (b) => diode('Dfwd', b, 540, 130, 'right') },
      Dfw:  { draw: (b) => diode('Dfw', b, 600, 200, 'up', 'right') },
      Lout: { draw: (b) => indH('Lout', b, 670, 130) },
      Cout: { draw: (b) => capV('Cout', b, 720, 220) },
    },
    wires: [
      { from: '@src.p0', to: 'Q1.drain', via: [[60, 70], [320, 70]] },       // Vin+ rail (D2 taps it)
      { from: '@src.p1', to: [320, 320], via: [[60, 320]] },                 // primary return rail (D1 taps it)
      { from: 'Q1.source', to: 'T1.p0', via: [[320, 165]] },
      { from: 'T1.p1', to: 'Q2.drain', via: [[320, 245]] },
      { from: 'Q2.source', to: [320, 320] },
      { from: 'D1.cathode', to: [320, 165], via: [[230, 165]] },
      { from: 'D1.anode', to: [230, 320] },
      { from: 'D2.cathode', to: [270, 70] },
      { from: 'D2.anode', to: [320, 245], via: [[270, 245]] },
      // secondary: forward diode + freewheel into Lout / Cout, on their own isolated return
      { from: 'T1.s0', to: 'Dfwd.anode', via: [[480, 165], [480, 130]] },
      { from: 'T1.s1', to: [800, 320], via: [[480, 245], [480, 320]] },
      { from: 'Dfwd.cathode', to: [600, 130] },
      { from: 'Dfw.cathode', to: [600, 130] },
      { from: 'Dfw.anode', to: [600, 320] },
      { from: 'Lout.p0', to: [600, 130] },
      { from: 'Lout.p1', to: '@port.VOUT', via: [[850, 130]] },
      { from: 'Cout.p0', to: [720, 130] },
      { from: 'Cout.p1', to: [720, 320] },
    ],
    synth: (b) => [
      srcDC(60, 200), gnd(150, 320), isoGnd(540, 320), loadR(800, 220, 130, 320), port(850, 130, 'VOUT'),
      sig(294, 120, 'g1', 'down'), sig(294, 290, 'g2'),
      ctrlIC(b, 470, 360, ['g1', 'g2']),
    ],
  },
  // ── isolated: active-clamp forward ────────────────────────────────────────
  acf: {
    size: [920, 400],
    place: {
      Q1:    { draw: (b) => mosfetV('Q1', b, 320, 145, 'right', true) },
      Sc:    { draw: (b) => mosfetV('Sc', b, 220, 110, 'right', true) },      // active-clamp switch
      Cc:    { draw: (b) => capV('Cc', b, 220, 170, 'left') },
      // centred (middle-anchored, so a wide value grows both ways and clears Q1's riser); offset right
      // it sat nearer SRfwd than T1
      T1:    { draw: (b) => xfmr('T1', b, 380, 195, { h: 90, labelDy: -22 }) },
      SRfwd: { draw: (b) => mosfetH('SRfwd', b, 500, 120, true, true) },
      SRfw:  { draw: (b) => mosfetV('SRfw', b, 570, 195, 'right', true) },
      Lout:  { draw: (b) => indH('Lout', b, 640, 120) },
      Cout:  { draw: (b) => capV('Cout', b, 700, 210) },
    },
    wires: [
      { from: '@src.p0', to: 'Q1.drain', via: [[60, 70], [320, 70]] },        // Vin+ rail (Sc taps it)
      { from: '@src.p1', to: [320, 300], via: [[60, 300]] },                  // primary return rail
      { from: 'Q1.source', to: [320, 185] },                                  // sw node
      { from: [320, 185], to: 'T1.p0', via: [[370, 185]] },
      { from: 'T1.p1', to: [320, 300], via: [[320, 240]] },
      // active-clamp leg: Sc (Vin → clamp node) in series with Cc (clamp node → switch node)
      { from: 'Sc.drain', to: [220, 70] },
      { from: 'Sc.source', to: 'Cc.p0' },
      { from: 'Cc.p1', to: [320, 185], via: [[260, 190], [260, 185]] },
      // secondary: synchronous forward + freewheel FETs into Lout / Cout, on their own isolated return
      { from: 'T1.s0', to: 'SRfwd.source', via: [[440, 150], [440, 120]] },
      { from: 'SRfwd.drain', to: [570, 120] },
      { from: 'T1.s1', to: [770, 300], via: [[440, 240], [440, 300]] },
      { from: 'SRfw.drain', to: [570, 120] },
      { from: 'SRfw.source', to: [570, 300] },
      { from: 'Lout.p0', to: [570, 120] },
      { from: 'Lout.p1', to: '@port.VOUT', via: [[830, 120]] },
      { from: 'Cout.p0', to: [700, 120] },
      { from: 'Cout.p1', to: [700, 300] },
    ],
    synth: (b) => [
      srcDC(60, 190), gnd(150, 300), isoGnd(500, 300), loadR(770, 210, 120, 300), port(830, 120, 'VOUT'),
      sig(294, 145, 'g1'), sig(194, 110, 'gc'), sig(500, 146, 'sr1', 'down'), sig(544, 195, 'sr2'),
      ctrlIC(b, 400, 355, ['g1', 'gc', 'sr1', 'sr2']),   // x=400: at 160 the 'U1' ref sat on the primary return rail
    ],
  },
  // ── isolated: push-pull (centre-tapped primary and secondary) ─────────────
  push_pull: {
    size: [880, 430],
    place: {
      // -32: centred, the value ran across the damper riser at x=250
      T1:   { draw: (b) => xfmr('T1', b, 280, 170, { h: 130, ct: 'both', labelDy: -14, labelDx: -32 }) },
      Q1:   { draw: (b) => mosfetV('Q1', b, 200, 130, 'right', true) },
      Q2:   { draw: (b) => mosfetV('Q2', b, 200, 266, 'right', true) },
      Rdmp: { draw: (b) => resV('Rdmp', b, 105, 130, 'left') },
      Cdmp: { draw: (b) => capV('Cdmp', b, 105, 200, 'left') },
      Dtop: { draw: (b) => diode('Dtop', b, 400, 90, 'right') },
      Dbot: { draw: (b) => diode('Dbot', b, 400, 250, 'right', 'below') },
      Lout: { draw: (b) => indH('Lout', b, 540, 90) },
      Cout: { draw: (b) => capV('Cout', b, 620, 190) },
    },
    wires: [
      // VIN+ → primary centre tap, clear of the winding-end rails AND of Q1's gate lead at x=174
      { from: '@src.p0', to: 'T1.pct', via: [[60, 90], [160, 90], [160, 170]] },
      { from: '@src.p1', to: [200, 320], via: [[60, 320]] },
      // centre-tapped primary: each half drives its own switch; Q1's source routes LEFT to gnd,
      // NOT through Q2
      { from: 'T1.p0', to: 'Q1.drain', via: [[200, 105]] },
      { from: 'Q1.source', to: [140, 320], via: [[200, 185], [140, 185]] },
      { from: 'T1.p1', to: 'Q2.drain', via: [[200, 235]] },
      { from: 'Q2.source', to: [200, 320] },
      // real RC damper across the primary (pri_top → Rdmp → Cdmp → pri_bot), looped left of VIN+
      { from: 'Rdmp.p0', to: [250, 105], via: [[105, 55], [250, 55]] },
      { from: 'Rdmp.p1', to: 'Cdmp.p0' },
      { from: 'Cdmp.p1', to: [200, 235], via: [[105, 235]] },
      // secondary full-wave rectifier: both winding ends → Dtop / Dbot → the output rail
      { from: 'T1.s0', to: 'Dtop.anode', via: [[290, 90]] },
      { from: 'Dtop.cathode', to: [480, 90] },
      { from: 'T1.s1', to: 'Dbot.anode', via: [[290, 250]] },
      { from: 'Dbot.cathode', to: [480, 90], via: [[480, 250]] },
      // secondary centre tap = the output return, isolated from the primary earth
      { from: 'T1.sct', to: [700, 330], via: [[340, 170], [340, 330]] },
      { from: 'Lout.p0', to: [480, 90] },
      { from: 'Lout.p1', to: '@port.VOUT', via: [[760, 90]] },
      { from: 'Cout.p0', to: [620, 90] },
      { from: 'Cout.p1', to: [620, 330] },
    ],
    synth: (b) => [
      srcDC(60, 170), gnd(110, 320), isoGnd(450, 330), loadR(700, 190, 90, 330), port(760, 90, 'VOUT'),
      sig(174, 130, 'g1'), sig(174, 266, 'g2'), ctrlIC(b, 400, 390, ['g1', 'g2']),
    ],
  },
  // ── isolated: Weinberg (coupled input choke + centre-tapped secondary) ────
  weinberg: {
    size: [1000, 440],
    place: {
      // -34: at -20 the value sat on the Vin rail at y=90
      L1:    { draw: (b) => xfmr('L1', b, 210, 170, { h: 80, labelDx: -18, labelDy: -34 }) },
      Rdcra: { draw: (b) => resH('Rdcra', b, 280, 250, 'above') },
      Rdcrb: { draw: (b) => resH('Rdcrb', b, 280, 320, 'below') },
      // -29: centred, the value sat on the sec_c riser at x=600. -18: at the default the value line sat
      // ON the top of the core it names — a 4-winding block is 200 px tall, so its label needs the room.
      T1:    { draw: (b) => xfmr4('T1', b, 560, 230, { labelDx: -29, labelDy: -18 }).el },
      S1:    { draw: (b) => mosfetV('S1', b, 470, 180, 'right', true) },
      S2:    { draw: (b) => mosfetV('S2', b, 470, 330, 'right', true, false, 0, 8) },   // +8y: the ref sat on the y=320 rail
      Dpos:  { draw: (b) => diode('Dpos', b, 660, 110, 'right', 'below') },
      Dneg:  { draw: (b) => diode('Dneg', b, 660, 350, 'right', 'below', false, 0, 25) },
      Cout:  { draw: (b) => capV('Cout', b, 820, 245) },
    },
    wires: [
      { from: '@src.p0', to: [220, 90], via: [[60, 90]] },                    // Vin+ rail reaches both L1 taps
      { from: '@src.p1', to: [470, 380], via: [[60, 380]] },                  // primary return rail
      { from: 'L1.p0', to: [200, 90] },
      { from: 'L1.s0', to: [220, 90] },
      // Each choke winding returns through its OWN DCR loop-breaker to a DIFFERENT primary half — the
      // defining dual-inductor structure. Each feed turns up its own clear column and enters its
      // terminal horizontally: run along the neighbour's lane and the corner lands ON that terminal,
      // shorting the two primary halves together.
      { from: 'L1.p1', to: 'Rdcra.p0', via: [[200, 250]] },
      { from: 'Rdcra.p1', to: 'T1.a1', via: [[520, 250], [520, 210]] },
      { from: 'L1.s1', to: 'Rdcrb.p0', via: [[220, 320]] },
      { from: 'Rdcrb.p1', to: 'T1.b0', via: [[320, 320], [320, 290], [530, 290], [530, 250]] },
      // each primary half's outer end → its own switch → ground. S1's source drops at x=430 (the earth
      // symbol's column): straight down x=470 it ran through S2's body.
      { from: 'T1.a0', to: 'S1.drain', via: [[470, 140]] },
      { from: 'S1.source', to: [430, 380], via: [[430, 206]] },
      { from: 'T1.b1', to: 'S2.drain', via: [[470, 320]] },
      { from: 'S2.source', to: [470, 380] },
      // centre-tapped full-wave secondary: outer ends → Dpos / Dneg, inner ends → CT → isolated return
      { from: 'T1.c0', to: 'Dpos.anode', via: [[600, 140], [600, 110]] },
      { from: 'T1.d1', to: 'Dneg.anode', via: [[600, 320], [600, 350]] },
      { from: 'Dpos.cathode', to: '@port.VOUT', via: [[960, 110]] },
      { from: 'Dneg.cathode', to: [740, 110], via: [[740, 350]] },
      { from: 'T1.c1', to: 'T1.d0', via: [[610, 210], [610, 250]] },
      { from: [610, 250], to: [900, 380], via: [[610, 380]] },                // CT → isolated return rail
      { from: 'Cout.p0', to: [820, 110] },
      { from: 'Cout.p1', to: [820, 380] },
    ],
    synth: (b) => [
      srcDC(60, 210), gnd(430, 380), isoGnd(700, 380), loadR(900, 245, 110, 380), port(960, 110, 'VOUT'),
      sig(444, 180, 'g1'), sig(444, 330, 'g2'), ctrlIC(b, 620, 60, ['g1', 'g2']),
    ],
  },
  // ── bridge: asymmetric half bridge ────────────────────────────────────────
  ahb: (present) => {
    const tx = 540, ty = 200, h = 90
    const sec = secondaryFrag(present, tx, ty, h,
      { fb: ['Dr1', 'Dr3', 'Dr2', 'Dr4'], d1: 'Dr1', d2: 'Dr2', lout: 'Lout', lo2: 'Lo2' })
    return {
      size: [1120, 420],
      place: {
        Q1:   { draw: (b) => mosfetV('Q1', b, 180, 128, 'right', true) },
        Q2:   { draw: (b) => mosfetV('Q2', b, 180, 250, 'right', true) },
        Cb:   { draw: (b) => capV('Cb', b, 300, 122, 'left') },         // DC-blocking cap into the primary
        T1:   { draw: (b) => xfmr('T1', b, tx, ty, { h, ct: present.has('Dr3') || present.has('Lo2') ? undefined : 'right', labelDy: -24 }) },
        Rdmp: { draw: (b) => resV('Rdmp', b, 430, 190, 'right') },      // RC damper across the primary
        Cdmp: { draw: (b) => capV('Cdmp', b, 430, 258, 'right') },
        ...sec.place,
      },
      wires: [
        { from: '@src.p0', to: [300, 80], via: [[60, 80]] },            // Vin+ rail (Q1 + Cb tap it)
        { from: '@src.p1', to: [180, 330], via: [[60, 330]] },
        { from: 'Q1.drain', to: [180, 80] },
        { from: 'Q1.source', to: 'Q2.drain' },                          // sw node
        { from: 'Q2.source', to: [180, 330] },
        { from: 'Cb.p0', to: [300, 80] },
        { from: 'Cb.p1', to: 'T1.p0', via: [[300, 155]] },              // cb_mid → primary top
        { from: 'T1.p1', to: [180, 200], via: [[tx - 10, 300], [220, 300], [220, 200]] },   // primary return → sw node
        { from: 'Rdmp.p0', to: [430, 155] },
        { from: 'Rdmp.p1', to: 'Cdmp.p0' },
        { from: 'Cdmp.p1', to: [430, 300] },
        ...sec.wires,
      ],
      synth: (b) => [
        srcDC(60, 190), gnd(120, 330), ...sec.synthEls(b),
        sig(154, 128, 'g1'), sig(154, 250, 'g2'), ctrlIC(b, 120, 45, ['g1', 'g2']),
      ],
    }
  },
  // ── bridge: phase-shifted full bridge ─────────────────────────────────────
  psfb: (present) => {
    const tx = 330, ty = 195, h = 90
    const sec = secondaryFrag(present, tx, ty, h,
      { fb: ['Dr1', 'Dr3', 'Dr2', 'Dr4'], d1: 'Dr1', d2: 'Dr2', lout: 'Lout', lo2: 'Lo2' })
    return {
      size: [1040, 430],
      place: {
        // Leg A stands at 130, not 150: the snubber loop between the legs is bounded on the right by the
        // gD gate flag, so at 150 it was 60 px wide and a 24 px capacitor plus a ~52 px label do not fit.
        QA: { draw: (b) => mosfetV('QA', b, 130, 128, 'right', true) },
        QB: { draw: (b) => mosfetV('QB', b, 130, 246, 'right', true) },
        QC: { draw: (b) => mosfetV('QC', b, 280, 128, 'right', true) },
        // +8/+60: the lane between the re-entry column (295) and T1's body (308) is narrower than the
        // ref itself, so the label drops clear of T1 instead.
        QD: { draw: (b) => mosfetV('QD', b, 280, 246, 'right', true, false, 8, 60) },
        Lr: { draw: (b) => indH('Lr', b, 228, 200) },
        // Centred and raised: at a +130 offset the value parked over the rectifier, nearer Dr1 than T1.
        T1: { draw: (b) => xfmr('T1', b, tx, ty, { h, ct: present.has('Dr3') || present.has('Lo2') ? undefined : 'right', labelDy: -70 }) },
        Crc_pri: { draw: (b) => capV('Crc_pri', b, 168, 270, 'right') },
        Rrc_pri: { draw: (b) => resH('Rrc_pri', b, 220, 300, 'below') },
        // Secondary bleeders, full-bridge only (Psfb.cpp adds them with Dr3/Dr4). Labelled left: on the
        // right the block had to clear Dr3's column and ended up across another part's wire.
        Rbsa: { draw: (b) => resV('Rbsa', b, 380, 310, 'left') },
        Rbsb: { draw: (b) => resV('Rbsb', b, 590, 310, 'left') },
        ...sec.place,
      },
      wires: [
        { from: '@src.p0', to: [280, 80], via: [[60, 80]] },
        { from: '@src.p1', to: [280, 340], via: [[60, 340]] },
        { from: 'QA.drain', to: [130, 80] },
        { from: 'QA.source', to: 'QB.drain' },                               // leg-A mid
        { from: 'QB.source', to: [130, 340] },
        { from: 'QC.drain', to: [280, 80] },
        { from: 'QC.source', to: 'QD.drain' },                               // leg-C mid
        { from: 'QD.source', to: [280, 340] },
        // The tank crosses the leg-C column to reach T1, and where it crosses decides whether the
        // drawing is readable: at y=200 it crossed 10 px above midC's junction dot, so the Lr→T1 wire
        // and the switch node read as one three-way node. Cross high in the QC/QD window instead.
        { from: [130, 200], to: 'Lr.p0' },
        { from: 'Lr.p1', to: 'T1.p0', via: [[270, 200], [270, 175], [300, 175], [300, 150]] },
        // primary return enters midC from the right at its own y-band, clear of QD's source-to-gnd drop
        { from: 'T1.p1', to: [280, 210], via: [[tx - 10, 270], [295, 270], [295, 210]] },
        // real RC snubber between the two leg midpoints, its return entering midC from the left
        { from: 'Crc_pri.p0', to: [168, 200] },                               // taps the tank rail from below
        { from: 'Crc_pri.p1', to: 'Rrc_pri.p0', via: [[168, 300]] },
        { from: 'Rrc_pri.p1', to: [280, 210], via: [[240, 300], [240, 210]] },
        ...sec.wires,
        ...(present.has('Rbsa') ? [
          { from: 'Rbsa.p0', to: [380, 195] },
          { from: 'Rbsa.p1', to: [420, 330], via: [[380, 330]] },
          { from: 'Rbsb.p0', to: [534, 240], via: [[590, 240]] },
          { from: 'Rbsb.p1', to: [590, 330] },
        ] : []),
      ],
      synth: (b) => [
        srcDC(60, 175), gnd(110, 340), ...sec.synthEls(b),
        sig(104, 128, 'gA'), sig(104, 246, 'gB'), sig(254, 128, 'gC'), sig(254, 246, 'gD'),
        ctrlIC(b, 90, 385, ['gA', 'gB', 'gC', 'gD']),
      ],
    }
  },
  // ── bridge: phase-shifted half bridge (3-level NPC) ───────────────────────
  pshb: (present) => {
    const tx = 540, ty = 240, h = 90
    const sec = secondaryFrag(present, tx, ty, h,
      { fb: ['Dr1', 'Dr3', 'Dr2', 'Dr4'], d1: 'Dr1', d2: 'Dr2', lout: 'Lout', lo2: 'Lo2' })
    return {
      size: [1140, 460],
      place: {
        CsHi: { draw: (b) => capV('CsHi', b, 150, 90, 'left', -22) },      // -22 to align with CsLo below it
        // Labelled left, clear of DC2's lane and of the source glyph: on the right the value ran into
        // the g3 flag, and nudging it back 10 px printed the name across the cap's own plates.
        CsLo: { draw: (b) => capV('CsLo', b, 150, 260, 'left', -22) },
        S1:   { draw: (b) => mosfetV('S1', b, 250, 100, 'right', true) },
        S2:   { draw: (b) => mosfetV('S2', b, 250, 180, 'right', true) },
        S3:   { draw: (b) => mosfetV('S3', b, 250, 270, 'right', true) },
        S4:   { draw: (b) => mosfetV('S4', b, 250, 350, 'right', true) },
        DC1:  { draw: (b) => diode('DC1', b, 180, 140, 'right') },
        // DC2 sits a column left with its label ABOVE: at x=200 with a label below, its VF value was
        // printed across the earth rail and into S4's body.
        DC2:  { draw: (b) => diode('DC2', b, 180, 310, 'left', 'above') },
        Lr:   { draw: (b) => indH('Lr', b, 340, 210) },
        T1:   { draw: (b) => xfmr('T1', b, tx, ty, { h, ct: present.has('Dr3') || present.has('Lo2') ? undefined : 'right', labelDy: -24 }) },
        Crc_pri: { draw: (b) => capV('Crc_pri', b, 450, 235, 'right') },
        Rrc_pri: { draw: (b) => resV('Rrc_pri', b, 450, 280, 'right') },
        ...sec.place,
      },
      wires: [
        { from: '@src.p0', to: [150, 70], via: [[60, 70]] },
        { from: '@src.p1', to: [190, 340], via: [[60, 340]] },              // primary return (ends at S4's column)
        { from: 'CsHi.p0', to: [150, 70] },
        { from: 'CsHi.p1', to: 'CsLo.p0' },                                 // split-bus neutral
        { from: 'CsLo.p1', to: [150, 340] },
        { from: 'S1.drain', to: [150, 70], via: [[250, 70]] },
        { from: 'S1.source', to: 'S2.drain' },
        // bridge_a taps the TOP of the S2/S3 window: the primary return crosses this column at y=235 on
        // its way to the neutral, and with the node in the middle the crossing sat 10 px under the
        // junction dot — the return read as tied to the switch node, the exact short the clamp diodes
        // exist to avoid.
        { from: 'S2.source', to: [250, 210] },
        { from: 'S3.drain', to: [250, 210] },
        { from: 'S3.source', to: 'S4.drain' },
        // S4 straddles the y=340 return, so its source reaches the rail's end BELOW the device
        { from: 'S4.source', to: [190, 340], via: [[250, 400], [190, 400]] },
        // clamp diodes tie the inner nodes to the neutral
        { from: 'DC1.anode', to: [150, 140] },
        { from: 'DC1.cathode', to: [250, 140] },
        { from: 'DC2.cathode', to: [150, 170], via: [[120, 310], [120, 170]] },   // up a CLEAR lane: at x=150 it ran through CsLo's body
        { from: 'DC2.anode', to: [250, 310] },
        // stack output → series Lr → primary; primary return → the neutral
        { from: [250, 210], to: 'Lr.p0' },
        { from: 'Lr.p1', to: 'T1.p0', via: [[440, 210], [440, 195]] },
        { from: 'T1.p1', to: [150, 235], via: [[tx - 10, 300], [280, 300], [280, 235]] },
        // real RC snubber across the primary, in its own column with the labels toward the clear gap
        { from: 'Crc_pri.p0', to: [450, 195] },
        { from: 'Crc_pri.p1', to: 'Rrc_pri.p0' },
        ...sec.wires,
      ],
      synth: (b) => [
        srcDC(60, 180), gnd(110, 340), ...sec.synthEls(b),
        sig(224, 100, 'g1'), sig(224, 180, 'g2'), sig(224, 270, 'g3'), sig(224, 350, 'g4'),
        ctrlIC(b, 90, 400, ['g1', 'g2', 'g3', 'g4']),
      ],
    }
  },
  // ── bridge: LLC and SRC resonant half bridges ─────────────────────────────
  llc: resonantHalfBridge,
  src: resonantHalfBridge,
  // ── bridge: dual active bridge ────────────────────────────────────────────
  dab: () => {
    const top = 90, gy = 300, mid = 195
    // one bridge leg: hi FET (rail → mid) + lo FET (mid → return) at column x, with its balancing
    // divider in a clear lane at bx. bdx nudges the bias labels off a lane that would cross them.
    const leg = (qh, ql, rh, rl, x, bx, qdx = 0, bdx = 0) => {
      const bside = bx < x ? 'left' : 'right'
      return {
        place: {
          [qh]: { draw: (b) => mosfetV(qh, b, x, 140, 'right', true, false, qdx) },
          [ql]: { draw: (b) => mosfetV(ql, b, x, 250, 'right', true, false, qdx) },
          [rh]: { draw: (b) => resV(rh, b, bx, 140, bside, bdx) },
          [rl]: { draw: (b) => resV(rl, b, bx, 250, bside, bdx) },
        },
        wires: [
          { from: `${qh}.drain`, to: [x, top] },
          { from: `${qh}.source`, to: [x, mid] },
          { from: `${ql}.drain`, to: [x, mid] },
          { from: `${ql}.source`, to: [x, gy] },
          { from: `${rh}.p0`, to: [bx, top] },
          { from: `${rh}.p1`, to: [bx, mid] },
          { from: `${rl}.p0`, to: [bx, mid] },
          { from: `${rl}.p1`, to: [bx, gy] },
          { from: [Math.min(bx, x), mid], to: [Math.max(bx, x), mid] },
        ],
        synthEls: () => [sig(x - 26, 140, 'g' + qh.slice(-1)), sig(x - 26, 250, 'g' + ql.slice(-1))],
      }
    }
    const frag = merge(
      leg('QA', 'QB', 'RbiasA_hi', 'RbiasA_lo', 230, 130),
      // +14: QD's ref sat on the snubber drop at x=370. The bias lane runs at 410, not 450: at 450 the
      // labels reached the Lr→p0 riser and nudging them left printed them through their own zigzags.
      leg('QC', 'QD', 'RbiasC_hi', 'RbiasC_lo', 350, 410, 14, 0),
      leg('QE', 'QF', 'RbiasE_hi', 'RbiasE_lo', 770, 700),
      leg('QG', 'QH', 'RbiasG_hi', 'RbiasG_lo', 890, 990, 0, 14),   // +14: the bias labels sat on Cout's column
      {
        place: {
          Lr: { draw: (b) => indH('Lr', b, 528, 380, 'below') },
          T1: { draw: (b) => xfmr('T1', b, 570, mid, { h: 80, labelDy: -26 }) },
          // 'below': above, the label sat on the primary return rail at y=300
          Rrc_pri: { draw: (b) => resH('Rrc_pri', b, 400, 330, 'below') },
          Crc_pri: { draw: (b) => capH('Crc_pri', b, 460, 330, 'below') },
          Cout: { draw: (b) => capV('Cout', b, 1010, mid) },
        },
        wires: [
          { from: '@src.p0', to: [410, top], via: [[50, top]] },     // rails span both primary legs + bias lanes
          { from: '@src.p1', to: [410, gy], via: [[50, gy]] },
          // series Lr: midA → down a clean column at 270 (x=230 would pass through QB's source-to-gnd
          // junction dot) → Lr → up a clear column at 520 (x=560 would run through T1's p1) → T1 p0
          { from: [230, mid], to: 'Lr.p0', via: [[270, mid], [270, 380]] },
          { from: 'Lr.p1', to: 'T1.p0', via: [[556, 360], [520, 360], [520, 155]] },
          { from: 'T1.p1', to: [350, 215], via: [[540, 235], [540, 215]] },   // clear of RbiasC_lo's body
          // RC snubber across the primary, in the clear lane below. Drop from midC at x=370, NOT 350,
          // which would overlap QD's source-to-gnd drop and short the primary to ground.
          { from: [370, mid], to: 'Rrc_pri.p0', via: [[370, 330]] },
          { from: 'Rrc_pri.p1', to: 'Crc_pri.p0' },
          { from: 'Crc_pri.p1', to: 'Lr.p0', via: [[500, 330]] },    // ends on Lr's own pin (= midA)
          // T1 sec-top → midE ; T1 sec-bot → midG, each approached horizontally at mid level
          { from: 'T1.s0', to: [700, mid], via: [[600, 155], [600, mid]] },
          { from: 'T1.s1', to: [890, mid], via: [[620, 235], [620, 340], [820, 340], [820, mid]] },   // x=820, not 840: that riser ran through the gH gate flag
          // output rails span the secondary bias lanes → Cout ∥ load → VOUT
          { from: [700, top], to: '@port.VOUT', via: [[1150, top]] },
          { from: [700, gy], to: [1090, gy] },
          { from: 'Cout.p0', to: [1010, top] },
          { from: 'Cout.p1', to: [1010, gy] },
        ],
        synthEls: (b) => [
          srcDC(50, 190), gnd(50, gy), isoGnd(960, gy), loadR(1090, mid, top, gy), port(1150, top, 'VOUT'),
          // U1 sits at x=180, NOT 300: centred on 300 the midA → Lr lane runs straight across its top,
          // reading as if the primary mid-node landed on the PWM.
          ctrlIC(b, 180, 415, ['gA', 'gB', 'gC', 'gD']),
          icBox('UDR', b, 830, 415, 70, 76, [], ['gE', 'gF', 'gG', 'gH'], 'SR DRV'),
        ],
      },
    )
    return { size: [1300, 470], place: frag.place, wires: frag.wires, synth: (b) => frag.synthEls(b) }
  },
  // ── bridge: CLLC / CLLLC (bidirectional resonant, SR output bridge) ───────
  cllc: (present) => cllcLike(present, false),
  clllc: (present) => cllcLike(present, true),
  // ── PFC: single-phase boost power-factor corrector ────────────────────────
  pfc: {
    size: [880, 520],
    place: {
      // -30y: at y=197 the value sat on the bus lane; shifted left it landed on the VAC source
      D1: { draw: (b) => diode('D1', b, 180, 197, 'up', 'left', false, 0, -30) },
      D3: { draw: (b) => diode('D3', b, 180, 273, 'up', 'left') },
      D2: { draw: (b) => diode('D2', b, 250, 197, 'up') },
      D4: { draw: (b) => diode('D4', b, 250, 273, 'up', 'above', false, 29) },   // +29: the VF value sat on the riser at x=290
      Rref: { draw: (b) => resV('Rref', b, 130, 340, 'left', -25) },             // -25: the label sat on the reference riser
      Rsense: { draw: (b) => resH('Rsense', b, 280, 150, 'above') },
      L: { draw: (b) => indH('L', b, 340, 150) },
      SW: { draw: (b) => mosfetV('SW', b, 410, 225) },
      D5: { draw: (b) => diode('D5', b, 480, 150, 'right') },
      Cout: { draw: (b) => capV('Cout', b, 560, 235) },
      Rv1: { draw: (b) => resV('Rv1', b, 700, 190, 'right') },                   // 'right': on the left the labels sat on the load column
      Rv2: { draw: (b) => resV('Rv2', b, 700, 262, 'right') },
      Iv:  { draw: (b) => icBox('Iv', b, 160, 460, 60, 56, ['vs'], ['iv'], '∫') },
      Sgv: { draw: (b) => icBox('Sgv', b, 300, 460, 60, 56, ['iv', 'vs'], ['gv'], 'EA') },
      Mv:  { draw: (b) => icBox('Mv', b, 440, 460, 60, 56, ['busP', 'gv'], ['vth'], '×') },
      Cmp: { draw: (b) => icBox('Cmp', b, 580, 460, 60, 56, ['nL', 'vth'], ['g'], 'PWM') },
    },
    wires: [
      // full-bridge line rectifier: acLine into the D1/D2 midpoints, acNeutral wrapping under it
      { from: '@src.p0', to: [180, 235], via: [[120, 185], [120, 235]] },
      { from: '@src.p1', to: [250, 235], via: [[95, 215], [95, 360], [290, 360], [290, 235]] },
      { from: 'D1.anode', to: [180, 235] },
      { from: 'D1.cathode', to: [180, 150] },
      { from: 'D3.cathode', to: [180, 235] },
      { from: 'D3.anode', to: [180, 320] },
      { from: 'D2.anode', to: [250, 235] },
      { from: 'D2.cathode', to: [250, 150] },
      { from: 'D4.cathode', to: [250, 235] },
      { from: 'D4.anode', to: [250, 320] },
      { from: [180, 150], to: 'Rsense.p0', via: [[260, 150]] },                 // rectified bus
      // return rail — it starts at Rref's column, not at the bridge, so the reference bleeder reaches it
      { from: 'Rref.p0', to: [700, 320] },
      // boost cell: bus → Rsense (current shunt) → L → switch node; SW shunt, D5 into the bus cap
      { from: 'Rsense.p1', to: 'L.p0' },
      { from: 'L.p1', to: 'D5.anode' },
      { from: 'SW.drain', to: [410, 150] },
      { from: 'SW.source', to: [410, 320] },
      { from: 'D5.cathode', to: '@port.VBUS', via: [[740, 150]] },
      { from: 'Cout.p0', to: [560, 150] },
      { from: 'Cout.p1', to: [560, 320] },
      // output-voltage divider feeding the control law (vout → Rv1 → vs → Rv2 → gnd), in its own lane
      { from: 'Rv1.p0', to: [700, 150] },
      { from: 'Rv1.p1', to: 'Rv2.p0' },
      { from: 'Rv2.p1', to: [700, 320] },
    ],
    synth: (b) => [
      srcAC(70, 200, 'VAC'), gnd(180, 320), loadR(630, 235, 150, 320, 21), port(740, 150, 'VBUS'),
      sig(255, 150, 'busP', 'down', 10), sig(303, 150, 'nL', 'down'),   // +10: the flag text sat on D2's riser
      sig(700, 226, 'vs', 'right'), sig(384, 225, 'g'),
      txt(60, 405, 'CONTROL — average-current-mode PFC law', 'sch-blk', 'start'),
      ctrlIC(b, 740, 460, ['g'], 'U1', 'CTRL'),
    ],
  },
  // ── PFC: three-phase Vienna rectifier ─────────────────────────────────────
  vienna: () => {
    const busP = 60, neu = 270, busN = 340, X = 130
    const cols = [{ x: 155, ph: 'a' }, { x: 365, ph: 'b' }, { x: 575, ph: 'c' }]
    // One phase leg: phase-in → Rs (current shunt) → L → node X, with Dp↑ to busP and Dn↑ from busN,
    // plus the bidirectional midpoint switch (SW+SQ, common source) clamping X to the neutral rail.
    const leg = ({ x, ph }) => {
      const sx = x + 48
      return {
        place: {
          [`Rs${ph}`]: { draw: (b) => resV(`Rs${ph}`, b, x - 48, 435, 'left') },
          [`L${ph}`]:  { draw: (b) => indV(`L${ph}`, b, x - 48, 375, 'left') },
          [`Dp${ph}`]: { draw: (b) => diode(`Dp${ph}`, b, x, 95, 'up') },
          [`Dn${ph}`]: { draw: (b) => diode(`Dn${ph}`, b, x, 300, 'up') },
          [`SW${ph}`]: { draw: (b) => mosfetV(`SW${ph}`, b, sx, 160, 'right', true) },
          [`SQ${ph}`]: { draw: (b) => mosfetV(`SQ${ph}`, b, sx, 220, 'right', true, true) },   // flipped: common source with SW
        },
        wires: [
          { from: `@port.${ph}`, to: `Rs${ph}.p1` },
          { from: `Rs${ph}.p0`, to: `L${ph}.p1` },
          { from: `L${ph}.p0`, to: [x, X], via: [[x - 48, X]] },
          { from: `Dp${ph}.cathode`, to: [x, busP] },
          { from: `Dp${ph}.anode`, to: [x, X] },
          { from: `Dn${ph}.cathode`, to: [x, X] },
          { from: `Dn${ph}.anode`, to: [x, busN] },
          { from: [x, X], to: `SW${ph}.drain`, via: [[sx, X]] },
          { from: `SW${ph}.source`, to: `SQ${ph}.source` },
          { from: `SQ${ph}.drain`, to: [sx, neu] },
        ],
        synthEls: () => [
          port(x - 48, 462, ph, 'start'),
          sig(x - 48, 410, `nl${ph}`, 'right'),
          sig(sx - 26, 160, `g${ph}`, 'left', -10), sig(sx - 26, 220, `g${ph}`, 'left', -10),   // -10: the flags sat on the Dn riser
        ],
      }
    }
    // per-phase current-loop block chain (Gvm/Sub/Sum/Add/Mul/Cmp), nets by label per viennaControl
    const phaseRow = (ph, y) => ({
      place: {
        [`Gvm${ph}`]: { draw: (b) => icBox(`Gvm${ph}`, b, 150, y, 60, 56, [ph, 'gcond'], [`gvp${ph}`], '×') },
        [`Sub${ph}`]: { draw: (b) => icBox(`Sub${ph}`, b, 305, y, 60, 56, [`nl${ph}`, ph], [`nmp${ph}`], '−') },
        [`Sum${ph}`]: { draw: (b) => icBox(`Sum${ph}`, b, 460, y, 60, 56, [`nmp${ph}`, `gvp${ph}`], [`err${ph}`], 'Σ') },
        [`Add${ph}`]: { draw: (b) => icBox(`Add${ph}`, b, 615, y, 60, 56, [`err${ph}`, 'bal'], [`errp${ph}`], '+') },
        [`Mul${ph}`]: { draw: (b) => icBox(`Mul${ph}`, b, 770, y, 60, 56, [ph, `errp${ph}`], [`m${ph}`], '×') },
        [`Cmp${ph}`]: { draw: (b) => icBox(`Cmp${ph}`, b, 925, y, 60, 56, [`m${ph}`, 'gnd'], [`g${ph}`], 'PWM') },
      },
    })
    const frag = merge(
      ...cols.map(leg), phaseRow('a', 650), phaseRow('b', 730), phaseRow('c', 810),
      {
        place: {
          Cp: { draw: (b) => capV('Cp', b, 720, 165, 'right') },
          Cn: { draw: (b) => capV('Cn', b, 720, 305, 'right') },
          Rload: { draw: (b) => resV('Rload', b, 790, 200, 'right') },
          Svs:   { draw: (b) => icBox('Svs', b, 150, 560, 60, 56, ['busP', 'busN'], ['vbus'], 'G') },
          Ivolt: { draw: (b) => icBox('Ivolt', b, 305, 560, 60, 56, ['vbus'], ['vint'], '∫') },
          Sg:    { draw: (b) => icBox('Sg', b, 460, 560, 60, 56, ['vint', 'vbus'], ['gcond'], 'EA') },
          Simb:  { draw: (b) => icBox('Simb', b, 615, 560, 60, 56, ['busP', 'busN'], ['imb'], '−') },
          Ibal:  { draw: (b) => icBox('Ibal', b, 770, 560, 60, 56, ['imb'], ['bal'], 'G') },
        },
        wires: [
          // busP / neutral / busN — each spans exactly its own junctions
          { from: [155, busP], to: '@port.BUS+', via: [[850, busP]] },
          { from: [203, neu], to: [720, neu] },
          { from: [155, busN], to: '@port.BUS−', via: [[850, busN]] },
          { from: 'Cp.p0', to: [720, busP] },
          { from: 'Cp.p1', to: [720, neu] },
          { from: 'Cn.p0', to: [720, neu] },
          { from: 'Cn.p1', to: [720, busN] },
          { from: 'Rload.p0', to: [790, busP] },
          { from: 'Rload.p1', to: [790, busN] },
        ],
        synthEls: (b) => [
          port(850, busP, 'BUS+'), port(850, busN, 'BUS−'),
          txt(196, 264, 'N', 'sch-port', 'end'),
          sig(770, busP, 'busP', 'up'), sig(770, busN, 'busN', 'down'),
          txt(60, 505, 'CONTROL — bus-voltage + balance loops, per-phase current loops', 'sch-blk', 'start'),
          ctrlIC(b, 900, 560, ['ga', 'gb', 'gc'], 'U1', 'CTRL'),
        ],
      },
    )
    return { size: [1010, 880], place: frag.place, wires: frag.wires, synth: (b) => frag.synthEls(b) }
  },
  // ── non-isolated: buck ────────────────────────────────────────────────────
  buck: {
    size: [720, 300],
    place: {
      Q1:   { draw: (b) => mosfetH('Q1', b, 180, 80) },
      L1:   { draw: (b) => indH('L1', b, 348, 80) },
      D1:   { draw: (b) => diode('D1', b, 240, 150, 'up', 'right') },        // diode variant
      Q2:   { draw: (b) => mosfetV('Q2', b, 240, 150, 'right', true) },      // synchronous variant
      Cout: { draw: (b) => capV('Cout', b, 430, 150) },
    },
    wires: [
      { from: '@src.p0', to: 'Q1.drain', via: [[70, 80]] },                  // Vin+ rail → high-side switch
      { from: '@src.p1', to: [520, 220], via: [[70, 220]] },                 // return rail → load bottom
      { from: 'Q1.source', to: 'L1.p0' },                                    // switch node (the rectifier taps it)
      { from: 'L1.p1', to: '@port.VOUT', via: [[600, 80]] },                 // Vout rail (Cout + load tap it)
      { from: 'Cout.p0', to: [430, 80], needs: ['Cout'] },
      { from: 'Cout.p1', to: [430, 220], needs: ['Cout'] },
      // free-wheeling path: a diode, or the low-side FET with its body diode drawn beside it
      { from: 'D1.cathode', to: [240, 80], needs: ['D1'] },
      { from: 'D1.anode', to: [240, 220], needs: ['D1'] },
      { from: 'Q2.drain', to: [240, 80], needs: ['Q2'] },
      { from: 'Q2.source', to: [240, 220], needs: ['Q2'] },
      { from: 'D2.cathode', to: [290, 80], needs: ['Q2'] },
      { from: 'D2.anode', to: [290, 220], needs: ['Q2'] },
    ],
    synth: (b, present) => [
      srcDC(70, 150), loadR(520, 150, 80, 220), gnd(300, 220), port(600, 80, 'VOUT'),
      // Q2's intrinsic body diode is a TAS component with role bodyDiode — real, and filtered out of the
      // BOM precisely because it is intrinsic, so it is drawn here rather than placed as a CIAS brick.
      ...(present.has('Q2') ? [diode('D2', b, 290, 150, 'up', 'right', true), sig(214, 150, 'g2')] : []),
      sig(180, 106, 'g1', 'down'),
      ctrlIC(b, 640, 220, present.has('Q2') ? ['g1', 'g2'] : ['g1']),
    ],
  },
  flyback: {
    size: [760, 350],
    // CIAS ref -> { draw(bom) }. The symbol registers its own terminals while drawing, so the wiring
    // below names them ('Q1.drain') instead of repeating their coordinates.
    place: {
      Cin:   { draw: (b) => capV('Cin', b, 150, 165, 'left') },
      Cclmp: { draw: (b) => capV('Cclmp', b, 205, 90, 'left') },
      Rclmp: { draw: (b) => resV('Rclmp', b, 205, 130, 'left') },
      Cres:  { draw: (b) => capV('Cres', b, 205, 215, 'left') },     // QRM only — absent in CCM/DCM/BCM
      T1:    { draw: (b) => xfmr('T1', b, 260, 140, { opp: true, labelDy: -30 }) },
      // +14: the secondary return runs down x=270, straight through "Q1" and its RDS(on) value.
      Q1:    { draw: (b) => mosfetV('Q1', b, 250, 228, 'right', false, false, 14) },
      D1:    { draw: (b) => diode('D1', b, 360, 90, 'right') },
      Cout:  { draw: (b) => capV('Cout', b, 470, 180) },
    },
    // Net-realizing wires. Ends name terminals; `via` carries the corners. `needs` gates each on the
    // presence of its owning CIAS components, so removing a component (e.g. Cres outside QRM) drops its
    // wiring automatically. Junction dots are derived from these routes, never listed.
    wires: [
      { from: '@src.p0', to: 'T1.p0', via: [[70, 70], [250, 70]] },                    // Vin+ rail → T1 primary_start
      { from: '@src.p1', to: 'Q1.source', via: [[70, 260], [250, 260]] },              // primary return rail (earth side)
      { from: 'Cin.p0', to: [150, 70], via: [], needs: ['Cin'] },                      // Cin top → Vin rail
      { from: 'Cin.p1', to: [150, 260], needs: ['Cin'] },                              // Cin bottom → return rail
      { from: 'T1.p1', to: 'Q1.drain', needs: ['T1', 'Q1'] },                          // primary_end → drain (sw)
      { from: 'Rclmp.p1', to: [250, 195], via: [[205, 195]], needs: ['Rclmp'] },       // RC clamp → sw node
      { from: 'Cres.p1', to: [205, 260], needs: ['Cres'] },                            // Cres bottom → return rail
      { from: 'T1.s0', to: 'D1.anode', via: [[270, 90]], needs: ['T1', 'D1'] },        // secondary_end → D1 anode
      { from: 'D1.cathode', to: '@port.VOUT', via: [[660, 90]], needs: ['D1'] },       // D1 cathode → Vout rail → port
      { from: 'T1.s1', to: '@port.RTN', via: [[270, 268]], needs: ['T1'] },            // secondary_start → return rail → port
      { from: 'Cout.p0', to: [470, 90], needs: ['Cout'] },                             // Cout top → Vout rail
      { from: 'Cout.p1', to: [470, 268], needs: ['Cout'] },                            // Cout bottom → sec return rail
    ],
    // Non-CIAS glyphs synthesized around the bricks — the dual of what the TAS assembler adds: the
    // input source, the output load (Rload = Vout^2/Pout), primary earth, the isolated secondary
    // return + Vout ports, and the gate-drive net-label + controller IC. Each registers its own
    // terminals, so the wires above reach them by name too.
    synth: (b) => [
      srcDC(70, 160), loadR(560, 180, 90, 268), gnd(180, 260),
      port(660, 90, 'VOUT'), port(660, 268, 'RTN'),
      sig(224, 228, 'g1', 'down'), ctrlIC(b, 400, 315, ['g1']),
    ],
  },
}

// Every topology has a native layout (ABT #684), so these are the same question — kept as two names
// because callers ask it for two reasons: "can the app draw this?" and "is it generated?".
export function hasNativeCiasLayout(topologyId) { return topologyId in LAYOUTS }
export function hasCiasSchematic(topologyId) { return topologyId in LAYOUTS }

// EXACTLY what the app renders, plus the anchor pins the offline checkers need. Every gate must render
// through here: while a second, hand-authored generator existed, the audits measured IT rather than the
// product, and a label fix applied there never reached the user.
export function renderForAudit(topologyId, tas) {
  return renderCiasSchematicWithPins(topologyId, tas)
}

// What the app draws. Throws rather than return an unverified drawing.
export function renderVerifiedSchematic(topologyId, tas) {
  return renderCiasSchematic(topologyId, tas)
}

// Build the drawing ONCE: draw every present part with terminal recording on, resolve each wire against
// the terminals that drawing produced, derive the junction dots, and hand back the SVG together with
// every recorded terminal. Both public entry points wrap this — they used to run the whole layout twice,
// once for the SVG and once to recover the pins, which is two chances to disagree about one drawing.
function buildCias(topologyId, tas) {
  const spec = LAYOUTS[topologyId]
  if (!spec) return null
  const bom = new Map(extractBom(tas).map((r) => [r.ref, r]))
  const present = new Set(ciasComponents(tas).map((c) => c.ref))
  // A layout may be a function of the present refs: that is how a bridge picks its rectifier — from
  // what the netlist actually carries, not from the variant string the form happened to send.
  const layout = typeof spec === 'function' ? spec(present) : spec

  // every CIAS power component must have a placement (loud gap, mirrors the falstad exporter)
  for (const ref of present) if (!layout.place[ref]) throw new Error(`ciasSchematic '${topologyId}': no placement for CIAS component '${ref}'`)

  const has = (needs) => (needs ?? []).every((r) => present.has(r))
  const parts = []
  // PASS 1 — the symbols. Recording is on, so every terminal coordinate below comes from the symbol
  // itself rather than from a second hand-typed copy of it.
  const pins = withPinRecording(() => {
    for (const [ref, p] of Object.entries(layout.place)) if (present.has(ref)) parts.push(p.draw(bom))
    parts.push(...layout.synth(bom, present))
  }).pins
  // A wire end is either a terminal name ('Q1.drain') or a bare point ([x, y]) for a rail corner that
  // belongs to no symbol. Naming a terminal that nothing registered is a layout bug, not a coordinate
  // to invent, so it throws.
  const at = (spec) => {
    if (Array.isArray(spec)) return spec
    const [ref, pin] = String(spec).split('.')
    const p = pins.find((q) => q.ref === ref && q.pin === pin)
    if (!p) throw new Error(`ciasSchematic '${topologyId}': wire end '${spec}' names a terminal no drawn symbol registered`)
    return [p.x, p.y]
  }
  // PASS 2 — the wiring, then the dots it implies.
  const routes = layout.wires.filter((w) => has(w.needs)).map((w) => resolveRoute(topologyId, w, at)).filter(Boolean)
  for (const pts of routes) parts.push(poly(pts))
  // Ground/return/port glyphs are references, not conductors joining a node: their own symbol says the
  // wire connects there, and house style across every hand-authored layout puts no dot on them. The
  // source and the load ARE two-terminal parts and do attract one where they tap a rail mid-run.
  const conductor = (p) => !['@gnd', '@sgnd', '@port'].includes(p.ref) && (present.has(p.ref) || p.ref.startsWith('@'))
  for (const [x, y] of deriveDots(parts.join(''), pins.filter(conductor))) parts.push(dot(x, y))

  const [w, h] = layout.size
  const svgStr = svg(w, h, parts.join(''))

  // Parity: every present CIAS component must be drawn with a data-ref hotspot (no hidden parts).
  const drawn = new Set([...svgStr.matchAll(/data-ref="([^"]+)"/g)].map((m) => m[1]))
  const missing = [...present].filter((r) => !drawn.has(r))
  if (missing.length) throw new Error(`ciasSchematic '${topologyId}': CIAS components not drawn: ${missing.join(', ')}`)

  // Equivalence guarantee: the generated drawing must pass the SAME netlist/isolation checker the
  // hand-authored layouts pass — including rule G, which identifies the passive-only nodes (a snubber
  // midpoint, a tank's Cr–Lr join) that no anchor names.
  const problems = checkSchematic({ svg: svgStr, pins, tas })
  if (problems.length) throw new Error(`ciasSchematic '${topologyId}' netlist mismatch: ${problems.join(' | ')}`)

  return { svg: svgStr, pins }
}

// The SVG alone (what the app renders), or null when this topology has no native layout.
export function renderCiasSchematic(topologyId, tas) {
  return buildCias(topologyId, tas)?.svg ?? null
}

// Same render, with the terminals the checkers need. Used by renderForAudit so every offline gate
// measures the drawing the app shows.
export function renderCiasSchematicWithPins(topologyId, tas) {
  return buildCias(topologyId, tas)
}
