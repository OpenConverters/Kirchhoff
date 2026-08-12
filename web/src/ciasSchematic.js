// CIAS-driven schematic generator. The power-path DRAWING is assembled from the TAS's inline CIAS
// bricks — the exact same structure the ngspice deck and the falstad visual sim are generated from —
// so all three views share one source of truth. WHICH components appear (e.g. the QRM resonant cap),
// their VALUES/labels, and the net TRUTH come from CIAS; only the GEOMETRY (grid coordinates — CIAS
// carries none) is declared per topology. Every render is verified against the flattened CIAS netlist
// with the SAME connectivity/isolation checker the hand-authored layouts pass (schematicCheck.js),
// and THROWS on drift of any ANCHORED net — one carrying a MOSFET drain/source, diode anode/cathode,
// or a magnetic winding terminal. NOTE: a net whose only members are passives (caps/resistors) + a
// ground/port glyph has no unambiguous anchor, so the checker (by inherited design — the hand-authored
// suite shares this blind spot) cannot resolve it; drift confined to such a purely-passive net would
// not throw. For flyback every return/rail is anchored (Q1|source, D1|cathode, T1 terminals), so this
// is fully covered here; strengthen rule C in schematicCheck.js before porting resonant/multi-return
// topologies where purely-passive tank/return nodes are common.
//
// The coordinates below are transcribed from the proven hand-authored flyback layout in schematics.js,
// so the generated art is identical in quality; the difference is that components/values/wiring are
// now driven and continuously verified by CIAS rather than hand-listed.
import { symbols as S, hasSchematic, collectPins, withPinRecording } from './schematics.js'
import { ciasComponents } from './cias.js'
import { extractBom } from './bom.js'
import { checkSchematic } from './schematicCheck.js'

const { svg, wire, dot, mosfetV, mosfetH, diode, indH, indV, capV, capH, resV, resH, xfmr, xfmr3, xfmr4, srcDC, gnd, isoGnd, loadR, port, sig, ctrlIC } = S

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
  return pts
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

// A native, fully-generated CIAS layout exists for this topology (flyback so far).
export function hasNativeCiasLayout(topologyId) { return topologyId in LAYOUTS }
// Any schematic — native-generated OR hand-authored — is available and will be CIAS-verified at render.
export function hasCiasSchematic(topologyId) { return topologyId in LAYOUTS || hasSchematic(topologyId) }

// Render a schematic and VERIFY it against the live CIAS netlist, whichever source it comes from:
//   • native generator layout (flyback) — components/values/wiring generated from CIAS, then checked;
//   • hand-authored layout (every other topology) — drawn from the CIAS-derived BOM, then checked with
//     the SAME connectivity/isolation rules so it can never silently drift from the netlist either.
// EXACTLY what the app renders, plus the anchor pins the offline checkers need.
//
// The app calls renderVerifiedSchematic, which prefers a CIAS layout when one exists and otherwise
// falls back to the hand-authored art. The offline gates used to call collectPins() directly, i.e. the
// hand-authored path ALWAYS — so for a topology with a CIAS layout (flyback) every audit measured a
// drawing the product never shows, and a label fix applied to schematics.js silently never reached the
// user. Every gate must go through here so the two can never diverge again.
export function renderForAudit(topologyId, tas, variant, bomRows) {
  if (topologyId in LAYOUTS) return renderCiasSchematicWithPins(topologyId, tas)
  if (!hasSchematic(topologyId)) return null
  const rows = bomRows ?? extractBom(tas)
  return collectPins(topologyId, rows, variant)
}

// Throws on any anchored-net mismatch (see the file header caveat). Returns null if no schematic exists.
export function renderVerifiedSchematic(topologyId, tas, variant, bomRows) {
  if (topologyId in LAYOUTS) return renderCiasSchematic(topologyId, tas)
  if (!hasSchematic(topologyId)) return null
  // Hand-authored art, but held to the CIAS netlist: collectPins re-renders with terminal recording on,
  // giving both the SVG and the anchor pins the checker needs.
  const rows = bomRows ?? extractBom(tas)
  const { svg, pins } = collectPins(topologyId, rows, variant)
  const problems = checkSchematic({ svg, pins, tas })
  if (problems.length) throw new Error(`schematic '${topologyId}' netlist mismatch: ${problems.join(' | ')}`)
  return svg
}

// Build the drawing ONCE: draw every present part with terminal recording on, resolve each wire against
// the terminals that drawing produced, derive the junction dots, and hand back the SVG together with
// every recorded terminal. Both public entry points wrap this — they used to run the whole layout twice,
// once for the SVG and once to recover the pins, which is two chances to disagree about one drawing.
function buildCias(topologyId, tas) {
  const layout = LAYOUTS[topologyId]
  if (!layout) return null
  const bom = new Map(extractBom(tas).map((r) => [r.ref, r]))
  const present = new Set(ciasComponents(tas).map((c) => c.ref))

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
  const routes = layout.wires.filter((w) => has(w.needs)).map((w) => resolveRoute(topologyId, w, at))
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
