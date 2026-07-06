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
import { symbols as S, hasSchematic, collectPins } from './schematics.js'
import { ciasComponents } from './cias.js'
import { extractBom } from './bom.js'
import { checkSchematic } from './schematicCheck.js'

const { svg, wire, dot, mosfetV, diode, capV, resV, xfmr, srcDC, gnd, loadR, port, sig, ctrlIC } = S

// wire(...pts) helper takes a flat point list; our layout stores polylines as flat arrays.
const poly = (pts) => wire(...pts)

const LAYOUTS = {
  flyback: {
    size: [760, 350],
    // CIAS ref -> { draw(bom), pins? }. `pins` (only for anchors the checker records: MOSFET
    // drain/source/gate, diode anode/cathode, magnetic winding terminals) feed the netlist check.
    place: {
      Cin:   { draw: (b) => capV('Cin', b, 150, 165, 'left') },
      Cclmp: { draw: (b) => capV('Cclmp', b, 205, 90, 'left') },
      Rclmp: { draw: (b) => resV('Rclmp', b, 205, 130, 'left') },
      Cres:  { draw: (b) => capV('Cres', b, 205, 215, 'left') },     // QRM only — absent in CCM/DCM/BCM
      T1:    { draw: (b) => xfmr('T1', b, 260, 140, { opp: true, labelDy: -30 }),
               pins: [['p0', 250, 100], ['p1', 250, 180], ['s0', 270, 100], ['s1', 270, 180]] },
      Q1:    { draw: (b) => mosfetV('Q1', b, 250, 228),
               pins: [['drain', 250, 202], ['source', 250, 254], ['gate', 224, 228]] },
      D1:    { draw: (b) => diode('D1', b, 360, 90, 'right'),
               pins: [['anode', 340, 90], ['cathode', 380, 90]] },
      Cout:  { draw: (b) => capV('Cout', b, 470, 180) },
    },
    // Net-realizing wire polylines. `needs` gates each on the presence of its owning CIAS components,
    // so removing a component (e.g. Cres outside QRM) drops its wiring automatically.
    wires: [
      { pts: [70, 145, 70, 70, 240, 70] },                       // Vin+ rail: source -> T1 primary feed
      { pts: [70, 175, 70, 260, 250, 260] },                     // primary return rail (earth side)
      { pts: [150, 70, 150, 145], needs: ['Cin'] },              // Cin top -> Vin rail
      { pts: [150, 185, 150, 260], needs: ['Cin'] },             // Cin bottom -> return rail
      { pts: [240, 70, 250, 70, 250, 100], needs: ['T1'] },      // Vin -> T1 primary_start (p0)
      { pts: [250, 180, 250, 202], needs: ['T1', 'Q1'] },        // T1 primary_end (p1) -> Q1 drain
      { pts: [250, 254, 250, 260], needs: ['Q1'] },              // Q1 source -> return rail
      { pts: [205, 150, 205, 195, 250, 195], needs: ['Rclmp'] }, // RC clamp -> drain (sw) node
      { pts: [205, 235, 205, 260], needs: ['Cres'] },            // Cres bottom -> return rail
      { pts: [270, 100, 270, 90, 340, 90], needs: ['T1', 'D1'] }, // T1 secondary_end (s0) -> D1 anode
      { pts: [380, 90, 620, 90], needs: ['D1'] },                // D1 cathode -> Vout rail
      { pts: [270, 180, 270, 268, 620, 268], needs: ['T1'] },    // T1 secondary_start (s1) -> sec return rail
      { pts: [470, 90, 470, 160], needs: ['Cout'] },             // Cout top -> Vout rail
      { pts: [470, 200, 470, 268], needs: ['Cout'] },            // Cout bottom -> sec return rail
      { pts: [620, 90, 660, 90] },                               // Vout rail -> VOUT port
      { pts: [620, 268, 660, 268] },                             // sec return rail -> RTN port
    ],
    dots: [
      { x: 150, y: 70, needs: ['Cin'] }, { x: 150, y: 260, needs: ['Cin'] },
      { x: 205, y: 70, needs: ['Cclmp'] }, { x: 250, y: 195, needs: ['Rclmp'] },
      { x: 205, y: 195, needs: ['Cres'] }, { x: 205, y: 260, needs: ['Cres'] },
      { x: 470, y: 90, needs: ['Cout'] }, { x: 470, y: 268, needs: ['Cout'] },
      { x: 560, y: 90 }, { x: 560, y: 268 },
    ],
    // Non-CIAS glyphs synthesized around the bricks — the dual of what the TAS assembler adds: the
    // input source, the output load (Rload = Vout^2/Pout), primary earth, the isolated secondary
    // return + Vout ports, and the gate-drive net-label + controller IC.
    synth: (b) => [
      srcDC(70, 160), loadR(560, 180, 90, 268), gnd(180, 260),
      port(660, 90, 'VOUT'), port(660, 268, 'RTN'),
      sig(224, 228, 'g1', 'down'), ctrlIC(b, 400, 315, ['g1']),
    ],
    // Recorded reference-glyph terminals for the checker (primary earth + secondary return + Vout).
    synthPins: [['@gnd', 'gnd', 180, 260], ['@port', 'VOUT', 660, 90], ['@port', 'RTN', 660, 268]],
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

// Generate the CIAS-driven SVG for a solved TAS. Returns the SVG string, or throws with the exact
// reason (unplaced component, netlist mismatch, undrawn part) — never a silently-wrong drawing.
export function renderCiasSchematic(topologyId, tas) {
  const layout = LAYOUTS[topologyId]
  if (!layout) return null
  const bom = new Map(extractBom(tas).map((r) => [r.ref, r]))
  const present = new Set(ciasComponents(tas).map((c) => c.ref))

  // every CIAS power component must have a placement (loud gap, mirrors the falstad exporter)
  for (const ref of present) if (!layout.place[ref]) throw new Error(`ciasSchematic '${topologyId}': no placement for CIAS component '${ref}'`)

  const has = (needs) => (needs ?? []).every((r) => present.has(r))
  const parts = []
  const pins = []
  for (const [ref, p] of Object.entries(layout.place)) {
    if (!present.has(ref)) continue
    parts.push(p.draw(bom))
    for (const [pin, x, y] of p.pins ?? []) pins.push({ ref, pin, x, y })
  }
  for (const w of layout.wires) if (has(w.needs)) parts.push(poly(w.pts))
  for (const d of layout.dots) if (has(d.needs)) parts.push(dot(d.x, d.y))
  parts.push(...layout.synth(bom))
  for (const [ref, pin, x, y] of layout.synthPins) pins.push({ ref, pin, x, y })

  const [w, h] = layout.size
  const svgStr = svg(w, h, parts.join(''))

  // Parity: every present CIAS component must be drawn with a data-ref hotspot (no hidden parts).
  const drawn = new Set([...svgStr.matchAll(/data-ref="([^"]+)"/g)].map((m) => m[1]))
  const missing = [...present].filter((r) => !drawn.has(r))
  if (missing.length) throw new Error(`ciasSchematic '${topologyId}': CIAS components not drawn: ${missing.join(', ')}`)

  // Equivalence guarantee: the generated drawing must pass the SAME netlist/isolation checker the
  // hand-authored layouts pass. Any problem on an anchored net means the layout drifted from the CIAS
  // netlist (see the file header on the purely-passive-net blind spot inherited from that checker).
  const problems = checkSchematic({ svg: svgStr, pins, tas })
  if (problems.length) throw new Error(`ciasSchematic '${topologyId}' netlist mismatch: ${problems.join(' | ')}`)

  return svgStr
}
