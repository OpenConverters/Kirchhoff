// CIAS-driven CircuitJS1 (Falstad) export: an animated "energy conversion in action" view of the
// solved design. Components, values and CONNECTIVITY all come from the TAS's inline CIAS bricks —
// the exact structure the ngspice deck is generated from — so the visual sim can never drift from
// the simulated circuit. The only hand-authored part is GEOMETRY (CIAS carries no coordinates and
// circuitjs1 has no netlist import: its text format is elements on a 16px grid). Every generation
// re-verifies the drawn wiring against the flattened CIAS nets and THROWS on any mismatch — the
// runtime equivalent of scripts/checkSchematicNets.mjs for the SVG schematic.
//
// Time scaling: circuitjs1 is a real-time toy simulator — at a real fsw (60 kHz–1 MHz) nothing is
// watchable. We apply a similarity transform: fsw/k with every L and C scaled *k. Duty, turns
// ratio, voltages, currents, ripple fractions and the CCM/DCM boundary are all invariant; the
// waveforms are identical, just stretched in time. Numbers still come from ngspice — this view is
// for intuition only.
//
// circuitjs1 is GPLv2 and a compiled GWT app; it is embedded as a SEPARATE page (iframe / new tab
// via the documented ?cct= URL parameter), never linked into this MIT bundle. It is SELF-HOSTED
// under /circuitjs/ (mirrored, no external hosts) so the bench has no runtime dependency on
// falstad.com; the canvas is themed to the KH phosphor palette via kh-theme.css + the color params.

import { flattenNets, ciasComponents, resolveDim } from './cias.js'

const VISUAL_HZ = 500 // what the design's fsw is scaled down to (watchable scope + current dots)

// Self-hosted CircuitJS1 host page (served from web/public/circuitjs/, deploys with the SPA).
const CIRCUITJS_BASE = 'circuitjs/circuitjs.html' // relative to the SPA base (served at site root)
// KH phosphor palette for the canvas: amber wires/current, warm/cool voltage polarity, near-black bg.
const KH_COLORS = {
  positiveColor: '%23ffd98a', negativeColor: '%237fd0ff', neutralColor: '%23c98a3a',
  currentColor: '%23ffb347', selectColor: '%23ffffff',
}
const khColorQuery = Object.entries(KH_COLORS).map(([k, v]) => `${k}=${v}`).join('&')

// ── per-topology placement maps: geometry ONLY (refdes -> falstad element + electrical pin coords).
// Pin coordinate contract per element type (16px grid):
//   c/r  two posts at the given endpoints
//   T    posts (x,y) (x,y+32) primary, (x2,y) (x2,y+32) secondary; flags 4 = secondary dot at bottom
//   f    flags 32 (body diode, no flip): gate at (x1,y1), drain (x2,y2-16), source (x2,y2+16)
//   d    anode at (x1,y1), cathode at (x2,y2)
const LAYOUTS = {
  flyback: {
    place: {
      Cin: { pins: { 1: [240, 128], 2: [240, 320] }, line: (q, c) => `c 240 128 240 320 0 ${c.C(q)} ${c.vin}` },
      Cclmp: { pins: { 1: [272, 128], 2: [272, 176] }, line: (q, c) => `c 272 128 272 176 0 ${c.C(q)} ${-c.n * c.vout}` },
      Rclmp: { pins: { 1: [272, 176], 2: [272, 224] }, line: (q, c) => `r 272 176 272 224 0 ${c.R(q)}` },
      T1: {
        pins: { primary_start: [320, 128], primary_end: [320, 160], secondary1_end: [400, 128], secondary1_start: [400, 160] },
        line: (q, c) => `T 320 128 400 128 4 ${c.Lm} ${1 / c.n} 0 0 0.999`, // flags 4: secondary dot at bottom = secondary1_start, per the CIAS dot convention
      },
      Q1: { pins: { gate: [256, 240], drain: [320, 224], source: [320, 256] }, line: () => 'f 256 240 320 240 32 1.5 50' },
      D1: { pins: { anode: [448, 128], cathode: [512, 128] }, line: () => 'd 448 128 512 128 2 default' },
      Cout: { pins: { 1: [560, 128], 2: [560, 320] }, line: (q, c) => `c 560 128 560 320 0 ${c.C(q)} ${c.vout}` },
      // QRM only: the real drain-source resonant cap (valley ringing) — optional in the brick.
      Cres: { optional: true, pins: { 1: [352, 224], 2: [352, 320] }, line: (q, c) => `c 352 224 352 320 0 ${c.C(q)} 0`, wires: [[320, 224, 352, 224]] },
    },
    wires: [
      [160, 128, 240, 128], [240, 128, 272, 128], [272, 128, 320, 128], // VIN rail
      [272, 224, 320, 224], // clamp bottom -> drain
      [320, 160, 320, 224], // primary_end -> drain
      [320, 256, 320, 320], // source -> ground rail
      [400, 128, 448, 128], // secondary1_end -> diode anode
      [400, 160, 400, 320], // secondary1_start (dot) -> ground rail
      [512, 128, 560, 128], [560, 128, 640, 128], // VOUT rail
      [160, 320, 240, 320], [240, 320, 256, 320], [256, 320, 320, 320], // ground rail
      [320, 320, 352, 320], [352, 320, 400, 320], [400, 320, 560, 320], [560, 320, 640, 320],
    ],
    // Synthesized elements — dual of what the TAS assembler synthesizes around the CIAS bricks:
    // the input source, the load (Rload = Vout²/Pout, Flyback.cpp), the gate drive from
    // tas.simulation.stimulus, plus ground + a labeled vout node (JS-API readable).
    synth: (c) => [
      { line: `v 160 320 160 128 0 0 40 ${c.vin} 0 0 0.5`, posts: [[160, 320], [160, 128]], attach: { '160,128': ['Cin', '1'], '160,320': ['Cin', '2'] } },
      { line: 'g 160 320 160 336 0', posts: [[160, 320]] },
      { line: `v 256 320 256 240 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[256, 320], [256, 240]], attach: { '256,240': ['Q1', 'gate'], '256,320': ['Cin', '2'] } },
      { line: `r 640 128 640 320 0 ${c.rload}`, tag: 'Rload', posts: [[640, 128], [640, 320]], attach: { '640,128': ['Cout', '1'], '640,320': ['Cout', '2'] } },
      { line: '207 640 128 688 128 0 vout', posts: [[640, 128]] },
    ],
    // Named scope sets the UI can switch between. 'magnetic' puts the transformer's CURRENT + VOLTAGE
    // in the middle panel: CircuitJS1's transformer element reports no device current, so the primary
    // current is read from the wire in series with the primary (magI), overlaid with the drain voltage.
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['D1', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['D1', 'voltage'], { magI: [320, 160, 320, 224], magVtag: 'Q1' }, ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['D1', 'voltage'], ['Rload', 'voltage']],
      rectifier: [['Q1', 'voltage'], ['D1', 'both'], ['Rload', 'voltage']],
      output:    [['Q1', 'voltage'], ['D1', 'voltage'], ['Rload', 'both']],
    },
  },

  // Non-isolated buck: high-side switch (Vin→sw), freewheel diode, LC filter. The high-side gate drive
  // is a FLOATING gate-source PWM (its reference wires to the switching node) so Vgs toggles regardless
  // of the source swinging to Vin — the standard toy-sim idiom for a high-side FET.
  buck: {
    place: {
      Q1:   { pins: { drain: [272, 192], source: [272, 224], gate: [208, 208] }, line: () => 'f 208 208 272 208 32 1.5 50' },
      // Freewheel path — a diode (default) OR a low-side synchronous-rectifier FET (Q2), never both.
      D1:   { optional: true, pins: { anode: [336, 336], cathode: [336, 224] }, line: () => 'd 336 336 336 224 2 default' },
      Q2:   { optional: true, pins: { drain: [336, 268], source: [336, 300], gate: [400, 284] }, line: () => 'f 400 284 336 284 32 1.5 50' },
      L1:   { pins: { primary_start: [400, 224], primary_end: [512, 224] }, line: (q, c) => `l 400 224 512 224 0 ${c.Lm} 0` },
      Cout: { pins: { 1: [576, 224], 2: [576, 336] }, line: (q, c) => `c 576 224 576 336 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [128, 144, 272, 144], [272, 144, 272, 192],          // Vin rail -> Q1 drain
      [272, 224, 336, 224], [336, 224, 400, 224],           // sw rail: Q1 source -> (D1|Q2) -> L1 start
      [512, 224, 576, 224], [576, 224, 656, 224],           // Vout rail: L1 end -> Cout -> load
      [128, 336, 336, 336], [336, 336, 576, 336], [576, 336, 656, 336], // gnd rail
      [272, 224, 272, 288], [272, 288, 208, 288],           // sw -> Q1 gate-drive reference (floating Vgs)
      { pts: [336, 224, 336, 268], needs: ['Q2'] },         // SR: sw -> Q2 drain
      { pts: [336, 300, 336, 336], needs: ['Q2'] },         // SR: Q2 source -> gnd
    ],
    synth: (c) => [
      { line: `v 128 336 128 144 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 336], [128, 144]], attach: { '128,144': ['Q1', 'drain'], '128,336': ['Cout', '2'] } },
      { line: 'g 128 336 128 352 0', posts: [[128, 336]] },
      { line: `v 208 288 208 208 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[208, 288], [208, 208]], attach: { '208,208': ['Q1', 'gate'], '208,288': ['Q1', 'source'] } },
      // SR gate: complementary drive, ground-referenced (Q2 is low-side). needs Q2.
      { needs: ['Q2'], line: `v 400 352 400 284 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[400, 352], [400, 284]], attach: { '400,284': ['Q2', 'gate'], '400,352': ['Q2', 'source'] } },
      { needs: ['Q2'], line: 'w 400 352 336 352 0', posts: [[400, 352], [336, 352]], wire: true },
      { needs: ['Q2'], line: 'w 336 352 336 336 0', posts: [[336, 352], [336, 336]], wire: true },
      { line: `r 656 224 656 336 0 ${c.rload}`, tag: 'Rload', posts: [[656, 224], [656, 336]], attach: { '656,224': ['Cout', '1'], '656,336': ['Cout', '2'] } },
      { line: '207 656 224 704 224 0 vout', posts: [[656, 224]] },
    ],
    scopeSets: {
      overview:  [['L1', 'current'], ['D1', 'voltage'], ['Q2', 'voltage'], ['Rload', 'voltage']],
      // L1 is a real inductor, so its own current + voltage both plot directly ('both').
      magnetic:  [['L1', 'both'], ['D1', 'voltage'], ['Q2', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['L1', 'current'], ['Rload', 'voltage']],
      rectifier: [['D1', 'both'], ['Q2', 'both'], ['L1', 'current'], ['Rload', 'voltage']],
      output:    [['L1', 'current'], ['Rload', 'both']],
    },
    oneOf: [['D1', 'Q2']],   // freewheel: diode XOR synchronous FET
  },

  // Non-isolated boost: input inductor, LOW-side switch (sw→gnd), output diode to Vout. Low-side switch
  // ⇒ a plain ground-referenced gate drive (like flyback).
  boost: {
    place: {
      L1:   { pins: { primary_start: [192, 224], primary_end: [320, 224] }, line: (q, c) => `l 192 224 320 224 0 ${c.Lm} 0` },
      Q1:   { pins: { drain: [320, 272], source: [320, 304], gate: [256, 288] }, line: () => 'f 256 288 320 288 32 1.5 50' },
      // Rectifier: a diode (default) OR a high-side synchronous FET (Q2, source=sw, drain=Vout).
      D1:   { optional: true, pins: { anode: [320, 224], cathode: [448, 224] }, line: () => 'd 320 224 448 224 2 default' },
      Q2:   { optional: true, pins: { drain: [384, 144], source: [384, 176], gate: [320, 160] }, line: () => 'f 320 160 384 160 32 1.5 50' },
      Cout: { pins: { 1: [512, 224], 2: [512, 336] }, line: (q, c) => `c 512 224 512 336 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [128, 224, 192, 224],                                 // Vin rail -> L1 start
      [320, 224, 320, 272],                                 // sw node (L1 end / rectifier) -> Q1 drain
      [320, 304, 320, 336],                                 // Q1 source -> gnd rail
      [448, 224, 512, 224], [512, 224, 592, 224],           // Vout rail: rectifier -> Cout -> load
      [128, 336, 256, 336], [256, 336, 320, 336], [320, 336, 512, 336], [512, 336, 592, 336], // gnd rail (split at 256 for the gate-drive tie)
      { pts: [384, 176, 384, 208], needs: ['Q2'] },         // SR source stub (split at 208 for the drive ref)
      { pts: [384, 208, 384, 224], needs: ['Q2'] },
      { pts: [384, 224, 320, 224], needs: ['Q2'] },         // SR source -> sw
      { pts: [384, 144, 448, 144], needs: ['Q2'] }, { pts: [448, 144, 448, 224], needs: ['Q2'] }, // SR drain -> Vout rail
    ],
    synth: (c) => [
      { line: `v 128 336 128 224 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 336], [128, 224]], attach: { '128,224': ['L1', 'primary_start'], '128,336': ['Cout', '2'] } },
      { line: 'g 128 336 128 352 0', posts: [[128, 336]] },
      { line: `v 256 336 256 288 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[256, 336], [256, 288]], attach: { '256,288': ['Q1', 'gate'], '256,336': ['Q1', 'source'] } },
      // SR gate: complementary + FLOATING (Q2 high-side), reference wired to its source (sw). needs Q2.
      { needs: ['Q2'], line: `v 320 208 320 160 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[320, 208], [320, 160]], attach: { '320,160': ['Q2', 'gate'], '320,208': ['Q2', 'source'] } },
      { needs: ['Q2'], line: 'w 320 208 384 208 0', posts: [[320, 208], [384, 208]], wire: true },
      { line: `r 592 224 592 336 0 ${c.rload}`, tag: 'Rload', posts: [[592, 224], [592, 336]], attach: { '592,224': ['Cout', '1'], '592,336': ['Cout', '2'] } },
      { line: '207 592 224 640 224 0 vout', posts: [[592, 224]] },
    ],
    scopeSets: {
      overview:  [['L1', 'current'], ['D1', 'voltage'], ['Q2', 'voltage'], ['Rload', 'voltage']],
      // L1 is a real inductor, so its own current + voltage both plot directly ('both').
      magnetic:  [['L1', 'both'], ['D1', 'voltage'], ['Q2', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['L1', 'current'], ['Rload', 'voltage']],
      rectifier: [['D1', 'both'], ['Q2', 'both'], ['L1', 'current'], ['Rload', 'voltage']],
      output:    [['L1', 'current'], ['Rload', 'both']],
    },
    oneOf: [['D1', 'Q2']],   // rectifier: diode XOR synchronous FET
  },

  // Non-isolated inverting Ćuk: input inductor L1, LOW-side switch (sw→common), an energy-transfer
  // coupling cap C1 (switch node → mid node), output inductor L2 (mid → Vout), a freewheel diode D1
  // (mid → common), a series-RC turn-off snubber across the diode node, and the output cap Cout. The
  // series power path lives on one rail at y=224: L1 (Vin→sw) — C1 (sw→mid) — L2 (mid→Vout); shared
  // endpoints at x=320 (switch node) and x=448 (mid node) tie the three in series, exactly as CIAS
  // wires them. Output is NEGATIVE (inverting) — the toy sim handles the sign; vout carries it.
  // Q1 is low-side (source at the common) ⇒ a plain ground-referenced gate drive (like boost/flyback).
  cuk: {
    place: {
      L1:     { pins: { primary_start: [192, 224], primary_end: [320, 224] }, line: (q, c) => `l 192 224 320 224 0 ${c.Lm} 0` },
      Q1:     { pins: { gate: [256, 288], drain: [320, 272], source: [320, 304] }, line: () => 'f 256 288 320 288 32 1.5 50' },
      // Coupling / energy-transfer cap: switch node (pin1) ↔ mid node (pin2). Steady-state V ≈ Vin−Vout.
      C1:     { pins: { 1: [320, 224], 2: [448, 224] }, line: (q, c) => `c 320 224 448 224 0 ${c.C(q)} ${c.vin - c.vout}` },
      L2:     { pins: { primary_start: [448, 224], primary_end: [576, 224] }, line: (q, c) => `l 448 224 576 224 0 ${c.Lm} 0` },
      // Freewheel diode: anode=mid node, cathode=common (conducts mid→common when Q1 is off).
      D1:     { pins: { anode: [496, 272], cathode: [496, 400] }, line: () => 'd 496 272 496 400 2 default' },
      // Series-RC diode snubber off the mid node: Rrc_sw (mid→snubber-mid) then Crc_sw (snubber-mid→common).
      Rrc_sw: { pins: { 1: [448, 320], 2: [448, 360] }, line: (q, c) => `r 448 320 448 360 0 ${c.R(q)}` },
      Crc_sw: { pins: { 1: [448, 360], 2: [448, 400] }, line: (q, c) => `c 448 360 448 400 0 ${c.C(q)} 0` },
      Cout:   { pins: { 1: [576, 224], 2: [576, 400] }, line: (q, c) => `c 576 224 576 400 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [320, 224, 320, 272],                                 // switch node: L1 end / C1 pin1 -> Q1 drain
      [320, 304, 320, 400],                                 // Q1 source -> common rail
      [448, 224, 448, 272],                                 // mid node: C1 pin2 / L2 start -> tee
      [448, 272, 496, 272],                                 // tee -> D1 anode
      [448, 272, 448, 320],                                 // tee -> Rrc_sw pin1 (snubber)
      [128, 224, 192, 224],                                 // Vin rail -> L1 start
      [576, 224, 640, 224],                                 // Vout rail: Cout pin1 -> load / label
      [128, 400, 256, 400], [256, 400, 320, 400],           // common rail (split at 256 for the gate-drive tie)
      [320, 400, 448, 400], [448, 400, 496, 400],
      [496, 400, 576, 400], [576, 400, 640, 400],
    ],
    synth: (c) => [
      { line: `v 128 400 128 224 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 400], [128, 224]], attach: { '128,224': ['L1', 'primary_start'], '128,400': ['Cout', '2'] } },
      { line: 'g 128 400 128 416 0', posts: [[128, 400]] },
      { line: `v 256 400 256 288 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[256, 400], [256, 288]], attach: { '256,288': ['Q1', 'gate'], '256,400': ['Q1', 'source'] } },
      { line: `r 640 224 640 400 0 ${c.rload}`, tag: 'Rload', posts: [[640, 224], [640, 400]], attach: { '640,224': ['Cout', '1'], '640,400': ['Cout', '2'] } },
      { line: '207 640 224 688 224 0 vout', posts: [[640, 224]] },
    ],
    scopeSets: {
      overview:  [['L1', 'current'], ['D1', 'voltage'], ['Rload', 'voltage']],
      // L1 is a real inductor, so its own current + voltage both plot directly ('both').
      magnetic:  [['L1', 'both'], ['D1', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['L1', 'current'], ['Rload', 'voltage']],
      rectifier: [['D1', 'both'], ['L1', 'current'], ['Rload', 'voltage']],
      output:    [['L1', 'current'], ['Rload', 'both']],
    },
  },

  // Non-isolated SEPIC: input inductor L1, LOW-side switch Q1, coupling cap Cs, second inductor L2 to
  // ground, output diode D1 to Vout. Non-inverting step up/down: Vout ≈ D/(1-D)·Vin. Low-side switch ⇒
  // a plain ground-referenced gate drive (like boost).
  sepic: {
    place: {
      L1: { pins: { primary_start: [192, 224], primary_end: [320, 224] }, line: (q, c) => `l 192 224 320 224 0 ${c.Lm} 0` },
      Q1: { pins: { drain: [320, 272], source: [320, 304], gate: [256, 288] }, line: () => 'f 256 288 320 288 32 1.5 50' },
      Cs: { pins: { 1: [320, 224], 2: [448, 224] }, line: (q, c) => `c 320 224 448 224 0 ${c.C(q)} ${c.vin}` },
      L2: { pins: { primary_end: [448, 224], primary_start: [448, 336] }, line: (q, c) => `l 448 224 448 336 0 ${c.Lm} 0` },
      D1: { pins: { anode: [448, 224], cathode: [576, 224] }, line: () => 'd 448 224 576 224 2 default' },
      Cout: { pins: { 1: [640, 224], 2: [640, 336] }, line: (q, c) => `c 640 224 640 336 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [128, 224, 192, 224],
      [320, 224, 320, 272],
      [320, 304, 320, 336],
      [576, 224, 640, 224], [640, 224, 704, 224],
      [128, 336, 256, 336], [256, 336, 320, 336], [320, 336, 448, 336], [448, 336, 640, 336], [640, 336, 704, 336],
    ],
    synth: (c) => [
      { line: `v 128 336 128 224 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 336], [128, 224]], attach: { '128,224': ['L1', 'primary_start'], '128,336': ['Cout', '2'] } },
      { line: 'g 128 336 128 352 0', posts: [[128, 336]] },
      { line: `v 256 336 256 288 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[256, 336], [256, 288]], attach: { '256,288': ['Q1', 'gate'], '256,336': ['Q1', 'source'] } },
      { line: `r 704 224 704 336 0 ${c.rload}`, tag: 'Rload', posts: [[704, 224], [704, 336]], attach: { '704,224': ['Cout', '1'], '704,336': ['Cout', '2'] } },
      { line: '207 704 224 752 224 0 vout', posts: [[704, 224]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['D1', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['L1', 'both'], ['L2', 'both'], ['D1', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['D1', 'voltage'], ['Rload', 'voltage']],
      rectifier: [['Q1', 'voltage'], ['D1', 'both'], ['Rload', 'voltage']],
      output:    [['Q1', 'voltage'], ['D1', 'voltage'], ['Rload', 'both']],
    },
  },

  // Non-isolated ZETA: high-side switch (Vin→sw), coupling cap Cc (sw→mid), storage inductor L1
  // (sw→gnd), freewheel diode D1 (gnd→mid), output inductor L2 (mid→Vout), output cap. Non-inverting,
  // Vout = D/(1-D)·Vin. High-side switch ⇒ a FLOATING gate-source PWM whose reference wires to the
  // switching node (Q1 source), same idiom as buck Q1. D1 oriented ground→mid.
  zeta: {
    place: {
      Q1:   { pins: { gate: [208, 160], drain: [272, 144], source: [272, 176] }, line: () => 'f 208 160 272 160 32 1.5 50' },
      Cc:   { pins: { 1: [272, 176], 2: [400, 176] }, line: (q, c) => `c 272 176 400 176 0 ${c.C(q)} ${-c.vout}` },
      L1:   { pins: { primary_start: [272, 240], primary_end: [272, 336] }, line: (q, c) => `l 272 240 272 336 0 ${c.Lm} 0` },
      D1:   { pins: { anode: [400, 336], cathode: [400, 176] }, line: () => 'd 400 336 400 176 2 default' },
      L2:   { pins: { primary_start: [400, 176], primary_end: [528, 176] }, line: (q, c) => `l 400 176 528 176 0 ${c.Lm} 0` },
      Cout: { pins: { 1: [528, 176], 2: [528, 336] }, line: (q, c) => `c 528 176 528 336 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [128, 144, 272, 144],
      [272, 176, 272, 240],
      [208, 240, 272, 240],
      [528, 176, 592, 176],
      [128, 336, 272, 336], [272, 336, 400, 336], [400, 336, 528, 336], [528, 336, 592, 336],
    ],
    synth: (c) => [
      { line: `v 128 336 128 144 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 336], [128, 144]], attach: { '128,144': ['Q1', 'drain'], '128,336': ['Cout', '2'] } },
      { line: 'g 128 336 128 352 0', posts: [[128, 336]] },
      { line: `v 208 240 208 160 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[208, 240], [208, 160]], attach: { '208,160': ['Q1', 'gate'], '208,240': ['Q1', 'source'] } },
      { line: `r 592 176 592 336 0 ${c.rload}`, tag: 'Rload', posts: [[592, 176], [592, 336]], attach: { '592,176': ['Cout', '1'], '592,336': ['Cout', '2'] } },
      { line: '207 592 176 640 176 0 vout', posts: [[592, 176]] },
    ],
    scopeSets: {
      overview:  [['L1', 'current'], ['L2', 'current'], ['D1', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['L1', 'both'], ['L2', 'both'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['L1', 'current'], ['Rload', 'voltage']],
      rectifier: [['D1', 'both'], ['L2', 'current'], ['Rload', 'voltage']],
      output:    [['L2', 'current'], ['Rload', 'both']],
    },
  },

  // Isolated buck-boost (flyback-type): a HIGH-side switch QS1 (Vin→sw) drives the transformer primary
  // (primary_start=sw, primary_end=common); a primary clamp Dpri→Cpri snubs the leakage spike. Energy
  // stored in T1 while QS1 is ON is released to the secondary while OFF (secondary OUT of phase ⇒ flags
  // 4, flyback dot convention), rectified by Dsec into Csec/Rsec. Rsec is the CIAS load. High-side switch
  // ⇒ a FLOATING gate-source PWM referenced to the switching node. Output +12 V.
  isolated_buck_boost: {
    place: {
      QS1:  { pins: { drain: [272, 128], source: [272, 160], gate: [208, 144] }, line: () => 'f 208 144 272 144 32 1.5 50' },
      T1: {
        pins: { primary_start: [352, 160], primary_end: [352, 192], secondary1_end: [432, 160], secondary1_start: [432, 192] },
        line: (q, c) => `T 352 160 432 160 4 ${c.Lm} ${1 / c.n} 0 0 0.999`,
      },
      Dpri: { pins: { anode: [320, 224], cathode: [320, 160] }, line: () => 'd 320 224 320 160 2 default' },
      Cpri: { pins: { 1: [320, 224], 2: [320, 336] }, line: (q, c) => `c 320 224 320 336 0 ${c.C(q)} 0` },
      Dsec: { pins: { anode: [480, 160], cathode: [544, 160] }, line: () => 'd 480 160 544 160 2 default' },
      Csec: { pins: { 1: [544, 160], 2: [544, 336] }, line: (q, c) => `c 544 160 544 336 0 ${c.C(q)} ${c.vout}` },
      Rsec: { pins: { 1: [608, 160], 2: [608, 336] }, line: (q, c) => `r 608 160 608 336 0 ${c.R(q)}` },
    },
    wires: [
      [128, 128, 272, 128],
      [272, 160, 320, 160], [320, 160, 352, 160],
      [352, 192, 352, 336],
      [432, 160, 480, 160],
      [432, 192, 432, 336],
      [544, 160, 608, 160],
      [272, 160, 272, 240], [272, 240, 208, 240],
      [128, 336, 320, 336], [320, 336, 352, 336],
      [352, 336, 432, 336], [432, 336, 544, 336], [544, 336, 608, 336],
    ],
    synth: (c) => [
      { line: `v 128 336 128 128 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 336], [128, 128]], attach: { '128,128': ['QS1', 'drain'], '128,336': ['Csec', '2'] } },
      { line: 'g 128 336 128 352 0', posts: [[128, 336]] },
      { line: `v 208 240 208 144 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[208, 240], [208, 144]], attach: { '208,144': ['QS1', 'gate'], '208,240': ['QS1', 'source'] } },
      { line: '207 608 160 656 160 0 vout', posts: [[608, 160]] },
    ],
    scopeSets: {
      overview:  [['QS1', 'voltage'], ['Dsec', 'voltage'], ['Rsec', 'voltage']],
      magnetic:  [['Dsec', 'voltage'], { magI: [320, 160, 352, 160], magVtag: 'QS1' }, ['Rsec', 'voltage']],
      switch:    [['QS1', 'both'], ['Dsec', 'voltage'], ['Rsec', 'voltage']],
      rectifier: [['QS1', 'voltage'], ['Dsec', 'both'], ['Rsec', 'voltage']],
      output:    [['QS1', 'voltage'], ['Dsec', 'voltage'], ['Rsec', 'both']],
    },
  },

  // Isolated buck (Fly-Buck): a complementary HALF-BRIDGE (QS1 high-side Vin→mid, QS2 low-side mid→gnd)
  // drives T1 primary through a DC-block cap Cpri. FORWARD-type (Dsec conducts while QS1 is on ⇒ secondary
  // IN PHASE, flags 0). 1:1 transformer, Vout ≈ D·Vin ≈ +12 V. QS1 high-side ⇒ FLOATING gate-source PWM
  // referenced to the mid node; QS2 low-side ⇒ ground-referenced complementary PWM. Load = CIAS Rsec.
  isolated_buck: {
    place: {
      QS1:  { pins: { gate: [208, 128], drain: [272, 112], source: [272, 144] }, line: () => 'f 208 128 272 128 32 1.5 50' },
      QS2:  { pins: { gate: [336, 224], drain: [272, 208], source: [272, 240] }, line: () => 'f 336 224 272 224 32 1.5 50' },
      T1: {
        pins: { primary_start: [400, 176], primary_end: [400, 208], secondary1_end: [496, 176], secondary1_start: [496, 208] },
        line: (q, c) => `T 400 176 496 176 0 ${c.Lm} ${1 / c.n} 0 0 0.999`,
      },
      Cpri: { pins: { 1: [400, 240], 2: [400, 400] }, line: (q, c) => `c 400 240 400 400 0 ${c.C(q)} ${c.vin * c.duty}` },
      Dsec: { pins: { anode: [560, 176], cathode: [624, 176] }, line: () => 'd 560 176 624 176 2 default' },
      Csec: { pins: { 1: [688, 176], 2: [688, 400] }, line: (q, c) => `c 688 176 688 400 0 ${c.C(q)} ${c.vout}` },
      Rsec: { pins: { 1: [752, 176], 2: [752, 400] }, line: (q, c) => `r 752 176 752 400 0 ${c.R(q)}` },
    },
    wires: [
      [128, 112, 272, 112],
      [272, 144, 272, 176], [272, 176, 272, 208],
      [272, 176, 400, 176],
      [208, 176, 272, 176],
      [400, 208, 400, 240],
      [496, 176, 560, 176],
      [272, 240, 272, 400],
      [496, 208, 496, 400],
      [336, 288, 336, 400],
      [624, 176, 688, 176], [688, 176, 752, 176],
      [128, 400, 272, 400], [272, 400, 336, 400], [336, 400, 400, 400],
      [400, 400, 496, 400], [496, 400, 688, 400], [688, 400, 752, 400],
    ],
    synth: (c) => [
      { line: `v 128 400 128 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 400], [128, 112]], attach: { '128,112': ['QS1', 'drain'], '128,400': ['Cpri', '2'] } },
      { line: 'g 128 400 128 416 0', posts: [[128, 400]] },
      { line: `v 208 176 208 128 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[208, 176], [208, 128]], attach: { '208,128': ['QS1', 'gate'], '208,176': ['QS1', 'source'] } },
      { line: `v 336 288 336 224 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[336, 288], [336, 224]], attach: { '336,224': ['QS2', 'gate'], '336,288': ['QS2', 'source'] } },
      { line: '207 752 176 800 176 0 vout', posts: [[752, 176]] },
    ],
    scopeSets: {
      overview:  [['QS1', 'voltage'], ['Dsec', 'voltage'], ['Rsec', 'voltage']],
      magnetic:  [['Dsec', 'voltage'], { magI: [272, 176, 400, 176], magVtag: 'QS1' }, ['Rsec', 'voltage']],
      switch:    [['QS1', 'both'], ['Dsec', 'voltage'], ['Rsec', 'voltage']],
      rectifier: [['QS1', 'voltage'], ['Dsec', 'both'], ['Rsec', 'voltage']],
      output:    [['QS1', 'voltage'], ['Dsec', 'voltage'], ['Rsec', 'both']],
    },
  },

  // Isolated two-switch forward: primary winding between HIGH-side Q1 (Vin→pri_top) and LOW-side Q2
  // (pri_bottom→gnd); both toggle on the SAME duty. Clamp diodes D1 (anode=gnd, cathode=pri_top) and D2
  // (anode=pri_bottom, cathode=Vin) recycle magnetizing energy during OFF. FORWARD-type (secondary IN
  // PHASE, flags 0): Dfwd conducts while the switches are ON. Secondary: T1 → Dfwd → Lout → Cout, with Dfw
  // freewheeling. Q1 high-side ⇒ FLOATING gate-source PWM; Q2 low-side ⇒ ground-ref PWM (separate drives).
  two_switch_forward: {
    place: {
      Q1:   { pins: { gate: [256, 128], drain: [320, 112], source: [320, 144] }, line: () => 'f 256 128 320 128 32 1.5 50' },
      Q2:   { pins: { gate: [264, 192], drain: [320, 176], source: [320, 208] }, line: () => 'f 264 192 320 192 32 1.5 50' },
      D1:   { pins: { anode: [224, 336], cathode: [224, 144] }, line: () => 'd 224 336 224 144 2 default' },
      D2:   { pins: { anode: [272, 176], cathode: [272, 96] }, line: () => 'd 272 176 272 96 2 default' },
      T1: {
        pins: { primary_start: [320, 144], primary_end: [320, 176], secondary1_start: [448, 144], secondary1_end: [448, 176] },
        line: (q, c) => `T 320 144 448 144 0 ${c.Lm} ${1 / c.n} 0 0 0.999`,
      },
      Dfwd: { pins: { anode: [448, 144], cathode: [576, 144] }, line: () => 'd 448 144 576 144 2 default' },
      Dfw:  { pins: { anode: [576, 336], cathode: [576, 144] }, line: () => 'd 576 336 576 144 2 default' },
      Lout: { pins: { primary_start: [576, 144], primary_end: [704, 144] }, line: (q, c) => `l 576 144 704 144 0 ${c.L(q)} 0` },
      Cout: { pins: { 1: [704, 144], 2: [704, 336] }, line: (q, c) => `c 704 144 704 336 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [176, 96, 272, 96], [272, 96, 320, 96], [320, 96, 320, 112],
      [224, 144, 256, 144], [256, 144, 320, 144],
      [272, 176, 320, 176],
      [320, 208, 320, 336],
      [448, 176, 448, 336],
      [704, 144, 768, 144],
      [176, 336, 224, 336], [224, 336, 264, 336], [264, 336, 320, 336],
      [320, 336, 448, 336], [448, 336, 576, 336], [576, 336, 704, 336], [704, 336, 768, 336],
    ],
    synth: (c) => [
      { line: `v 176 336 176 96 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 336], [176, 96]], attach: { '176,96': ['Q1', 'drain'], '176,336': ['Cout', '2'] } },
      { line: 'g 176 336 176 352 0', posts: [[176, 336]] },
      { line: `v 256 144 256 128 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[256, 144], [256, 128]], attach: { '256,128': ['Q1', 'gate'], '256,144': ['Q1', 'source'] } },
      { line: `v 264 336 264 192 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[264, 336], [264, 192]], attach: { '264,192': ['Q2', 'gate'], '264,336': ['Q2', 'source'] } },
      { line: `r 768 144 768 336 0 ${c.rload}`, tag: 'Rload', posts: [[768, 144], [768, 336]], attach: { '768,144': ['Cout', '1'], '768,336': ['Cout', '2'] } },
      { line: '207 704 144 752 144 0 vout', posts: [[704, 144]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['Dfwd', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['Lout', 'both'], ['Dfwd', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q2', 'both'], ['Rload', 'voltage']],
      rectifier: [['Dfwd', 'both'], ['Dfw', 'both'], ['Lout', 'current'], ['Rload', 'voltage']],
      output:    [['Lout', 'current'], ['Rload', 'both']],
    },
  },

  // Isolated LLC resonant half-bridge (DEFAULT = full-bridge secondary rectifier). Q1 (high-side) / Q2
  // (low-side) form a half-bridge across the Vin bus; a split DC-block divider (Chi/Clo + balance Rbal)
  // fixes the midpoint at Vin/2. The SW node drives a SERIES tank Cr→Lr→T1 primary_start; T1 primary_end
  // returns to the split-cap midpoint. Secondary feeds a 4-diode full-wave bridge (DH1/DH2→Vout,
  // DL1/DL2→gnd) — rectifies both half-cycles so dot phase isn't critical (flags 0). NOTE: ctx picks the
  // FIRST magnetic (Lr) as the timescale driver so ctx.n=1; the step-down ratio is read from T1's own
  // turnsRatios. High-side Q1 floating drive; low-side Q2 ground-ref; complementary ~50/50.
  llc: {
    place: {
      Q1:      { pins: { gate: [384, 160], drain: [448, 144], source: [448, 176] }, line: () => 'f 384 160 448 160 32 1.5 50' },
      Q2:      { pins: { gate: [384, 272], drain: [448, 256], source: [448, 288] }, line: () => 'f 384 272 448 272 32 1.5 50' },
      Chi:     { pins: { 1: [272, 112], 2: [272, 304] }, line: (q, c) => `c 272 112 272 304 0 ${c.C(q)} ${c.vin / 2}` },
      Clo:     { pins: { 1: [272, 304], 2: [272, 400] }, line: (q, c) => `c 272 304 272 400 0 ${c.C(q)} ${c.vin / 2}` },
      Rbal_hi: { pins: { 1: [336, 112], 2: [336, 304] }, line: (q, c) => `r 336 112 336 304 0 ${c.R(q)}` },
      Rbal_lo: { pins: { 1: [336, 304], 2: [336, 400] }, line: (q, c) => `r 336 304 336 400 0 ${c.R(q)}` },
      Cr:      { pins: { 1: [512, 208], 2: [624, 208] }, line: (q, c) => `c 512 208 624 208 0 ${c.C(q)} 0` },
      Lr:      { pins: { primary_start: [624, 208], primary_end: [720, 208] }, line: (q, c) => `l 624 208 720 208 0 ${c.Lm} 0` },
      T1: {
        pins: { primary_start: [736, 208], primary_end: [736, 240], secondary1_start: [816, 208], secondary1_end: [816, 240] },
        line: (q, c) => `T 736 208 816 208 0 ${c.Lm} ${1 / resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)} 0 0 0.999`,
      },
      DH1:  { pins: { anode: [880, 208], cathode: [880, 160] }, line: () => 'd 880 208 880 160 2 default' },
      DH2:  { pins: { anode: [976, 240], cathode: [976, 160] }, line: () => 'd 976 240 976 160 2 default' },
      DL1:  { pins: { anode: [880, 400], cathode: [880, 288] }, line: () => 'd 880 400 880 288 2 default' },
      DL2:  { pins: { anode: [976, 400], cathode: [976, 320] }, line: () => 'd 976 400 976 320 2 default' },
      Cout: { pins: { 1: [1040, 160], 2: [1040, 400] }, line: (q, c) => `c 1040 160 1040 400 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [176, 112, 272, 112], [272, 112, 336, 112], [336, 112, 448, 112], [448, 112, 448, 144],
      [448, 176, 448, 208], [448, 208, 448, 256],
      [448, 208, 512, 208],
      [384, 208, 448, 208],
      [720, 208, 736, 208],
      [736, 240, 736, 304], [272, 304, 336, 304], [336, 304, 736, 304],
      [448, 288, 448, 336], [448, 336, 448, 400],
      [384, 336, 448, 336],
      [816, 208, 880, 208], [880, 208, 880, 288],
      [816, 240, 976, 240], [976, 240, 976, 320],
      [880, 160, 976, 160], [976, 160, 1040, 160], [1040, 160, 1104, 160],
      [176, 400, 272, 400], [272, 400, 336, 400], [336, 400, 448, 400],
      [448, 400, 880, 400], [880, 400, 976, 400], [976, 400, 1040, 400], [1040, 400, 1104, 400],
    ],
    synth: (c) => [
      { line: `v 176 400 176 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 400], [176, 112]], attach: { '176,112': ['Chi', '1'], '176,400': ['Clo', '2'] } },
      { line: 'g 176 400 176 416 0', posts: [[176, 400]] },
      { line: `v 384 208 384 160 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[384, 208], [384, 160]], attach: { '384,160': ['Q1', 'gate'], '384,208': ['Q1', 'source'] } },
      { line: `v 384 336 384 272 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[384, 336], [384, 272]], attach: { '384,272': ['Q2', 'gate'], '384,336': ['Q2', 'source'] } },
      { line: `r 1104 160 1104 400 0 ${c.rload}`, tag: 'Rload', posts: [[1104, 160], [1104, 400]], attach: { '1104,160': ['Cout', '1'], '1104,400': ['Cout', '2'] } },
      { line: '207 1040 160 1088 160 0 vout', posts: [[1040, 160]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['DH1', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['DH1', 'voltage'], { magI: [720, 208, 736, 208], magVtag: 'Q1' }, ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q2', 'both'], ['Rload', 'voltage']],
      rectifier: [['DH1', 'both'], ['DL1', 'both'], ['Rload', 'voltage']],
      output:    [['Q1', 'voltage'], ['DH1', 'voltage'], ['Rload', 'both']],
    },
  },

  // Isolated SRC (series-resonant) half-bridge — identical structure to LLC (series Cr-Lr tank, full-bridge rect) (DEFAULT = full-bridge secondary rectifier). Q1 (high-side) / Q2
  // (low-side) form a half-bridge across the Vin bus; a split DC-block divider (Chi/Clo + balance Rbal)
  // fixes the midpoint at Vin/2. The SW node drives a SERIES tank Cr→Lr→T1 primary_start; T1 primary_end
  // returns to the split-cap midpoint. Secondary feeds a 4-diode full-wave bridge (DH1/DH2→Vout,
  // DL1/DL2→gnd) — rectifies both half-cycles so dot phase isn't critical (flags 0). NOTE: ctx picks the
  // FIRST magnetic (Lr) as the timescale driver so ctx.n=1; the step-down ratio is read from T1's own
  // turnsRatios. High-side Q1 floating drive; low-side Q2 ground-ref; complementary ~50/50.
  src: {
    place: {
      Q1:      { pins: { gate: [384, 160], drain: [448, 144], source: [448, 176] }, line: () => 'f 384 160 448 160 32 1.5 50' },
      Q2:      { pins: { gate: [384, 272], drain: [448, 256], source: [448, 288] }, line: () => 'f 384 272 448 272 32 1.5 50' },
      Chi:     { pins: { 1: [272, 112], 2: [272, 304] }, line: (q, c) => `c 272 112 272 304 0 ${c.C(q)} ${c.vin / 2}` },
      Clo:     { pins: { 1: [272, 304], 2: [272, 400] }, line: (q, c) => `c 272 304 272 400 0 ${c.C(q)} ${c.vin / 2}` },
      Rbal_hi: { pins: { 1: [336, 112], 2: [336, 304] }, line: (q, c) => `r 336 112 336 304 0 ${c.R(q)}` },
      Rbal_lo: { pins: { 1: [336, 304], 2: [336, 400] }, line: (q, c) => `r 336 304 336 400 0 ${c.R(q)}` },
      Cr:      { pins: { 1: [512, 208], 2: [624, 208] }, line: (q, c) => `c 512 208 624 208 0 ${c.C(q)} 0` },
      Lr:      { pins: { primary_start: [624, 208], primary_end: [720, 208] }, line: (q, c) => `l 624 208 720 208 0 ${c.Lm} 0` },
      T1: {
        pins: { primary_start: [736, 208], primary_end: [736, 240], secondary1_start: [816, 208], secondary1_end: [816, 240] },
        line: (q, c) => `T 736 208 816 208 0 ${c.Lm} ${1 / resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)} 0 0 0.999`,
      },
      DH1:  { pins: { anode: [880, 208], cathode: [880, 160] }, line: () => 'd 880 208 880 160 2 default' },
      DH2:  { pins: { anode: [976, 240], cathode: [976, 160] }, line: () => 'd 976 240 976 160 2 default' },
      DL1:  { pins: { anode: [880, 400], cathode: [880, 288] }, line: () => 'd 880 400 880 288 2 default' },
      DL2:  { pins: { anode: [976, 400], cathode: [976, 320] }, line: () => 'd 976 400 976 320 2 default' },
      Cout: { pins: { 1: [1040, 160], 2: [1040, 400] }, line: (q, c) => `c 1040 160 1040 400 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [176, 112, 272, 112], [272, 112, 336, 112], [336, 112, 448, 112], [448, 112, 448, 144],
      [448, 176, 448, 208], [448, 208, 448, 256],
      [448, 208, 512, 208],
      [384, 208, 448, 208],
      [720, 208, 736, 208],
      [736, 240, 736, 304], [272, 304, 336, 304], [336, 304, 736, 304],
      [448, 288, 448, 336], [448, 336, 448, 400],
      [384, 336, 448, 336],
      [816, 208, 880, 208], [880, 208, 880, 288],
      [816, 240, 976, 240], [976, 240, 976, 320],
      [880, 160, 976, 160], [976, 160, 1040, 160], [1040, 160, 1104, 160],
      [176, 400, 272, 400], [272, 400, 336, 400], [336, 400, 448, 400],
      [448, 400, 880, 400], [880, 400, 976, 400], [976, 400, 1040, 400], [1040, 400, 1104, 400],
    ],
    synth: (c) => [
      { line: `v 176 400 176 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 400], [176, 112]], attach: { '176,112': ['Chi', '1'], '176,400': ['Clo', '2'] } },
      { line: 'g 176 400 176 416 0', posts: [[176, 400]] },
      { line: `v 384 208 384 160 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[384, 208], [384, 160]], attach: { '384,160': ['Q1', 'gate'], '384,208': ['Q1', 'source'] } },
      { line: `v 384 336 384 272 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[384, 336], [384, 272]], attach: { '384,272': ['Q2', 'gate'], '384,336': ['Q2', 'source'] } },
      { line: `r 1104 160 1104 400 0 ${c.rload}`, tag: 'Rload', posts: [[1104, 160], [1104, 400]], attach: { '1104,160': ['Cout', '1'], '1104,400': ['Cout', '2'] } },
      { line: '207 1040 160 1088 160 0 vout', posts: [[1040, 160]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['DH1', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['DH1', 'voltage'], { magI: [720, 208, 736, 208], magVtag: 'Q1' }, ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q2', 'both'], ['Rload', 'voltage']],
      rectifier: [['DH1', 'both'], ['DL1', 'both'], ['Rload', 'voltage']],
      output:    [['Q1', 'voltage'], ['DH1', 'voltage'], ['Rload', 'both']],
    },
  },

  // Non-isolated four-switch buck-boost (FSBB): a BUCK leg (Q1 high-side Vin→sw1, Q2 low-side sw1→gnd)
  // and a BOOST leg (Q3 high-side sw2→Vout, Q4 low-side sw2→gnd) bridged by a single inductor L between
  // the two switch nodes. Each leg has an RC turn-off snubber (Crc_sw→Rrc_sw). Body diodes D1-D4 and
  // numerical-aid caps Csw1/Csw2 are NOT listed comps — ignored. No CIAS load resistor ⇒ synthesize
  // Rload. Each switch runs on its OWN stimulus duty (c.dutyOf): Q1≈0.93 / Q2≈0.05 (buck pair), Q3=1.00,
  // Q4=0.00 ⇒ synchronous BUCK, L conducting to Vout through the always-on Q3. Q1 & Q3 HIGH-side (floating
  // drives referenced to their sw node); Q2 & Q4 LOW-side (ground-ref).
  fsbb: {
    place: {
      Q1:      { pins: { gate: [208, 192], drain: [272, 176], source: [272, 208] }, line: () => 'f 208 192 272 192 32 1.5 50' },
      Q2:      { pins: { gate: [400, 256], drain: [336, 240], source: [336, 272] }, line: () => 'f 400 256 336 256 32 1.5 50' },
      Q4:      { pins: { gate: [848, 256], drain: [784, 240], source: [784, 272] }, line: () => 'f 848 256 784 256 32 1.5 50' },
      Q3:      { pins: { gate: [976, 192], drain: [912, 176], source: [912, 208] }, line: () => 'f 976 192 912 192 32 1.5 50' },
      L:       { pins: { primary_start: [464, 208], primary_end: [720, 208] }, line: (q, c) => `l 464 208 720 208 0 ${c.Lm} 0` },
      Crc_sw1: { pins: { 1: [464, 208], 2: [464, 336] }, line: (q, c) => `c 464 208 464 336 0 ${c.C(q)} 0` },
      Rrc_sw1: { pins: { 1: [464, 336], 2: [464, 416] }, line: (q, c) => `r 464 336 464 416 0 ${c.R(q)}` },
      Crc_sw2: { pins: { 1: [720, 208], 2: [720, 336] }, line: (q, c) => `c 720 208 720 336 0 ${c.C(q)} 0` },
      Rrc_sw2: { pins: { 1: [720, 336], 2: [720, 416] }, line: (q, c) => `r 720 336 720 416 0 ${c.R(q)}` },
      Cout:    { pins: { 1: [1040, 128], 2: [1040, 416] }, line: (q, c) => `c 1040 128 1040 416 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [128, 128, 272, 128], [272, 128, 272, 176],
      [272, 208, 336, 208], [336, 208, 464, 208],
      [336, 208, 336, 240],
      [336, 272, 336, 416],
      [272, 208, 272, 272], [272, 272, 208, 272],
      [720, 208, 784, 208], [784, 208, 912, 208],
      [784, 208, 784, 240],
      [784, 272, 784, 416],
      [912, 176, 912, 128], [912, 128, 1040, 128],
      [1040, 128, 1104, 128],
      [912, 208, 912, 272], [912, 272, 976, 272],
      [128, 416, 336, 416], [336, 416, 400, 416], [400, 416, 464, 416],
      [464, 416, 720, 416], [720, 416, 784, 416], [784, 416, 848, 416],
      [848, 416, 1040, 416], [1040, 416, 1104, 416],
    ],
    synth: (c) => [
      { line: `v 128 416 128 128 0 0 40 ${c.vin} 0 0 0.5`, posts: [[128, 416], [128, 128]], attach: { '128,128': ['Q1', 'drain'], '128,416': ['Cout', '2'] } },
      { line: 'g 128 416 128 432 0', posts: [[128, 416]] },
      { line: `v 208 272 208 192 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('Q1')}`, posts: [[208, 272], [208, 192]], attach: { '208,192': ['Q1', 'gate'], '208,272': ['Q1', 'source'] } },
      { line: `v 400 416 400 256 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('Q2')}`, posts: [[400, 416], [400, 256]], attach: { '400,256': ['Q2', 'gate'], '400,416': ['Q2', 'source'] } },
      { line: `v 848 416 848 256 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('Q4')}`, posts: [[848, 416], [848, 256]], attach: { '848,256': ['Q4', 'gate'], '848,416': ['Q4', 'source'] } },
      { line: `v 976 272 976 192 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('Q3')}`, posts: [[976, 272], [976, 192]], attach: { '976,192': ['Q3', 'gate'], '976,272': ['Q3', 'source'] } },
      { line: `r 1104 128 1104 416 0 ${c.rload}`, tag: 'Rload', posts: [[1104, 128], [1104, 416]], attach: { '1104,128': ['Cout', '1'], '1104,416': ['Cout', '2'] } },
      { line: '207 1040 128 1088 128 0 vout', posts: [[1040, 128]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['Q3', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['L', 'both'], ['Q1', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q2', 'both'], ['L', 'current'], ['Rload', 'voltage']],
      rectifier: [['Q3', 'both'], ['Q4', 'both'], ['L', 'current'], ['Rload', 'voltage']],
      output:    [['L', 'current'], ['Rload', 'both']],
    },
  },

  ahb: {
    place: {
      Q1:   { pins: { gate: [384, 160], drain: [448, 144], source: [448, 176] }, line: () => 'f 384 160 448 160 32 1.5 50' },
      Q2:   { pins: { gate: [384, 272], drain: [448, 256], source: [448, 288] }, line: () => 'f 384 272 448 272 32 1.5 50' },
      // DC-block: Cb|1 = Vin bus, Cb|2 = primary-start node (net2). Steady-state V ≈ Vin·(1−D).
      Cb:   { pins: { 1: [680, 112], 2: [680, 160] }, line: (q, c) => `c 680 112 680 160 0 ${c.C(q)} ${c.vin * (1 - c.duty)}` },
      T1: {
        pins: { primary_start: [736, 160], primary_end: [736, 192], secondary1_start: [816, 160], secondary1_end: [816, 192] },
        line: (q, c) => `T 736 160 816 160 0 ${c.Lm} ${1 / c.n} 0 0 0.999`,
      },
      // Series-RC damper across the primary: Rdmp (net2 → snubber-mid) then Cdmp (snubber-mid → sw/net1).
      Rdmp: { pins: { 1: [608, 160], 2: [608, 224] }, line: (q, c) => `r 608 160 608 224 0 ${c.R(q)}` },
      Cdmp: { pins: { 1: [608, 224], 2: [608, 272] }, line: (q, c) => `c 608 224 608 272 0 ${c.C(q)} 0` },
      // Secondary full-wave bridge: Dr1/Dr2 anodes on the two secondary legs → rectified rail (net6);
      // Dr3/Dr4 anodes on gnd → the same two secondary legs (cathodes).
      Dr1:  { pins: { anode: [880, 160], cathode: [880, 96] }, line: () => 'd 880 160 880 96 2 default' },
      Dr2:  { pins: { anode: [976, 192], cathode: [976, 96] }, line: () => 'd 976 192 976 96 2 default' },
      // Dr3/Dr4 are the LOW legs of the 4-diode full bridge — present only in the default fullBridge
      // rectifier. The center-tapped variant's 2-diode rectifier omits them; the oneOf below turns that
      // absence into a clean "variant not laid out" skip instead of a hard export failure.
      Dr3:  { optional: true, pins: { anode: [880, 400], cathode: [880, 288] }, line: () => 'd 880 400 880 288 2 default' },
      Dr4:  { optional: true, pins: { anode: [976, 400], cathode: [976, 288] }, line: () => 'd 976 400 976 288 2 default' },
      Lout: { pins: { primary_start: [1040, 96], primary_end: [1168, 96] }, line: (q, c) => `l 1040 96 1168 96 0 ${c.L(q)} 0` },
      Cout: { pins: { 1: [1168, 96], 2: [1168, 400] }, line: (q, c) => `c 1168 96 1168 400 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      // Vin bus (net0): source top → Q1 drain, → Cb top.
      [176, 112, 448, 112], [448, 112, 448, 144], [448, 112, 680, 112],
      // sw node (net1): Q1 source → tap → Q2 drain; tap → T1 primary_end; tap → Cdmp bottom.
      [448, 176, 448, 192], [448, 192, 448, 256],
      [448, 192, 608, 192], [608, 192, 736, 192], [608, 192, 608, 272],
      [384, 192, 448, 192],   // Q1 floating gate-drive reference → sw node
      // net2 (DC-block / primary-start): Rdmp top ─ Cb bottom ─ T1 primary_start.
      [608, 160, 680, 160], [680, 160, 736, 160],
      // gnd rail (net8): Vin gnd ─ Q2 source ─ Dr3 anode ─ Dr4 anode ─ Cout bottom ─ Rload bottom.
      [448, 288, 448, 336], [448, 336, 448, 400], [384, 336, 448, 336],
      [176, 400, 448, 400], [448, 400, 880, 400], [880, 400, 976, 400], [976, 400, 1168, 400], [1168, 400, 1232, 400],
      // secondary legs net4 (Dr1 anode / Dr3 cathode) and net5 (Dr2 anode / Dr4 cathode).
      [816, 160, 880, 160], [880, 160, 880, 288],
      [816, 192, 976, 192], [976, 192, 976, 288],
      // rectified rail net6 → Lout; Vout net7 → Rload.
      [880, 96, 976, 96], [976, 96, 1040, 96], [1168, 96, 1232, 96],
    ],
    synth: (c) => [
      { line: `v 176 400 176 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 400], [176, 112]], attach: { '176,112': ['Q1', 'drain'], '176,400': ['Cout', '2'] } },
      { line: 'g 176 400 176 416 0', posts: [[176, 400]] },
      // Q1 high-side: FLOATING gate-source PWM referenced to the sw node (Q1 source), D = duty.
      { line: `v 384 192 384 160 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[384, 192], [384, 160]], attach: { '384,160': ['Q1', 'gate'], '384,192': ['Q1', 'source'] } },
      // Q2 low-side: ground-referenced complementary PWM, D = 1 − duty.
      { line: `v 384 336 384 272 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[384, 336], [384, 272]], attach: { '384,272': ['Q2', 'gate'], '384,336': ['Q2', 'source'] } },
      { line: `r 1232 96 1232 400 0 ${c.rload}`, tag: 'Rload', posts: [[1232, 96], [1232, 400]], attach: { '1232,96': ['Cout', '1'], '1232,400': ['Cout', '2'] } },
      { line: '207 1168 96 1216 96 0 vout', posts: [[1168, 96]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['Dr1', 'voltage'], ['Rload', 'voltage']],
      // Lout is a real inductor, so its own current + voltage both plot directly ('both').
      magnetic:  [['Lout', 'both'], ['Dr1', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q2', 'both'], ['Rload', 'voltage']],
      rectifier: [['Dr1', 'both'], ['Dr3', 'both'], ['Lout', 'current'], ['Rload', 'voltage']],
      output:    [['Lout', 'current'], ['Rload', 'both']],
    },
    // This layout draws the DEFAULT 4-diode full-bridge secondary. Dr3 and Dr4 exist only in that
    // rectifier; requiring each (exactly-one of a singleton group) makes the center-tapped / current-
    // doubler variants report as "not laid out yet" rather than exporting a wrong circuit.
    oneOf: [['Dr3'], ['Dr4']],
  },

  acf: {
    place: {
      Sc:    { pins: { gate: [208, 192], drain: [272, 176], source: [272, 208] }, line: () => 'f 208 192 272 192 32 1.5 50' },
      Q1:    { pins: { gate: [288, 192], drain: [352, 176], source: [352, 208] }, line: () => 'f 288 192 352 192 32 1.5 50' },
      Cc:    { pins: { 1: [272, 288], 2: [352, 288] }, line: (q, c) => `c 272 288 352 288 0 ${c.C(q)} ${c.vin * c.duty / (1 - c.duty)}` },
      T1: {
        pins: { primary_start: [448, 208], primary_end: [448, 240], secondary1_start: [544, 208], secondary1_end: [544, 240] },
        line: (q, c) => `T 448 208 544 208 0 ${c.Lm} ${1 / c.n} 0 0 0.999`,
      },
      SRfwd: { pins: { gate: [512, 208], drain: [576, 192], source: [576, 224] }, line: () => 'f 512 208 576 208 32 1.5 50' },
      SRfw:  { pins: { gate: [704, 208], drain: [640, 192], source: [640, 224] }, line: () => 'f 704 208 640 208 32 1.5 50' },
      Lout:  { pins: { primary_start: [640, 192], primary_end: [768, 192] }, line: (q, c) => `l 640 192 768 192 0 ${c.L(q)} 0` },
      Cout:  { pins: { 1: [768, 192], 2: [768, 352] }, line: (q, c) => `c 768 192 768 352 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      [176, 128, 272, 128], [272, 128, 352, 128],           // Vin rail
      [272, 176, 272, 128],                                 // Sc drain -> Vin
      [352, 176, 352, 128],                                 // Q1 drain -> Vin
      [272, 208, 272, 288],                                 // Sc source -> Cc pin1 (clamp node)
      [352, 208, 352, 288],                                 // Q1 source -> Cc pin2 (switch node)
      [352, 208, 448, 208],                                 // sw rail: Q1 source -> T1 primary_start
      [448, 240, 448, 352],                                 // T1 primary_end -> gnd
      [544, 208, 544, 224], [544, 224, 576, 224],           // T1 secondary1_start -> SRfwd source (net3)
      [544, 240, 544, 352],                                 // T1 secondary1_end -> gnd
      [576, 192, 640, 192],                                 // SRfwd drain -> rectified node (Lout / SRfw drain)
      [640, 224, 640, 352],                                 // SRfw source -> gnd
      [768, 192, 816, 192],                                 // Vout rail: Cout pin1 -> Rload
      [176, 352, 448, 352], [448, 352, 544, 352], [544, 352, 640, 352],
      [640, 352, 704, 352], [704, 352, 768, 352], [768, 352, 816, 352], // gnd rail
    ],
    synth: (c) => [
      { line: `v 176 352 176 128 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 352], [176, 128]], attach: { '176,128': ['Q1', 'drain'], '176,352': ['Cout', '2'] } },
      { line: 'g 176 352 176 368 0', posts: [[176, 352]] },
      // Q1 main switch — HIGH-side, FLOATING drive referenced to its source (sw node), duty D.
      { line: `v 288 320 288 192 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[288, 320], [288, 192]], attach: { '288,192': ['Q1', 'gate'], '288,320': ['Q1', 'source'] } },
      { line: 'w 288 320 352 320 0', posts: [[288, 320], [352, 320]], wire: true },
      { line: 'w 352 320 352 288 0', posts: [[352, 320], [352, 288]], wire: true },
      // Sc clamp switch — HIGH-side, FLOATING drive referenced to its source (clamp node), COMPLEMENTARY (1−D).
      { line: `v 208 320 208 192 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[208, 320], [208, 192]], attach: { '208,192': ['Sc', 'gate'], '208,320': ['Sc', 'source'] } },
      { line: 'w 208 320 272 320 0', posts: [[208, 320], [272, 320]], wire: true },
      { line: 'w 272 320 272 288 0', posts: [[272, 320], [272, 288]], wire: true },
      // SRfwd forward SR — source is the swinging secondary node ⇒ FLOATING drive, IN PHASE with Q1 (duty D).
      { line: `v 512 320 512 208 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[512, 320], [512, 208]], attach: { '512,208': ['SRfwd', 'gate'], '512,320': ['SRfwd', 'source'] } },
      { line: 'w 512 320 576 320 0', posts: [[512, 320], [576, 320]], wire: true },
      { line: 'w 576 320 576 224 0', posts: [[576, 320], [576, 224]], wire: true },
      // SRfw freewheel SR — source is gnd ⇒ GROUND-referenced drive, complementary (1−D).
      { line: `v 704 288 704 208 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[704, 288], [704, 208]], attach: { '704,208': ['SRfw', 'gate'], '704,288': ['SRfw', 'source'] } },
      { line: 'w 704 288 704 352 0', posts: [[704, 288], [704, 352]], wire: true },
      { line: `r 816 192 816 352 0 ${c.rload}`, tag: 'Rload', posts: [[816, 192], [816, 352]], attach: { '816,192': ['Cout', '1'], '816,352': ['Cout', '2'] } },
      { line: '207 816 192 864 192 0 vout', posts: [[816, 192]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['SRfwd', 'voltage'], ['Rload', 'voltage']],
      // Lout is a real inductor, so its own current + voltage both plot directly ('both').
      magnetic:  [['Lout', 'both'], ['SRfwd', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Sc', 'both'], ['Rload', 'voltage']],
      rectifier: [['SRfwd', 'both'], ['SRfw', 'both'], ['Lout', 'current'], ['Rload', 'voltage']],
      output:    [['Lout', 'current'], ['Rload', 'both']],
    },
  },

  // Isolated 3-level NPC phase-shifted (half) bridge (DEFAULT = full-bridge secondary rectifier). Four
  // switches S1..S4 form a VERTICAL stack across the DC bus (S1 drain=Vin, S4 source=gnd); the split-cap
  // divider CsHi/CsLo pins the bus midpoint (net1) at Vin/2. The three inner junctions are net2 (S1src/
  // S2drain), net3 (S2src/S3drain — the FLYING switching node) and net4 (S3src/S4drain); NPC clamp diodes
  // DC1 (mid→net2) and DC2 (net4→mid) tie the outer junctions to the midpoint. The primary tank hangs
  // between the flying node and the midpoint: net3 → Lr → T1 primary_start (net5), T1 primary_end → mid,
  // with a series-RC damper (Crc_pri→Rrc_pri) from net5 back to mid. S1..S3 have SWINGING sources ⇒
  // FLOATING gate drives referenced to each own source; S4 is low-side (ground-referenced). Each FET runs
  // on its OWN phase-shift stimulus duty (c.dutyOf). Secondary is a 4-diode full bridge (Dr1..Dr4) → Lout
  // → Cout. NOTE: ctx picks the FIRST magnetic (Lr) as the timescale driver so ctx.n=1; the step-down
  // ratio is read from T1's own turnsRatios. The mid rail (y=304) and the net8 secondary leg both CROSS a
  // switching column without a shared vertex — a crossing is NOT a junction (same idiom as the llc mid
  // rail crossing the Q2 leg), so they carry no short.
  pshb: {
    place: {
      // DC-bus split divider: CsHi = Vin→mid, CsLo = mid→gnd; midpoint (net1) sits at Vin/2.
      CsHi: { pins: { 1: [272, 112], 2: [272, 304] }, line: (q, c) => `c 272 112 272 304 0 ${c.C(q)} ${c.vin / 2}` },
      CsLo: { pins: { 1: [272, 304], 2: [272, 480] }, line: (q, c) => `c 272 304 272 480 0 ${c.C(q)} ${c.vin / 2}` },
      // Vertical switch stack S1 (top, drain=Vin) … S4 (bottom, source=gnd). Sources of S1/S2/S3 swing.
      S1: { pins: { gate: [384, 144], drain: [448, 128], source: [448, 160] }, line: () => 'f 384 144 448 144 32 1.5 50' },
      S2: { pins: { gate: [384, 240], drain: [448, 224], source: [448, 256] }, line: () => 'f 384 240 448 240 32 1.5 50' },
      S3: { pins: { gate: [384, 336], drain: [448, 320], source: [448, 352] }, line: () => 'f 384 336 448 336 32 1.5 50' },
      S4: { pins: { gate: [384, 432], drain: [448, 416], source: [448, 448] }, line: () => 'f 384 432 448 432 32 1.5 50' },
      // NPC clamp diodes: DC1 anode=mid, cathode=net2; DC2 anode=net4, cathode=mid.
      DC1: { pins: { anode: [336, 304], cathode: [336, 192] }, line: () => 'd 336 304 336 192 2 default' },
      DC2: { pins: { anode: [336, 384], cathode: [336, 304] }, line: () => 'd 336 384 336 304 2 default' },
      // Resonant/leakage inductor: flying node (net3) → net5.
      Lr: { pins: { primary_start: [640, 272], primary_end: [736, 272] }, line: (q, c) => `l 640 272 736 272 0 ${c.Lm} 0` },
      T1: {
        pins: { primary_start: [800, 272], primary_end: [800, 304], secondary1_start: [880, 272], secondary1_end: [880, 304] },
        line: (q, c) => `T 800 272 880 272 0 ${c.Lm} ${1 / resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)} 0 0 0.999`,
      },
      // Series-RC primary damper across the tank: Crc_pri (net5→net6) then Rrc_pri (net6→mid).
      Crc_pri: { pins: { 1: [744, 272], 2: [744, 288] }, line: (q, c) => `c 744 272 744 288 0 ${c.C(q)} 0` },
      Rrc_pri: { pins: { 1: [744, 288], 2: [744, 304] }, line: (q, c) => `r 744 288 744 304 0 ${c.R(q)}` },
      // Secondary full-wave bridge: Dr1/Dr2 to the rectified rail (net9); Dr3/Dr4 from gnd to the two legs.
      Dr1: { pins: { anode: [960, 272], cathode: [960, 176] }, line: () => 'd 960 272 960 176 2 default' },
      Dr2: { pins: { anode: [1056, 304], cathode: [1056, 176] }, line: () => 'd 1056 304 1056 176 2 default' },
      // Dr3/Dr4 are the LOW legs of the 4-diode bridge — present only in the default fullBridge rectifier.
      // The center-tapped / current-doubler variants omit them; the oneOf below turns that absence into a
      // clean "variant not laid out" skip instead of a hard export failure.
      Dr3: { optional: true, pins: { anode: [960, 480], cathode: [960, 336] }, line: () => 'd 960 480 960 336 2 default' },
      Dr4: { optional: true, pins: { anode: [1056, 480], cathode: [1056, 336] }, line: () => 'd 1056 480 1056 336 2 default' },
      Lout: { pins: { primary_start: [1120, 176], primary_end: [1248, 176] }, line: (q, c) => `l 1120 176 1248 176 0 ${c.L(q)} 0` },
      Cout: { pins: { 1: [1248, 176], 2: [1248, 480] }, line: (q, c) => `c 1248 176 1248 480 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      // Vin bus (net0): source top → CsHi top → S1 drain.
      [176, 112, 272, 112], [272, 112, 448, 112], [448, 112, 448, 128],
      // Switch-stack column: net2 (S1src→S2drain), net3 (S2src→S3drain, flying), net4 (S3src→S4drain),
      // then S4 source → gnd. Each net has a mid junction for the floating gate-drive tie.
      [448, 160, 448, 192], [448, 192, 448, 224],
      [448, 256, 448, 272], [448, 272, 448, 288], [448, 288, 448, 320],
      [448, 352, 448, 384], [448, 384, 448, 416],
      [448, 448, 448, 480],
      // Gate-drive source references: helper coord → each switch's own source net.
      [384, 192, 448, 192], [384, 288, 448, 288], [384, 384, 448, 384], [384, 480, 448, 480],
      // Mid rail (net1): divider mid → clamps → damper → T1 primary_end. Crosses the flying column at
      // (448,304) with NO vertex there — a crossing, not a junction, so no short (as in the llc mid rail).
      [272, 304, 336, 304], [336, 304, 744, 304], [744, 304, 800, 304],
      // Clamp-diode ties: DC1 cathode → net2, DC2 anode → net4.
      [336, 192, 448, 192], [336, 384, 448, 384],
      // Primary tank: flying node (net3) → Lr → net5 → T1 primary_start; net5 also feeds the RC damper.
      [448, 272, 640, 272], [736, 272, 744, 272], [744, 272, 800, 272],
      // Secondary leg net7 (T1 sec1_start / Dr1 anode / Dr3 cathode).
      [880, 272, 960, 272], [960, 272, 960, 336],
      // Secondary leg net8 (T1 sec1_end / Dr2 anode / Dr4 cathode); crosses the net7 leg at (960,304)
      // with no shared vertex → no short.
      [880, 304, 1056, 304], [1056, 304, 1056, 336],
      // Rectified rail (net9) → Lout.
      [960, 176, 1056, 176], [1056, 176, 1120, 176],
      // Vout rail.
      [1248, 176, 1312, 176],
      // gnd rail (net11): source gnd ─ CsLo bottom ─ S4 source ─ Dr3 anode ─ Dr4 anode ─ Cout ─ Rload.
      [176, 480, 272, 480], [272, 480, 448, 480], [448, 480, 960, 480],
      [960, 480, 1056, 480], [1056, 480, 1248, 480], [1248, 480, 1312, 480],
    ],
    synth: (c) => [
      { line: `v 176 480 176 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 480], [176, 112]], attach: { '176,112': ['S1', 'drain'], '176,480': ['CsLo', '2'] } },
      { line: 'g 176 480 176 496 0', posts: [[176, 480]] },
      // S1/S2/S3 high-side: FLOATING gate-source PWM referenced to each own (swinging) source; S4 low-side:
      // ground-referenced. Each FET runs on its OWN phase-shift stimulus duty via c.dutyOf.
      { line: `v 384 192 384 144 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('S1')}`, posts: [[384, 192], [384, 144]], attach: { '384,144': ['S1', 'gate'], '384,192': ['S1', 'source'] } },
      { line: `v 384 288 384 240 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('S2')}`, posts: [[384, 288], [384, 240]], attach: { '384,240': ['S2', 'gate'], '384,288': ['S2', 'source'] } },
      { line: `v 384 384 384 336 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('S3')}`, posts: [[384, 384], [384, 336]], attach: { '384,336': ['S3', 'gate'], '384,384': ['S3', 'source'] } },
      { line: `v 384 480 384 432 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('S4')}`, posts: [[384, 480], [384, 432]], attach: { '384,432': ['S4', 'gate'], '384,480': ['S4', 'source'] } },
      { line: `r 1312 176 1312 480 0 ${c.rload}`, tag: 'Rload', posts: [[1312, 176], [1312, 480]], attach: { '1312,176': ['Cout', '1'], '1312,480': ['Cout', '2'] } },
      { line: '207 1248 176 1296 176 0 vout', posts: [[1248, 176]] },
    ],
    scopeSets: {
      overview:  [['S1', 'voltage'], ['Dr1', 'voltage'], ['Rload', 'voltage']],
      // Lr (resonant/leakage) and Lout (output choke) are real inductors, so their own I+V plot directly.
      magnetic:  [['Lr', 'both'], ['Lout', 'both'], ['Rload', 'voltage']],
      switch:    [['S1', 'both'], ['S4', 'both'], ['Rload', 'voltage']],
      rectifier: [['Dr1', 'both'], ['Dr3', 'both'], ['Lout', 'current'], ['Rload', 'voltage']],
      output:    [['Lout', 'current'], ['Rload', 'both']],
    },
    // The layout draws the DEFAULT 4-diode full bridge. Dr3/Dr4 exist only in that rectifier; requiring
    // exactly one of each singleton group makes the center-tapped / current-doubler variants report as
    // "not laid out yet" rather than exporting a wrong circuit (same idiom as ahb).
    oneOf: [['Dr3'], ['Dr4']],
  },

  cllc: {
    place: {
      // ── primary full bridge (two legs, side by side) ──────────────────────────────────────────
      Q1: { pins: { gate: [208, 176], drain: [272, 160], source: [272, 192] }, line: () => 'f 208 176 272 176 32 1.5 50' },
      Q2: { pins: { gate: [208, 320], drain: [272, 304], source: [272, 336] }, line: () => 'f 208 320 272 320 32 1.5 50' },
      Q3: { pins: { gate: [416, 176], drain: [480, 160], source: [480, 192] }, line: () => 'f 416 176 480 176 32 1.5 50' },
      Q4: { pins: { gate: [416, 320], drain: [480, 304], source: [480, 336] }, line: () => 'f 416 320 480 320 32 1.5 50' },
      // ── primary resonant tank Cr1 → Lr1 → T1 primary ──────────────────────────────────────────
      Cr1: { pins: { 1: [336, 256], 2: [400, 256] }, line: (q, c) => `c 336 256 400 256 0 ${c.C(q)} 0` },
      Lr1: { pins: { primary_start: [608, 256], primary_end: [720, 256] }, line: (q, c) => `l 608 256 720 256 0 ${c.L(q)} 0` },
      T1: {
        pins: { primary_start: [800, 256], primary_end: [800, 288], secondary1_start: [880, 256], secondary1_end: [880, 288] },
        line: (q, c) => `T 800 256 880 256 0 ${c.Lm} ${1 / resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)} 0 0 0.999`,
      },
      // ── secondary resonant tank Lr2 → Cr2 (mirror of the primary) ─────────────────────────────
      Lr2: { pins: { primary_start: [944, 256], primary_end: [1056, 256] }, line: (q, c) => `l 944 256 1056 256 0 ${c.L(q)} 0` },
      Cr2: { pins: { 1: [1184, 256], 2: [1248, 256] }, line: (q, c) => `c 1184 256 1248 256 0 ${c.C(q)} 0` },
      // ── secondary full bridge (synchronous rectifier) ─────────────────────────────────────────
      Qc: { pins: { gate: [1184, 176], drain: [1120, 160], source: [1120, 192] }, line: () => 'f 1184 176 1120 176 32 1.5 50' },
      Qd: { pins: { gate: [1184, 320], drain: [1120, 304], source: [1120, 336] }, line: () => 'f 1184 320 1120 320 32 1.5 50' },
      Qa: { pins: { gate: [1376, 176], drain: [1312, 160], source: [1312, 192] }, line: () => 'f 1376 176 1312 176 32 1.5 50' },
      Qb: { pins: { gate: [1376, 320], drain: [1312, 304], source: [1312, 336] }, line: () => 'f 1376 320 1312 320 32 1.5 50' },
      Cout: { pins: { 1: [1440, 96], 2: [1440, 432] }, line: (q, c) => `c 1440 96 1440 432 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      // primary Vin bus (net0): source → both high-side drains
      [176, 96, 272, 96], [272, 96, 480, 96], [272, 96, 272, 160], [480, 96, 480, 160],
      // leg1 vertical: Q1 source → net1 (leg mid) → Q2 drain → … → gnd
      [272, 192, 272, 256], [272, 256, 272, 304], [272, 336, 272, 384], [272, 384, 272, 432],
      // leg2 vertical: Q3 source → net2 (leg mid) → Q4 drain → … → gnd
      [480, 192, 480, 240], [480, 240, 480, 304], [480, 336, 480, 384], [480, 384, 480, 432],
      // primary tank rail: net1 → Cr1, Cr1|2(net3) crosses leg2 to Lr1, Lr1(net4) → T1.primary_start
      [272, 256, 336, 256], [400, 256, 608, 256], [720, 256, 800, 256],
      // net2 wrap: leg2 mid → T1.primary_end (routed clear of the tank rail)
      [480, 240, 768, 240], [768, 240, 768, 288], [768, 288, 800, 288],
      // high-side floating gate-drive reference ties (ref → each high FET's own source node)
      [208, 256, 272, 256], [416, 240, 480, 240],
      // low-side ground-referenced gate-drive reference ties (ref → the source-to-gnd wire)
      [208, 384, 272, 384], [416, 384, 480, 384],
      // ── secondary side (mirror) ──────────────────────────────────────────────────────────────
      // secondary tank: T1.secondary1_start(net5) → Lr2, Lr2|end(net6) crosses leg-c to Cr2, Cr2|2(net7) → leg-a
      [880, 256, 944, 256], [1056, 256, 1184, 256], [1248, 256, 1312, 256],
      // leg-c vertical: Qc source → net8 (= T1.secondary1_end) → Qd drain → … → gnd
      [1120, 192, 1120, 240], [1120, 240, 1120, 304], [1120, 336, 1120, 384], [1120, 384, 1120, 432],
      // net8 wrap: leg-c mid → T1.secondary1_end
      [1120, 240, 912, 240], [912, 240, 912, 288], [912, 288, 880, 288],
      // leg-a vertical: Qa source → net7 (leg mid) → Qb drain → … → gnd
      [1312, 192, 1312, 256], [1312, 256, 1312, 304], [1312, 336, 1312, 384], [1312, 384, 1312, 432],
      // secondary high-side ref ties
      [1184, 240, 1120, 240], [1376, 256, 1312, 256],
      // secondary low-side ref ties
      [1184, 384, 1120, 384], [1376, 384, 1312, 384],
      // Vout bus (net9): both high-side drains → Cout
      [1120, 96, 1120, 160], [1120, 96, 1312, 96], [1312, 96, 1312, 160], [1312, 96, 1440, 96],
      // ground rail (net10): spans both bridges' low-side sources + Cout bottom
      [176, 432, 272, 432], [272, 432, 480, 432], [480, 432, 1120, 432],
      [1120, 432, 1312, 432], [1312, 432, 1440, 432], [1440, 432, 1472, 432],
      // Rload tie
      [1440, 96, 1472, 96],
    ],
    synth: (c) => [
      // primary DC input source (Vin bus → gnd) + ground
      { line: `v 176 432 176 96 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 432], [176, 96]], attach: { '176,96': ['Q1', 'drain'], '176,432': ['Cout', '2'] } },
      { line: 'g 176 432 176 448 0', posts: [[176, 432]] },
      // eight gate drives — one per FET, on its diagonal's duty. Q1-diagonal {Q1,Q4,Qa,Qd} = c.duty;
      // Q2-diagonal {Q2,Q3,Qb,Qc} = 1-c.duty. High-side float (ref → own source); low-side ground-ref.
      // The PRIMARY four switch at phase 0; the SECONDARY synchronous-rectifier four (Qa–Qd) are driven
      // π out of phase (phaseShift = Math.PI) so each SR FET conducts on the half-cycle its body diode
      // would — in phase, they short the tank and collapse Vout (~0.35 V); π-shifted they rectify to the
      // full +Vout. Verified live in CircuitJS1: in-phase → 0.35 V, π-shifted → 48.0 V (design target).
      { line: `v 208 256 208 176 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[208, 256], [208, 176]], attach: { '208,176': ['Q1', 'gate'], '208,256': ['Q1', 'source'] } },
      { line: `v 208 384 208 320 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[208, 384], [208, 320]], attach: { '208,320': ['Q2', 'gate'], '208,384': ['Q2', 'source'] } },
      { line: `v 416 240 416 176 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[416, 240], [416, 176]], attach: { '416,176': ['Q3', 'gate'], '416,240': ['Q3', 'source'] } },
      { line: `v 416 384 416 320 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[416, 384], [416, 320]], attach: { '416,320': ['Q4', 'gate'], '416,384': ['Q4', 'source'] } },
      { line: `v 1376 256 1376 176 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.duty}`, posts: [[1376, 256], [1376, 176]], attach: { '1376,176': ['Qa', 'gate'], '1376,256': ['Qa', 'source'] } },
      { line: `v 1376 384 1376 320 0 2 ${c.fVis} 5 5 ${Math.PI} ${1 - c.duty}`, posts: [[1376, 384], [1376, 320]], attach: { '1376,320': ['Qb', 'gate'], '1376,384': ['Qb', 'source'] } },
      { line: `v 1184 240 1184 176 0 2 ${c.fVis} 5 5 ${Math.PI} ${1 - c.duty}`, posts: [[1184, 240], [1184, 176]], attach: { '1184,176': ['Qc', 'gate'], '1184,240': ['Qc', 'source'] } },
      { line: `v 1184 384 1184 320 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.duty}`, posts: [[1184, 384], [1184, 320]], attach: { '1184,320': ['Qd', 'gate'], '1184,384': ['Qd', 'source'] } },
      // load across Vout + JS-API-readable vout node
      { line: `r 1472 96 1472 432 0 ${c.rload}`, tag: 'Rload', posts: [[1472, 96], [1472, 432]], attach: { '1472,96': ['Cout', '1'], '1472,432': ['Cout', '2'] } },
      { line: '207 1440 96 1488 96 0 vout', posts: [[1440, 96]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['Qa', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['Qa', 'voltage'], { magI: [720, 256, 800, 256], magVtag: 'Q1' }, ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q3', 'both'], ['Rload', 'voltage']],
      rectifier: [['Qa', 'both'], ['Qc', 'both'], ['Rload', 'voltage']],
      output:    [['Q1', 'voltage'], ['Qa', 'voltage'], ['Rload', 'both']],
    },
  },

  psfb: {
    place: {
      QA:      { pins: { gate: [384, 160], drain: [448, 144], source: [448, 176] }, line: () => 'f 384 160 448 160 32 1.5 50' },
      QB:      { pins: { gate: [384, 272], drain: [448, 256], source: [448, 288] }, line: () => 'f 384 272 448 272 32 1.5 50' },
      QC:      { pins: { gate: [624, 160], drain: [560, 144], source: [560, 176] }, line: () => 'f 624 160 560 160 32 1.5 50' },
      QD:      { pins: { gate: [624, 272], drain: [560, 256], source: [560, 288] }, line: () => 'f 624 272 560 272 32 1.5 50' },
      Lr:      { pins: { primary_start: [504, 336], primary_end: [632, 336] }, line: (q, c) => `l 504 336 632 336 0 ${c.Lm} 0` },
      T1: {
        pins: { primary_start: [680, 336], primary_end: [680, 368], secondary1_start: [760, 336], secondary1_end: [760, 368] },
        line: (q, c) => `T 680 336 760 336 0 ${c.Lm} ${1 / resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)} 0 0 0.999`,
      },
      Crc_pri: { pins: { 1: [504, 432], 2: [632, 432] }, line: (q, c) => `c 504 432 632 432 0 ${c.C(q)} 0` },
      Rrc_pri: { pins: { 1: [632, 432], 2: [760, 432] }, line: (q, c) => `r 632 432 760 432 0 ${c.R(q)}` },
      Dr1:  { pins: { anode: [824, 336], cathode: [824, 288] }, line: () => 'd 824 336 824 288 2 default' },
      Dr2:  { pins: { anode: [920, 368], cathode: [920, 288] }, line: () => 'd 920 368 920 288 2 default' },
      // Dr3/Dr4 (the 2nd diode leg) + Rbsa/Rbsb (secondary bleeds) exist ONLY in the fullBridge secondary.
      // Marked optional so the centerTapped variant (Dr1/Dr2 only — a strict subset) doesn't trip the
      // "layout places X but the TAS has no such component" guard; the oneOf below then routes it cleanly
      // to the allowlisted "not laid out" skip instead of a hard export failure.
      Dr3:  { optional: true, pins: { anode: [824, 528], cathode: [824, 416] }, line: () => 'd 824 528 824 416 2 default' },
      Dr4:  { optional: true, pins: { anode: [920, 528], cathode: [920, 448] }, line: () => 'd 920 528 920 448 2 default' },
      Lout: { pins: { primary_start: [984, 288], primary_end: [1112, 288] }, line: (q, c) => `l 984 288 1112 288 0 ${c.L(q)} 0` },
      Rbsa: { optional: true, pins: { 1: [792, 336], 2: [792, 528] }, line: (q, c) => `r 792 336 792 528 0 ${c.R(q)}` },
      Rbsb: { optional: true, pins: { 1: [888, 368], 2: [888, 528] }, line: (q, c) => `r 888 368 888 528 0 ${c.R(q)}` },
      Cout: { pins: { 1: [1112, 288], 2: [1112, 528] }, line: (q, c) => `c 1112 288 1112 528 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      // Vin bus (net0): QA/QC drains up to the top rail, tied to the source
      [448, 144, 448, 112], [560, 144, 560, 112], [176, 112, 448, 112], [448, 112, 560, 112],
      // leg-A mid (net1): QA source / QB drain / Lr primary_start / Crc_pri pin1
      [448, 176, 448, 208], [448, 208, 448, 256], [448, 208, 504, 208], [504, 208, 504, 336], [504, 336, 504, 432],
      // leg-C mid (net2): QC source / QD drain / T1 primary_end / Rrc_pri pin2 (dropped below the bridge)
      [560, 176, 560, 208], [560, 208, 560, 256], [560, 208, 600, 208], [600, 208, 600, 400],
      [600, 400, 680, 400], [680, 400, 680, 368], [680, 400, 760, 400], [760, 400, 760, 432],
      // Lr primary_end → T1 primary_start (net4)
      [632, 336, 680, 336],
      // secondary A (net5): T1 sec_start / Dr1 anode / Dr3 cathode / Rbsa pin1 (split at 792 for the bleed tap)
      [760, 336, 792, 336], [792, 336, 824, 336], [824, 336, 824, 416],
      // secondary B (net6): T1 sec_end / Dr2 anode / Dr4 cathode / Rbsb pin1. The horizontal run crosses the
      // net5 column at (824,368) — falstad wires join ONLY at shared endpoints, so a mid-span cross is not a node.
      [760, 368, 888, 368], [888, 368, 920, 368], [920, 368, 920, 448],
      // rectified node (net7): Dr1 cathode / Dr2 cathode / Lout primary_start
      [824, 288, 920, 288], [920, 288, 984, 288],
      // Vout (net8): Lout primary_end / Cout pin1 → load
      [1112, 288, 1176, 288],
      // low-side sources down to ground, each split so a ground-referenced gate drive can tap it
      [448, 288, 448, 336], [448, 336, 448, 528],
      [560, 288, 560, 336], [560, 336, 560, 528],
      // ground rail (net9): source⁻ / QB,QD sources / Dr3,Dr4 anodes / Rbsa,Rbsb pin2 / Cout pin2 / load
      [176, 528, 448, 528], [448, 528, 560, 528], [560, 528, 792, 528], [792, 528, 824, 528],
      [824, 528, 888, 528], [888, 528, 920, 528], [920, 528, 1112, 528], [1112, 528, 1176, 528],
    ],
    synth: (c) => [
      { line: `v 176 528 176 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 528], [176, 112]], attach: { '176,112': ['QA', 'drain'], '176,528': ['Cout', '2'] } },
      { line: 'g 176 528 176 544 0', posts: [[176, 528]] },
      // Leg A — QA high-side FLOATING drive (ref wired to the leg-A mid), phase 0; QB low-side, complementary
      // (INVERTED via a π phase offset), ground-referenced. Both ~50% duty.
      { line: `v 384 208 384 160 0 2 ${c.fVis} 5 5 0 0.5`, posts: [[384, 208], [384, 160]], attach: { '384,160': ['QA', 'gate'], '384,208': ['QA', 'source'] } },
      { line: 'w 384 208 448 208 0', posts: [[384, 208], [448, 208]], wire: true },
      { line: `v 384 336 384 272 0 2 ${c.fVis} 5 5 3.14 0.5`, posts: [[384, 336], [384, 272]], attach: { '384,272': ['QB', 'gate'], '384,336': ['QB', 'source'] } },
      { line: 'w 384 336 448 336 0', posts: [[384, 336], [448, 336]], wire: true },
      // Leg C — PHASE-SHIFTED by ~2.2 rad (≈126°): this phase, not the duty, transfers power. QC high-side
      // FLOATING drive (ref = leg-C mid) at phase 2.2; QD low-side, complementary (2.2 + π = 5.34).
      { line: `v 624 208 624 160 0 2 ${c.fVis} 5 5 2.2 0.5`, posts: [[624, 208], [624, 160]], attach: { '624,160': ['QC', 'gate'], '624,208': ['QC', 'source'] } },
      { line: 'w 600 208 624 208 0', posts: [[600, 208], [624, 208]], wire: true },
      { line: `v 624 336 624 272 0 2 ${c.fVis} 5 5 5.34 0.5`, posts: [[624, 336], [624, 272]], attach: { '624,272': ['QD', 'gate'], '624,336': ['QD', 'source'] } },
      { line: 'w 560 336 624 336 0', posts: [[560, 336], [624, 336]], wire: true },
      { line: `r 1176 288 1176 528 0 ${c.rload}`, tag: 'Rload', posts: [[1176, 288], [1176, 528]], attach: { '1176,288': ['Cout', '1'], '1176,528': ['Cout', '2'] } },
      { line: '207 1112 288 1160 288 0 vout', posts: [[1112, 288]] },
    ],
    scopeSets: {
      overview:  [['QA', 'voltage'], ['Dr1', 'voltage'], ['Rload', 'voltage']],
      // Lout is a real inductor (own current+voltage 'both'); the transformer PRIMARY current is read from
      // the Lr→T1 wire (CircuitJS1 transformers report no device current), overlaid with QA's node voltage.
      magnetic:  [['Lout', 'both'], { magI: [632, 336, 680, 336], magVtag: 'QA' }, ['Rload', 'voltage']],
      switch:    [['QA', 'both'], ['QC', 'both'], ['Rload', 'voltage']],
      rectifier: [['Dr1', 'both'], ['Dr3', 'both'], ['Lout', 'current'], ['Rload', 'voltage']],
      output:    [['Lout', 'current'], ['Rload', 'both']],
    },
    // Only the fullBridge secondary is drawn. Dr3 is present iff the design is fullBridge, so requiring
    // exactly one member routes centerTapped (0 present) to the allowlisted "expects exactly one of" skip
    // (its center-tapped secondary is a different, not-yet-laid-out topology). currentDoubler already
    // skips earlier on its unplaced Lo2/Rlb.
    oneOf: [['Dr3']],
  },

  // Isolated CLLLC resonant converter — a FULL bridge on BOTH sides with a resonant tank each side. The
  // primary full bridge Q1(HS)/Q2(LS) (left leg) + Q3(HS)/Q4(LS) (right leg) drives, diagonally, a
  // primary tank Cr1→Lr1 in series with the T1 magnetizing/primary. The secondary is a synchronous-
  // rectifier full bridge QE(HS)/QF(LS) + QG(HS)/QH(LS) with its OWN resonant tank Lr2→Cr2 (+ a tiny
  // series sense resistor Rsense) between T1 secondary and the bridge midpoint, feeding Cout. The two
  // legs of each bridge share a diagonal gate net (Q1/Q4/QE/QH share net12; Q2/Q3/QF/QG share net13),
  // but every FET gets its OWN drive here: the four HIGH-side FETs (Q1,Q3,QE,QG) get FLOATING drives
  // referenced to their swinging source node; the four LOW-side FETs (Q2,Q4,QF,QH) are ground-ref.
  // Diagonal 1 (Q1,Q4,QE,QH) runs at duty; diagonal 2 (Q2,Q3,QF,QG) complementary (1−duty). The
  // SECONDARY SR four (QE–QH) are driven π out of phase (phaseShift = Math.PI) with their primary
  // diagonal — in phase they short the tank and collapse Vout (like cllc: ~0.4 V); π-shifted they
  // rectify to the full +Vout. The magnetics are placed on an overhead lane (y=64): the two resonant
  // branches never touch the bridge legs they pass over (falstad joins only at shared endpoints). ctx
  // picks the first magnetic (Lr1) as the timescale driver so ctx.n=1; every L uses c.L(q) and T1's
  // step-down ratio is read from its own turnsRatios. Vout ≈ Vin·Ns/Np ≈ +48 V at resonance (positive).
  clllc: {
    place: {
      // PRIMARY full bridge — left leg Q1/Q2 (mid net1), right leg Q3/Q4 (mid net2).
      Q1: { pins: { gate: [240, 160], drain: [304, 144], source: [304, 176] }, line: () => 'f 240 160 304 160 32 1.5 50' },
      Q2: { pins: { gate: [240, 272], drain: [304, 256], source: [304, 288] }, line: () => 'f 240 272 304 272 32 1.5 50' },
      Q3: { pins: { gate: [416, 160], drain: [480, 144], source: [480, 176] }, line: () => 'f 416 160 480 160 32 1.5 50' },
      Q4: { pins: { gate: [416, 272], drain: [480, 256], source: [480, 288] }, line: () => 'f 416 272 480 272 32 1.5 50' },
      // Primary tank on the overhead lane: net1 → Cr1 → Lr1 → T1.primary_start (net4). Seed Cr1 at 0.
      Cr1: { pins: { 1: [352, 64], 2: [448, 64] }, line: (q, c) => `c 352 64 448 64 0 ${c.C(q)} 0` },
      Lr1: { pins: { primary_start: [448, 64], primary_end: [544, 64] }, line: (q, c) => `l 448 64 544 64 0 ${c.L(q)} 0` },
      T1: {
        pins: { primary_start: [608, 192], primary_end: [608, 224], secondary1_start: [688, 192], secondary1_end: [688, 224] },
        line: (q, c) => `T 608 192 688 192 0 ${c.L(q)} ${1 / resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)} 0 0 0.999`,
      },
      // Secondary tank on the overhead lane: T1.secondary1_start (net5) → Lr2 → Cr2 → Rsense → net8.
      Lr2: { pins: { primary_start: [688, 64], primary_end: [784, 64] }, line: (q, c) => `l 688 64 784 64 0 ${c.L(q)} 0` },
      Cr2: { pins: { 1: [784, 64], 2: [880, 64] }, line: (q, c) => `c 784 64 880 64 0 ${c.C(q)} 0` },
      Rsense: { pins: { 1: [880, 64], 2: [976, 64] }, line: (q, c) => `r 880 64 976 64 0 ${c.R(q)}` },
      // SECONDARY full bridge — near leg QG/QH (mid net9), far leg QE/QF (mid net8).
      QG: { pins: { gate: [752, 160], drain: [816, 144], source: [816, 176] }, line: () => 'f 752 160 816 160 32 1.5 50' },
      QH: { pins: { gate: [752, 272], drain: [816, 256], source: [816, 288] }, line: () => 'f 752 272 816 272 32 1.5 50' },
      QE: { pins: { gate: [944, 160], drain: [1008, 144], source: [1008, 176] }, line: () => 'f 944 160 1008 160 32 1.5 50' },
      QF: { pins: { gate: [944, 272], drain: [1008, 256], source: [1008, 288] }, line: () => 'f 944 272 1008 272 32 1.5 50' },
      Cout: { pins: { 1: [1120, 112], 2: [1120, 400] }, line: (q, c) => `c 1120 112 1120 400 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      // net0 (Vin+ bus): Vsource → Q1 drain, → Q3 drain.
      [160, 112, 304, 112], [304, 112, 304, 144], [304, 112, 480, 112], [480, 112, 480, 144],
      // net1 (left mid): Q1 source ─ Q2 drain ─ up to the overhead lane → Cr1.1.
      [304, 176, 304, 208], [304, 208, 304, 256], [304, 208, 352, 208], [352, 208, 352, 64],
      // net2 (right mid): Q3 source ─ Q4 drain ─ across to T1 primary_end.
      [480, 176, 480, 208], [480, 208, 480, 256], [480, 208, 544, 208], [544, 208, 544, 224], [544, 224, 608, 224],
      // net4: Lr1 primary_end → down to T1 primary_start.
      [544, 64, 608, 64], [608, 64, 608, 192],
      // net5: T1 secondary1_start → up to Lr2 primary_start.
      [688, 192, 688, 64],
      // net8 (far mid): Rsense.2 → down to QE source / QF drain hub.
      [976, 64, 976, 208], [976, 208, 1008, 208], [1008, 176, 1008, 208], [1008, 208, 1008, 256],
      // net9 (near mid): QG source ─ QH drain ─ across to T1 secondary1_end.
      [816, 176, 816, 208], [816, 208, 816, 256], [816, 208, 752, 208], [752, 208, 752, 224], [752, 224, 688, 224],
      // net10 (Vout+ bus): QG drain ─ QE drain ─ Cout.1.
      [816, 144, 816, 112], [816, 112, 1008, 112], [1008, 112, 1008, 144], [1008, 112, 1120, 112],
      // net11 (common gnd rail): Vsource− ─ Q2/Q4/QH/QF sources ─ Cout.2.
      [160, 400, 304, 400], [304, 288, 304, 336], [304, 336, 304, 400], [304, 400, 480, 400],
      [480, 288, 480, 336], [480, 336, 480, 400], [480, 400, 816, 400],
      [816, 288, 816, 336], [816, 336, 816, 400], [816, 400, 1008, 400],
      [1008, 288, 1008, 336], [1008, 336, 1008, 400], [1008, 400, 1120, 400],
      // gate-drive reference ties: HS floating refs to their mid node, LS refs to the gnd rail.
      [240, 208, 304, 208],   // Q1 (HS) → net1
      [240, 336, 304, 336],   // Q2 (LS) → gnd
      [416, 208, 480, 208],   // Q3 (HS) → net2
      [416, 336, 480, 336],   // Q4 (LS) → gnd
      [944, 208, 976, 208],   // QE (HS) → net8
      [944, 336, 1008, 336],  // QF (LS) → gnd
      [752, 336, 816, 336],   // QH (LS) → gnd   (QG's HS ref rides the net9 tie at 752,208)
      // Rload leads off the Vout bus.
      [1120, 112, 1200, 112], [1120, 400, 1200, 400],
    ],
    synth: (c) => [
      { line: `v 160 400 160 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[160, 400], [160, 112]], attach: { '160,112': ['Q1', 'drain'], '160,400': ['Cout', '2'] } },
      { line: 'g 160 400 160 416 0', posts: [[160, 400]] },
      // Diagonal 1 (Q1,Q4,QE,QH) at duty; diagonal 2 (Q2,Q3,QF,QG) complementary (1−duty). The SECONDARY
      // SR four (QE,QF,QG,QH) run π out of phase (phaseShift = Math.PI) so each conducts on the half-cycle
      // its body diode would — in phase they short the tank and collapse Vout (cllc-class bug).
      { line: `v 240 208 240 160 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[240, 208], [240, 160]], attach: { '240,160': ['Q1', 'gate'], '240,208': ['Q1', 'source'] } },
      { line: `v 240 336 240 272 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[240, 336], [240, 272]], attach: { '240,272': ['Q2', 'gate'], '240,336': ['Q2', 'source'] } },
      { line: `v 416 208 416 160 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[416, 208], [416, 160]], attach: { '416,160': ['Q3', 'gate'], '416,208': ['Q3', 'source'] } },
      { line: `v 416 336 416 272 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[416, 336], [416, 272]], attach: { '416,272': ['Q4', 'gate'], '416,336': ['Q4', 'source'] } },
      { line: `v 944 208 944 160 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.duty}`, posts: [[944, 208], [944, 160]], attach: { '944,160': ['QE', 'gate'], '944,208': ['QE', 'source'] } },
      { line: `v 944 336 944 272 0 2 ${c.fVis} 5 5 ${Math.PI} ${1 - c.duty}`, posts: [[944, 336], [944, 272]], attach: { '944,272': ['QF', 'gate'], '944,336': ['QF', 'source'] } },
      { line: `v 752 208 752 160 0 2 ${c.fVis} 5 5 ${Math.PI} ${1 - c.duty}`, posts: [[752, 208], [752, 160]], attach: { '752,160': ['QG', 'gate'], '752,208': ['QG', 'source'] } },
      { line: `v 752 336 752 272 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.duty}`, posts: [[752, 336], [752, 272]], attach: { '752,272': ['QH', 'gate'], '752,336': ['QH', 'source'] } },
      { line: `r 1200 112 1200 400 0 ${c.rload}`, tag: 'Rload', posts: [[1200, 112], [1200, 400]], attach: { '1200,112': ['Cout', '1'], '1200,400': ['Cout', '2'] } },
      { line: '207 1120 112 1168 112 0 vout', posts: [[1120, 112]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['QE', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['QE', 'voltage'], { magI: [544, 64, 608, 64], magVtag: 'Q1' }, ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q3', 'both'], ['Rload', 'voltage']],
      rectifier: [['QE', 'both'], ['QG', 'both'], ['Rload', 'voltage']],
      output:    [['Q1', 'voltage'], ['QE', 'voltage'], ['Rload', 'both']],
    },
  },

  // Isolated Dual Active Bridge (DAB): a full bridge on the PRIMARY (leg A = QA/QB, leg C = QC/QD) drives
  // T1 through a series tank inductor Lr and a parallel RC damper (Crc_pri→Rrc_pri, net1↔net2); a full
  // bridge on the SECONDARY (leg E = QE/QF, leg G = QG/QH) is the active/synchronous rectifier into Cout.
  // Primary & secondary share the SAME ground (net8), so one gnd rail spans the whole width. Each FET
  // carries a gate-bias divider Rbias*_hi/lo (bus→mid, mid→gnd — negligible current, geometry only). Body
  // diodes DA-DH and snubber caps Csn* are auto-added (NOT listed comps) — never placed. QA,QC,QE,QG are
  // HIGH-side (floating drives referenced to their own leg-mid); QB,QD,QF,QH are LOW-side (source=gnd,
  // ground-referenced). Each bridge's two legs run ANTI-phase (real ±Vin/±Vsec square-wave across the AC
  // port); the SECONDARY four are π out of phase from the primary so each SR FET conducts on the half-cycle
  // its body diode would (in phase they short the tank and collapse Vout, per the verified cllc note).
  // ctx picks the FIRST magnetic (Lr) as the timescale driver so ctx.n=1; the step-down ratio is read from
  // T1's OWN turnsRatios (like LLC/cllc/psfb). Vout ≈ Vin·Ns/Np (buck-type via the 8.33 ratio) ⇒ POSITIVE
  // ~48 V. No CIAS load resistor ⇒ synthesize Rload. (Falstad wires join ONLY at shared endpoints, so the
  // long net2/net6 return runs may cross a leg or bias lane mid-span without forming a node.)
  dab: {
    place: {
      // ── primary full bridge: leg C (net2) on the left, leg A (net1) on the right (nearer the tank) ──
      QC: { pins: { gate: [320, 160], drain: [384, 144], source: [384, 176] }, line: () => 'f 320 160 384 160 32 1.5 50' },
      QD: { pins: { gate: [320, 272], drain: [384, 256], source: [384, 288] }, line: () => 'f 320 272 384 272 32 1.5 50' },
      QA: { pins: { gate: [528, 160], drain: [592, 144], source: [592, 176] }, line: () => 'f 528 160 592 160 32 1.5 50' },
      QB: { pins: { gate: [528, 272], drain: [592, 256], source: [592, 288] }, line: () => 'f 528 272 592 272 32 1.5 50' },
      // Series tank inductor (net1 → net4) — a plain inductor, so c.L(q) (its own scaled L).
      Lr: { pins: { primary_start: [720, 208], primary_end: [784, 208] }, line: (q, c) => `l 720 208 784 208 0 ${c.L(q)} 0` },
      // Transformer: primary net4(start)→net2(end), secondary net5(start)→net6(end). Ratio from T1's own turnsRatios.
      T1: {
        pins: { primary_start: [784, 208], primary_end: [784, 240], secondary1_start: [848, 208], secondary1_end: [848, 240] },
        line: (q, c) => `T 784 208 848 208 0 ${c.Lm} ${1 / resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)} 0 0 0.999`,
      },
      // Parallel RC damper across the primary AC port: Crc_pri (net1 → net3) then Rrc_pri (net3 → net2).
      Crc_pri: { pins: { 1: [592, 224], 2: [512, 224] }, line: (q, c) => `c 592 224 512 224 0 ${c.C(q)} 0` },
      Rrc_pri: { pins: { 1: [512, 224], 2: [384, 224] }, line: (q, c) => `r 512 224 384 224 0 ${c.R(q)}` },
      // ── secondary full bridge: leg E (net5) then leg G (net6), rectifying into Cout ──
      QE: { pins: { gate: [880, 160], drain: [944, 144], source: [944, 176] }, line: () => 'f 880 160 944 160 32 1.5 50' },
      QF: { pins: { gate: [880, 272], drain: [944, 256], source: [944, 288] }, line: () => 'f 880 272 944 272 32 1.5 50' },
      QG: { pins: { gate: [1072, 160], drain: [1136, 144], source: [1136, 176] }, line: () => 'f 1072 160 1136 160 32 1.5 50' },
      QH: { pins: { gate: [1072, 272], drain: [1136, 256], source: [1136, 288] }, line: () => 'f 1072 272 1136 272 32 1.5 50' },
      Cout: { pins: { 1: [1264, 112], 2: [1264, 400] }, line: (q, c) => `c 1264 112 1264 400 0 ${c.C(q)} ${c.vout}` },
      // ── gate-bias dividers (bus→mid, mid→gnd) — one dedicated x-lane per leg ──
      RbiasA_hi: { pins: { 1: [656, 112], 2: [656, 208] }, line: (q, c) => `r 656 112 656 208 0 ${c.R(q)}` },
      RbiasA_lo: { pins: { 1: [656, 208], 2: [656, 400] }, line: (q, c) => `r 656 208 656 400 0 ${c.R(q)}` },
      RbiasC_hi: { pins: { 1: [448, 112], 2: [448, 208] }, line: (q, c) => `r 448 112 448 208 0 ${c.R(q)}` },
      RbiasC_lo: { pins: { 1: [448, 208], 2: [448, 400] }, line: (q, c) => `r 448 208 448 400 0 ${c.R(q)}` },
      RbiasE_hi: { pins: { 1: [1008, 112], 2: [1008, 208] }, line: (q, c) => `r 1008 112 1008 208 0 ${c.R(q)}` },
      RbiasE_lo: { pins: { 1: [1008, 208], 2: [1008, 400] }, line: (q, c) => `r 1008 208 1008 400 0 ${c.R(q)}` },
      RbiasG_hi: { pins: { 1: [1200, 112], 2: [1200, 208] }, line: (q, c) => `r 1200 112 1200 208 0 ${c.R(q)}` },
      RbiasG_lo: { pins: { 1: [1200, 208], 2: [1200, 400] }, line: (q, c) => `r 1200 208 1200 400 0 ${c.R(q)}` },
    },
    wires: [
      // primary Vin bus (net0): Vin → QC drain → RbiasC_hi → QA drain → RbiasA_hi.
      [176, 112, 384, 112], [384, 112, 448, 112], [448, 112, 592, 112], [592, 112, 656, 112],
      [384, 112, 384, 144], [592, 112, 592, 144],
      // leg C spine (net2): QC source → QD drain (split at 208/224/240 for taps).
      [384, 176, 384, 208], [384, 208, 384, 224], [384, 224, 384, 240], [384, 240, 384, 256],
      [384, 208, 448, 208],                                   // leg C mid → RbiasC divider mid
      [384, 288, 384, 336], [384, 336, 384, 400],             // QD source → gnd (split at 336 for QD drive)
      // leg A spine (net1): QA source → QB drain (split at 208/224).
      [592, 176, 592, 208], [592, 208, 592, 224], [592, 224, 592, 256],
      [592, 208, 656, 208], [656, 208, 720, 208],             // leg A mid → RbiasA mid → Lr primary_start
      [592, 288, 592, 336], [592, 336, 592, 400],             // QB source → gnd (split at 336 for QB drive)
      // net2 return: leg C mid (384,240) → T1 primary_end (784,240) (crosses leg A / lanes at interior points only).
      [384, 240, 784, 240],
      // secondary Vout bus (net7): QE drain → RbiasE_hi → QG drain → RbiasG_hi → Cout → Rload.
      [944, 112, 1008, 112], [1008, 112, 1136, 112], [1136, 112, 1200, 112], [1200, 112, 1264, 112], [1264, 112, 1328, 112],
      [944, 112, 944, 144], [1136, 112, 1136, 144],
      // leg E spine (net5): QE source → QF drain (split at 208).
      [944, 176, 944, 208], [944, 208, 944, 256],
      [848, 208, 944, 208], [944, 208, 1008, 208],            // T1 sec1_start → leg E mid → RbiasE mid
      [944, 288, 944, 336], [944, 336, 944, 400],             // QF source → gnd (split at 336 for QF drive)
      // leg G spine (net6): QG source → QH drain (split at 208/240).
      [1136, 176, 1136, 208], [1136, 208, 1136, 240], [1136, 240, 1136, 256],
      [1136, 208, 1200, 208],                                 // leg G mid → RbiasG mid
      [848, 240, 1136, 240],                                  // T1 sec1_end → leg G mid
      [1136, 288, 1136, 336], [1136, 336, 1136, 400],         // QH source → gnd (split at 336 for QH drive)
      // COMMON gnd rail (net8): spans primary & secondary; taps every leg source and bias-lo pin.
      [176, 400, 384, 400], [384, 400, 448, 400], [448, 400, 592, 400], [592, 400, 656, 400], [656, 400, 944, 400],
      [944, 400, 1008, 400], [1008, 400, 1136, 400], [1136, 400, 1200, 400], [1200, 400, 1264, 400], [1264, 400, 1328, 400],
      // high-side gate-drive references → each leg-mid (floating Vgs); low-side references → gnd rail.
      [320, 208, 384, 208], [528, 208, 592, 208], [880, 208, 944, 208], [1072, 208, 1136, 208],
      [320, 336, 384, 336], [528, 336, 592, 336], [880, 336, 944, 336], [1072, 336, 1136, 336],
    ],
    synth: (c) => [
      { line: `v 176 400 176 112 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 400], [176, 112]], attach: { '176,112': ['QC', 'drain'], '176,400': ['Cout', '2'] } },
      { line: 'g 176 400 176 416 0', posts: [[176, 400]] },
      // primary bridge — legs A and C ANTI-phase (real AC across the tank), phase 0. HS float / LS ground-ref.
      { line: `v 528 208 528 160 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[528, 208], [528, 160]], attach: { '528,160': ['QA', 'gate'], '528,208': ['QA', 'source'] } },
      { line: `v 528 336 528 272 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[528, 336], [528, 272]], attach: { '528,272': ['QB', 'gate'], '528,336': ['QB', 'source'] } },
      { line: `v 320 208 320 160 0 2 ${c.fVis} 5 5 0 ${1 - c.duty}`, posts: [[320, 208], [320, 160]], attach: { '320,160': ['QC', 'gate'], '320,208': ['QC', 'source'] } },
      { line: `v 320 336 320 272 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[320, 336], [320, 272]], attach: { '320,272': ['QD', 'gate'], '320,336': ['QD', 'source'] } },
      // secondary bridge — legs E and G ANTI-phase, whole bridge π-shifted from the primary (synchronous
      // rectification; in phase they short the tank and collapse Vout, per cllc's verified note).
      { line: `v 880 208 880 160 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.duty}`, posts: [[880, 208], [880, 160]], attach: { '880,160': ['QE', 'gate'], '880,208': ['QE', 'source'] } },
      { line: `v 880 336 880 272 0 2 ${c.fVis} 5 5 ${Math.PI} ${1 - c.duty}`, posts: [[880, 336], [880, 272]], attach: { '880,272': ['QF', 'gate'], '880,336': ['QF', 'source'] } },
      { line: `v 1072 208 1072 160 0 2 ${c.fVis} 5 5 ${Math.PI} ${1 - c.duty}`, posts: [[1072, 208], [1072, 160]], attach: { '1072,160': ['QG', 'gate'], '1072,208': ['QG', 'source'] } },
      { line: `v 1072 336 1072 272 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.duty}`, posts: [[1072, 336], [1072, 272]], attach: { '1072,272': ['QH', 'gate'], '1072,336': ['QH', 'source'] } },
      { line: `r 1328 112 1328 400 0 ${c.rload}`, tag: 'Rload', posts: [[1328, 112], [1328, 400]], attach: { '1328,112': ['Cout', '1'], '1328,400': ['Cout', '2'] } },
      { line: '207 1264 112 1312 112 0 vout', posts: [[1264, 112]] },
    ],
    scopeSets: {
      overview:  [['QA', 'voltage'], ['QE', 'voltage'], ['Rload', 'voltage']],
      // T1 reports no device current ⇒ read the primary/tank current from the wire feeding Lr, overlaid with QA's node voltage.
      magnetic:  [['QE', 'voltage'], { magI: [656, 208, 720, 208], magVtag: 'QA' }, ['Rload', 'voltage']],
      switch:    [['QA', 'both'], ['QB', 'both'], ['Rload', 'voltage']],
      rectifier: [['QE', 'both'], ['QG', 'both'], ['Rload', 'voltage']],
      output:    [['QA', 'voltage'], ['QE', 'voltage'], ['Rload', 'both']],
    },
  },

  // Single-switch FORWARD converter — the first topology needing a 3-WINDING transformer, drawn with
  // CircuitJS1's Custom Transformer (dump type 406). T1 has: winding0 = primary (Q1 drives it from Vin),
  // winding1 = reset/demagnetizing winding (on the PRIMARY side, 1:1, returns magnetizing energy to Vin
  // through Ddemag each cycle), winding2 = power secondary (turns ratio 1.86 ⇒ step-down). The 406 line is
  // `406 x1 y1 x2 y1 flags baseInductance couplingCoef "p0,p1:s0" coilCount cur…`, where the description
  // encodes turns per coil (‘,’ separates coils, ‘:’ splits primary-side from secondary-side, sign = dot
  // polarity) and `baseInductance` is the 1-turn inductance (so primary turns=1 ⇒ base = ctx.Lm). The six
  // posts land, in order, at: primary_start (x1,y1) primary_end (x1,y1+32) secondary1_start (x1,y1+48)
  // secondary1_end (x1,y1+80) secondary2_start (x2,y1) secondary2_end (x2,y1+80) — measured live and mapped
  // to the CIAS pin names below. Reset winding wound in phase (+1) so its far end swings above Vin only on
  // the OFF interval (forward-biasing Ddemag → energy back to the bus); secondary in phase (+) so Dfwd
  // conducts during the ON interval. Q1 is HIGH-side (source = primary_start swings) ⇒ floating gate drive
  // referenced to its own source. Verified live: Vout ≈ Vin·D·Ns/Np = 48·0.5·(1/1.86) ≈ +12 V.
  forward: {
    place: {
      Q1: { pins: { gate: [432, 176], drain: [496, 160], source: [496, 192] }, line: () => 'f 432 176 496 176 32 1.5 50' },
      T1: {
        pins: {
          primary_start: [608, 192], primary_end: [608, 224],
          secondary1_start: [608, 240], secondary1_end: [608, 272],
          secondary2_start: [720, 192], secondary2_end: [720, 272],
        },
        // base inductance = ctx.Lm (primary turns = 1); demag turns from ratios[0] (1:1); secondary turns
        // = 1/ratios[1] (ratios[1] = Np/Ns). Secondary is wound in ANTI-phase (negative turns) so that with
        // CircuitJS1's post ordering (start post = non-dot end) Dfwd is forward-biased on the ON interval —
        // verified live: positive secondary → 0 V (Dfwd blocked), negative secondary → +11.8 V.
        line: (q, c) => {
          const demag = resolveDim(q.req.turnsRatios[0], `${q.ref} turnsRatios[0]`)
          const sec = 1 / resolveDim(q.req.turnsRatios[1], `${q.ref} turnsRatios[1]`)
          return `406 608 192 720 192 0 ${c.Lm} 0.999 1,${demag}:${-sec} 3 0 0 0`
        },
      },
      // Reset diode: anode = demag far end (secondary1_end / net2), cathode = Vin bus (net0). Returns the
      // magnetizing energy to the input during the OFF interval.
      Ddemag: { pins: { anode: [544, 272], cathode: [544, 96] }, line: () => 'd 544 272 544 96 2 default' },
      // Forward rectifier: anode = secondary2_start (shared coord, net3), cathode = rectified rail (net4).
      Dfwd: { pins: { anode: [720, 192], cathode: [848, 192] }, line: () => 'd 720 192 848 192 2 default' },
      // Freewheel diode: anode = gnd (net6), cathode = rectified rail (net4).
      Dfw: { pins: { anode: [848, 320], cathode: [848, 192] }, line: () => 'd 848 320 848 192 2 default' },
      Lout: { pins: { primary_start: [848, 192], primary_end: [976, 192] }, line: (q, c) => `l 848 192 976 192 0 ${c.L(q)} 0` },
      Cout: { pins: { 1: [976, 192], 2: [976, 400] }, line: (q, c) => `c 976 192 976 400 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      // net0 (Vin+ bus): source top → Q1 drain, → Ddemag cathode.
      [176, 96, 496, 96], [496, 96, 496, 160], [496, 96, 544, 96],
      // net1: Q1 source → T1 primary_start.
      [496, 192, 608, 192],
      // net2: T1 secondary1_end (demag far end) → Ddemag anode.
      [544, 272, 608, 272],
      // net4: Dfwd cathode / Dfw cathode / Lout start all share (848,192) — Vout rail out at net5.
      [976, 192, 1040, 192],
      // net6 (gnd rail): primary_end + secondary1_start + secondary2_end + Dfw anode + Cout bottom.
      [608, 224, 608, 240],           // join primary_end ↔ secondary1_start (both gnd)
      [608, 240, 608, 400],           // down to the gnd rail
      [720, 272, 720, 400],           // secondary2_end → gnd rail
      [848, 320, 848, 400],           // Dfw anode → gnd rail
      [176, 400, 608, 400], [608, 400, 720, 400], [720, 400, 848, 400], [848, 400, 976, 400], [976, 400, 1040, 400],
    ],
    synth: (c) => [
      { line: `v 176 400 176 96 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 400], [176, 96]], attach: { '176,96': ['Q1', 'drain'], '176,400': ['Cout', '2'] } },
      { line: 'g 176 400 176 416 0', posts: [[176, 400]] },
      // Q1 high-side: floating gate drive referenced to its own source (primary_start node).
      { line: `v 432 240 432 176 0 2 ${c.fVis} 5 5 0 ${c.duty}`, posts: [[432, 240], [432, 176]], attach: { '432,176': ['Q1', 'gate'], '432,240': ['Q1', 'source'] } },
      { line: 'w 432 240 496 240 0', posts: [[432, 240], [496, 240]], wire: true },
      { line: 'w 496 240 496 192 0', posts: [[496, 240], [496, 192]], wire: true },
      { line: `r 1040 192 1040 400 0 ${c.rload}`, tag: 'Rload', posts: [[1040, 192], [1040, 400]], attach: { '1040,192': ['Cout', '1'], '1040,400': ['Cout', '2'] } },
      { line: '207 976 192 1024 192 0 vout', posts: [[976, 192]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['Dfwd', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['Lout', 'both'], ['Dfwd', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Ddemag', 'voltage'], ['Rload', 'voltage']],
      rectifier: [['Dfwd', 'both'], ['Dfw', 'both'], ['Lout', 'current'], ['Rload', 'voltage']],
      output:    [['Lout', 'current'], ['Rload', 'both']],
    },
  },

  push_pull: {
    place: {
      // Both switches are LOW-side (source = gnd), drains up to the two outer primary ends.
      Q1: { pins: { gate: [356, 300], drain: [420, 284], source: [420, 316] }, line: () => 'f 356 300 420 300 32 1.5 50' },
      Q2: { pins: { gate: [228, 332], drain: [292, 316], source: [292, 348] }, line: () => 'f 228 332 292 332 32 1.5 50' },
      T1: {
        pins: {
          primary_start: [512, 160], primary_end: [512, 192],
          secondary1_start: [512, 208], secondary1_end: [512, 240],
          secondary2_start: [640, 160], secondary2_end: [640, 192],
          secondary3_start: [640, 208], secondary3_end: [640, 240],
        },
        // base inductance = ctx.Lm (each primary half = 1 turn); each secondary half = 1/ratios[1]
        // turns (ratios[1] = Np/Ns ⇒ step-down). All four coils wound in phase (+) as a clean start.
        line: (q, c) => {
          const sec = 1 / resolveDim(q.req.turnsRatios[1], `${q.ref} turnsRatios[1]`)
          return `406 512 160 640 160 0 ${c.Lm} 0.999 1,1:${sec},${sec} 4 0 0 0 0`
        },
      },
      // Primary series-RC damper across the two outer primary ends: Rdmp (n1→n3) then Cdmp (n3→n2).
      Rdmp: { pins: { 1: [464, 160], 2: [464, 196] }, line: (q, c) => `r 464 160 464 196 0 ${c.R(q)}` },
      Cdmp: { pins: { 1: [464, 196], 2: [464, 240] }, line: (q, c) => `c 464 196 464 240 0 ${c.C(q)} 0` },
      // Center-tapped full-wave rectifier: anodes at the outer secondary ends (n4/n5), cathodes join
      // the rectified rail (n6) → Lout → Cout.
      Dtop: { pins: { anode: [704, 160], cathode: [816, 160] }, line: () => 'd 704 160 816 160 2 default' },
      Dbot: { pins: { anode: [704, 240], cathode: [816, 240] }, line: () => 'd 704 240 816 240 2 default' },
      Lout: { pins: { primary_start: [816, 200], primary_end: [944, 200] }, line: (q, c) => `l 816 200 944 200 0 ${c.L(q)} 0` },
      Cout: { pins: { 1: [944, 200], 2: [944, 400] }, line: (q, c) => `c 944 200 944 400 0 ${c.C(q)} ${c.vout}` },
    },
    wires: [
      // primary: Vin+ into the center tap (n0); damper + FET drains on the two outer ends.
      [176, 208, 512, 208],                                  // Vin+ -> primary center tap (secondary1_start, n0)
      [512, 192, 512, 208],                                  // join the two center-tap posts (primary_end + secondary1_start, n0)
      [420, 160, 420, 284], [420, 160, 464, 160], [464, 160, 512, 160], // n1: Q1 drain / Rdmp|1 -> primary_start
      [292, 240, 292, 316], [292, 240, 464, 240], [464, 240, 512, 240], // n2: Q2 drain / Cdmp|2 -> secondary1_end
      [420, 316, 420, 400],                                  // Q1 source -> gnd
      [292, 348, 292, 400],                                  // Q2 source -> gnd
      // secondary: center-tapped full-wave rectifier into Lout/Cout.
      [640, 160, 704, 160],                                  // secondary2_start (n4) -> Dtop anode
      [640, 240, 704, 240],                                  // secondary3_end (n5) -> Dbot anode
      [816, 160, 816, 200], [816, 200, 816, 240],            // rectified rail n6 (Dtop cath / Lout start / Dbot cath)
      [944, 200, 1008, 200],                                 // Vout node n7 (Lout end == Cout|1) -> Rload
      [640, 192, 640, 208], [640, 208, 760, 208], [760, 208, 760, 400], // secondary center tap (n8) -> gnd
      // ground rail (n8), split at every tap.
      [176, 400, 228, 400], [228, 400, 292, 400], [292, 400, 356, 400], [356, 400, 420, 400],
      [420, 400, 760, 400], [760, 400, 944, 400], [944, 400, 1008, 400],
    ],
    synth: (c) => [
      // Vin feeds the primary center tap (n0); return is gnd (n8 = Cout|2).
      { line: `v 176 400 176 208 0 0 40 ${c.vin} 0 0 0.5`, posts: [[176, 400], [176, 208]], attach: { '176,208': ['T1', 'secondary1_start'], '176,400': ['Cout', '2'] } },
      { line: 'g 176 400 176 416 0', posts: [[176, 400]] },
      // Q1 / Q2 low-side, ground-referenced gate drives. Push-pull ⇒ 180° apart: Q2 phase = π. (Both in
      // phase energises the two half-primaries together — no alternating flux, not push-pull; anti-phase
      // is the correct, pedagogical waveform. Verified live: anti-phase → +11.4 V, alternating switch nodes.)
      { line: `v 356 400 356 300 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('Q1')}`, posts: [[356, 400], [356, 300]], attach: { '356,300': ['Q1', 'gate'], '356,400': ['Q1', 'source'] } },
      { line: `v 228 400 228 332 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.dutyOf('Q2')}`, posts: [[228, 400], [228, 332]], attach: { '228,332': ['Q2', 'gate'], '228,400': ['Q2', 'source'] } },
      { line: `r 1008 200 1008 400 0 ${c.rload}`, tag: 'Rload', posts: [[1008, 200], [1008, 400]], attach: { '1008,200': ['Cout', '1'], '1008,400': ['Cout', '2'] } },
      { line: '207 944 200 992 200 0 vout', posts: [[944, 200]] },
    ],
    scopeSets: {
      overview:  [['Q1', 'voltage'], ['Dtop', 'voltage'], ['Rload', 'voltage']],
      magnetic:  [['Lout', 'both'], ['Dtop', 'voltage'], ['Rload', 'voltage']],
      switch:    [['Q1', 'both'], ['Q2', 'both'], ['Rload', 'voltage']],
      rectifier: [['Dtop', 'both'], ['Dbot', 'both'], ['Lout', 'current'], ['Rload', 'voltage']],
      output:    [['Lout', 'current'], ['Rload', 'both']],
    },
  },

  weinberg: {
    place: {
      L1: {
        pins: {
          primary_start: [384, 176], primary_end: [384, 208],
          secondary1_start: [464, 176], secondary1_end: [464, 208],
        },
        line: (q, c) => `406 384 176 464 176 0 ${c.L(q)} 0.999 1:1 2 0 0`,
      },
      T1: {
        pins: {
          primary_start: [640, 176], primary_end: [640, 208],
          secondary1_start: [640, 224], secondary1_end: [640, 256],
          secondary2_start: [768, 176], secondary2_end: [768, 208],
          secondary3_start: [768, 224], secondary3_end: [768, 256],
        },
        // base inductance = ctx.Lm (primary turns = 1); the two secondary coils have turns sec =
        // 1/ratios[1] (ratios[1] = Np/Ns). Primary halves are 1,1 (ratios[0] = 1). All-positive to start.
        line: (q, c) => {
          const sec = 1 / resolveDim(q.req.turnsRatios[1], `${q.ref} turnsRatios[1]`)
          return `406 640 176 768 176 0 ${c.Lm} 0.999 1,1:${sec},${sec} 4 0 0 0 0`
        },
      },
      // DC-resistance sense resistors from each L1 winding far end to a T1 primary center tap.
      Rdcra: { pins: { 1: [496, 208], 2: [592, 208] }, line: (q, c) => `r 496 208 592 208 0 ${c.R(q)}` },
      Rdcrb: { pins: { 1: [496, 272], 2: [592, 272] }, line: (q, c) => `r 496 272 592 272 0 ${c.R(q)}` },
      // Full-wave secondary rectifier: each diode anode shares its T1 secondary outer-end coord.
      Dpos: { pins: { anode: [768, 208], cathode: [896, 208] }, line: () => 'd 768 208 896 208 2 default' },
      Dneg: { pins: { anode: [768, 224], cathode: [896, 224] }, line: () => 'd 768 224 896 224 2 default' },
      Cout: { pins: { 1: [960, 208], 2: [960, 400] }, line: (q, c) => `c 960 208 960 400 0 ${c.C(q)} ${c.vout}` },
      // Low-side push-pull switches: drain up to a T1 primary outer end, source down to the GND rail.
      S1: { pins: { gate: [336, 288], drain: [400, 272], source: [400, 304] }, line: () => 'f 336 288 400 288 32 1.5 50' },
      S2: { pins: { gate: [704, 336], drain: [640, 320], source: [640, 352] }, line: () => 'f 704 336 640 336 32 1.5 50' },
    },
    wires: [
      // n0 (Vin tap): L1 primary_start ↔ secondary1_start, fed by Vin at (256,176).
      [256, 176, 384, 176], [384, 176, 464, 176],
      // n1: L1 primary_end → Rdcra|1 (detour below y=208 to skip L1 secondary1_end's post).
      [384, 208, 384, 240], [384, 240, 496, 240], [496, 240, 496, 208],
      // n7: Rdcra|2 → T1 primary_end.
      [592, 208, 640, 208],
      // n2: L1 secondary1_end → Rdcrb|1.
      [464, 208, 464, 272], [464, 272, 496, 272],
      // n8: Rdcrb|2 → T1 secondary1_start.
      [592, 272, 592, 224], [592, 224, 640, 224],
      // n9: T1 primary_start (top) routed up-and-over to S1 drain.
      [640, 176, 640, 120], [640, 120, 400, 120], [400, 120, 400, 272],
      // n10: T1 secondary1_end (bottom) → S2 drain.
      [640, 256, 640, 320],
      // n15 (Vout): Dpos|cathode ↔ Dneg|cathode ↔ Cout|1 ↔ Rload.
      [896, 208, 896, 224], [896, 208, 960, 208], [960, 208, 1088, 208],
      // n16 (GND): switch sources, secondary center tap (both T1 GND ends), down to the GND rail.
      [400, 304, 400, 400],           // S1 source → rail
      [640, 352, 640, 400],           // S2 source → rail
      [768, 256, 768, 400],           // T1 secondary3_end (center tap) → rail
      [768, 176, 816, 176], [816, 176, 816, 400], // T1 secondary2_start (center tap) around to rail
      // GND rail (endpoints at every tap x so each lands on a node).
      [256, 400, 336, 400], [336, 400, 400, 400], [400, 400, 640, 400], [640, 400, 704, 400],
      [704, 400, 768, 400], [768, 400, 816, 400], [816, 400, 960, 400], [960, 400, 1088, 400],
    ],
    synth: (c) => [
      { line: `v 256 400 256 176 0 0 40 ${c.vin} 0 0 0.5`, posts: [[256, 400], [256, 176]], attach: { '256,176': ['L1', 'primary_start'], '256,400': ['Cout', '2'] } },
      { line: 'g 256 400 256 416 0', posts: [[256, 400]] },
      // Low-side push-pull gate drives, ground-referenced, 180° apart (S2 phase = π).
      { line: `v 336 400 336 288 0 2 ${c.fVis} 5 5 0 ${c.dutyOf('S1')}`, posts: [[336, 400], [336, 288]], attach: { '336,288': ['S1', 'gate'], '336,400': ['Cout', '2'] } },
      { line: `v 704 400 704 336 0 2 ${c.fVis} 5 5 ${Math.PI} ${c.dutyOf('S2')}`, posts: [[704, 400], [704, 336]], attach: { '704,336': ['S2', 'gate'], '704,400': ['Cout', '2'] } },
      { line: `r 1088 208 1088 400 0 ${c.rload}`, tag: 'Rload', posts: [[1088, 208], [1088, 400]], attach: { '1088,208': ['Cout', '1'], '1088,400': ['Cout', '2'] } },
      { line: '207 960 208 1008 208 0 vout', posts: [[960, 208]] },
    ],
    scopeSets: {
      overview:  [['S1', 'voltage'], ['Dpos', 'voltage'], ['Rload', 'voltage']],
      // Both magnetics are 406 custom transformers (no device current) ⇒ read L1's input current from the
      // Vin-tap wire in series with it, overlaid with S1's switch-node voltage.
      magnetic:  [{ magI: [256, 176, 384, 176], magVtag: 'S1' }, ['Dpos', 'voltage'], ['Rload', 'voltage']],
      switch:    [['S1', 'both'], ['S2', 'both'], ['Rload', 'voltage']],
      rectifier: [['Dpos', 'both'], ['Dneg', 'both'], ['Rload', 'voltage']],
      output:    [['S1', 'voltage'], ['Rload', 'both']],
    },
  },
}

export function hasVisualSim(topoId) { return topoId in LAYOUTS }

// Build the circuitjs1 circuit text + URL for a solved TAS. Throws (loudly, with the reason) on
// anything the layout cannot faithfully represent — never draws an approximation silently.
export function falstadExport(topoId, tas, scopeSet = 'overview') {
  const layout = LAYOUTS[topoId]
  if (!layout) throw new Error(`no visual-sim layout for topology '${topoId}' yet`)

  const dr = tas?.inputs?.designRequirements
  if (!dr) throw new Error('TAS has no inputs.designRequirements')
  const vin = resolveDim(dr.inputVoltage, 'inputVoltage')
  const outs = dr.outputs ?? []
  if (outs.length !== 1) throw new Error(`visual sim supports a single output for now (design has ${outs.length})`)
  const vout = resolveDim(outs[0].voltage, 'output voltage')
  const pout = tas.inputs?.operatingPoints?.[0]?.outputs?.[0]?.power
  if (typeof pout !== 'number') throw new Error('TAS has no operatingPoints[0].outputs[0].power')
  const stim = (tas.simulation?.stimulus ?? [])[0]
  if (stim?.waveform?.type !== 'pwm') throw new Error('TAS has no pwm stimulus (gate drive unknown)')
  const fsw = stim.waveform.frequency
  const duty = stim.waveform.dutyCycle
  if (typeof fsw !== 'number' || typeof duty !== 'number') throw new Error('pwm stimulus lacks frequency/dutyCycle')
  // Per-switch duty map: a multi-switch bridge drives each FET off its OWN stimulus duty, not stim[0]'s.
  const dutyByComp = new Map()
  for (const s of tas.simulation?.stimulus ?? []) {
    if (s?.waveform?.type === 'pwm' && typeof s.waveform.dutyCycle === 'number') dutyByComp.set(s.component, s.waveform.dutyCycle)
  }

  // CIAS components (the same set the ngspice deck instantiates)
  const comps = new Map(ciasComponents(tas).map((x) => [x.ref, x]))
  // The main magnetic drives the time-scale: a transformer (turnsRatios present) or a plain inductor.
  const mag = [...comps.values()].find((c) => c.data?.magnetic !== undefined)
  if (!mag) throw new Error('TAS has no magnetic component')
  const ratios = mag.req.turnsRatios ?? []
  const n = ratios.length ? resolveDim(ratios[0], `${mag.ref} turnsRatios[0]`) : 1 // Np/Ns; falstad ratio is Ns/Np
  const Lreq = mag.req.magnetizingInductance ?? mag.req.inductance
  if (Lreq === undefined) throw new Error(`${mag.ref} has no magnetizingInductance/inductance`)
  const k = fsw / VISUAL_HZ // similarity transform: fsw/k, L*k, C*k -> identical waveforms, stretched in time

  const ctx = {
    vin, vout, n, duty, fVis: VISUAL_HZ, rload: vout * vout / pout,
    Lm: resolveDim(Lreq, `${mag.ref} inductance`) * k,
    C: (q) => resolveDim(q.req.capacitance, `${q.ref} capacitance`) * k,
    R: (q) => resolveDim(q.req.resistance, `${q.ref} resistance`),
    // A specific magnetic's own (time-scaled) inductance — for topologies with a SECOND inductor
    // (an output choke, a coupled winding) that shouldn't reuse the main magnetic's Lm.
    L: (q) => resolveDim(q.req.magnetizingInductance ?? q.req.inductance, `${q.ref} inductance`) * k,
    // A specific switch's own PWM duty (from tas.simulation.stimulus); falls back to stim[0]'s duty.
    dutyOf: (ref) => (dutyByComp.has(ref) ? dutyByComp.get(ref) : duty),
  }

  // ── emit elements: every CIAS power component must have a placement (loud gap otherwise) ──────
  const elements = [] // { line, tag?, posts: [[x,y],...] }
  const pinCoord = new Map() // "ref|pin" -> "x,y"
  for (const [ref, q] of comps) {
    const p = layout.place[ref]
    if (!p) throw new Error(`no placement for component '${ref}' in the ${topoId} visual layout`)
    for (const [pin, xy] of Object.entries(p.pins)) pinCoord.set(`${ref}|${pin}`, String(xy))
    elements.push({ line: p.line(q, ctx), tag: ref, posts: Object.values(p.pins) })
    for (const w of p.wires ?? []) elements.push({ line: `w ${w.join(' ')} 0`, posts: [[w[0], w[1]], [w[2], w[3]]], wire: true })
  }
  for (const [ref, p] of Object.entries(layout.place)) {
    if (!comps.has(ref) && !p.optional) throw new Error(`layout places '${ref}' but the TAS has no such component`)
  }
  // Mutually-exclusive optional groups (e.g. freewheel = diode D1 XOR synchronous FET Q2): assert
  // EXACTLY one is present, so a malformed design with neither (no freewheel path) or both (two
  // devices in parallel) throws loudly rather than silently exporting a broken circuit.
  for (const group of layout.oneOf ?? []) {
    const n = group.filter((r) => comps.has(r)).length
    if (n !== 1) throw new Error(`${topoId} visual layout expects exactly one of {${group.join(', ')}}, found ${n}`)
  }
  // Wires and synth glyphs may be gated on component presence via `needs` (variant-specific parts —
  // an SR FET, a clamp, a resonant cap — draw only when the design actually contains them).
  const present = (needs) => (needs ?? []).every((r) => comps.has(r))
  for (const w of layout.wires) {
    const pts = Array.isArray(w) ? w : w.pts
    if (!Array.isArray(w) && !present(w.needs)) continue
    elements.push({ line: `w ${pts.join(' ')} 0`, posts: [[pts[0], pts[1]], [pts[2], pts[3]]], wire: true })
  }
  const attachChecks = [] // [coordKey, "ref|pin"] — synthesized posts that must land on a CIAS net
  for (const s of layout.synth(ctx)) {
    if (!present(s.needs)) continue
    elements.push(s)
    for (const [coord, [ref, pin]] of Object.entries(s.attach ?? {})) attachChecks.push([coord, `${ref}|${pin}`])
  }

  // ── verify the drawn wiring against the flattened CIAS nets (same nets ngspice simulates) ─────
  const par = new Map()
  const find = (x) => { if (!par.has(x)) par.set(x, x); while (par.get(x) !== x) { par.set(x, par.get(par.get(x))); x = par.get(x) } return x }
  for (const e of elements) {
    for (const p of e.posts) find(String(p))
    if (e.wire) par.set(find(String(e.posts[0])), find(String(e.posts[1]))) // only wires (and shared coords) join nodes
  }
  const pinNet = flattenNets(tas)
  const netRoot = new Map() // CIAS net id -> falstad node root
  const rootNet = new Map() // falstad node root -> CIAS net id (short check)
  for (const [key, coord] of pinCoord) {
    // Gate pins are CONTROL signals, not power nets — excluded from the connectivity check (as in the
    // SVG checker). A CIAS gate net is often shared across a high-side + low-side switch that physically
    // need SEPARATE drives (floating vs ground); enforcing it would false-flag a "split net".
    if (key.endsWith('|gate')) continue
    const net = pinNet.get(key)
    if (net === undefined) throw new Error(`CIAS has no net for pin ${key}`)
    const root = find(coord)
    if (netRoot.has(net) && netRoot.get(net) !== root) throw new Error(`net '${net}' is split in the drawing at ${key}`)
    if (rootNet.has(root) && rootNet.get(root) !== net) throw new Error(`drawing shorts nets '${rootNet.get(root)}' and '${net}' at ${key}`)
    netRoot.set(net, root); rootNet.set(root, net)
  }
  for (const [coord, key] of attachChecks) {
    if (key.endsWith('|gate')) continue // gate drives aren't connectivity-verified (control signal)
    if (find(coord) !== netRoot.get(pinNet.get(key))) throw new Error(`synthesized element at (${coord}) missed the net of ${key}`)
  }

  // ── assemble: header, elements, scopes ─────────────────────────────────────────────────────────
  const vrange = Math.ceil(Math.max(5, vin, vout, vin + n * vout))
  const ts = 1 / (VISUAL_HZ * 500)
  const lines = [`$ 1 ${ts} 1.7 50 ${vrange} 50`, ...elements.map((e) => e.line)]
  // Scopes: the chosen scope set. Entries are either
  //   ['tag', 'voltage'|'current'|'both']  — plot an element (by tag), or
  //   { magI:[x1,y1,x2,y2], magVtag:'tag' } — the magnetic panel: CURRENT read from the wire in series
  //     with the winding (CircuitJS1 transformers report no device current) OVERLAID with the voltage of
  //     a reference element (the switch node). Skip any whose target isn't in this variant.
  // CircuitJS1 `o` format: o <elm> <speed> <value> <flags> <scaleV> <scaleA> <position> <plotCount> [<elm> <val> …]
  //   value: 0=voltage 3=current;  flags: 4096 FLAG_PLOTS | 8192 maxScale | (1 showI | 2 showV)
  const F = 4096 | 8192
  const wireIdx = (pts) => elements.findIndex((e) => e.wire && e.line === `w ${pts.join(' ')} 0`)
  const scopeList = layout.scopeSets?.[scopeSet] ?? layout.scopeSets?.overview ?? layout.scopes ?? []
  let slot = 0
  for (const sc of scopeList) {
    if (Array.isArray(sc)) {
      const [tag, kind] = sc
      const idx = elements.findIndex((e) => e.tag === tag)
      if (idx < 0) continue
      if (kind === 'both') lines.push(`o ${idx} 64 0 ${F | 1 | 2} ${vrange} 1 ${slot++} 2 ${idx} 3`)
      else { const isI = kind === 'current'; lines.push(`o ${idx} 64 ${isI ? 3 : 0} ${F | (isI ? 1 : 2)} ${vrange} 1 ${slot++} 1`) }
    } else if (sc.magI) {
      const wi = wireIdx(sc.magI)
      const vi = elements.findIndex((e) => e.tag === sc.magVtag)
      if (wi < 0 || vi < 0) continue
      lines.push(`o ${wi} 64 3 ${F | 1 | 2} ${vrange} 1 ${slot++} 2 ${vi} 0`) // primary current + node voltage
    }
  }
  const text = lines.join('\n')
  return {
    text,
    url: `${CIRCUITJS_BASE}?${khColorQuery}&cct=${encodeURIComponent(text)}`,
    fsw, fVis: VISUAL_HZ, scale: k, vin, vout,
  }
}
