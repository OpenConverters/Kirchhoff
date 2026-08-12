// Pure geometry rules for the schematic legibility audit — NO engine, NO browser, no imports.
//
// Kept separate from auditSchematicLabels.mjs so the SAME rules can be applied to the LIVE app DOM
// (tests/e2e/schematic.spec.js) as to the Node-built SVG. That script initialises the WASM engine at
// module load, which a Playwright spec cannot import; sharing the rules any other way would mean two
// copies of them, which is exactly how a rule fix reaches one path and not the other.
const TOL = 2

const over = (a, b) => Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x) > TOL &&
                       Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y) > TOL
// Does an axis-aligned WIRE run through a text box? This cannot be asked with over(): a wire has zero
// thickness on one axis, so demanding >TOL overlap on BOTH axes is unsatisfiable — T-WIRE was
// structurally incapable of firing for any wire in any schematic, and a rail printed straight through
// DAB's "Crc_pri" label passed as clean. Correct test: the wire's fixed coordinate lies strictly inside
// the box (with a margin, since a glyph box includes ascender/descender padding) and it spans the box.
const crossesText = (t, [p, q]) => {
  const [x0, x1] = [Math.min(p[0], q[0]), Math.max(p[0], q[0])]
  const [y0, y1] = [Math.min(p[1], q[1]), Math.max(p[1], q[1])]
  if (p[1] === q[1]) return p[1] > t.y + TOL && p[1] < t.y + t.h - TOL &&
                            Math.min(t.x + t.w, x1) - Math.max(t.x, x0) > TOL
  if (p[0] === q[0]) return p[0] > t.x + TOL && p[0] < t.x + t.w - TOL &&
                            Math.min(t.y + t.h, y1) - Math.max(t.y, y0) > TOL
  return false
}
const endpointAt = (wires, d) => wires.some((s) => s.some((e) => Math.abs(e[0] - d[0]) < 3 && Math.abs(e[1] - d[1]) < 3))
const onSpan = (p, [a, b]) => (a[0] === b[0] && Math.abs(p[0] - a[0]) <= 2 && p[1] > Math.min(a[1], b[1]) + 2 && p[1] < Math.max(a[1], b[1]) - 2) ||
                              (a[1] === b[1] && Math.abs(p[1] - a[1]) <= 2 && p[0] > Math.min(a[0], b[0]) + 2 && p[0] < Math.max(a[0], b[0]) - 2)

// One rendered schematic's measured geometry -> string[] problems. Exported so checkSweep.mjs can run
// the same rules over many design points without duplicating them.
export function auditLabels({ texts, boxes, glyphs = [], drawn = [], wires, dots, W, H }, gates) {
  const ownsFlag = (tx, b) => tx.str.length <= 3 && gates.some((q) => q.ref === b.ref &&
    Math.hypot(Math.max(0, Math.max(tx.x - q.x, q.x - (tx.x + tx.w))),
               Math.max(0, Math.max(tx.y - q.y, q.y - (tx.y + tx.h)))) < 25)
  const p = []
  for (const tx of texts) {
    const hit = wires.find((s) => crossesText(tx, s))
    if (hit) p.push(`T-WIRE "${tx.str}" [x ${tx.x.toFixed(1)}..${(tx.x + tx.w).toFixed(1)}, y ${tx.y.toFixed(1)}..${(tx.y + tx.h).toFixed(1)}] lies across wire ${JSON.stringify(hit)}`)
  }
  for (let i = 0; i < texts.length; i++) for (let j = i + 1; j < texts.length; j++) {
    // A component's ref and its value are ONE two-line label block, drawn 12 px apart with ~14 px em
    // boxes — so they always overlap by ~2 px of ascender/descender padding, no ink. That is not a
    // collision, and at TOL=2 it sat exactly on the knife edge: under it in the Node harness, over it
    // in the live app, which is what made every label block in the product flip to "colliding".
    if (texts[i].ref && texts[i].ref === texts[j].ref) continue
    if (over(texts[i], texts[j])) p.push(`T-TEXT "${texts[i].str}" ∩ "${texts[j].str}"`)
  }
  for (const tx of texts) for (const b of boxes)
    if (tx.ref !== b.ref && !ownsFlag(tx, b) && over(tx, b)) p.push(`T-SYM "${tx.str}" [x ${tx.x.toFixed(1)}..${(tx.x + tx.w).toFixed(1)}] over ${b.ref}'s footprint [x ${b.x}..${b.x + b.w}]`)
  for (let i = 0; i < boxes.length; i++) for (let j = i + 1; j < boxes.length; j++)
    if (over(boxes[i], boxes[j])) p.push(`SYM-SYM ${boxes[i].ref} ∩ ${boxes[j].ref}`)
  // T-GLYPH: a label over a source / ground / port / load symbol. These are not BOM parts, so they
  // have no hitbox and T-SYM never saw them — pfc printed D1's VF value straight across the VAC source.
  for (const tx of texts) for (const g of glyphs)
    if (tx.str !== g.owner && over(tx, g)) p.push(`T-GLYPH "${tx.str}" over the ${g.owner || 'ground/return'} symbol`)
  // OWN-BODY: a part's ref/value printed across the part's OWN glyph. Every other rule exempts a part's
  // own labels — deliberately, since a gate flag belongs on its switch — and that exemption hid DAB's
  // RbiasC_hi/RbiasC_lo, whose names and values were struck straight through their own resistor
  // zigzags. Measured against the symbol's real ink (drawn), not the padded hitbox: the label sits
  // beside the glyph by design, so any real overlap is the label landing on the part it names. Gate
  // flags (.sch-sig) are drawn AT the gate pin on purpose. A label drawn wholly INSIDE its own symbol
  // is a caption, not a collision — that is how a control block carries its "PWM"/"CTRL"/"Σ" — so the
  // rule asks for an overlap that BREAKS OUT of the glyph, which is what a mispositioned label does.
  const inside = (a, b) => a.x >= b.x && a.y >= b.y && a.x + a.w <= b.x + b.w && a.y + a.h <= b.y + b.h
  for (const tx of texts) {
    if (!tx.ref || tx.cls === 'sch-sig') continue
    const body = drawn.find((b) => b.ref === tx.ref)
    if (body && over(tx, body) && !inside(tx, body)) p.push(`OWN-BODY "${tx.str}" is printed over ${tx.ref}'s own symbol`)
  }
  for (const d of dots)
    if (!endpointAt(wires, d) && wires.filter((s) => onSpan(d, s)).length >= 2) p.push(`FALSE-DOT at (${d}) marks a pure crossing`)
  const seen = new Set()
  for (const b of boxes) { if (seen.has(b.ref)) p.push(`DUP-REF ${b.ref} drawn twice`); seen.add(b.ref) }
  for (const tx of texts)
    if (tx.x < -TOL || tx.y < -TOL || tx.x + tx.w > W + TOL || tx.y + tx.h > H + TOL) p.push(`OOB "${tx.str}" outside ${W}x${H}`)
  return [...new Set(p)]
}
