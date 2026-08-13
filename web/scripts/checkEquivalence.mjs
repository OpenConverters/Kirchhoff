// CIAS ≡ SCHEMATIC ≡ SIMULATION — the equivalence gate (run: `node scripts/checkEquivalence.mjs`).
//
// The other checkers each police ONE view against the CIAS. Nothing asserted that the three views
// describe the SAME converter, so a component could exist in the netlist, be drawn, and quietly not be
// simulated (or the reverse) with every other check still green. For every topology × variant this
// asserts, and fails the run on any breach:
//
//   1 DRAWN     every power + controller component of the CIAS is drawn in the schematic
//   2 INVENTED  nothing is drawn that is not a real TAS component (numerical aids must never be drawn)
//   3 VISUAL    the visual-sim export instantiates every power component, except those the layout
//               EXPLICITLY declares consumed by its ideal `drive` — that list is printed, never implied
//   4 DECK      every power component appears as an instance in the generated ngspice deck
//   5 RUNS      the in-process libngspice transient actually runs (success, points > 0)
//   6 VOUT      the simulated output settles within tolerance of the design's own Vout
//
// Rules 4-6 are what make this more than a drawing check: the deck and the transient come from the same
// TAS the schematic was drawn from, so a divergence anywhere shows up here.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
// Renders through the SAME entry point the app uses (renderForAudit). Every topology is generated from
// CIAS now, but the rule stands: measure what the product draws, never a reconstruction of it.
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { extractBom } from '../src/bom.js'
import { falstadExport, hasVisualSim } from '../src/falstad.js'
import { resolveDim } from '../src/cias.js'
const M = await init()

const VOUT_TOL = 0.15          // ±15% of design Vout on the ideal deck
const only = process.argv[2]

// Classify every component the TAS carries — the BOM/CIAS helpers filter these out, but equivalence
// needs to know which bucket each ref is in.
function classify(tas) {
  const power = [], aid = [], body = [], ctrl = []
  for (const st of tas?.topology?.stages ?? []) for (const c of st.circuit?.components ?? []) {
    const req = c.data?.inputs?.designRequirements ?? {}
    if (req.role === 'bodyDiode') body.push(c.name)
    else if (req.name === '__kh_numerical_aid__') aid.push(c.name)
    else if (c.data?.controller !== undefined) ctrl.push(c.name)
    else power.push(c.name)
  }
  return { power, aid, body, ctrl, all: [...power, ...aid, ...body, ...ctrl] }
}

const deckInstances = (deck) => deck.split('\n')
  .filter((l) => /^[A-Za-z]/.test(l))
  .map((l) => l.split(/\s+/)[0])

let failed = 0, checked = 0
for (const t of TOPOLOGIES) {
  if (only && t.id !== only) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const name = `${t.id}${opt ? '/' + opt : ''}`
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) { console.error(`${name}: design failed: ${out.slice(0, 120)}`); failed++; continue }
    const tas = JSON.parse(out).tas
    const cls = classify(tas)
    const problems = [], notes = []

    // ── 1/2: the schematic ────────────────────────────────────────────────────────────────────
    if (hasCiasSchematic(t.id)) {
      const { svg } = renderForAudit(t.id, tas, opt ?? 'standard')
      const drawn = new Set([...svg.matchAll(/data-ref="([^"]+)"/g)].map((m) => m[1]))
      for (const ref of [...cls.power, ...cls.ctrl]) if (!drawn.has(ref)) problems.push(`DRAWN: ${ref} is in the CIAS but not in the schematic`)
      for (const ref of drawn) if (!cls.all.includes(ref)) problems.push(`INVENTED: ${ref} is drawn but is not a TAS component`)
      for (const ref of cls.aid) if (drawn.has(ref)) problems.push(`INVENTED: numerical aid ${ref} must never be drawn`)
      const bodyDrawn = cls.body.filter((r) => drawn.has(r))
      if (bodyDrawn.length) notes.push(`body-diode annotations drawn: ${bodyDrawn.join(',')}`)
    } else notes.push('no schematic')

    // ── 3: the visual sim ─────────────────────────────────────────────────────────────────────
    if (hasVisualSim(t.id)) {
      try {
        const e = falstadExport(t.id, tas)
        const seen = new Set([...e.components.placed, ...e.components.consumed])
        for (const ref of cls.power) if (!seen.has(ref)) problems.push(`VISUAL: ${ref} is in the CIAS but the visual sim neither draws nor declares it`)
        for (const ref of e.components.placed) if (!cls.power.includes(ref)) problems.push(`VISUAL: ${ref} is drawn in the visual sim but is not a CIAS power component`)
        if (e.components.consumed.length) notes.push(`visual sim replaces with an ideal drive: ${e.components.consumed.join(',')}`)
      } catch (err) {
        if (/no placement for component|expects exactly one of/.test(err.message)) notes.push(`variant not laid out (${err.message.slice(0, 60)})`)
        else problems.push(`VISUAL: export failed: ${err.message}`)
      }
    } else notes.push('no visual sim')

    // ── 4/5/6: the real ngspice deck + transient ───────────────────────────────────────────────
    const fidelity = JSON.stringify({ origin: 'REQUIREMENTS' })
    const deck = M.generate_ngspice_circuit(JSON.stringify(tas), fidelity)
    if (deck.startsWith('Exception')) problems.push(`DECK: generation failed: ${deck.slice(0, 100)}`)
    else {
      const inst = deckInstances(deck)
      for (const ref of cls.power)
        if (!inst.some((i) => new RegExp(`^[A-Za-z]${ref}(_|$)`).test(i))) problems.push(`DECK: ${ref} is in the CIAS but has no instance in the ngspice deck`)
    }
    const simRaw = M.simulate_ngspice(JSON.stringify(tas), fidelity)
    if (simRaw.startsWith('Exception')) problems.push(`RUNS: simulate_ngspice threw: ${simRaw.slice(0, 100)}`)
    else {
      const sim = JSON.parse(simRaw)
      if (!sim.success) problems.push(`RUNS: transient failed: ${sim.error || '(no reason given)'}`)
      else if (!(sim.points > 0)) problems.push('RUNS: transient produced no points')
      else {
        const design = resolveDim(tas.inputs.designRequirements.outputs[0].voltage, 'design Vout')
        // Measure ACROSS THE LOAD, whatever the deck calls its rails: a single-ended converter gives
        // v(vout)-0, a differential bus (vienna: RRload busP busN) gives v(busP)-v(busN). Keying on a
        // node literally named 'vout' left the three-phase decks unverifiable.
        const load = deck.split('\n').map((l) => l.split(/\s+/)).find((tk) => /^R?Rload$/i.test(tk[0]))
        const nodeV = (n) => (n === '0' ? 0 : sim.vectors?.[n.toLowerCase()]?.average)
        if (!load) problems.push('VOUT: the deck has no Rload to measure across')
        else if (nodeV(load[1]) === undefined || nodeV(load[2]) === undefined)
          problems.push(`VOUT: the transient exposes no vectors for the load nodes ${load[1]}/${load[2]}`)
        else {
          const got = nodeV(load[1]) - nodeV(load[2])
          if (Math.sign(got) !== Math.sign(design))
            problems.push(`VOUT SIGN: simulated ${got.toFixed(2)} V but the design declares ${design} V — the requirement's polarity is opposite to the circuit it generates`)
          else {
            const err = Math.abs(Math.abs(got) - Math.abs(design)) / Math.abs(design)
            if (err > VOUT_TOL) problems.push(`VOUT: simulated ${got.toFixed(3)} V vs design ${design} V (${(err * 100).toFixed(0)}% off)`)
            else notes.push(`Vout ${got.toFixed(2)}/${design} V`)
          }
        }
      }
    }

    checked++
    if (problems.length) { failed++; console.log(`\n✗ ${name}`); for (const p of problems) console.log('    ' + p); if (notes.length) console.log('    · ' + notes.join(' · ')) }
    else console.log(`✓ ${name.padEnd(26)} ${notes.join(' · ')}`)
  }
}
console.log(failed ? `\n${failed} of ${checked} topology/variant combos BREACH CIAS ≡ schematic ≡ simulation` : `\nAll ${checked} combos: CIAS ≡ schematic ≡ simulation`)
process.exit(failed ? 1 : 0)
