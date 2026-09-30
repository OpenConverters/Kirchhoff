// EMI line filter: the CIAS brick (schema-valid in every state), its verified schematic, and the ngspice
// AC decks. The end-to-end insertion-loss run through the real WASM engine is scripts/checkEmiFilterAc.mjs
// (it needs the built engine; this file does not).
//
// Schemas: KH_SCHEMA_ROOT, colon-separated roots whose children are schema packages with a schemas/ dir
// (the first root to define an $id wins). Default: the sibling PSMA checkout (../../PSMA from the repo
// root), i.e. the latest schemas of the whole family. Not deps/ alone: PEAS $refs every module, TDAS
// included, which Kirchhoff does not vendor, and mixing its pinned PEAS with newer module schemas leaves
// dangling refs. A missing root is a hard FAIL, never a skip.
import { test } from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync, readdirSync, existsSync } from 'node:fs'
import { join, resolve } from 'node:path'
import Ajv2020 from 'ajv/dist/2020.js'
import {
  buildEmiFilterCias, parseEmiFilterCias, emiFilterComponents, emiFilterNets, emiFilterBom,
  buildEmiFilterAcDeck, buildLisnReferenceDeck, insertionLossDb, capacitorSubckt,
  CISPR16_LISN, IDEAL_NOISE_SOURCE, EMI_PROBES, MKF_CMC_PIN_MAP,
} from '../../src/emiFilter.js'
import {
  renderVerifiedEmiFilterSchematic, renderEmiFilterSchematicWithPins, hasCiasSchematic, EMI_FILTER_SCHEMATIC,
} from '../../src/ciasSchematic.js'
import { checkSchematic, wireGraph } from '../../src/schematicCheck.js'

const roots = (process.env.KH_SCHEMA_ROOT ?? resolve(import.meta.dirname, '../../../../../PSMA')).split(':')
const ajv = new Ajv2020({ strict: false, allErrors: true, logger: false })
const seen = new Set()
function loadDir(dir) {
  for (const e of readdirSync(dir, { withFileTypes: true })) {
    const p = join(dir, e.name)
    if (e.isDirectory()) loadDir(p)
    else if (e.name.endsWith('.json')) {
      const schema = JSON.parse(readFileSync(p, 'utf8'))
      if (schema.$id && !seen.has(schema.$id)) { seen.add(schema.$id); ajv.addSchema(schema) }
    }
  }
}
for (const root of roots) {
  assert.ok(existsSync(root), `schema root not found: ${root} — set KH_SCHEMA_ROOT`)
  for (const pkg of readdirSync(root)) if (existsSync(join(root, pkg, 'schemas'))) loadDir(join(root, pkg, 'schemas'))
}
const validateCias = ajv.getSchema('https://psma.com/cias/CIAS.json')
assert.ok(validateCias, 'CIAS schema not found under the schema roots (git submodule update --init deps/CIAS deps/PEAS)')
const assertValid = (brick, what) => assert.ok(validateCias(brick), `${what}: ${ajv.errorsText(validateCias.errors)}`)

const COMBOS = [1, 2, 3, 4].flatMap((stages) => ['mains', 'dc'].map((topology) => ({ stages, topology })))
const fullBindings = (stages) => Object.fromEntries(emiFilterComponents(stages).map((c) => [c.ref, { mpn: c.kind === 'cmc' ? '744834101' : '890334025039CS' }]))

// ── the brick ───────────────────────────────────────────────────────────────────────────────────────

test('every EMI filter brick is CIAS-valid: unbound, partly bound and fully bound', () => {
  for (const { stages, topology } of COMBOS) {
    assertValid(buildEmiFilterCias({ stages, topology, bindings: {} }), `${stages}/${topology} unbound`)
    assertValid(buildEmiFilterCias({ stages, topology, bindings: { CMC1: { mpn: '744834101' } } }), `${stages}/${topology} partly bound`)
    assertValid(buildEmiFilterCias({ stages, topology, bindings: fullBindings(stages) }), `${stages}/${topology} bound`)
  }
})

test('the validator is not vacuous: an extra key and a bad URI are rejected', () => {
  const b = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
  assert.equal(validateCias({ ...b, stage: 'x' }), false)
  assert.equal(validateCias({ ...b, components: [{ name: 'CMC1', data: 'magnetics.ndjson?partNumber=a&b=c' }] }), false)
})

test('unbound slots are placeholder URIs, bound slots partNumber URIs, per catalogue', () => {
  const b = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: { CMC1: { mpn: '744834101' }, C_X1: { mpn: 'X1' } } })
  const data = Object.fromEntries(b.components.map((c) => [c.name, c.data]))
  assert.deepEqual(data, {
    C_X1: 'TAS/data/capacitors.ndjson?partNumber=X1',
    CMC1: 'TAS/data/magnetics.ndjson?partNumber=744834101',
    C_YL1: 'TAS/data/capacitors.ndjson?placeholder=C_YL1',
    C_YN1: 'TAS/data/capacitors.ndjson?placeholder=C_YN1',
  })
})

test('ANP015 Fig. 13 wiring: X on the stage input, choke, Y on the stage output; stages share nets', () => {
  const nets = Object.fromEntries(emiFilterNets(2).map((n) => [n.name, n.endpoints.map((e) => e.port ? `port:${e.port}` : `${e.component}|${e.pin}`).sort()]))
  assert.deepEqual(nets.line_0, ['C_X1|1', 'CMC1|P1', 'port:line_in'].sort())
  assert.deepEqual(nets.neutral_0, ['C_X1|2', 'CMC1|S1', 'port:neutral_in'].sort())
  assert.deepEqual(nets.line_1, ['CMC1|P2', 'C_YL1|1', 'C_X2|1', 'CMC2|P1'].sort())
  assert.deepEqual(nets.neutral_1, ['CMC1|S2', 'C_YN1|1', 'C_X2|2', 'CMC2|S1'].sort())
  assert.deepEqual(nets.line_2, ['CMC2|P2', 'C_YL2|1', 'port:line_out'].sort())
  assert.deepEqual(nets.pe, ['port:pe', 'C_YL1|2', 'C_YN1|2', 'C_YL2|2', 'C_YN2|2'].sort())
})

test('bad input throws — no defaults, no silent drops', () => {
  assert.throws(() => buildEmiFilterCias({ stages: 0, topology: 'mains', bindings: {} }), /stages/)
  assert.throws(() => buildEmiFilterCias({ stages: 5, topology: 'mains', bindings: {} }), /stages/)
  assert.throws(() => buildEmiFilterCias({ stages: 1, topology: '3ph', bindings: {} }), /topology/)
  assert.throws(() => buildEmiFilterCias({ stages: 1, topology: 'mains' }), /bindings/)
  assert.throws(() => buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: { CMC2: { mpn: 'a' } } }), /not a slot/)
  assert.throws(() => buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: { CMC1: {} } }), /no mpn/)
  assert.throws(() => buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: { CMC1: { mpn: 'a=b' } } }), /URI/)
})

test('parseEmiFilterCias reads a brick back and refuses one that is not the filter', () => {
  const b = buildEmiFilterCias({ stages: 2, topology: 'dc', bindings: { C_X2: { mpn: 'P' } } })
  const p = parseEmiFilterCias(b)
  assert.equal(p.stages, 2); assert.equal(p.topology, 'dc')
  assert.equal(p.slots.find((s) => s.ref === 'C_X2').mpn, 'P')
  assert.equal(p.slots.find((s) => s.ref === 'C_X1').mpn, null)
  const swapped = structuredClone(b)
  const net = swapped.connections.find((n) => n.name === 'line_0')
  net.endpoints.find((e) => e.component === 'CMC1').pin = 'S1'
  swapped.connections.find((n) => n.name === 'neutral_0').endpoints.find((e) => e.component === 'CMC1').pin = 'P1'
  assert.throws(() => parseEmiFilterCias(swapped), /connections/)
  const wrongCat = structuredClone(b)
  wrongCat.components.find((c) => c.name === 'CMC1').data = 'TAS/data/capacitors.ndjson?partNumber=x'
  assert.throws(() => parseEmiFilterCias(wrongCat), /expected 'TAS\/data\/magnetics.ndjson'/)
})

test('BOM rows: kind/role per slot, partLabel only when bound, values only when given', () => {
  const b = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: { CMC1: { mpn: '744834101' } } })
  const rows = emiFilterBom(b, { values: { C_X1: '470 nF' } })
  const by = Object.fromEntries(rows.map((r) => [r.ref, r]))
  assert.deepEqual(by.CMC1, { ref: 'CMC1', kind: 'CommonModeChoke', role: 'cmc', stage: 1, partLabel: '744834101', bound: true })
  assert.deepEqual(by.C_X1, { ref: 'C_X1', kind: 'Capacitor', role: 'cx', stage: 1, value: '470 nF', bound: false })
  assert.throws(() => emiFilterBom(b, { values: { C_Z1: '1' } }), /unknown slot/)
})

// ── the schematic ───────────────────────────────────────────────────────────────────────────────────

const pinKey = (p, kinds, portOf) => (p.ref === '@port' ? `port:${portOf[p.pin]}`
  : `${p.ref}|${kinds.get(p.ref) === 'cmc' ? p.pin : { p0: '1', p1: '2' }[p.pin]}`)
const PORT_LABELS = {
  mains: { 'L IN': 'line_in', 'N IN': 'neutral_in', 'L OUT': 'line_out', 'N OUT': 'neutral_out', PE: 'pe' },
  dc: { '+ IN': 'line_in', '− IN': 'neutral_in', '+ OUT': 'line_out', '− OUT': 'neutral_out', CHASSIS: 'pe' },
}

test('the drawing is the brick: every combo renders, and its wire pieces ARE the CIAS nets', () => {
  assert.equal(hasCiasSchematic(EMI_FILTER_SCHEMATIC), true)
  for (const { stages, topology } of COMBOS) {
    const b = buildEmiFilterCias({ stages, topology, bindings: {} })
    const { svg, pins } = renderEmiFilterSchematicWithPins(b)
    // independent re-derivation of the partition (the renderer does its own; this one must agree)
    const g = wireGraph(svg)
    const kinds = new Map(parseEmiFilterCias(b).slots.map((s) => [s.ref, s.kind]))
    const groups = new Map()
    for (const p of pins) {
      const r = g.rootAt([p.x, p.y], 4)
      assert.notEqual(r, null, `${stages}/${topology}: ${p.ref}.${p.pin} touches no wire`)
      ;(groups.get(r) || groups.set(r, []).get(r)).push(pinKey(p, kinds, PORT_LABELS[topology]))
    }
    const canon = (sets) => sets.map((x) => [...x].sort().join(',')).sort().join(';')
    assert.equal(canon([...groups.values()]),
      canon(b.connections.map((n) => n.endpoints.map((e) => (e.port ? `port:${e.port}` : `${e.component}|${e.pin}`)))),
      `${stages}/${topology}: drawn nets differ from the brick`)
    for (const c of b.components) assert.match(svg, new RegExp(`data-ref="${c.name}"[^>]*role="button"`), `${c.name} is a button`)
  }
})

test('the renderer refuses a brick that is not the EMI filter (nothing drawn)', () => {
  const b = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
  const bad = structuredClone(b)
  bad.connections.find((n) => n.name === 'pe').endpoints.pop()
  assert.throws(() => renderVerifiedEmiFilterSchematic(bad), /connections/)
})

test('checkSchematic over the brick nets flags a capacitor drawn shorted out', () => {
  const b = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
  const { svg, pins } = renderEmiFilterSchematicWithPins(b)
  const pinNet = new Map()
  for (const n of b.connections) for (const e of n.endpoints) if (e.component) pinNet.set(`${e.component}|${e.pin}`, n.name)
  assert.deepEqual(checkSchematic({ svg, pins, pinNet, magRefs: new Set(['CMC1']) }), [])
  // bridge C_X1's two terminals with a wire: the part is now drawn shorted out
  const x = pins.filter((p) => p.ref === 'C_X1')
  const shorted = svg.replace('</svg>', `<path class="sch-wire" d="M ${x[0].x - 30} ${x[0].y} L ${x[0].x - 30} ${x[1].y}"/><path class="sch-wire" d="M ${x[0].x} ${x[0].y} L ${x[0].x - 30} ${x[0].y}"/><path class="sch-wire" d="M ${x[1].x} ${x[1].y} L ${x[1].x - 30} ${x[1].y}"/></svg>`)
  assert.ok(checkSchematic({ svg: shorted, pins, pinNet, magRefs: new Set(['CMC1']) }).length > 0)
  assert.throws(() => checkSchematic({ svg, pins }), /exactly one of tas or pinNet/)
})

test('bound parts carry their part number as a visible label; unbound ones do not', () => {
  const b = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: { CMC1: { mpn: '744834101' } } })
  const svg = renderVerifiedEmiFilterSchematic(b)
  assert.match(svg, /<g class="sch-hot sch-bound" data-ref="CMC1"[^>]*aria-label="Component CMC1, CommonModeChoke, part 744834101"/)
  assert.match(svg, /<text class="sch-part"[^>]*>744834101<\/text>/)
  assert.equal((svg.match(/class="sch-part"/g) ?? []).length, 1)
  assert.doesNotMatch(svg, /data-ref="C_X1"[^>]*sch-bound|sch-bound" data-ref="C_X1"/)
})

// ── the decks ───────────────────────────────────────────────────────────────────────────────────────

const CMC_TEXT = `.subckt 744834101 P1+ P1- P2+ P2-
L1 P1+ P1- 1m
L2 P2+ P2- 1m
K L1 L2 0.99
.param X=1
.ends 744834101
`
const cap = (n, c) => capacitorSubckt({ subcktName: n, capacitanceF: c, esrOhm: 0, eslH: 0, insulationResistanceOhm: Infinity })
const models1 = () => ({
  CMC1: { subcktName: '744834101', text: CMC_TEXT, pinMap: MKF_CMC_PIN_MAP },
  C_X1: cap('cx', 1e-6), C_YL1: cap('cy', 4.7e-9), C_YN1: cap('cy', 4.7e-9),
})
const SWEEP = { lisn: CISPR16_LISN, source: IDEAL_NOISE_SOURCE, fStart: 150e3, fStop: 30e6, pointsPerDecade: 10 }

test('filter deck: every part instanced on its CIAS nets in the subckt port order; PE is ground', () => {
  const cias = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
  const deck = buildEmiFilterAcDeck({ cias, models: models1(), mode: 'cm', ...SWEEP })
  const lines = deck.split('\n')
  assert.ok(lines.includes('Xcmc1 n_line_0 n_line_1 n_neutral_0 n_neutral_1 744834101'), deck)
  assert.ok(lines.includes('Xc_x1 n_line_0 n_neutral_0 cx'))
  assert.ok(lines.includes('Xc_yl1 n_line_1 0 cy'))
  assert.ok(lines.includes('Xc_yn1 n_neutral_1 0 cy'))
  assert.equal(lines.filter((l) => /^\.subckt cy /.test(l)).length, 1, 'a shared model is defined once')
  assert.ok(lines.includes('Xlisn_line n_line_0 mains_line meas_line kh_emi_lisn'))
  assert.ok(lines.includes('Xlisn_neut n_neutral_0 mains_neut meas_neut kh_emi_lisn'))
  assert.ok(lines.includes('L1 eut mains 0.00005') && lines.includes('C1 eut meas 1e-7') && lines.includes('R1 meas 0 50'))
  assert.ok(lines.includes('Vnoise cm_src 0 AC 1'))
  assert.ok(lines.includes('Rsrc_line cm_src n_line_1 1m') && lines.includes('Rsrc_neut cm_src n_neutral_1 1m'))
  assert.ok(lines.includes('.ac dec 10 150000 30000000') && lines.at(-2) === '.end')
})

test('the pin map, not the header order, decides which net lands on which subckt port', () => {
  const cias = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
  const m = models1()
  m.CMC1 = { subcktName: 'odd', text: '.subckt odd B A D C\nL1 A B 1m\nL2 C D 1m\nK L1 L2 0.99\n.ends odd\n', pinMap: { P1: 'A', P2: 'B', S1: 'C', S2: 'D' } }
  const deck = buildEmiFilterAcDeck({ cias, models: m, mode: 'dm', ...SWEEP })
  assert.ok(deck.split('\n').includes('Xcmc1 n_line_1 n_line_0 n_neutral_1 n_neutral_0 odd'), deck)
  assert.ok(deck.split('\n').includes('Vnoise n_line_1 n_neutral_1 AC 1'))
})

test('stated source impedances are netted as Hertz nets them', () => {
  const cias = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
  const cm = buildEmiFilterAcDeck({ cias, models: models1(), mode: 'cm', ...SWEEP, source: { cmStrayF: 22e-12, dmSourceOhm: 0 } }).split('\n')
  assert.ok(cm.includes('Vnoise cm_drive 0 AC 1') && cm.includes('Cstray cm_drive cm_src 2.2e-11'))
  const dm = buildEmiFilterAcDeck({ cias, models: models1(), mode: 'dm', ...SWEEP, source: { cmStrayF: 0, dmSourceOhm: 0.1 } }).split('\n')
  assert.ok(dm.includes('Vnoise dm_drive n_neutral_1 AC 1') && dm.includes('Rsrc_dm dm_drive n_line_1 0.1'))
})

test('reference deck: same bench, source straight onto the LISNs', () => {
  const ref = buildLisnReferenceDeck({ mode: 'cm', ...SWEEP }).split('\n')
  assert.ok(ref.includes('Xlisn_line src_line mains_line meas_line kh_emi_lisn'))
  assert.ok(ref.includes('Rsrc_line cm_src src_line 1m') && ref.includes('Rsrc_neut cm_src src_neut 1m'))
  assert.ok(!ref.some((l) => /^X(c|cmc)/i.test(l)))
})

test('deck inputs are required: missing model, incomplete pin map, clashing subckt, bad sweep/LISN/source', () => {
  const cias = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
  const base = { cias, mode: 'cm', ...SWEEP }
  const m = models1()
  delete m.C_YN1
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: m }), /no model for 'C_YN1'/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: { ...models1(), CMC1: { ...models1().CMC1, pinMap: { P1: 'P1+', P2: 'P1-', S1: 'P2+' } } } }), /unmapped: S2/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: { ...models1(), CMC1: { ...models1().CMC1, subcktName: 'nope' } } }), /defines .subckt nope 0 times/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: { ...models1(), C_YN1: cap('cy', 1e-9) } }), /two different models/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: { ...models1(), C_Z9: cap('cz', 1e-9) } }), /not in the brick/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: models1(), mode: 'xm' }), /mode/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: models1(), fStop: 100 }), /fStop/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: models1(), pointsPerDecade: 2.5 }), /pointsPerDecade/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: models1(), lisn: { inductanceH: 5e-5 } }), /lisn/)
  assert.throws(() => buildEmiFilterAcDeck({ ...base, models: models1(), source: undefined }), /source/)
  assert.throws(() => capacitorSubckt({ subcktName: 'c', capacitanceF: 1e-9, esrOhm: 0, eslH: 0 }), /insulationResistanceOhm/)
})

test('capacitor model: Rs + Ls + Cs in series, Riso across Cs', () => {
  const m = capacitorSubckt({ subcktName: 'c1', capacitanceF: 1e-6, esrOhm: 0.01, eslH: 5e-9, insulationResistanceOhm: 1e10 })
  assert.equal(m.text, '.subckt c1 a b\nRs a n_rs 0.01\nLs n_rs n_ls 5e-9\nCs n_ls b 0.000001\nRiso n_ls b 1e+10\n.ends c1\n')
  assert.deepEqual(m.pinMap, { 1: 'a', 2: 'b' })
})

test('insertionLossDb: ratio of receiver-port magnitudes, loud on anything missing', () => {
  const mk = (re, im) => ({ success: true, error: '', frequenciesHz: [1e5, 1e6], vectors: { meas_line: { re, im } } })
  const { frequenciesHz, ilDb } = insertionLossDb(mk([1, 0], [0, 1]), mk([0.1, 0], [0, 0.01]), EMI_PROBES.line)
  assert.deepEqual(frequenciesHz, [1e5, 1e6])
  assert.ok(Math.abs(ilDb[0] - 20) < 1e-9 && Math.abs(ilDb[1] - 40) < 1e-9)
  assert.ok(Math.abs(insertionLossDb(mk([1, 0], [0, 1]), mk([0.1, 0], [0, 0.01]), 'V(MEAS_LINE)').ilDb[0] - 20) < 1e-9)
  assert.throws(() => insertionLossDb({ success: false, error: 'singular matrix' }, mk([1, 1], [0, 0]), 'meas_line'), /singular matrix/)
  assert.throws(() => insertionLossDb(mk([1, 1], [0, 0]), mk([1, 1], [0, 0]), 'meas_neut'), /no vector 'meas_neut'/)
  const shifted = mk([1, 1], [0, 0]); shifted.frequenciesHz = [1e5, 2e6]
  assert.throws(() => insertionLossDb(mk([1, 1], [0, 0]), shifted, 'meas_line'), /different frequencies/)
})
