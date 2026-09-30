// EMI line filter insertion loss through the REAL engine (run: `node scripts/checkEmiFilterAc.mjs`).
//
// Builds the CIAS brick, the filter + LISN-reference .ac decks (src/emiFilter.js), runs both through the
// WASM engine's in-process libngspice (run_ngspice_ac — the same entry the app's worker calls), and
// checks the result against physics that does not come from ngspice:
//   1. ideal choke + ideal capacitors, DM: the leakage L and the X capacitor form an LC section into the
//      LISN pair — the ngspice IL must equal the closed-form ABCD insertion loss at every point, and roll
//      off at 40 dB/decade above the corner.
//   2. same filter, CM: series L(1+K)/2 into the LISNs in parallel — the closed form again, 20 dB/decade.
//   3. MKF's real export (tests/fixtures/mkf_cmc_744834101.cir.txt, `.subckt 744834101 P1+ P1- P2+ P2-`)
//      through MKF_CMC_PIN_MAP with no effective X capacitor: CM IL large, DM IL small. The same choke
//      with ONE winding reversed must give the opposite — so a swapped dot cannot pass.
//   4. the engine client (createKirchhoff().emiFilterInsertionLoss) returns the same numbers as 3.
import init from '../../build-wasm-ng/kirchhoff.js'
import { readFileSync } from 'node:fs'
import {
  buildEmiFilterCias, buildEmiFilterAcDeck, buildLisnReferenceDeck, insertionLossDb, capacitorSubckt,
  CISPR16_LISN, IDEAL_NOISE_SOURCE, EMI_PROBES, MKF_CMC_PIN_MAP,
} from '../src/emiFilter.js'
import { createKirchhoff } from '../src/kh.js'

const M = await init()
const failures = []
const check = (ok, what) => { console.log(`${ok ? 'OK  ' : 'FAIL'} ${what}`); if (!ok) failures.push(what) }

const L = 1e-3, K = 0.99, CX = 1e-6, CY = 4.7e-9
const idealCmc = { subcktName: 'ideal_cmc', pinMap: MKF_CMC_PIN_MAP,
  text: `.subckt ideal_cmc P1+ P1- P2+ P2-\nL1 P1+ P1- ${L}\nL2 P2+ P2- ${L}\nK1 L1 L2 ${K}\n.ends ideal_cmc\n` }
const cap = (n, c) => capacitorSubckt({ subcktName: n, capacitanceF: c, esrOhm: 0, eslH: 0, insulationResistanceOhm: Infinity })
const cias = buildEmiFilterCias({ stages: 1, topology: 'mains', bindings: {} })
const bench = { lisn: CISPR16_LISN, source: IDEAL_NOISE_SOURCE, fStart: 150e3, fStop: 30e6, pointsPerDecade: 20 }

function il(models, mode) {
  const run = (deck) => {
    const r = JSON.parse(M.run_ngspice_ac(deck))
    if (!r.success) throw new Error(`ngspice: ${r.error}`)
    return r
  }
  const filt = run(buildEmiFilterAcDeck({ cias, models, mode, ...bench }))
  const ref = run(buildLisnReferenceDeck({ mode, ...bench }))
  return insertionLossDb(ref, filt, EMI_PROBES.line)
}

// complex arithmetic for the closed forms
const C = (re, im = 0) => ({ re, im })
const add = (a, b) => C(a.re + b.re, a.im + b.im)
const mul = (a, b) => C(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re)
const div = (a, b) => { const d = b.re * b.re + b.im * b.im; return C((a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d) }
const par = (a, b) => div(mul(a, b), add(a, b))
const abs = (a) => Math.hypot(a.re, a.im)
const lisnZ = (f, rMains) => {                    // EUT-side impedance of one LISN of the bench
  const w = 2 * Math.PI * f
  const lBranch = C(rMains, w * CISPR16_LISN.inductanceH)
  const mBranch = C(CISPR16_LISN.measuringImpedanceOhm, -1 / (w * CISPR16_LISN.couplingCapacitanceF))
  return par(lBranch, mBranch)
}
// IL of a two-port [[A,B],[Cc,D]] between source Zs and load Zl, relative to Zs driving Zl directly
const ilAbcd = ([A, B, Cc, D], zs, zl) =>
  20 * Math.log10(abs(add(add(mul(A, zl), B), add(mul(mul(Cc, zs), zl), mul(D, zs)))) / abs(add(zs, zl)))

// dB/decade between the first sweep point at/above f1 and the last at/below f2
const slope = (res, f1, f2) => {
  const i = res.frequenciesHz.findIndex((x) => x >= f1 * (1 - 1e-9))
  const j = res.frequenciesHz.findLastIndex((x) => x <= f2 * (1 + 1e-9))
  if (i < 0 || j <= i) throw new Error(`no sweep points in ${f1}..${f2}`)
  return (res.ilDb[j] - res.ilDb[i]) / Math.log10(res.frequenciesHz[j] / res.frequenciesHz[i])
}

// ── 1 + 2: ideal parts vs the closed form ──
const ideal = { CMC1: idealCmc, C_X1: cap('cx', CX), C_YL1: cap('cy', CY), C_YN1: cap('cy', CY) }
const dm = il(ideal, 'dm')
const cm = il(ideal, 'cm')
let worstDm = 0, worstCm = 0
dm.frequenciesHz.forEach((f, i) => {
  const w = 2 * Math.PI * f
  // DM, source side first: (Y caps across an ideal source do nothing) series loop leakage 2L(1-K), shunt X
  const z1 = C(0, w * 2 * L * (1 - K)), y = C(0, w * CX)
  const zl = add(lisnZ(f, 0), lisnZ(f, 1e-3))
  const expDm = ilAbcd([add(C(1), mul(z1, y)), z1, y, C(1)], C(0), zl)
  worstDm = Math.max(worstDm, Math.abs(expDm - dm.ilDb[i]))
  // CM: 2 x 1 mOhm drive in parallel, shunt 2·Cy, series L(1+K)/2, into the two LISNs in parallel
  const yc = C(0, w * 2 * CY), zc = C(0, w * L * (1 + K) / 2)
  const expCm = ilAbcd([C(1), zc, yc, add(C(1), mul(yc, zc))], C(0.5e-3), par(lisnZ(f, 0), lisnZ(f, 1e-3)))
  worstCm = Math.max(worstCm, Math.abs(expCm - cm.ilDb[i]))
})
check(worstDm < 0.1, `ideal parts, DM: ngspice IL = closed-form LC insertion loss (worst |Δ| ${worstDm.toFixed(4)} dB)`)
check(worstCm < 0.1, `ideal parts, CM: ngspice IL = closed-form insertion loss (worst |Δ| ${worstCm.toFixed(4)} dB)`)
const sDm = slope(dm, 3e6, 30e6), sCm = slope(cm, 3e6, 30e6)
check(Math.abs(sDm - 40) < 1.5, `ideal parts, DM: LC section rolls off at 40 dB/dec above its corner (3-30 MHz: ${sDm.toFixed(2)} dB/dec)`)
check(Math.abs(sCm - 20) < 1.5, `ideal parts, CM: series choke rolls off at 20 dB/dec (3-30 MHz: ${sCm.toFixed(2)} dB/dec)`)

// ── 3: MKF's export, right and reversed ──
const mkfText = readFileSync(new URL('../tests/fixtures/mkf_cmc_744834101.cir.txt', import.meta.url), 'utf8')
const mkfModels = (pinMap) => ({ CMC1: { subcktName: '744834101', text: mkfText, pinMap },
  C_X1: cap('cx_none', 1e-12), C_YL1: cap('cy', CY), C_YN1: cap('cy', CY) })
const at150k = (r) => r.ilDb[0]
const maxBelow = (r, fMax) => Math.max(...r.ilDb.filter((_, i) => r.frequenciesHz[i] <= fMax))
const mkfCm = il(mkfModels(MKF_CMC_PIN_MAP), 'cm')
const mkfDm = il(mkfModels(MKF_CMC_PIN_MAP), 'dm')
check(at150k(mkfCm) > 25, `MKF 744834101, CM: magnetising inductance works (${at150k(mkfCm).toFixed(1)} dB at 150 kHz > 25)`)
check(maxBelow(mkfDm, 10e6) < 1, `MKF 744834101, DM, no X: only leakage (max ${maxBelow(mkfDm, 10e6).toFixed(2)} dB below 10 MHz < 1)`)
const reversed = { P1: 'P1+', P2: 'P1-', S1: 'P2-', S2: 'P2+' }
const revCm = il(mkfModels(reversed), 'cm')
const revDm = il(mkfModels(reversed), 'dm')
check(at150k(revCm) < 3 && at150k(revDm) > 20,
  `one winding reversed inverts it (CM ${at150k(revCm).toFixed(1)} dB, DM ${at150k(revDm).toFixed(1)} dB at 150 kHz) — a swapped dot cannot pass`)

// ── 4: the engine client, over the same worker protocol (src/worker.js) run in-process ──
const fakeWorker = () => {
  const w = { onmessage: null, onerror: null, terminate() {} }
  w.postMessage = ({ id, fn, args }) => {
    if (fn === '__config__') return
    queueMicrotask(() => {
      try { w.onmessage({ data: { id, ok: true, result: fn === '__init__' ? 'ready' : M[fn](...args) } }) }
      catch (e) { w.onmessage({ data: { id, ok: false, error: e.message } }) }
    })
  }
  return w
}
const kh = createKirchhoff({ wasmUrl: 'in-process', createWorker: fakeWorker })
const viaClient = await kh.emiFilterInsertionLoss({ cias, models: mkfModels(MKF_CMC_PIN_MAP), mode: 'cm', ...bench, probe: EMI_PROBES.line })
check(viaClient.ilDb.length === mkfCm.ilDb.length && viaClient.ilDb.every((x, i) => Math.abs(x - mkfCm.ilDb[i]) < 1e-9),
  'kh.emiFilterInsertionLoss returns exactly the direct run')
// NOT checked here: that kh.runNgspiceAc rejects a deck instantiating an undefined subckt. The engine's
// run_ngspice_ac never returns on such a deck (busy-loops) — ABT #1556. Restore the check once it is fixed.

if (failures.length) { console.error(`\n${failures.length} EMI filter AC check(s) failed`); process.exit(1) }
console.log('\nEMI filter: CIAS brick -> ngspice decks -> insertion loss agrees with the closed forms; MKF choke phasing verified')
