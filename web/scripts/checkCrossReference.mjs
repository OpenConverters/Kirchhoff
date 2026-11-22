// kh.crossReference through the REAL engine, worker and client (run: `node scripts/checkCrossReference.mjs`).
//
// The method shipped calling a WASM export that did not exist (ABT #2503), which no test of kh.js or of the
// C++ alone could see. So nothing is faked here: src/worker.js itself runs in-process and imports
// build-wasm-ng/kirchhoff.js, or the engine named by the first argument.
import { resolve } from 'node:path'
import { pathToFileURL } from 'node:url'
import { createKirchhoff } from '../src/kh.js'

const wasmUrl = process.argv[2]
  ? pathToFileURL(resolve(process.argv[2])).href
  : new URL('../../build-wasm-ng/kirchhoff.js', import.meta.url).href

const page = { onmessage: null, onerror: null, terminate() {} }
globalThis.self = { postMessage: (data) => page.onmessage({ data }) }
await import('../src/worker.js')
page.postMessage = (data) => { globalThis.self.onmessage({ data }) }
const kh = createKirchhoff({ wasmUrl, createWorker: () => page })

const failures = []
const check = (ok, what) => { console.log(`${ok ? 'OK  ' : 'FAIL'} ${what}`); if (!ok) failures.push(what) }
async function outcome(run) {
  try { return { value: await run() } } catch (e) { return { error: e?.message ?? String(e) } }
}

const part = (mpn, over = {}) => ({ mpn, vds: 60, id: 10, rds_on: 0.01, lifecycle_status: 'production', ...over })
const original = part('ORIGINAL')
const candidates = [part('UNDERRATED', { vds: 20 }), part('EQUAL')]

const ranked = await outcome(() => kh.crossReference('MOSFET', original, candidates))
const list = ranked.value?.candidates ?? []
check(ranked.value?.category === 'mosfet', `returns a mosfet result${ranked.error ? ` (${ranked.error})` : ''}`)
check(list[0]?.mpn === 'EQUAL' && list[0]?.status === 'recommended', 'the equal part ranks first, recommended')
check(list[1]?.mpn === 'UNDERRATED' && list[1]?.status === 'no_substitute', 'a 20 V part is no substitute for a 60 V one')

const unverified = await outcome(() => kh.crossReference('MOSFET', original, candidates, { originalVerified: false }))
check(unverified.value?.candidates?.[0]?.status === 'partial', 'originalVerified: false caps the result at partial')

const one = await outcome(() => kh.crossReference('MOSFET', original, candidates, { maxResults: 1 }))
check(one.value?.candidates?.length === 1, 'maxResults limits the list')

const selectShape = await outcome(() => kh.crossReference('MOSFET', original, [{ mpn: 'SELECTED', ratings: { vds: 60 } }]))
check(/'SELECTED' has no 'lifecycle_status'/.test(selectShape.error ?? ''), 'a candidate without lifecycle_status rejects, naming part and key')

const noRules = await outcome(() => kh.crossReference('Controller', original, candidates))
check(/no ranking rules for category 'controller'/.test(noRules.error ?? ''), 'a kind without ranking rules rejects, naming the category')

const after = await outcome(() => kh.crossReference('MOSFET', original, candidates))
check(after.value?.candidates?.[0]?.mpn === 'EQUAL', 'the engine still answers after a rejection')

if (failures.length) { console.error(`\n${failures.length} cross-reference check(s) failed`); process.exit(1) }
console.log('\nkh.crossReference: client -> worker -> WASM ranker verified')
