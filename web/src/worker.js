// Engine worker: hosts the Kirchhoff WASM module off the main thread so long
// ngspice transients don't freeze the UI. Pure RPC — {id, fn, args} in,
// {id, ok, result|error} out. All results are the engine's raw strings; the
// "Exception: " protocol is handled by the caller (kh.js).
//
// The worker does not know where the engine lives: the facade's first message is
// {fn: '__config__', args: [wasmUrl]} (createKirchhoff's wasmUrl option). A call that
// arrives before it is a facade bug and fails loudly rather than guessing a path.

let wasmUrl = null
let enginePromise = null

function engine() {
  if (!wasmUrl) return Promise.reject(new Error('engine worker has no wasmUrl — createKirchhoff() configures it before the first call'))
  if (!enginePromise) {
    enginePromise = import(/* @vite-ignore */ wasmUrl).then((m) => m.default())
  }
  return enginePromise
}

self.onmessage = async (ev) => {
  const { id, fn, args } = ev.data
  if (fn === '__config__') {
    wasmUrl = args[0]
    return
  }
  try {
    const M = await engine()
    if (fn === '__init__') {
      self.postMessage({ id, ok: true, result: 'ready' })
      return
    }
    const out = M[fn](...args)
    self.postMessage({ id, ok: true, result: out })
  } catch (e) {
    self.postMessage({ id, ok: false, error: e?.message ?? String(e) })
  }
}
