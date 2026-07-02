// Engine worker: hosts the Kirchhoff WASM module off the main thread so long
// ngspice transients don't freeze the UI. Pure RPC — {id, fn, args} in,
// {id, ok, result|error} out. All results are the engine's raw strings; the
// "Exception: " protocol is handled by the caller (kh.js).

let enginePromise = null

function engine() {
  if (!enginePromise) {
    enginePromise = import(/* @vite-ignore */ new URL('kirchhoff.js', self.location.origin + '/').href)
      .then((m) => m.default())
  }
  return enginePromise
}

self.onmessage = async (ev) => {
  const { id, fn, args } = ev.data
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
