// How components find their engine: an explicit `engine` prop wins, otherwise the instance a host
// installed with app.use(createKirchhoff(...)). No instance at all is a host wiring error — thrown,
// never papered over with a default engine pointed at a guessed URL.
import { inject } from 'vue'
import { KIRCHHOFF_KEY } from '../kh.js'

export { KIRCHHOFF_KEY }

export function useKirchhoff(explicit = null) {
  if (explicit) return explicit
  const kh = inject(KIRCHHOFF_KEY, null)
  if (!kh) throw new Error('no Kirchhoff engine: app.use(createKirchhoff({ wasmUrl })) or pass the component an `engine` prop')
  return kh
}
