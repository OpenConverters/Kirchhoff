#!/usr/bin/env python3
"""Validate that the web app's waveform export conforms to the PEAS/MAS schemas.

The Waveforms tab can hand the user the whole design's waveform set as JSON. Every member of that
file claims to be a schema type, so every member must validate — including the parts the engine
serialises with explicit nulls (an unset optional written as `null` is a PRESENT property to a
schema: `{"ancillaryLabel": null, "numberPeriods": null, data, time}` matches NEITHER branch of the
waveform oneOf, and each null fails its declared type).

Guards, per topology:
  1. magnetics[].operatingPoint  validates against MAS   inputs/operatingPoint.json
  2. components[].operatingPoint validates against PEAS  twoTerminalOperatingPoint.json (2-terminal)
     or PEAS multiPortOperatingPoint.json (MOSFETs — the V_DS/I_D pair as the 'drain' port)
  3. nothing exported carries a null anywhere

The export is built by the SAME module the browser download button calls
(web/src/waveExport.js, through web/scripts/buildExportJson.mjs), so a green run here is a
statement about the file the user actually receives, not about a re-implementation of it.

Run: PYTHONPATH=<build-dir> python3 tests/test_export_schema.py
     (needs jsonschema + referencing + PyKirchhoff + node)
"""
import os, sys, json, glob, subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Kirchhoff consumes the schemas as submodules under deps/; KIRCHHOFF_PSMA_ROOT overrides.
PSMA = os.environ.get("KIRCHHOFF_PSMA_ROOT", os.path.join(ROOT, "deps"))
BUILDER = os.path.join(ROOT, "web", "scripts", "buildExportJson.mjs")

try:
    import PyKirchhoff
    from jsonschema import Draft202012Validator
    from referencing import Registry, Resource
except Exception as e:
    print(f"FAIL: cannot import dependencies ({e}). Need PyKirchhoff on PYTHONPATH + jsonschema + referencing.")
    sys.exit(2)

res = {}
for f in glob.glob(f"{PSMA}/*/schemas/**/*.json", recursive=True):
    try:
        s = json.load(open(f))
    except Exception:
        continue
    if isinstance(s, dict) and "$id" in s:
        res[s["$id"]] = Resource.from_contents(s)
reg = Registry().with_resources(res.items())

def V(sid):
    if sid not in res:
        print(f"FAIL: schema {sid} not found under {PSMA} — is the submodule initialised?")
        sys.exit(2)
    return Draft202012Validator(res[sid].contents, registry=reg)

SPEC = {
    "designRequirements": {
        "efficiency": 0.9,
        "inputVoltage": {"minimum": 36, "nominal": 48, "maximum": 57},
        "switchingFrequency": {"nominal": 100000},
        "outputs": [{"name": "out", "voltage": {"nominal": 12}}],
    },
    "operatingPoints": [{"name": "full_load", "inputVoltage": 48, "ambientTemperature": 25,
                         "outputs": [{"name": "out", "power": 60}]}],
}

failures = 0

def fail(msg):
    global failures
    failures += 1
    print(f"  FAIL: {msg}")

def nulls_in(value, path="$"):
    if value is None:
        return [path]
    if isinstance(value, list):
        return [p for i, v in enumerate(value) for p in nulls_in(v, f"{path}[{i}]")]
    if isinstance(value, dict):
        return [p for k, v in value.items() for p in nulls_in(v, f"{path}.{k}")]
    return []

def check(topology, with_components=True):
    print(f"\n=== {topology} ===")
    before = failures
    full = PyKirchhoff.design_tas_full(topology, SPEC)
    tas = full["tas"]
    ctx = {
        "topology": topology,
        "magnetics": PyKirchhoff.topology_waveforms(tas),
        "analyticalWaveforms": full.get("analyticalWaveforms") or {},
        "periods": 1,
        "opIdx": 0,
    }
    if with_components:
        cw = PyKirchhoff.component_waveforms(tas, {"origin": "REQUIREMENTS"})
        if cw.get("success") is False:
            fail(f"component_waveforms failed: {cw.get('error')}")
        else:
            ctx["componentWaves"] = cw

    proc = subprocess.run(["node", BUILDER], input=json.dumps(ctx), capture_output=True, text=True)
    if proc.returncode != 0:
        fail(f"buildExportJson.mjs exited {proc.returncode}: {proc.stderr.strip()[:400]}")
        return
    export = json.loads(proc.stdout)

    n_mag = n_comp = 0
    for m in export["magnetics"]:
        errs = list(V(export["schemas"]["magnetics"]).iter_errors(m["operatingPoint"]))
        n_mag += 1
        for e in errs[:4]:
            fail(f"magnetic {m['name']}: {'/'.join(map(str, e.path))} :: {e.message[:160]}")
    for c in export["components"]:
        two = "excitation" in c["operatingPoint"]
        sid = export["schemas"]["twoTerminalComponents" if two else "multiPortComponents"]
        errs = list(V(sid).iter_errors(c["operatingPoint"]))
        n_comp += 1
        for e in errs[:4]:
            fail(f"component {c['ref']} ({c['kind']}): {'/'.join(map(str, e.path))} :: {e.message[:160]}")

    if n_mag == 0:
        fail("no magnetic operating point in the export — the gate would pass vacuously")
    if with_components and n_comp == 0:
        fail("no component operating point in the export — the gate would pass vacuously")

    stray = nulls_in(export["magnetics"], "magnetics") + nulls_in(export["components"], "components")
    if stray:
        fail(f"{len(stray)} null(s) survive into the export, e.g. {stray[:3]}")

    if failures == before:
        print(f"  {n_mag} magnetic + {n_comp} component operating point(s) validated, no nulls")
    else:
        print(f"  {failures - before} problem(s) in {n_mag} magnetic + {n_comp} component operating point(s)")

# One topology per shape of magnetic: a transformer (2 windings), a single inductor, and a
# resonant family whose winding waveforms are 'custom' (no closed form to fall back on).
check("flyback")
check("buck")
check("llc")

print("\n" + ("ALL WAVEFORM-EXPORT SCHEMA CHECKS PASSED" if failures == 0 else f"{failures} CHECK(S) FAILED"))
sys.exit(1 if failures else 0)
