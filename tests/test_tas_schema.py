#!/usr/bin/env python3
"""Validate that Kirchhoff's generated TAS documents conform to the PSMA schemas.

Guards, for every generated TAS:
  1. Every component's inputs.designRequirements validates against its family schema
     (SAS/CAS/MAS inputs/designRequirements.json) — the part-sourcing contract is well-formed.
  2. Each component is a valid PEAS "pre-sourcing seed": an empty part object (semiconductor:{mosfet:{}},
     capacitor:{}, magnetic:{}) + inputs.designRequirements filled with minimum values. (SAS/CAS were
     made seed-friendly to allow this — a component is valid iff it has architecture OR
     manufacturerInfo.datasheetInfo OR min-value designRequirements; peas.json always requires the latter.)
  3. The whole TAS (topology/inputs/simulation, seeds and all) validates against TAS.json.

Run: PYTHONPATH=<build-dir> python3 tests/test_tas_schema.py   (needs jsonschema + referencing + PyKirchhoff)
"""
import os, sys, json, glob

PSMA = os.environ.get("KIRCHHOFF_PSMA_ROOT", "/home/alf/PSMA")

try:
    import PyKirchhoff
    from jsonschema import Draft202012Validator
    from referencing import Registry, Resource
except Exception as e:
    print(f"FAIL: cannot import dependencies ({e}). Need PyKirchhoff on PYTHONPATH + jsonschema + referencing.")
    sys.exit(2)

# Registry of every sibling schema, keyed by $id (cross-repo $ref resolution).
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
    return Draft202012Validator(res[sid].contents, registry=reg)

TAS_V = V("https://psma.com/tas/TAS.json")
PEAS_V = V("https://psma.com/peas/peas.json")
FAM = {"semiconductor": "https://psma.com/sas/inputs/designRequirements.json",
       "capacitor":     "https://psma.com/cas/inputs/designRequirements.json",
       "magnetic":      "https://psma.com/mas/inputs/designRequirements.json",
       "controller":    "https://psma.com/ctas/inputs/designRequirements.json",
       "resistor":      "https://psma.com/ras/inputs/designRequirements.json"}
OP_V = V("https://psma.com/mas/inputs/operatingPoint.json")

FLYBACK_IN = {"designRequirements": {"efficiency": 0.88, "inputType": "dc",
    "inputVoltage": {"minimum": 36, "nominal": 48, "maximum": 60}, "switchingFrequency": {"nominal": 100000},
    "isolationVoltage": 1500, "outputs": [{"name": "12V", "voltage": {"nominal": 12}, "regulation": "voltage"}]},
    "operatingPoints": [{"name": "f", "inputVoltage": 48, "ambientTemperature": 25, "outputs": [{"name": "12V", "power": 24}]}]}
BOOST_IN = {"designRequirements": {"efficiency": 0.9, "inputType": "dc",
    "inputVoltage": {"minimum": 11.4, "nominal": 12, "maximum": 12.6}, "switchingFrequency": {"nominal": 100000},
    "outputs": [{"name": "24V", "voltage": {"nominal": 24}, "regulation": "voltage"}]},
    "operatingPoints": [{"name": "f", "inputVoltage": 12, "outputs": [{"name": "24V", "power": 24}]}]}

failures = 0

def disc(data):
    return next((k for k in FAM if k in data), None)

def check(label, tas):
    global failures
    if isinstance(tas, str):
        tas = json.loads(tas)
    print(f"\n== {label} ==")

    # (1) per-component requirements + (2) magnetic seed + operating points
    for st in tas["topology"]["stages"]:
        for c in st["circuit"].get("components", []) if isinstance(st.get("circuit"), dict) else []:
            data = c["data"]
            k = disc(data)
            dr = data.get("inputs", {}).get("designRequirements")
            if dr is not None:
                errs = list(V(FAM[k]).iter_errors(dr))
                if errs:
                    failures += 1
                    print(f"  FAIL requirements {c['name']} ({k}): {errs[0].message[:90]}")
                else:
                    print(f"  ok   requirements {c['name']} ({k})")
            for op in data.get("inputs", {}).get("operatingPoints", []):
                if list(OP_V.iter_errors(op)):
                    failures += 1
                    print(f"  FAIL operatingPoint {c['name']}")
            # each component is a valid PEAS pre-sourcing seed (empty part + min-value requirements)
            if list(PEAS_V.iter_errors(data)):
                failures += 1
                print(f"  FAIL PEAS seed {c['name']} ({k})")
            else:
                print(f"  ok   PEAS seed {c['name']} ({k})")

    # (3) the whole TAS (with its seeds) validates against TAS.json — no part binding needed
    errs = list(TAS_V.iter_errors(tas))
    if errs:
        failures += 1
        print(f"  FAIL TAS: {errs[0].message[:90]} @ {list(errs[0].absolute_path)[:5]}")
    else:
        print("  ok   full TAS validates against TAS.json")

check("flyback", PyKirchhoff.design_flyback_tas(FLYBACK_IN))
check("boost", PyKirchhoff.design_boost_tas(BOOST_IN))

# Multi-output (N isolated secondaries) — ABT #86. A 2-output forward-family spec must still emit a
# schema-valid TAS (extra secondary windings, per-output rectifiers/filters, extra external output ports).
MULTI_IN = {"designRequirements": {"efficiency": 1.0, "inputType": "dc",
    "inputVoltage": {"minimum": 38, "nominal": 40, "maximum": 42}, "switchingFrequency": {"nominal": 100000},
    "outputs": [{"name": "out", "voltage": {"nominal": 5}, "regulation": "voltage"},
                {"name": "aux", "voltage": {"nominal": 12}, "regulation": "voltage"}]},
    "operatingPoints": [{"name": "f", "inputVoltage": 40, "ambientTemperature": 25,
                         "outputs": [{"name": "out", "power": 25}, {"name": "aux", "power": 12}]}]}
check("forward-2out", PyKirchhoff.design_forward_tas(MULTI_IN))
check("tsf-2out", PyKirchhoff.design_two_switch_forward_tas(MULTI_IN))
check("acf-2out", PyKirchhoff.design_acf_tas(MULTI_IN))

print("\n" + ("ALL KIRCHHOFF TAS CHECKS PASSED" if failures == 0 else f"{failures} CHECK(S) FAILED"))
sys.exit(1 if failures else 0)
