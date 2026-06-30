"""abt #67 verification (boost half): controlled diode-vs-synchronous efficiency A/B.

Designs ONE boost spec twice (diode default vs config.rectifier=synchronous),
binds the SAME real datasheet semiconductors to both, and runs the regulated
operating point. Both arms share the identical magnetic, switch Q1, and output
cap; only the output rectifier differs (real diode D1 vs sync FET Q2 + body
diode). So the efficiency delta isolates the rectifier. Reports regulated vout
(must hit target) and efficiency for each.
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build"))
import PyKirchhoff  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import regulate  # noqa: E402

VIN, VOUT, IOUT, FSW = 12.0, 48.0, 2.0, 250000.0
FID = os.environ.get("FID", "DATASHEET")


def _spec(config):
    s = {
        "designRequirements": {
            "efficiency": 0.9,
            "inputVoltage": {"minimum": VIN * 0.95, "nominal": VIN, "maximum": VIN * 1.05},
            "switchingFrequency": {"nominal": FSW},
            "outputs": [{"name": "out", "voltage": {"nominal": VOUT}, "regulation": "voltage"}],
        },
        "operatingPoints": [{"inputVoltage": VIN, "outputs": [{"power": VOUT * IOUT}]}],
    }
    if config:
        s["config"] = config
    return s


def _run(config, tag):
    tas = PyKirchhoff.design_boost_tas(_spec(config))
    if FID == "DATASHEET":
        regulate.bind_datasheet_semis(tas)
    op = regulate.simulate_regulated(tas, VOUT, "boost", fidelity={"origin": FID}, tol=0.01)
    print(f"[{tag}] regulated={op.get('regulated')} vout={op.get('vout')} "
          f"value(duty)={op.get('value')} eff={op.get('efficiency')} "
          f"pin={op.get('pin')} pout={op.get('pout')}", flush=True)
    return op


def main():
    print(f"=== boost {VIN}->{VOUT}V/{IOUT}A  fidelity={FID} ===", flush=True)
    diode = _run(None, "diode")
    sync = _run({"rectifier": "synchronous"}, "sync ")
    ed, es = diode.get("efficiency"), sync.get("efficiency")
    print("\n=== SUMMARY ===", flush=True)
    print(f"diode: vout={diode.get('vout'):.3f} eff={ed}", flush=True)
    print(f"sync : vout={sync.get('vout'):.3f} eff={es}", flush=True)
    if ed and es:
        print(f"efficiency delta (sync - diode) = {(es - ed) * 100:+.2f} pp", flush=True)
    ok = (diode.get("regulated") and sync.get("regulated") and es and ed and es > ed)
    print(f"\nVERDICT: {'PASS' if ok else 'FAIL'} "
          f"(both regulate + sync more efficient)", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
