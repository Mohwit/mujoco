"""Copy the recorded MuJoCo rollouts from results/curves.json into the lesson page's script.

    .venv/bin/python lessons/15-learning-a-policy/15_make_page_data.py

Rewrites the block between '// @@DATA15-START' and '// @@DATA15-END' in _fragments/script.js.
Angles are kept every 2nd env step (0.04 s) in units of 0.01 rad, actions every 2nd env step in units of 0.01,
to keep the page small. Summary numbers (returns, mean |action|, ...) are the full-resolution
values computed by 15_train.py over seeds 0-4.
"""
import json
import pathlib

HERE = pathlib.Path(__file__).parent
R = json.loads((HERE / "results" / "curves.json").read_text())
VARIANTS = ["base", "no_act", "big_act", "sparse", "approach", "signed"]


def summ(r, with_roll=True):
    out = {"R": round(r["return"], 3), "B": round(r["base_return"], 3), "A": round(r["mean_abs_a"], 3),
           "dA": round(r["mean_da"], 4), "E": round(r["final_err"], 3), "X": r["crossings"]}
    if with_roll:
        out["q"] = [round(x * 100) for x in r["rollout"]["q"]]   # centi-radians (integers keep the page small)
        out["a"] = [round(x * 100) for x in r["rollout"]["a"]]   # percent of full motor command
    return out


data = {
    "steps": R["checkpoints"],
    "runs": {v: [summ(r) for r in R["runs"][f"{v}_seed0"]] for v in VARIANTS},
    "seeds": {k: [round(r["return"], 3) for r in res] for k, res in R["runs"].items()},
    "refs": {v: {n: summ(r, n != "zero") for n, r in R["refs"][v].items()} for v in VARIANTS},
    "secs": R["train_seconds"],
}
js = "const D15 = " + json.dumps(data, separators=(",", ":")) + ";"
p = HERE / "_fragments" / "script.js"
s = p.read_text()
a, b = s.index("// @@DATA15-START"), s.index("// @@DATA15-END")
s = s[:a] + "// @@DATA15-START\n// Recorded from MuJoCo by 15_train.py (copied by 15_make_page_data.py). Do not edit by hand.\n" + js + "\n" + s[b:]
p.write_text(s)
print(f"wrote {len(js):,} characters of data into {p}")
