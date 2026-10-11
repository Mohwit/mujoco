"""Assemble lessons/index.html from index.base.html plus per-lesson fragments.

Each lesson NN (4-16) keeps its course-site pieces in lessons/NN-topic/_fragments/:
  nav.html      sidebar <button class="nav-item" data-lesson="N">
  pill.html     mobile <button class="pill" data-lesson="N">
  panel.html    <div class="lesson-content xp" id="lessonN">...</div>
  sandbox.html  <div class="sandbox-container" id="sandboxN">...</div>
  script.js     sandbox functions + "LESSON N EXPLAINER" IIFE

Usage:
  python build.py                      # all lessons with fragments -> ../index.html
  python build.py --only 5 --out X.html  # lessons 1-3 + lesson 5 only (preview)
"""
import argparse, pathlib, re, sys

HERE = pathlib.Path(__file__).resolve().parent
LESSONS_DIR = HERE.parent
DIRS = {
    4: "04-inspection-and-visualization",
    5: "05-actuators-and-control",
    6: "06-sensors-and-observation",
    7: "07-contact-and-constraints",
    8: "08-controllers-and-experiments",
    9: "09-environments-and-next-steps",
    10: "10-capstone-cart-pole",
    11: "11-building-bigger-models",
    12: "12-kinematics-and-jacobians",
    13: "13-grippers-and-grasping",
    14: "14-mobile-bases",
    15: "15-learning-a-policy",
    16: "16-capstone-warehouse-robot",
}
TITLES = {4: "Inspection and Visualization", 5: "Actuators and Control", 6: "Sensors and Observation",
          7: "Contact and Constraints", 8: "Controllers and Experiments", 9: "Environments and Next Steps",
          10: "Capstone 1: Balance a Cart-Pole", 11: "Building Bigger Models", 12: "Kinematics and Jacobians",
          13: "Grippers and Grasping", 14: "Mobile Bases", 15: "Learning a Policy", 16: "Capstone 2: Warehouse Robot"}
# Sidebar section labels, inserted before the first lesson of each later part.
PART_LABELS = {11: "Part 2: Robots that move and grab"}
PARTS = ["nav.html", "pill.html", "panel.html", "sandbox.html", "script.js"]


def load(n):
    d = LESSONS_DIR / DIRS[n] / "_fragments"
    if not d.is_dir():
        return None
    missing = [p for p in PARTS if not (d / p).exists()]
    if missing:
        sys.exit(f"lesson {n}: missing fragments {missing} in {d}")
    return {p: (d / p).read_text().rstrip() + "\n" for p in PARTS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, nargs="*")
    ap.add_argument("--out", default=str(LESSONS_DIR / "index.html"))
    a = ap.parse_args()
    s = (HERE / "index.base.html").read_text()
    wanted = a.only if a.only else sorted(DIRS)
    frags = {n: f for n in wanted if (f := load(n))}
    nums = [1, 2, 3] + sorted(frags)
    nav = pill = panel = sandbox = script = ""
    for n in sorted(frags):
        f = frags[n]
        if n in PART_LABELS:
            nav += f'    <p class="nav-label nav-part">{PART_LABELS[n]}</p>\n'
        nav += f["nav.html"]; pill += f["pill.html"]; panel += f"<!-- ~~~~~~~~ LESSON {n} ~~~~~~~~ -->\n" + f["panel.html"] + "\n"
        sandbox += f"<!-- ~~~~~~~~ SANDBOX {n} ~~~~~~~~ -->\n" + f["sandbox.html"] + "\n"
        script += f["script.js"] + "\n"
    rep = {
        "    <!-- @@NAV_INSERT -->\n": nav,
        "  <!-- @@PILL_INSERT -->\n": pill,
        "<!-- @@PANEL_INSERT -->\n": panel,
        "<!-- @@SANDBOX_INSERT -->\n": sandbox,
        "// @@SCRIPT_INSERT\n": script,
        "const lessons = [1, 2, 3]; // @@LESSONS": "const lessons = [" + ", ".join(map(str, nums)) + "];",
        '0 / 3</span><!-- @@PROGRESS -->': f"0 / {len(nums)}</span>",
        "      <!-- @@NEXT3 -->\n": (f'      <button class="btn btn-sm" type="button" onclick="switchLesson(4)">Next: {TITLES[4]}</button>\n' if 4 in frags else ""),
    }
    for k, v in rep.items():
        assert s.count(k) == 1, k
        s = s.replace(k, v)
    pathlib.Path(a.out).write_text(s)
    print(f"built {a.out} with lessons {nums}")


if __name__ == "__main__":
    main()
