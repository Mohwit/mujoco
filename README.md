# MuJoCo Learning Lab

A hands-on course for learning the [MuJoCo](https://mujoco.org) physics simulator, from your first
pendulum to a warehouse robot that picks things up with a learned policy.

**Live site:** https://mohwit.github.io/mujoco/

Each lesson pairs a plain-language explainer with an interactive in-browser sandbox and a Python
script. Every number shown on the site comes from running that lesson's script with MuJoCo 3.15.0.

## Curriculum

**Part 1: Foundations**

| # | Lesson |
|---|---|
| 01 | MJCF hierarchy |
| 02 | Frames and axes |
| 03 | State and stepping |
| 04 | Inspection and visualization |
| 05 | Actuators and control |
| 06 | Sensors and observation |
| 07 | Contact and constraints |
| 08 | Controllers and experiments |
| 09 | Environments and next steps |
| C1 | Capstone 1: Balance a cart-pole |

**Part 2: Robots that move and grab**

| # | Lesson |
|---|---|
| 11 | Building bigger models |
| 12 | Kinematics and Jacobians |
| 13 | Grippers and grasping |
| 14 | Mobile bases |
| 15 | Learning a policy |
| C2 | Capstone 2: Warehouse robot |

## Repository layout

```
lessons/
  index.html             # the course site (generated, do not edit by hand)
  NN-topic/              # one folder per lesson: scripts, notes, images
    _fragments/          # this lesson's piece of index.html (lessons 4-16)
  _build/
    build.py             # assembles index.html from the base page + fragments
    index.base.html      # site shell, shared CSS/JS, and lessons 1-3
    check.cjs            # headless Playwright check of one lesson
    CURRICULUM-PLAN.md   # course design notes
hello_mujoco.py          # minimal pendulum in the native MuJoCo viewer
design/                  # early layout previews
tracking/                # personal learning progress
```

## Running the lesson scripts

Requires Python 3.10+.

```bash
python -m venv .venv
source .venv/bin/activate
pip install mujoco numpy gymnasium stable-baselines3 torch

python hello_mujoco.py                                        # opens the MuJoCo viewer
python lessons/05-actuators-and-control/05_actuators_and_control.py
```

`gymnasium`, `stable-baselines3` and `torch` are only needed for lessons 9, 15 and Capstone 2.
On macOS, scripts that open the interactive viewer (`launch_passive`) must be run with `mjpython`
instead of `python`.

## Editing the site

`lessons/index.html` is generated. To change it:

- Lessons 1-3, shared styles or shared JavaScript: edit `lessons/_build/index.base.html`.
- Lessons 4-16: edit the files in `lessons/NN-topic/_fragments/`.

Then rebuild and open the page:

```bash
python lessons/_build/build.py
open lessons/index.html
```

Preview a single lesson with `python lessons/_build/build.py --only 5 --out preview.html`.

## Deployment

Pushing to `main` runs `.github/workflows/pages.yml`, which rebuilds `index.html`, publishes the
`lessons/` folder (without `_build/` and `_fragments/`) to GitHub Pages, and updates the live site
in about a minute.
