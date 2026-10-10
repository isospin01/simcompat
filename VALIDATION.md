# Validation on real robots

This document answers one question: **does SimCompat do what it claims, on real robot models?** Every number below comes from [`validation/RESULTS.md`](validation/RESULTS.md), which [`validation/analyze.py`](validation/analyze.py) generates from the raw runs in [`validation/results/`](validation/results/). [`validation/README.md`](validation/README.md) explains how to reproduce them; the whole pipeline takes about 15 minutes on a 2-core machine.

Setup: SimCompat 0.2.0 with Pinocchio 4.1.0 (reference), MuJoCo 3.15.0, PyBullet 3.2.7 with its defaults, and PyBullet 3.2.7 with `URDF_USE_INERTIA_FROM_FILE`, on Python 3.11.

## Summary

| Claim | How it was tested | Result |
|---|---|---|
| It runs on real models, not only on fixtures | Every URDF model of robot_descriptions, each in its own process | 131 of 132 checked end to end; **0 crashes**; median **1.0 s** per robot for four engine configurations |
| Its ground truth (its own reading of the file) is right | Every link compared with an independent URDF library, yourdfpy | **4,012 of 4,012 links agree**: identical masses, positions within 1.4e-8 m |
| It catches real importer bugs and says where | 1,028 bugs of 12 classes taken from real issues, injected into MuJoCo's compiled models of 109 robots | **1,027 caught** by the intended check, at the right joint, as a problem of their own |
| Its explanations are right | Every difference must be attributed to a cause; each kind of inertia attribution re-tested by removing the cause | **0 of 904** differences unexplained; **214 of 214** inertia attributions confirmed |
| Correct imports really agree, so strict tolerances are fair | Mass-matrix error on models where an engine matches the file | median **1.2e-15** (MuJoCo), **2.5e-12** (PyBullet with inertia from file) |
| The result is useful | What it finds on public models | **66%** of models are simulated differently from the file by MuJoCo or PyBullet |
| The report is readable | Problems shown vs. raw differences found | median **2 problems** per report, from a median of **25** raw differences |

## The dataset

[robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py) packages the robot models that the community actually uses: 132 of its models have a URDF (plain or xacro). They span arms (Franka Panda and FR3, Universal Robots, KUKA iiwa, Kinova, xArm, Flexiv Rizon, Fanuc), quadrupeds (Unitree Go1/Go2/A1/B1/B2, ANYmal B/C/D, Mini Cheetah, HyQ, Solo, Minitaur), humanoids (Unitree G1/H1, Talos, iCub, ergoCub, Atlas, JAXON, Romeo, Valkyrie, Fourier GR1, Booster T1), hands (Allegro, LEAP, Ability, Barrett, Robotiq 2F-85) and mobile manipulators (Fetch, TIAGo, Stretch, HSR, PR2). The models were written by dozens of teams with different tools (SolidWorks exporters, xacro macros, hand-written URDF), which is exactly the variety an importer meets in practice. Nothing was selected or filtered.

## 1. It runs on real models

`validation/corpus.py` runs `simcompat check rd:<name>` on every model, each in its own process with a 15-minute timeout, with four engine configurations.

- **131 of 132** models were checked end to end. The one exception, Valkyrie, is a xacro file that uses a macro it never defines; simcompat reports it as an input error (exit code 2) that names the missing macro.
- **0** crashed checks, **0** crashed processes, **0** timeouts.
- The engines themselves are less robust than that: MuJoCo refuses 20 of the 131 models (17 for physically impossible inertia tensors), and PyBullet fails to answer some queries on the two largest humanoids (iCub and ergoCub). simcompat reports both as findings attributed to the engine, and still runs every other check.
- Median **1.0 s** per robot (90th percentile 3.0 s, maximum 10.7 s) for four engine configurations, including Python start-up.

## 2. Its ground truth is right

The import, inertia and kinematics checks compare each engine with simcompat's own reading of the file. If that reading were wrong, every verdict would be wrong, so it was checked against two independent implementations:

- `validation/oracle.py` compares every link of every model with **yourdfpy**, an independent URDF parser and forward-kinematics library: link masses, and link poses at three random configurations (mimic joints follow their leaders). All **4,012 links of 131 models** agree: masses are identical, positions differ by at most **1.4e-8 m** and orientations by at most 6.3e-8 rad (the resolution of the angle metric).
- **Pinocchio** is checked against simcompat's reading like any other engine, inside the corpus run: its mass properties and kinematics match on **131 of 131** models.

## 3. It catches real importer bugs and says where

`validation/mutations.py` is a seeded-bug benchmark. For each robot that both MuJoCo and Pinocchio load and that has movable joints (109 robots), it injects **one importer bug at a time** into MuJoCo's compiled model, which is exactly where importer bugs live, and checks it against Pinocchio. The 12 bug classes mirror bugs reported in the wild:

| Bug class | Injected | Mirrors | Caught by the intended check, at the right joint |
|---|---|---|---|
| link mass 5% off | 104 | wrong defaults (Go2/G1 cases) | 104/104 |
| COM moved 5 mm | 104 | inertial origin conversion | 104/104 |
| inertial rotation dropped | 98 | MuJoCo #3559 report | 98/98 |
| inertia tensor 10% off | 104 | PyBullet recomputing inertia | 103/104 |
| joint axis reversed | 109 | sign convention bugs | 109/109 |
| joint axis tilted 1° | 109 | axis conversion bugs | 109/109 |
| joint origin moved 1 cm | 109 | origin conversion bugs | 109/109 |
| `<dynamics damping>` dropped | 34 | Isaac Sim #841 | 34/34 |
| `<dynamics friction>` dropped | 14 | Isaac Sim #841 | 14/14 |
| 0.01 armature injected | 109 | Genesis #2398 | 109/109 |
| position limits dropped | 104 | | 104/104 |
| joints exposed breadth-first | 30 | Isaac Gym vs. Isaac Sim ordering | 30/30 |
| **all** | **1,028** | | **1,027 (99.9%)** |

A bug counts as caught only if a new WARN/FAIL appears that the clean model did not have, it comes from the check meant for that bug class, it names the mutated joint, and the report lists it as a problem of its own rather than only as a symptom. Every caught bug met all four conditions.

The one miss is a 10% change to the inertia of Pepper's `LHand`, a 2-milligram placeholder link whose whole tensor is 1.1e-9 kg·m². The change is 1.1e-10 kg·m², below the documented absolute floor of the inertia check (1e-9 kg·m²), which exists so that numerically negligible bodies do not produce noise.

## 4. Its explanations are right

A report is only useful if its causes are right. Two tests:

**Nothing is left unexplained.** Each dynamics or rollout difference must be attributed to a root-cause finding on the same engine, on the engine it was compared with, or in the file. Anything left over would be either a new kind of divergence or a false positive, and is listed for manual review. On the corpus, **904** such differences were attributed to a cause and **0** were left over. (A further 12, on iCub and ergoCub, belong to the PyBullet failure on those models: the checks that could explain them are the ones PyBullet could not answer.)

**Each kind of inertia attribution survives a counterfactual.** Inertia causes explain 871 of the 904 differences. For each kind of inertia cause simcompat attributed on the corpus, `validation/confirm.py` changes exactly what simcompat blames and re-runs:

| Engine | What simcompat blamed | Counterfactual | Confirmed |
|---|---|---|---|
| MuJoCo | links without `<inertial>` (MuJoCo derives their mass from geometry) | re-run with `inertiafromgeom=false` | 8/8 models |
| MuJoCo | links with zero mass but non-zero inertia (MuJoCo drops that inertia) | re-run with those links at 1e-9 kg | 1/1 |
| PyBullet, inertia from file | links without `<inertial>` (PyBullet assigns 1 kg) | engine mass − declared mass = exactly 1 kg per such link | 65/65 |
| PyBullet, inertia from file | impossible or degenerate inertia tensors (PyBullet sets them to zero) | re-run with those tensors made physical | 13/13 |
| PyBullet, default | inertia recomputed from collision shapes | the same model with `URDF_USE_INERTIA_FROM_FILE`: only the differences attributed to the file remain | 127/127 |
| **all** | | | **214/214** |

The remaining 33 differences are attributed to causes that are visible in the report itself rather than re-tested: a floating joint that PyBullet welds (12), links without `<inertial>` that leave a massless moving body Pinocchio cannot integrate (8), limits that engines read differently (6), mimic couplings (5) and others (2).

The validation also found two behaviours we had not anticipated, which led to new checks before this release: PyBullet zeroes the inertia of links whose tensor violates (or exactly meets) the triangle inequality, and MuJoCo drops the inertia of zero-mass links. Both are now named in the report, and both are among the confirmed attributions above.

## 5. Correct imports really agree

simcompat fails a model when an engine's mass matrix differs from Pinocchio's by more than 1e-6 (relative). That is only fair if engines that import a model correctly agree far more closely than that. On the models where an engine passes every core check:

| Engine | Models where it matches the file | M(q) relative error vs. Pinocchio: median | max |
|---|---|---|---|
| MuJoCo | 92 of 111 | 1.2e-15 | 5.5e-8 |
| PyBullet, inertia from file | 46 of 131 | 2.5e-12 | 3.1e-8 |
| PyBullet, default | 2 of 131 | | |

Correct imports agree to between 1e-15 and 1e-8, so a difference above 1e-6 is an import problem, not numerical noise. (PyBullet with its defaults matches almost no model because it recomputes inertia from collision shapes; that is a documented default, which simcompat reports with the flag that turns it off.)

## 6. What it finds on public robots

On the 131 models it could check, simcompat finds that **66%** are simulated differently from their file by MuJoCo or by PyBullet with inertia from file. The main causes, all attributed and confirmed above:

- **Missing `<inertial>` (67 models).** The URDF specification says such a link has zero mass. MuJoCo derives a mass from the collision geometry, PyBullet assigns 1 kg, and Pinocchio follows the specification. On the Go2, a calf weighs 0.194 kg in the file, 0.241 kg in MuJoCo and 2.194 kg in PyBullet.
- **Impossible or degenerate inertia tensors (23 models).** MuJoCo refuses to load 17 of these models (ANYmal C, Talos and Fetch among them), PyBullet silently zeroes the tensors, and Pinocchio keeps them.
- **Joint order (52 models, 40%).** MuJoCo and PyBullet number joints depth-first in file order; Pinocchio, like other urdfdom-based tools, sorts siblings by name. A policy that indexes joints by position moves the wrong legs. The report names each engine's convention.
- **Friction that is read but not simulated (19 models).** PyBullet stores `<dynamics friction>` but `loadURDF` never applies it. simcompat finds this by comparing PyBullet with itself, with friction on and off.
- **Mimic joints (24 models).** MuJoCo enforces `<mimic>` as an equality constraint; PyBullet leaves the joint independent, so grippers and hands move differently.
- **Smaller classes.** A floating joint becomes a free joint in MuJoCo and Pinocchio but a fixed one in PyBullet (3 models). A range written on a continuous joint is enforced by MuJoCo and PyBullet but ignored by Pinocchio and by the specification (2 models). A zero-width range (lower == upper, 8 models) locks the joint in PyBullet and leaves it free elsewhere.

## 7. The report is readable

A tool that is right but unreadable does not help. The raw comparison on a public model produces a median of **25** WARN/FAIL/ERROR differences (up to 168). Grouped by cause, a report shows a median of **2** problems (up to 7), each with its fix. Every difference is still available, with `-v` or in the HTML and JSON reports.

## Limits of this validation

- **Engines.** Three engines, CPU only. The seeded bugs are injected into MuJoCo's model only, so the benchmark measures the checks, not each engine adapter. The adapters are covered by `tests/test_engines.py` and by the corpus run itself (Pinocchio matches the file on every model).
- **Scope.** Fixed base, no contact, no actuators: the deterministic layer. Contact and actuator tiers will need statistical validation of their own.
- **Ground truth.** It is simcompat's reading of the URDF specification. Where the specification is ambiguous (a zero-width range, a range on a continuous joint), simcompat reports how each engine reads the file instead of declaring one of them wrong.
- **Corpus.** One public corpus, as of October 2026. Private CAD exports may contain defects this corpus does not.

## Reproduce

```bash
pip install -e ".[all]" yourdfpy
python validation/corpus.py --out validation/results/corpus
python validation/oracle.py --out validation/results/oracle.jsonl
python validation/confirm.py --corpus validation/results/corpus --out validation/results/confirm.jsonl
python validation/mutations.py --out validation/results/mutations.jsonl
python validation/analyze.py        # -> validation/RESULTS.md
```
