# SimCompat

[![ci](https://github.com/isospin01/simcompat/actions/workflows/ci.yml/badge.svg)](https://github.com/isospin01/simcompat/actions/workflows/ci.yml) · [中文说明](README.zh-CN.md)

**Does every physics engine simulate what your robot file declares?**

A robot description is not a simulation. Each engine's importer turns your URDF into *its own* model. Along the way it applies defaults, drops fields and occasionally gets things wrong, all without an error. SimCompat loads one file into several engines (MuJoCo, PyBullet and Pinocchio today), reads back what each one actually built, and tells you in plain language, with the fix, where an engine simulates something different from your file.

```bash
pip install "simcompat[all] @ git+https://github.com/isospin01/simcompat"    # Python 3.10-3.12
simcompat check path/to/robot.urdf      # or robot.urdf.xacro, a package directory, or rd:go2
```

```text
SimCompat 0.2.0 · go2 · 12 movable joints
Engines: pinocchio 4.1.0 (reference) · mujoco 3.15.0 · pybullet 3.2.7
Input: robot_descriptions model go2_description

✗ FAIL  Some engines simulate something different from what the file declares.

1. ✗ Engines simulate different mass, center of mass or inertia than the file declares
   pybullet  FL_calf_joint   mass 2.194 kg vs declared 0.194 kg; COM off by 30.9 mm; inertia tensor rel. error 0.681
   mujoco    FL_calf_joint   mass 0.241352 kg vs declared 0.194 kg; COM off by 6.51 mm; inertia tensor rel. error 0.0515
   ... and 14 more (all with --verbose)
   Fix: These bodies include links without <inertial>, for which each engine substitutes its own default mass ...
   Fix: PyBullet's loadURDF ignores URDF inertia tensors by default ... (simcompat: -b pybullet:inertia_from_file=1).
   Also explains: dynamics.mass_matrix (mujoco, pybullet), dynamics.gravity (mujoco, pybullet), rollout.rigid ...
   More: simcompat explain inertia.clusters

2. ! Moving links have no <inertial>, so each engine invents its own mass for them
   file  FL_calflower   moved by 'FL_calf_joint' but has no <inertial>
   ...

Checks: File ! · Import i · Inertia ✗ · Kinematics ✓ · Dynamics ✗ · Rollout ✗   (✓ ok · i note · ! warning · ✗ differs)
```

Problems are grouped by **cause**: a dynamics or rollout difference that an inertia or import problem explains is listed under that problem ("Also explains"), not as noise of its own. On public robots, a report shows a median of 2 problems where the raw comparison has 25 differences. The exit code is `0` if every engine matches the file, `1` if one does not (`--strict` also fails on warnings) and `2` for a mistake in the input or options, so it can gate CI before you train a policy on a model that one engine silently changed.

## Everyday use

| You want to... | Run |
|---|---|
| Check a robot | `simcompat check robot.urdf` |
| Get a report to share | `simcompat check robot.urdf -o report.html` (also `.md`, `.json`, `.txt`) |
| Check a xacro file (no ROS needed) | `simcompat check robot.urdf.xacro --xacro-arg prefix:=left_` |
| Try it on public robots | `simcompat robots unitree`, then `simcompat check rd:go2` |
| Understand a finding | `simcompat explain inertia.clusters` (or `simcompat explain` to list all checks) |
| Run part of the checks | `--only dynamics`, `--skip rollout`, `--only 'import.*'` |
| Add a CI gate to your repository | `simcompat init`: writes `simcompat.toml` and `.github/workflows/simcompat.yml` |
| Compare engine configurations | `simcompat check robot.urdf -b mujoco -b pybullet -b pybullet:inertia_from_file=1` |
| Check a fleet | `simcompat matrix rd:go2 rd:g1 my_arm.urdf -o matrix.html` |
| See every finding and the joint ordering | add `-v` |

Inputs are resolved for you: `package://` mesh URIs are found by walking up to the package (`--package-dir NAME=PATH` if needed), xacro is expanded with `xacrodoc` (no ROS installation needed), and a directory is searched for its single URDF. Mistakes are caught before anything runs, with a suggestion: `-b mujoko` asks "Did you mean 'mujoco'?", `rd:go22` suggests `rd:go2`, and a typo in `simcompat.toml` names the file and the key.

A project profile (`simcompat init` writes a commented one) sets tolerances and re-grades checks, for example `"import.joint_order" = "info"` if your code maps joints by name. Re-grading a cause also re-grades what it explains: with `"inertia.clusters" = "warn"`, the dynamics and rollout differences that the inertia differences cause are warnings too, so a known issue stops failing the run.

From Python:

```python
import simcompat

report = simcompat.check("robot.urdf", backends=["pinocchio", "mujoco"])
report.ok                                  # False if an engine differs from the file
for problem in report.problems:            # grouped by cause, worst first
    print(problem.severity, problem.headline, problem.engines, problem.hints)
report.save("report.html")
```

In GitHub Actions, either run `simcompat init` (it writes a workflow) or use the action directly; the report lands in the job summary and the job fails when an engine differs from the file:

```yaml
- uses: actions/checkout@v4
- uses: isospin01/simcompat@v0.2.0
  with:
    urdf: path/to/robot.urdf.xacro
    args: --profile simcompat.toml
```

`simcompat.check` takes the same options as the command line (`backends`, `reference`, `package_dirs`, `xacro_args`, `config`, `only`, `skip`); see its docstring. [`examples/`](examples/) has real reports (HTML, Markdown, JSON, terminal) and a compatibility matrix of popular robots.

## What it checks

| Tier | What is compared | Against |
|---|---|---|
| File | links without `<inertial>`, impossible inertia, ambiguous joint ranges, non-unit axes, unresolved meshes | (properties of the file that predict divergence) |
| Import | load, degrees of freedom, **joint order** (with the traversal convention each engine follows), floating joints, limits, effort/velocity, damping/friction as read, injected armature, mimic couplings | the file |
| Inertia | composite mass, COM and inertia of everything each joint moves (fixed children merged, so fusing engines compare fairly) | the file |
| Kinematics | child-link pose and joint-axis direction at sampled configurations (catches reversed axes) | the file |
| Dynamics | M(q), g(q), C(q,q̇)q̇, fixed base, no contact | Pinocchio (reference) |
| Rollout | zero-torque rollouts; **does declared damping/friction actually act** (each engine compared with itself) | reference / first simulator |

Design choices:

1. **The file is the ground truth.** simcompat parses the URDF itself, without any engine, so "what the file says" is never contaminated by an importer.
2. **Strict where physics is deterministic.** Contact-free rigid-body dynamics from a correct import agree across engines to about 1e-12, so a FAIL means an import problem, not "engines differ". Integrated behaviour with damping gets a looser tolerance and is graded WARN.
3. **Causes, not symptoms.** Findings are attributed to a joint, grouped by cause, ordered largest first, and come with a fix.
4. **Nothing hidden.** Every runtime default simcompat harmonizes is listed in the report. For PyBullet that means its velocity motors, zero gravity, link damping and 100 rad/s speed clamp.

## Validated on real robots

SimCompat was run on **every URDF model in [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py)**: 132 arms, quadrupeds, humanoids, hands and mobile manipulators. [VALIDATION.md](VALIDATION.md) has the method, the numbers and how to reproduce them. In short:

| Question | Result |
|---|---|
| Does it run on real models? | 131 of 132 checked end to end, with 0 crashes, in a median 1.0 s per robot with four engine configurations. The one exception is a model whose xacro is broken upstream, reported as an input error. |
| Is its ground truth right? | Its URDF parser matches an independent library (yourdfpy) on all 4,012 links: identical masses, positions within 1.4e-8 m. Pinocchio matches it on 131 of 131 models. |
| Does it catch real importer bugs? | 1,027 of 1,028 bugs injected into MuJoCo's compiled models of 109 robots were caught by the intended check, at the right joint, as a problem of their own. The bug classes come from real issues: wrong mass, COM or inertia, lost inertial rotation, reversed or tilted axis, shifted origin, dropped damping, friction or limits, injected armature, breadth-first ordering. The one miss is below the 1e-9 kg·m² inertia floor, on a 2 mg placeholder link. |
| Are its explanations right? | All 904 dynamics and rollout differences on public models are attributed to a cause; none is left unexplained. The inertia causes behind 871 of them were tested by changing what simcompat blames and re-running: the difference disappeared in 214 of 214 cases. |
| Is it useful? | MuJoCo or PyBullet simulates something different from the file for 66% of public models. MuJoCo refuses 20 of them, engines number the joints differently for 40%, and PyBullet ignores the declared joint friction of 19. |

## Why this exists

sim2sim (for example Isaac Lab → MuJoCo) is a standard gate before hardware, and multi-backend training is now normal. When a policy fails in another engine, the cause is often that the file was *read differently*, not physics. These failures are silent and slow to find:

| Case | What happened |
|---|---|
| [Isaac Sim #841](https://github.com/isaac-sim/IsaacSim/issues/841) | Isaac Sim 6.1 silently dropped URDF damping and friction (regression confirmed by NVIDIA) |
| [Genesis #2398](https://github.com/Genesis-Embodied-AI/genesis-world/discussions/2398) | A Genesis-vs-MuJoCo divergence took about 5 weeks to trace to a default armature |
| [Isaac Lab #4383](https://github.com/isaac-sim/IsaacLab/issues/4383) | A custom joint order was not honoured: "the robot immediately falls over in Mujoco" |
| [humanoid-locomotion-sim2sim](https://github.com/earosenfeld/humanoid-locomotion-sim2sim) | Matching the G1's armature between Menagerie (0.01) and Isaac Lab (0.03) moved tracked speed from 0.63 to 0.80 m/s |

Engine teams run parity tests on their own importer and curated models; URDF linters check the file, not what an engine builds from it. [docs/PROBLEM.md](docs/PROBLEM.md) has the full landscape.

## Engines

| Backend | Role | Options (`-b name:option=value`, listed by `simcompat backends`) |
|---|---|---|
| `pinocchio` | reference (analytical CRBA/RNEA/ABA), itself checked against the file | none |
| `mujoco` | simulator (built-in URDF importer) | `fusestatic`, `discardvisual`, `balanceinertia`, `inertiafromgeom`, `boundmass`, `boundinertia` |
| `pybullet` | simulator (`loadURDF`) | `inertia_from_file`, `merge_fixed_links` |

Planned: Isaac Sim/Lab (PhysX and Newton), Genesis, Drake and Gazebo; see [ROADMAP.md](ROADMAP.md). An engine is one class with about ten small methods, and a separate package can add one through an entry point; see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#adding-a-backend).

## Limitations

- **Inputs:** URDF and xacro only; MJCF and USD inputs are planned.
- **Fixed base:** comparisons are fixed-base. Floating joints are reported (`import.multi_dof`) rather than compared.
- **Not compared yet:** contact, actuators and PD controllers. These are the next tiers.
- **MuJoCo meshes:** MuJoCo cannot read `.dae` collision meshes. simcompat drops them for MuJoCo with a warning; this only matters for links without `<inertial>`.

## Contributing

```bash
git clone https://github.com/isospin01/simcompat && cd simcompat
pip install -e ".[dev]" && pytest && ruff check .
```

Read [CONTRIBUTING.md](CONTRIBUTING.md), then [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), a ten-minute tour of the code. The test suite includes an engine-free fake backend with injectable importer bugs, and engine tests that pin every engine behaviour this README claims. Apache-2.0.
