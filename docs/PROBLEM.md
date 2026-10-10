# The problem simcompat solves

*Research notes, October 2026. "Verified" means we read the primary source or reproduced it ourselves. "Reported" means it was taken from the linked thread.*

## 1. Problem statement

> The same robot description file produces different simulations in different engines, silently.
> No tool lets a user ask, for **their own** model and **in CI**: *"what does each engine actually simulate, where does it disagree with the file, and where do engines disagree with each other?"*

Each engine's importer interprets URDF/MJCF/USD with its own:

- defaults for missing data
- unsupported elements, which are dropped
- traversal order for joints
- runtime defaults, such as motors, gravity and damping
- regressions, which change between releases

None of these raise errors. Users discover them weeks later, when a policy that works in engine A falls over in engine B.

## 2. Evidence that the pain is real

### Importer-level divergences

| Source | What | Status |
|---|---|---|
| [Isaac Sim #841](https://github.com/isaac-sim/IsaacSim/issues/841) | Isaac Sim 6.1: the URDF converter writes `newton:damping/friction` but the importer reads `urdf:dynamics:*`, so drive damping and joint friction are silently dropped. The unit test hand-wrote the old names and missed it. | Verified (maintainer confirmed as a regression) |
| [Genesis #2398](https://github.com/Genesis-Embodied-AI/genesis-world/discussions/2398) | Open-loop divergence from MuJoCo with matched gains and timestep. Root cause: Genesis applies a default armature (fix: `default_armature=0.0`). Feb 12 to Mar 21, 2026. | Verified |
| [Isaac Lab #4383](https://github.com/isaac-sim/IsaacLab/issues/4383) | A custom joint order was not honoured on one code path: "the robot immediately falls over in Mujoco". The official advice is still a joint-mapping file. | Verified |
| [humanoid-locomotion-sim2sim](https://github.com/earosenfeld/humanoid-locomotion-sim2sim) | The G1 armature is 0.01 in MuJoCo Menagerie and 0.03 in the Isaac Lab asset. Speed at a 1.0 m/s command went from 0.63 to 0.80 m/s once matched. Joint order had to be remapped by name. | Verified |
| [MuJoCo #3559](https://github.com/google-deepmind/mujoco/issues/3559) | Reported: URDF `<inertial><origin rpy>` rotation lost during compilation. We could **not** reproduce this on MuJoCo 3.15.0 with child-link, floating and root variants. simcompat's inertia check would catch it. | Reported |
| [PyBullet quickstart guide](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html) | Inertia is recomputed from collision shapes unless `URDF_USE_INERTIA_FROM_FILE` is set; every joint gets a velocity motor by default; the base floats by default; gravity defaults to 0 | Verified by experiment |
| [Isaac Sim gym-transfer guide](https://docs.isaacsim.omniverse.nvidia.com/2023.1.1/isaac_gym_tutorials/tutorial_gym_transfer_policy.html) | Isaac Gym orders joints depth-first and Isaac Sim breadth-first; stiffness is per degree in one and per radian in the other | Reported (NVIDIA docs) |
| [Isaac Lab PhysX↔Newton guide](https://isaac-sim.github.io/IsaacLab/develop/source/how-to/transfer_policies_between_physx_and_newton.html) | Lists the mismatches: joint/body ordering, mimic joints (PhysX drives both fingers, MJWarp only the leader), armature, drive formulation, friction model. **Validation is manual**: run the train/play matrix. | Verified |
| [MuJoCo XML reference](https://github.com/google-deepmind/mujoco/blob/main/doc/XMLreference.rst) | URDF-specific defaults: `fusestatic` and `discardvisual` are true, velocity limits are dropped, effort maps to `actfrcrange` | Verified by experiment |

### What simcompat found on public models (MuJoCo 3.15.0, PyBullet 3.2.7, Pinocchio 4.1.0)

All 132 URDF models of robot_descriptions; see [VALIDATION.md](../VALIDATION.md) for the numbers and [`examples/matrix.md`](../examples/matrix.md) for a dozen popular robots. Highlights:

- **Missing `<inertial>` gives three answers.** MuJoCo derives mass from geometry, PyBullet uses 1 kg and Pinocchio uses 0 (67 of 131 models have such a moving link). On the Go2 a calf weighs 0.194 kg in the file, 0.241 kg in MuJoCo and 2.194 kg in PyBullet.
- **Impossible inertias get three answers too.** MuJoCo refuses to load the model (17 models, including ANYmal C, Talos and Fetch), PyBullet with `URDF_USE_INERTIA_FROM_FILE` silently sets that link's inertia to zero, and Pinocchio keeps it. MuJoCo also drops the inertia of a link whose mass is zero (Bolt's feet).
- **Joint order splits in two.** MuJoCo and PyBullet use file order; Pinocchio, being urdfdom-based, sorts siblings by name. Engines disagree on 40% of models, including Go1, A1, Allegro, Talos and ANYmal C.
- **PyBullet ignores `<dynamics friction>`.** It reads the value, but the value has no effect (19 models, including A1, Allegro, ANYmal D and Franka FR3).
- **Limits on continuous joints.** The URDF spec ignores lower/upper on a continuous joint and so does Pinocchio, but MuJoCo and PyBullet enforce them (Kinova Gen2, JAXON).

### Cost signals

- **sim2sim is treated as a gate before hardware.** Humanoid-Gym ships a sim2sim tool ([paper](https://arxiv.org/html/2404.05695v1)), and teams write "validate the trained policy in mjlab before sim2real" ([example](https://github.com/WATonomous/humanoid/issues/181)).
- **Per-robot glue gets rewritten.** The [`sim2sim` GitHub topic](https://github.com/topics/sim2sim) is mostly per-robot deploy scripts, each re-implementing joint mapping and gains by hand.
- **Root causes take weeks.** Genesis #2398 took about 5 weeks. [Isaac Lab #2637](https://github.com/isaac-sim/IsaacLab/discussions/2637) and [HybridRobotics motion_tracking_controller #9](https://github.com/HybridRobotics/motion_tracking_controller/issues/9) were left without a root cause.

## 3. The landscape: what exists and why it does not close the gap

| Tool | What it does | Why it is not the answer |
|---|---|---|
| Static URDF validators (urdf_validator, RoboInfra, LinkForge, `check_urdf`) | Check the file: structure, inertia sanity, reach | Never ask an engine what it built |
| Converters (urdf2any, urdf-usd-converter, mujoco-usd-converter, RoboVerse scripts, mujoco_ros2_control) | Produce MJCF or USD | Produce files; at best compare against one importer's summary |
| [urdf-usd-bridge](https://github.com/fermarc98/urdf-usd-bridge) | Repairs USD physics data; reports a 14-robot × 3-backend hold-pose benchmark | USD/Isaac only, needs an NVIDIA GPU, v0.1 |
| Isaac Sim Asset Validator (SimReady rules) | Rules for USD assets; some run one PhysX step | Single backend; USD only |
| Engine parity tests (Newton `test_menagerie_mujoco.py`, Genesis `mujoco_parity.py`) | Compare their importer with MuJoCo on curated models | Internal to the engine; not usable on your robot |
| [gaitkeeper](https://github.com/dundysm/gaitkeeper) | Classifies policy-level sim2sim failures | Runs after training; diagnoses symptoms, not the import |
| robot_descriptions, Menagerie tests | Check that models load or step | Do not compare dynamics across engines |

**The gap** is an engine-neutral, CPU-friendly, `pip install`-able tool. It would load *any* user model into several engines and compare what each one built against the file at the level that *must* match (parameters, inertia, kinematics, M, g, C, contact-free rollouts). It would also separate import errors from legitimate solver differences, and run in CI.

## 4. Why it is worth solving

1. **The errors are silent.** They are not exceptions: a robot that is 20% heavier at the calves still walks, just differently.
2. **They are expensive to find late.** By the time sim2sim fails, the suspects are the policy, gains, timestep, observation normalization and the model. simcompat removes the model from that list in seconds, before training.
3. **Exposure is growing.** Multi-backend is now the default path: Isaac Lab runs PhysX and Newton, and mjlab, Genesis, MuJoCo Playground and Newton's multiple solvers are all in use. Each new backend multiplies the number of importers a model must survive.
4. **Importers regress.** Isaac Sim 6.1 #841 shows that behaviour changes between engine releases. A regression test pinned to engine versions catches this; a one-time manual check does not.
5. **The hard part is tractable.** Contact differences are genuinely irreducible, but import errors are not. Contact-free rigid-body dynamics from a correct import agrees across independent engines to 1e-11 to 1e-15 (measured). A strict tolerance therefore turns "the engines behave differently" into a precise, attributable defect.

## 5. Scope decisions

- **Input: URDF (and xacro).** It is still the lingua franca and the source most other formats are converted from.
- **Engines: MuJoCo and PyBullet as simulators, Pinocchio as the analytical reference.** All of them are pip-installable and CPU-only, so simcompat runs in any CI. Isaac (GPU, heavy install) comes next as an optional backend.
- **Fixed base, no contact, no actuators.** This covers the deterministic layer, where every mismatch is a bug. Contact and actuator tiers are on the roadmap and will report statistics rather than pass/fail.
- **Report runtime harmonization explicitly** (PyBullet's motors, gravity and link damping) so that users see both the default behaviour and the comparison.
