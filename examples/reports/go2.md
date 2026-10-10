# SimCompat report: `go2`

**❌ FAIL**: Some engines simulate something different from what the file declares.

2 problems · findings: 24 fail, 9 warn, 6 info · engines: pinocchio 4.1.0 (reference), mujoco 3.15.0, pybullet 3.2.7 · simcompat 0.2.0 · 2026-10-10T01:22:22+00:00

Input `rd:go2` · 42 links · 41 joints (12 movable)
- robot_descriptions model go2_description

## Problems

### 1. ❌ Engines simulate different mass, center of mass or inertia than the file declares

`inertia.clusters` · pybullet, mujoco

| Engine | Joint / link | Finding |
|---|---|---|
| pybullet | FL_calf_joint | mass 2.194 kg vs declared 0.194 kg; COM off by 30.9 mm; inertia tensor rel. error 0.681 |
| pybullet | FR_calf_joint | mass 2.194 kg vs declared 0.194 kg; COM off by 30.9 mm; inertia tensor rel. error 0.683 |
| pybullet | RL_calf_joint | mass 2.194 kg vs declared 0.194 kg; COM off by 30.9 mm; inertia tensor rel. error 0.683 |
| pybullet | RR_calf_joint | mass 2.194 kg vs declared 0.194 kg; COM off by 30.9 mm; inertia tensor rel. error 0.683 |
| mujoco | FL_calf_joint | mass 0.241352 kg vs declared 0.194 kg; COM off by 6.51 mm; inertia tensor rel. error 0.0515 |
| mujoco | FR_calf_joint | mass 0.241352 kg vs declared 0.194 kg; COM off by 6.51 mm; inertia tensor rel. error 0.0515 |
| | | ... and 10 more (all in the HTML or JSON report) |

- **Fix:** These bodies include links without &lt;inertial&gt;, for which each engine substitutes its own default mass (the links are listed under file.inertial). Add explicit &lt;inertial&gt; blocks.
- **Fix:** PyBullet's loadURDF ignores URDF inertia tensors by default and recomputes them from collision shapes. Pass flags=URDF_USE_INERTIA_FROM_FILE (simcompat: -b pybullet:inertia_from_file=1).
- Also explains: dynamics.mass_matrix (mujoco, pybullet), dynamics.gravity (mujoco, pybullet), dynamics.coriolis (mujoco, pybullet), rollout.rigid (mujoco, pybullet), rollout.passive (pybullet)

### 2. ⚠️ Moving links have no &lt;inertial&gt;, so each engine invents its own mass for them

`file.inertial` · file

| Engine | Joint / link | Finding |
|---|---|---|
| file | FL_calflower | moved by 'FL_calf_joint' but has no &lt;inertial&gt; |
| file | FL_calflower1 | moved by 'FL_calf_joint' but has no &lt;inertial&gt; |
| file | FR_calflower | moved by 'FR_calf_joint' but has no &lt;inertial&gt; |
| file | FR_calflower1 | moved by 'FR_calf_joint' but has no &lt;inertial&gt; |
| file | RL_calflower | moved by 'RL_calf_joint' but has no &lt;inertial&gt; |
| file | RL_calflower1 | moved by 'RL_calf_joint' but has no &lt;inertial&gt; |
| | | ... and 2 more (all in the HTML or JSON report) |

- **Fix:** Add an explicit &lt;inertial&gt; so every engine uses the same mass.

<details><summary><b>All checks</b></summary>

| Tier | Check | file | pinocchio 4.1.0 (reference) | mujoco 3.15.0 | pybullet 3.2.7 |
|---|---|---|---|---|---|
| File | Moving links without &lt;inertial&gt; <sub>`file.inertial`</sub> | ⚠️ WARN |  |  |  |
| File | Physically valid inertia <sub>`file.inertia_validity`</sub> | ✅ PASS |  |  |  |
| File | Ambiguous joint ranges <sub>`file.limits`</sub> | ✅ PASS |  |  |  |
| File | Unit joint axes <sub>`file.axis`</sub> | ✅ PASS |  |  |  |
| File | Collision mesh resolution <sub>`file.meshes`</sub> | ✅ PASS |  |  |  |
| File | Joint types covered <sub>`file.joint_types`</sub> | ✅ PASS |  |  |  |
| Import | Model loads <sub>`import.load`</sub> |  | ✅ PASS | ✅ PASS | ℹ️ INFO |
| Import | Degrees of freedom <sub>`import.dofs`</sub> |  | ✅ PASS | ✅ PASS | ✅ PASS |
| Import | Joint ordering <sub>`import.joint_order`</sub> | ℹ️ INFO |  |  |  |
| Import | Floating and planar joints <sub>`import.multi_dof`</sub> | ⏭️ SKIP |  |  |  |
| Import | Default base mode <sub>`import.base_mode`</sub> |  | ℹ️ INFO | ℹ️ INFO | ℹ️ INFO |
| Import | Position limits <sub>`import.limits`</sub> |  | ✅ PASS | ✅ PASS | ✅ PASS |
| Import | Effort and velocity limits <sub>`import.effort_velocity`</sub> |  | ✅ PASS | ℹ️ INFO | ✅ PASS |
| Import | Joint damping and friction (as read) <sub>`import.damping_friction`</sub> |  | ✅ PASS | ✅ PASS | ✅ PASS |
| Import | Engine-injected armature <sub>`import.armature`</sub> |  | ✅ PASS | ✅ PASS | ✅ PASS |
| Import | Mimic joints <sub>`import.mimic`</sub> | ⏭️ SKIP |  |  |  |
| Inertia | Mass, COM and inertia per moving body <sub>`inertia.clusters`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Kinematics | Link frames and joint axes <sub>`kinematics.frames`</sub> |  | ✅ PASS | ✅ PASS | ✅ PASS |
| Dynamics | Joint-space mass matrix M(q) <sub>`dynamics.mass_matrix`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Dynamics | Gravity torques g(q) <sub>`dynamics.gravity`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Dynamics | Coriolis/centrifugal forces C(q, qd) qd <sub>`dynamics.coriolis`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Rollout | Rigid-body rollout <sub>`rollout.rigid`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Rollout | Declared damping/friction actually act <sub>`rollout.passive_effect`</sub> | ⏭️ SKIP |  |  |  |
| Rollout | Passive rollout across simulators <sub>`rollout.passive`</sub> |  |  | ✅ PASS | ⚠️ WARN |

</details>

<details><summary><b>Joint ordering</b></summary>

Index policy and controller vectors by joint **name**. **pinocchio** follows depth-first, file order = depth-first, siblings sorted by name; **mujoco** follows depth-first, file order = depth-first, siblings sorted by name; **pybullet** follows depth-first, file order = depth-first, siblings sorted by name.

| # | declared | depth-first | depth-first (sorted) | breadth-first | pinocchio | mujoco | pybullet |
|---|---|---|---|---|---|---|---|
| 0 | `FL_hip_joint` | `FL_hip_joint` | `FL_hip_joint` | `FL_hip_joint` | `FL_hip_joint` | `FL_hip_joint` | `FL_hip_joint` |
| 1 | `FL_thigh_joint` | `FL_thigh_joint` | `FL_thigh_joint` | `FR_hip_joint` | `FL_thigh_joint` | `FL_thigh_joint` | `FL_thigh_joint` |
| 2 | `FL_calf_joint` | `FL_calf_joint` | `FL_calf_joint` | `RL_hip_joint` | `FL_calf_joint` | `FL_calf_joint` | `FL_calf_joint` |
| 3 | `FR_hip_joint` | `FR_hip_joint` | `FR_hip_joint` | `RR_hip_joint` | `FR_hip_joint` | `FR_hip_joint` | `FR_hip_joint` |
| 4 | `FR_thigh_joint` | `FR_thigh_joint` | `FR_thigh_joint` | `FL_thigh_joint` | `FR_thigh_joint` | `FR_thigh_joint` | `FR_thigh_joint` |
| 5 | `FR_calf_joint` | `FR_calf_joint` | `FR_calf_joint` | `FR_thigh_joint` | `FR_calf_joint` | `FR_calf_joint` | `FR_calf_joint` |
| 6 | `RL_hip_joint` | `RL_hip_joint` | `RL_hip_joint` | `RL_thigh_joint` | `RL_hip_joint` | `RL_hip_joint` | `RL_hip_joint` |
| 7 | `RL_thigh_joint` | `RL_thigh_joint` | `RL_thigh_joint` | `RR_thigh_joint` | `RL_thigh_joint` | `RL_thigh_joint` | `RL_thigh_joint` |
| 8 | `RL_calf_joint` | `RL_calf_joint` | `RL_calf_joint` | `FL_calf_joint` | `RL_calf_joint` | `RL_calf_joint` | `RL_calf_joint` |
| 9 | `RR_hip_joint` | `RR_hip_joint` | `RR_hip_joint` | `FR_calf_joint` | `RR_hip_joint` | `RR_hip_joint` | `RR_hip_joint` |
| 10 | `RR_thigh_joint` | `RR_thigh_joint` | `RR_thigh_joint` | `RL_calf_joint` | `RR_thigh_joint` | `RR_thigh_joint` | `RR_thigh_joint` |
| 11 | `RR_calf_joint` | `RR_calf_joint` | `RR_calf_joint` | `RR_calf_joint` | `RR_calf_joint` | `RR_calf_joint` | `RR_calf_joint` |

</details>

<details><summary><b>Notes</b> (documented conventions)</summary>

- **import.load** (pybullet): loaded (3.2.7); 1 importer message(s)
- **import.joint_order** (file): the tree is branched, so traversal conventions disagree (for example breadth-first, documented for Isaac Sim/PhysX, vs. depth-first); see the ordering table
- **import.base_mode** (pinocchio): default loader gives a fixed base
- **import.base_mode** (mujoco): default loader gives a fixed base
- **import.base_mode** (pybullet): default loader gives a floating base
- **import.effort_velocity** (mujoco): velocity limits are not represented by this engine (12 joint(s))

</details>

<details><summary><b>Engines and settings</b></summary>

**pinocchio 4.1.0 (reference)**

- simcompat set: pinocchio.buildModelFromUrdf (fixed base)
- simcompat set: rigid rollouts: semi-implicit Euler on ABA with the shared timestep (no limits)
- engine default, base: fixed unless a root joint (e.g. JointModelFreeFlyer) is passed
- engine default, missing &lt;inertial&gt;: zero mass and inertia (the URDF specification)
- engine default, mimic: independent joint unless built with mimic=True

**mujoco 3.15.0**

- simcompat set: loaded with MuJoCo's URDF importer (MjModel.from_xml_path)
- simcompat set: &lt;compiler strippath='false'&gt; injected (mesh file resolution only)
- simcompat set: timestep = 0.001
- simcompat set: rollouts: contacts disabled; integrator left at the model default
- engine default, base: fixed (the root link is welded to the world unless a 'floating' joint is declared)
- engine default, fusestatic: true for URDF (bodies joined by fixed joints are merged)
- engine default, velocity limit: not represented (MuJoCo has no joint velocity limit)
- engine default, effort limit: mapped to the joint's actuatorfrcrange
- engine default, missing &lt;inertial&gt;: mass and inertia inferred from collision geometry (inertiafromgeom=auto)

**pybullet 3.2.7**

- simcompat set: loadURDF(useFixedBase=True, flags=0)
- simcompat set: setGravity(0, 0, -9.81)  [PyBullet's default gravity is 0]
- simcompat set: setTimeStep(0.001)
- simcompat set: default joint velocity motors disabled (force=0)  [PyBullet holds every joint with a motor]
- simcompat set: link linear/angular damping set to 0  [PyBullet's default is 0.04]
- simcompat set: joint speed cap raised to 10000 rad/s  [PyBullet clamps joint speeds at 100 by default]
- engine default, base: floating (loadURDF useFixedBase=False)
- engine default, gravity: 0 until setGravity is called
- engine default, joint motors: a velocity motor on every joint, max force = URDF effort
- engine default, inertia: recomputed from collision shapes
- engine default, missing &lt;inertial&gt;: mass 1 kg
- engine default, mimic: not supported by loadURDF (the joint stays independent)
- engine default, floating joint: built as a fixed joint
- engine default, joint speed: clamped at 100 rad/s
- importer: No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frame: FL_calflower, FL_calflower1, FR_calflower, FR_calflower1, RL_calflower, RL_calflower1, RR_calflower, RR_calflower1 and 1 more

</details>

<sub>Import, inertia and kinematics checks compare each engine with the file as parsed by simcompat (no engine involved). Dynamics checks compare with the reference at 6 sampled configurations (fixed base, no contact; correct imports agree to about 1e-12; M(q) tolerance 1e-06). Rollouts: 2 random starts, 0.5 s at dt=0.001, zero torque, contacts off, compared until a joint nears a limit. ✅ PASS: matches. ⏭️ SKIP: not applicable. ℹ️ INFO: documented convention. ⚠️ WARN: likely to bite. ❌ FAIL: differs where it must match. 💥 ERROR: could not run.</sub>
