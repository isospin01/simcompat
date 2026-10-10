# SimCompat report: `divergent`

**❌ FAIL**: Some engines simulate something different from what the file declares.

4 problems · findings: 13 fail, 4 warn, 8 info · engines: pinocchio 4.1.0 (reference), mujoco 3.15.0, pybullet 3.2.7 · simcompat 0.2.0 · 2026-10-10T01:22:23+00:00

Input `tests/fixtures/divergent.urdf` · 6 links · 5 joints (4 movable)

## Problems

### 1. ❌ Engines simulate different mass, center of mass or inertia than the file declares

`inertia.clusters` · pybullet, mujoco

| Engine | Joint / link | Finding |
|---|---|---|
| pybullet | j3 | mass 1 kg vs declared 0 kg; inertia tensor rel. error 6.24e+05 |
| mujoco | j3 | mass 0.113097 kg vs declared 0 kg; inertia tensor rel. error 7.05e+04 |
| pybullet | jm | inertia tensor rel. error 0.999 |
| pybullet | j1 | inertia tensor rel. error 0.884 |
| pybullet | j2 | inertia tensor rel. error 0.484 |

- **Fix:** These bodies include links without &lt;inertial&gt;, for which each engine substitutes its own default mass (the links are listed under file.inertial). Add explicit &lt;inertial&gt; blocks.
- **Fix:** PyBullet's loadURDF ignores URDF inertia tensors by default and recomputes them from collision shapes. Pass flags=URDF_USE_INERTIA_FROM_FILE (simcompat: -b pybullet:inertia_from_file=1).
- Also explains: dynamics.mass_matrix (mujoco, pybullet), dynamics.gravity (mujoco, pybullet), dynamics.coriolis (mujoco, pybullet), rollout.rigid (pybullet), rollout.passive (pybullet)

### 2. ❌ An engine reads joint damping or friction but does not simulate it

`rollout.passive_effect` · pybullet

| Engine | Joint / link | Finding |
|---|---|---|
| pybullet |  | declared &lt;dynamics friction&gt; has NO effect on the motion (read but not simulated) on j1 |

- **Fix:** pybullet stores &lt;dynamics friction&gt; (getJointInfo) but loadURDF does not simulate it. Workaround: a VELOCITY_CONTROL motor with targetVelocity=0 and force=friction on each joint.

### 3. ⚠️ Moving links have no &lt;inertial&gt;, so each engine invents its own mass for them

`file.inertial` · file

| Engine | Joint / link | Finding |
|---|---|---|
| file | noinert | moved by 'j3' but has no &lt;inertial&gt; |

- **Fix:** Add an explicit &lt;inertial&gt; so every engine uses the same mass.
- Also explains: rollout.rigid (pinocchio)

### 4. ⚠️ Mimic couplings are enforced by some engines and ignored by others

`import.mimic` · pybullet

| Engine | Joint / link | Finding |
|---|---|---|
| pybullet | jm | follows 'j1' (multiplier -1); built as: independent joint |

- **Fix:** pybullet.loadURDF does not create &lt;mimic&gt; couplings; add them yourself, e.g. createConstraint(..., JOINT_GEAR, ...) with gearRatio = -multiplier.

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
| Import | Mimic joints <sub>`import.mimic`</sub> |  | ℹ️ INFO | ℹ️ INFO | ⚠️ WARN |
| Inertia | Mass, COM and inertia per moving body <sub>`inertia.clusters`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Kinematics | Link frames and joint axes <sub>`kinematics.frames`</sub> |  | ✅ PASS | ✅ PASS | ✅ PASS |
| Dynamics | Joint-space mass matrix M(q) <sub>`dynamics.mass_matrix`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Dynamics | Gravity torques g(q) <sub>`dynamics.gravity`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Dynamics | Coriolis/centrifugal forces C(q, qd) qd <sub>`dynamics.coriolis`</sub> |  | ✅ PASS | ❌ FAIL | ❌ FAIL |
| Rollout | Rigid-body rollout <sub>`rollout.rigid`</sub> |  | ⚠️ WARN | ✅ PASS | ❌ FAIL |
| Rollout | Declared damping/friction actually act <sub>`rollout.passive_effect`</sub> |  |  | ✅ PASS | ❌ FAIL |
| Rollout | Passive rollout across simulators <sub>`rollout.passive`</sub> |  |  | ✅ PASS | ⚠️ WARN |

</details>

<details><summary><b>Joint ordering</b></summary>

Index policy and controller vectors by joint **name**. **pinocchio** follows depth-first, file order = depth-first, siblings sorted by name; **mujoco** follows depth-first, file order = depth-first, siblings sorted by name; **pybullet** follows depth-first, file order = depth-first, siblings sorted by name.

| # | declared | depth-first | depth-first (sorted) | breadth-first | pinocchio | mujoco | pybullet |
|---|---|---|---|---|---|---|---|
| 0 | `j1` | `j1` | `j1` | `j1` | `j1` | `j1` | `j1` |
| 1 | `j2` | `j2` | `j2` | `j2` | `j2` | `j2` | `j2` |
| 2 | `j3` | `j3` | `j3` | `jm` | `j3` | `j3` | `j3` |
| 3 | `jm` | `jm` | `jm` | `j3` | `jm` | `jm` | `jm` |

</details>

<details><summary><b>Notes</b> (documented conventions)</summary>

- **import.load** (pybullet): loaded (3.2.7); 1 importer message(s)
- **import.joint_order** (file): the tree is branched, so traversal conventions disagree (for example breadth-first, documented for Isaac Sim/PhysX, vs. depth-first); see the ordering table
- **import.base_mode** (pinocchio): default loader gives a fixed base
- **import.base_mode** (mujoco): default loader gives a fixed base
- **import.base_mode** (pybullet): default loader gives a floating base
- **import.effort_velocity** (mujoco): velocity limits are not represented by this engine (3 joint(s))
- **import.mimic** (pinocchio) `jm`: follows 'j1' (multiplier -1); built as: independent joint
- **import.mimic** (mujoco) `jm`: follows 'j1' (multiplier -1); built as: equality constraint

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
- importer: No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frame: noinert

</details>

<sub>Import, inertia and kinematics checks compare each engine with the file as parsed by simcompat (no engine involved). Dynamics checks compare with the reference at 6 sampled configurations (fixed base, no contact; correct imports agree to about 1e-12; M(q) tolerance 1e-06). Rollouts: 2 random starts, 0.5 s at dt=0.001, zero torque, contacts off, compared until a joint nears a limit. ✅ PASS: matches. ⏭️ SKIP: not applicable. ℹ️ INFO: documented convention. ⚠️ WARN: likely to bite. ❌ FAIL: differs where it must match. 💥 ERROR: could not run.</sub>
