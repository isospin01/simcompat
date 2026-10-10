# Roadmap

Done in v0.2 (see [CHANGELOG.md](CHANGELOG.md)): xacro input (no ROS needed), problems grouped by cause, HTML reports and matrices, `explain`/`init`/`robots` commands, floating-joint and ambiguous-range handling, engine-failure reporting, backend plugins, and validation on the full robot_descriptions corpus.

The order of these items is driven by the evidence in [docs/PROBLEM.md](docs/PROBLEM.md). The deterministic layer comes first, then engines where users hit real pain, then layers that need statistics instead of pass/fail.

## v0.3: more engines (the main value multiplier)

- [ ] **Isaac Lab / Isaac Sim backend** (PhysX, and Newton through Isaac Lab's `physics=` presets). This is optional and GPU-only, so it ships as its own extra and its own CI runner. It would catch, for example, Isaac Sim #841 (dropped damping and friction) as `import.damping_friction` plus `rollout.passive_effect` FAILs.
- [ ] **Genesis backend.** Armature defaults and `enable_mujoco_compatibility`.
- [ ] **Drake backend.** `MultibodyPlant` from URDF; Drake has its own inertia validation policy.
- [ ] **Newton standalone backend.** Several solvers behind one API; compare solvers against each other.
- [ ] Ordering conventions of PhysX and MJWarp *measured* rather than predicted.

## v0.4: more inputs

- [ ] **MJCF input.** Its own declared-model parser, and URDF↔MJCF converter round-trips: convert, re-check, diff.
- [ ] **USD input.** UsdPhysics and Newton/PhysX schema attributes; layer/variant aware.

## v0.5: actuators and floating base

- [ ] **Actuator tier.** PD gains and units (per degree vs per radian), effort clamps, armature, implicit vs explicit PD. Read from engine configs such as Isaac Lab `ArticulationCfg` and mjlab, not from URDF.
- [ ] **Floating-base comparisons.** Centroidal quantities, plus base-mode defaults as an explicit finding.

## v0.6: contact (statistical, INFO/WARN only)

- [ ] Drop/settle tests, standing support forces, foot-slip onset. Report distributions and differences with documented solver context, never PASS/FAIL against exact equality.

## Always on

- [ ] **Public compatibility matrix.** A nightly CI run over `robot_descriptions` against the latest *and* pinned engine releases, published as a page, so that importer regressions are seen the day they ship.
- [ ] **`simcompat fix`.** Emit a patched URDF: explicit `<inertial>` for links that lack one, explicit ranges for zero-width limits, balanced inertias. Always opt-in, always shown as a diff.
- [ ] **pre-commit hook and PR comment bot.**
