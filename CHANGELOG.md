# Changelog

## 0.2.0

Validated on every URDF model of robot_descriptions (see [VALIDATION.md](VALIDATION.md)).

**Using it**

- Inputs: `.urdf.xacro` files are expanded with xacrodoc (no ROS needed), a package directory is searched for its URDF, and `rd:<name>` checks any robot_descriptions model. Misspelled xacro arguments are reported.
- Reports lead with *problems grouped by cause*, largest difference first, each with a fix. Dynamics and rollout differences are attached to the inertia, kinematics or import problem that explains them.
- New outputs: self-contained HTML reports (`-o report.html`) and compatibility matrices (`simcompat matrix -o matrix.html`).
- New commands: `explain` (what a check compares and how to fix it), `init` (profile + GitHub Actions workflow), `robots` (find robot_descriptions models by name, maker or tag).
- `--only`/`--skip` accept a tier (`dynamics`), a check id or a glob; so do `[severity]` keys in a profile.
- Re-grading a cause in a profile also re-grades the symptoms it explains.
- Mistakes are usage errors (exit code 2) with a suggestion: unknown engines, options, option values, references, checks, profile keys, robot names and package directories. Internal errors exit with 3.

**New checks and findings**

- `import.multi_dof`: floating joints are free in MuJoCo and Pinocchio but welded by PyBullet.
- `file.limits` / `import.limits`: lower/upper written on a continuous joint are enforced by MuJoCo and PyBullet, ignored by Pinocchio (and by the URDF spec).
- `file.inertia_validity` / `inertia.clusters`: PyBullet (with inertia from file) zeroes non-physical inertia tensors, MuJoCo refuses them, and MuJoCo drops the inertia of zero-mass links.
- Engine failures (for example PyBullet's getLinkState on iCub and ergoCub) are reported as such, never blamed on another engine.
- `rollout.passive` compares integrated damping relative to how much damping changes the motion.

**For contributors**

- Checks declare the causes that explain them (`@check(..., explained_by=...)`); the runner fills in check ids; layers hand data over through typed `Finding` fields.
- Backends declare typed options (`Option`) and can be added from another package through the `simcompat.backends` entry point.
- One shared `SimCompatError` base for user-facing errors; shared report wording in `report/wording.py`.

## 0.1.0

First release: URDF checks across MuJoCo, PyBullet and Pinocchio (file, import, inertia, kinematics, dynamics, rollout), text/Markdown/JSON reports, GitHub Action.
