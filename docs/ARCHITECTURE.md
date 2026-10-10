# Architecture: a ten-minute tour

```
 "robot.urdf" / ".xacro" / dir / "rd:go2"
            │
   inputs.py        resolve()        -> RobotInput   (which URDF; xacro expanded; package dirs)
            │       prepare_urdf()   -> PreparedURDF (absolute mesh paths, <visual> stripped)
   declared.py      parse_urdf()     -> DeclaredModel: what the FILE says (our own parser, no engine)
            │
   backends/        one adapter per engine, everything keyed by URDF joint NAME
            │       load() with a fixed base and harmonized runtime settings
            │
   runner.py        validate options -> load backends -> draw samples -> run every check (crash-isolated) -> Report
            │
   checks/          @check functions, one module per tier, each returns [Finding] (the runner adds the check id)
            │       ctx.ev (evaluate.py) memoizes engine queries in canonical joint order
            │
   diagnosis.py     findings -> Problems: grouped by cause, symptoms attached via @check(explained_by=...)
            │
   report/          Report (model.py) -> console.py · markdown.py · html.py · JSON; shared wording.py
            │
   cli.py           thin argparse layer: check, matrix, explain, backends, robots, init
```

| Module | Responsibility | Read it when... |
|---|---|---|
| `declared.py` | URDF parsing, kinematic tree, FK, declared mass properties, traversal orders | a check compares against "the file" |
| `inputs.py` | robot specs → plain URDF; xacro; `package://` resolution | an input does not load |
| `config.py` | `RunConfig`, `Tolerances`, profiles (`simcompat.toml`) | adding a setting or tolerance |
| `runner.py` | option validation, backend loading, crash isolation, severity overrides | changing the order of a run |
| `sampling.py` | seeded configurations and rollout starts | changing what is sampled |
| `spatial.py` | rotations, transforms, combining inertias | doing geometry |
| `backends/base.py` | the `Backend` interface, `Option`, view dataclasses, capabilities, hint topics | adding an engine |
| `checks/base.py` | `Severity`, `Finding`, `@check`, `Context`, `problems_or_pass`, check patterns | adding a check |
| `checks/evaluate.py` | memoized engine queries (M, g, C, rollouts) in canonical joint order | a check needs dynamics |
| `checks/*_checks.py` | the checks, one module per tier (file, import, inertia, kinematics, dynamics, rollout) | changing what is compared |
| `diagnosis.py` | grouping findings into problems: causes, symptoms, engine failures, re-graded causes | a problem is grouped wrongly |
| `report/` | rendering only, no physics; all shared sentences live in `wording.py` | changing how results look |
| `matrix.py` | many robots → one compatibility matrix | changing `simcompat matrix` |
| `errors.py`, `scaffold.py`, `cli.py` | user-facing errors, `simcompat init` templates, argument parsing | changing the command line |

## Key ideas

- **Two references.** Import, inertia and kinematics checks compare each engine with `DeclaredModel`, simcompat's own reading of the file. Dynamics checks compare with a *reference backend* (Pinocchio), because they need rigid-body algorithms. Pinocchio is itself checked against the file like every other backend.
- **Names, not indices.** Engines order joints differently. Every engine result is reordered to the canonical order (depth-first, file order) by `reorder_vector`, `reorder_matrix` and `reorder_columns`. Ordering is reported as a finding, never assumed.
- **Fusion-invariant inertia.** Some engines merge bodies joined by fixed joints. simcompat compares, for each movable joint, the composite of everything that joint moves (`DeclaredModel.moving_bodies` vs `inertia_checks.engine_moving_bodies`).
- **Ablation instead of guessing.** `rollout(damping=, friction=, mimic=)` switches an engine's *own* handling of those elements. "Read but not simulated" is found by comparing an engine with itself. For PyBullet, `friction=False` runs a friction-free twin model, so the verdict is measured rather than assumed.
- **Engine failures are findings.** A backend raises `EngineQueryError` when the engine cannot answer, for example when PyBullet's inverse dynamics rejects a tensor. The runner reports that as an ERROR attributed to the engine; any other exception is reported as a simcompat bug. When a check fails, the runner re-runs it per engine (the reference first), so one engine's failure is never hidden behind, or blamed on, another.
- **Causes, not symptoms.** A check that compares an integrated quantity (dynamics, rollouts) declares which root-cause checks can explain it: `@check(..., explained_by=("inertia.clusters", ...))`. `diagnosis.py` attaches each such symptom to the first cause present on the same engine, on the engine it was compared with (`Finding.compared_with`), or in the file. A symptom whose possible causes the engine failed to answer belongs to that engine's failure; a symptom nothing explains is reported on its own. The verdict is the worst *problem*, and a cause that a project re-grades in its profile caps the symptoms it explains.
- **Typed hand-offs between layers.** Checks, the runner and diagnosis communicate through `Finding` fields (`compared_with`, `engine_failure`, `regraded_from`), never through ad-hoc keys in `Finding.data`, which only carries numbers for reports.
- **Fail early on mistakes.** Unknown engines, options and option values, references, check patterns, profile keys and package directories raise a `SimCompatError` subclass with a "did you mean" suggestion before any engine is loaded. The CLI turns those, and files it cannot read or write, into exit code 2; any other exception is a bug (exit code 3, with a traceback).
- **Deterministic.** All sampling is seeded (`RunConfig.seed`), so reports are reproducible.

## Adding a backend

1. Create `src/simcompat/backends/<engine>_backend.py`:

```python
class GenesisBackend(Backend):
    name = "genesis"
    is_simulator = True
    capabilities = frozenset({CAP_IMPORT, CAP_INERTIA, CAP_KINEMATICS, CAP_DYNAMICS})  # what you implement
    options_spec = {"default_armature": Option("armature Genesis adds to every joint", kind="number")}

    @classmethod
    def availability(cls) -> tuple[bool, str]:   # never raise
        try:
            import genesis
            return True, genesis.__version__
        except Exception as e:
            return False, f"pip install genesis-world ({type(e).__name__})"

    def load(self, prepared, declared, settings):
        ...                                # FIXED base; apply settings.dt and settings.gravity
        return LoadInfo(version=..., settings=[...what you changed...], defaults={...engine defaults...})

    def joint_order(self): ...             # movable joint names, engine order
    def joint_views(self): ...             # {name: JointView(...)}; None = "not represented"
    def default_base(self): ...            # "fixed" | "floating"
    def multi_dof_joints(self): ...        # {name: BUILT_FREE | BUILT_FIXED | ...} for floating/planar joints
    def bodies(self): ...                  # [BodyView(name, carrier, mass, com_world, inertia_world)] at q = 0
    def joint_frames(self, q): ...         # {name: (child frame position, rotation, axis)} in world
    def mass_matrix(self, q): ...          # (n, n), joint_order()
    def bias(self, q, qd): ...             # C(q,qd)qd + g(q), joint_order(), no damping or friction
    def rollout(self, q0, qd0, steps, *, damping, friction, mimic): ...  # (steps, n), contacts off
```

2. Register it in `backends/__init__.py`. A separate package needs no change to simcompat: declare an entry point and it is picked up automatically:

```toml
[project.entry-points."simcompat.backends"]
genesis = "simcompat_genesis:GenesisBackend"
```

3. **Acceptance test:** `simcompat check tests/fixtures/arm6.urdf -b pinocchio -b genesis` and the same for `tests/fixtures/hand.urdf` must pass every core check. `arm6` has rotated inertial frames, skewed axes, a continuous joint, a prismatic joint and a fixed tool; `hand` is a branched tree, so it also exercises joint ordering. Then add a test to `tests/test_engines.py`.
4. Put engine defaults in `LoadInfo.defaults` and fix suggestions in `self.hints[HINT_*]`. When an engine call fails, raise `EngineQueryError` with the engine's own message. Capture anything the engine prints with `backends/_quiet.captured_native_output()` and put it in `LoadInfo.warnings`.
5. Option values arrive validated and normalized (`Option.normalize`): booleans are `"1"`/`"0"`, so use `self.option_enabled(key)`.

Pitfalls we hit:

- PyBullet's `jointAxis` is expressed in the child's *inertial* frame, not the link frame.
- PyBullet prints from C++. Use `backends/_quiet.captured_native_output()` to capture it.
- PyBullet explains an inverse-dynamics failure only on the first call.
- MuJoCo's `strippath` defaults to true for URDF.
- Pinocchio represents continuous joints as (cos, sin), so nq = 2 and nv = 1.

## Adding a check

```python
@check(
    "dynamics.my_check",
    Tier.DYNAMICS,
    "Short title of what is compared",
    problem="Plain-language headline shown when it fails",
    requires=[CAP_DYNAMICS],          # skipped (not failed) when no loaded engine can answer
    explained_by=MODEL_CAUSES,        # only for symptoms: root causes that explain a difference here
)
def my_check(ctx: Context) -> list[Finding]:
    """One-line summary shown in reports.

    Longer explanation shown by `simcompat explain dynamics.my_check`: what is compared, why it
    matters, typical causes. End with "Fix: ...".
    """
    findings = []
    for backend in ctx.with_cap(CAP_DYNAMICS):
        problems = [
            Finding(Severity.FAIL, "what differs, with numbers", backend=backend.label, subject=joint,
                    hint="how to fix it", data={...}, score=relative_error)   # score: largest shown first
            for joint in ...
        ]
        findings += problems_or_pass(backend, problems, "what passing means")
    return findings
```

Import the module in `checks/__init__.py`. Root causes need nothing else; `validate_registry()` rejects an `explained_by` that names an unknown check or another symptom.

## Severity semantics

| Severity | Meaning |
|---|---|
| FAIL | The engine simulates something different from the file where it must match exactly |
| WARN | Likely to bite: ordering, mimic, floating joints, zero-width ranges, integrated passive behaviour |
| INFO | A documented convention, such as the default base mode or dropped velocity limits |
| SKIP | Not applicable |
| ERROR | The engine refused the model or failed to answer, or a check crashed |

Projects can re-grade checks in a profile (`[severity]`). Overrides never hide an ERROR. Lowering a root cause below WARN also accepts the symptoms it explains (see `diagnosis.py`).

## Tests

- `tests/fake_backend.py` is an engine-free backend with injectable importer bugs (axis, origin, mass, limits, damping, armature, joint order, load and query failures). The import, inertia and kinematics checks are tested against the bug classes they exist to catch; dynamics and rollout checks need a real engine and are covered by `tests/test_engines.py` and the seeded-bug benchmark.
- `tests/test_engines.py` pins real engine behaviours. If an engine release changes one, the test fails.
- `tests/test_report.py` covers diagnosis (grouping, attribution, re-graded causes) and the renderers; `tests/test_cli.py` covers every command, exit codes and the error messages for common mistakes.
- `validation/` runs the whole public corpus, an independent-parser oracle, counterfactual confirmations and the seeded-bug benchmark ([VALIDATION.md](../VALIDATION.md)).
