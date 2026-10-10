"""Tier 1 - what each importer actually built from the file, compared with the file."""

from __future__ import annotations

from ..backends.base import CAP_IMPORT, HINT_MIMIC, MIMIC_IGNORED, NOT_BUILT, JointView
from ..declared import DECLARATION_ORDER, MULTI_DOF_TYPES, UNBOUNDED, Joint
from .base import Context, Finding, Severity, Tier, check, problems_or_pass


def _same(a: float | None, b: float | None, rel: float) -> bool:
    a, b = a or 0.0, b or 0.0
    return abs(a - b) <= max(1e-12, rel * max(abs(a), abs(b)))


def _kind(joint_type: str) -> str:
    return "prismatic" if joint_type == "prismatic" else "revolute"


@check("import.load", Tier.IMPORT, "Model loads", problem="An engine refused to load the model")
def load(ctx: Context) -> list[Finding]:
    """Each engine loads the file with its standard URDF importer.

    A model that loads in one engine and is rejected by another is the most visible kind of
    divergence. The importer's own messages are listed in the report.
    """
    findings = []
    for result in ctx.load_results:
        if not result.ok:
            findings.append(
                Finding(Severity.ERROR, result.error or "failed to load", backend=result.label, hint=result.hint)
            )
            continue
        warnings = result.info.warnings if result.info else []
        message = f"loaded ({result.version})" + (f"; {len(warnings)} importer message(s)" if warnings else "")
        findings.append(
            Finding(
                Severity.INFO if warnings else Severity.PASS,
                message,
                backend=result.label,
                data={"warnings": warnings},
            )
        )
    return findings


@check(
    "import.dofs",
    Tier.IMPORT,
    "Degrees of freedom",
    requires=[CAP_IMPORT],
    problem="An engine built different degrees of freedom than the file declares",
)
def dofs(ctx: Context) -> list[Finding]:
    """Every declared movable joint exists in the engine, with the same joint type.

    A missing or retyped joint changes the robot's degrees of freedom. Dynamics checks are
    skipped for such an engine, because its joint space no longer matches the file.
    """
    findings = []
    for backend in ctx.with_cap(CAP_IMPORT):
        order, views = backend.joint_order(), backend.joint_views()
        problems = []
        for name in ctx.joints:
            joint = ctx.declared.joints[name]
            if name not in order:
                problems.append(
                    Finding(
                        Severity.FAIL, f"declared {joint.type} joint is missing", backend=backend.label, subject=name
                    )
                )
            elif views[name].kind != _kind(joint.type):
                problems.append(
                    Finding(
                        Severity.FAIL,
                        f"declared {joint.type} but the engine built a {views[name].kind} joint",
                        backend=backend.label,
                        subject=name,
                    )
                )
        problems += [
            Finding(Severity.WARN, "engine has an extra degree of freedom", backend=backend.label, subject=name)
            for name in order
            if name not in ctx.joints
        ]
        count = len(ctx.joints)
        findings += problems_or_pass(backend, problems, f"{count}/{count} movable joints, types match")
    return findings


@check(
    "import.joint_order",
    Tier.IMPORT,
    "Joint ordering",
    requires=[CAP_IMPORT],
    problem="Engines number the joints differently",
)
def joint_order(ctx: Context) -> list[Finding]:
    """Engines expose the joints in the same order.

    Policies and controllers index joints by position. Engines traverse the kinematic tree
    differently: depth-first in file order (MuJoCo, PyBullet), depth-first with siblings sorted by
    name (Pinocchio and other urdfdom-based tools), or breadth-first (documented for Isaac Sim's
    PhysX articulations). The report names the convention each engine follows.

    Fix: map joints by name, never by index, when moving a policy between engines.
    """
    conventions = ctx.declared.order_conventions()
    traversals = {name: order for name, order in conventions.items() if name != DECLARATION_ORDER}
    orders = {b.label: b.joint_order() for b in ctx.with_cap(CAP_IMPORT)}

    def convention_of(order: list[str]) -> str:
        """Name the convention(s) an order follows ("A = B" when two conventions coincide)."""
        matches = [name for name, conv in traversals.items() if conv == order]
        if len(matches) == len(traversals):
            return "the only traversal order: the tree has no branches"
        if not matches and conventions[DECLARATION_ORDER] == order:
            matches = [DECLARATION_ORDER]
        return " = ".join(matches) if matches else "an engine-specific order"

    follows = {label: convention_of(order) for label, order in orders.items()}
    table = {
        "declared": conventions[DECLARATION_ORDER],
        "conventions": conventions,
        "engines": orders,
        "follows": follows,
    }
    findings = []
    if len({tuple(order) for order in orders.values()}) > 1:
        base_label, base = next(iter(orders.items()))
        for label, order in orders.items():
            if order == base:
                continue
            moved = sum(1 for i, name in enumerate(order) if i >= len(base) or base[i] != name)
            findings.append(
                Finding(
                    Severity.WARN,
                    f"{moved} of {len(order)} joint indices differ from {base_label} "
                    f"({label}: {follows[label]}; {base_label}: {follows[base_label]})",
                    backend=label,
                    data=table,
                    hint="Map joints by name, never by index, when moving a policy between engines.",
                    score=moved,
                )
            )
    else:
        common = next(iter(follows.values()), "?")
        findings.append(Finding(Severity.PASS, f"all engines use the same order ({common})", data=table))
    if len({tuple(order) for order in traversals.values()}) > 1:
        findings.append(
            Finding(
                Severity.INFO,
                "the tree is branched, so traversal conventions disagree (for example breadth-first, documented for "
                "Isaac Sim/PhysX, vs. depth-first); see the ordering table",
                data=table,
                hint="Isaac Lab exposes ArticulationCfg.joint_ordering to pin an order.",
            )
        )
    return findings


@check(
    "import.multi_dof",
    Tier.IMPORT,
    "Floating and planar joints",
    requires=[CAP_IMPORT],
    problem="Engines build the file's floating/planar joints differently (free in some, welded in others)",
)
def multi_dof(ctx: Context) -> list[Finding]:
    """How each engine builds the file's floating and planar joints.

    A URDF `floating` joint becomes a free 6-DoF joint in MuJoCo and Pinocchio but a fixed joint
    in PyBullet's loadURDF. The robot then floats in some engines and is welded to the world in
    others, so every rollout differs even though each 1-DoF joint matches.

    Fix: make the base explicit in every engine (weld it, or float it yourself) instead of relying
    on the importer's reading of `floating`.
    """
    joints = [j for j in ctx.declared.joints.values() if j.type in MULTI_DOF_TYPES]
    if not joints:
        return [Finding(Severity.SKIP, "no floating or planar joints declared")]
    built = {b.label: b.multi_dof_joints() for b in ctx.with_cap(CAP_IMPORT)}
    findings = []
    for joint in joints:
        readings = {label: kinds.get(joint.name, NOT_BUILT) for label, kinds in built.items()}
        disagree = len(set(readings.values())) > 1
        findings += [
            Finding(
                Severity.WARN if disagree else Severity.INFO,
                f"declared {joint.type} joint built as: {kind}",
                backend=label,
                subject=joint.name,
                data={"readings": readings},
                hint="Make the base explicit in every engine instead of relying on the importer." if disagree else None,
            )
            for label, kind in readings.items()
        ]
    return findings


@check(
    "import.base_mode",
    Tier.IMPORT,
    "Default base mode",
    requires=[CAP_IMPORT],
    problem="Engines' default loaders disagree on whether the base floats",
)
def base_mode(ctx: Context) -> list[Finding]:
    """Whether each engine's standard loader welds the root link to the world or lets it float.

    simcompat always compares with a fixed base; this tells you what you get with default
    arguments (PyBullet floats the base, MuJoCo welds it unless the URDF declares a floating joint).
    """
    modes = {b.label: b.default_base() for b in ctx.with_cap(CAP_IMPORT)}
    if len(set(modes.values())) > 1:
        return [
            Finding(Severity.INFO, f"default loader gives a {mode} base", backend=label, data={"modes": modes})
            for label, mode in modes.items()
        ]
    mode = next(iter(modes.values()), "?")
    return [Finding(Severity.PASS, f"all engines default to a {mode} base", data={"modes": modes})]


@check(
    "import.limits",
    Tier.IMPORT,
    "Position limits",
    requires=[CAP_IMPORT],
    problem="Engines enforce different joint limits (dropped, changed, or an ambiguous range read differently)",
)
def limits(ctx: Context) -> list[Finding]:
    """Declared lower/upper limits survive the import; continuous joints stay unlimited.

    Ambiguous ranges (see file.limits) are reported with each engine's reading (locked, limited
    or unlimited), as a WARN when engines disagree. A range at least 1e9 wide counts as unlimited.
    """
    views = {b.label: b.joint_views() for b in ctx.with_cap(CAP_IMPORT)}
    ambiguous = [
        name for name in ctx.joints if ctx.declared.joints[name].zero_width or ctx.declared.joints[name].ignored_range
    ]
    readings = {name: _readings(ctx.declared.joints[name], views) for name in ambiguous}
    findings = []
    for backend in ctx.with_cap(CAP_IMPORT):
        problems = []
        for name in ctx.joints:
            view = views[backend.label].get(name)
            reading = _reading(view) if view is not None else None
            if reading is None:
                continue
            joint = ctx.declared.joints[name]
            if name in readings:
                problems.append(_ambiguous_range(joint, view, reading, readings[name], backend.label))
            elif joint.limited and reading == UNLIMITED:
                problems.append(
                    Finding(
                        Severity.FAIL,
                        f"declared limits [{joint.lower:g}, {joint.upper:g}] were dropped",
                        backend=backend.label,
                        subject=name,
                        score=float("inf"),
                    )
                )
            elif not joint.limited and reading == LIMITED:
                problems.append(
                    Finding(
                        Severity.FAIL,
                        f"declared unlimited ({joint.type}) but limited to [{view.lower:g}, {view.upper:g}]",
                        backend=backend.label,
                        subject=name,
                        score=float("inf"),
                    )
                )
            elif joint.limited:
                error = max(abs(view.lower - joint.lower), abs(view.upper - joint.upper))
                if error > ctx.tol.limit_abs:
                    problems.append(
                        Finding(
                            Severity.FAIL,
                            f"limits [{view.lower:g}, {view.upper:g}] vs declared [{joint.lower:g}, {joint.upper:g}]",
                            backend=backend.label,
                            subject=name,
                            score=error,
                        )
                    )
        findings += problems_or_pass(backend, problems, "position limits match the file")
    return findings


LIMITED, UNLIMITED, LOCKED = "limited", "unlimited", "locked"


def _reading(view: JointView) -> str | None:
    """How an engine holds a joint: LIMITED, UNLIMITED, or None if it does not say."""
    if view.limited is None:
        return None
    if not view.limited or (view.lower <= -UNBOUNDED and view.upper >= UNBOUNDED):
        return UNLIMITED
    return LIMITED


def _readings(joint: Joint, views: dict[str, dict[str, JointView]]) -> dict[str, str]:
    """Each engine's reading of an ambiguous range: LOCKED (zero width), LIMITED or UNLIMITED."""
    readings = {label: _reading(v[joint.name]) for label, v in views.items() if joint.name in v}
    locked = {LIMITED: LOCKED} if joint.zero_width else {}
    return {label: locked.get(r, r) for label, r in readings.items() if r is not None}


def _ambiguous_range(joint: Joint, view: JointView, reading: str, readings: dict[str, str], label: str) -> Finding:
    """One engine's reading of a zero-width range or of a range written on a continuous joint."""
    disagree = len(set(readings.values())) > 1
    if joint.zero_width:
        what = (
            f"zero-width range: engine holds the joint at [{joint.lower:g}, {joint.upper:g}] (locked)"
            if reading == LIMITED
            else "zero-width range: engine leaves the joint free (unlimited)"
        )
        hint = "Declare the intended range explicitly."
    else:
        what = (
            f"range written on a continuous joint: engine enforces [{view.lower:g}, {view.upper:g}]"
            if reading == LIMITED
            else "range written on a continuous joint: engine ignores it (unlimited), as the URDF spec says"
        )
        hint = "Use a revolute joint if the range is intended; otherwise remove lower/upper."
    return Finding(
        Severity.WARN if disagree else Severity.INFO,
        what,
        backend=label,
        subject=joint.name,
        data={"interpretations": readings},
        hint=hint,
    )


@check(
    "import.effort_velocity",
    Tier.IMPORT,
    "Effort and velocity limits",
    requires=[CAP_IMPORT],
    problem="An engine stores different effort or velocity limits",
)
def effort_velocity(ctx: Context) -> list[Finding]:
    """Declared effort and velocity limits, as the engine stores them.

    Engines without such a concept drop them (MuJoCo has no joint velocity limit); that is
    reported as INFO. A different stored value is a WARN.
    """
    findings = []
    for backend in ctx.with_cap(CAP_IMPORT):
        views = backend.joint_views()
        problems: list[Finding] = []
        dropped: dict[str, list[str]] = {"effort": [], "velocity": []}
        for name in ctx.joints:
            if name not in views:
                continue
            for attr in dropped:
                declared, stored = getattr(ctx.declared.joints[name], attr), getattr(views[name], attr)
                if declared is None:
                    continue
                if stored is None:
                    dropped[attr].append(name)
                elif not _same(declared, stored, ctx.tol.param_rel):
                    problems.append(
                        Finding(
                            Severity.WARN,
                            f"{attr} limit {stored:g} vs declared {declared:g}",
                            backend=backend.label,
                            subject=name,
                            score=abs(stored - declared) / max(abs(declared), 1e-12),
                        )
                    )
        problems += [
            Finding(
                Severity.INFO,
                f"{attr} limits are not represented by this engine ({len(names)} joint(s))",
                backend=backend.label,
                data={"joints": names},
            )
            for attr, names in dropped.items()
            if names
        ]
        findings += problems_or_pass(backend, problems, "effort/velocity limits match the file")
    return findings


@check(
    "import.damping_friction",
    Tier.IMPORT,
    "Joint damping and friction (as read)",
    requires=[CAP_IMPORT],
    problem="An engine read different joint damping or friction than declared",
)
def damping_friction(ctx: Context) -> list[Finding]:
    """<dynamics damping/friction> values as the engine stores them.

    Reading a value is not the same as simulating it; rollout.passive_effect tests whether the
    values actually change the motion.
    """
    findings = []
    for backend in ctx.with_cap(CAP_IMPORT):
        views = backend.joint_views()
        problems = []
        for name in ctx.joints:
            if name not in views:
                continue
            for attr in ("damping", "friction"):
                declared, stored = getattr(ctx.declared.joints[name], attr) or 0.0, getattr(views[name], attr)
                if stored is None and declared:
                    problems.append(
                        Finding(
                            Severity.FAIL,
                            f"declared {attr}={declared:g} is not represented by the engine",
                            backend=backend.label,
                            subject=name,
                            score=declared,
                        )
                    )
                elif stored is not None and not _same(declared, stored, ctx.tol.param_rel):
                    problems.append(
                        Finding(
                            Severity.FAIL,
                            f"{attr} {stored:g} vs declared {declared:g}",
                            backend=backend.label,
                            subject=name,
                            score=abs(stored - declared),
                        )
                    )
        findings += problems_or_pass(backend, problems, "damping/friction read as declared")
    return findings


@check(
    "import.armature",
    Tier.IMPORT,
    "Engine-injected armature",
    requires=[CAP_IMPORT],
    problem="An engine adds armature (rotor inertia) that the file does not declare",
)
def armature(ctx: Context) -> list[Finding]:
    """No engine adds armature (rotor inertia) on its own.

    URDF has no armature. A non-zero value means the importer injected a default, which changes
    the mass matrix (Genesis' default armature is a documented example).

    Fix: make armature explicit and identical in every engine's configuration.
    """
    findings = []
    for backend in ctx.with_cap(CAP_IMPORT):
        views = backend.joint_views()
        injected = [
            Finding(
                Severity.WARN,
                f"engine adds armature {view.armature:g} (the file declares none)",
                backend=backend.label,
                subject=name,
                hint="Make armature explicit and identical in every engine.",
                score=view.armature,
            )
            for name, view in views.items()
            if view.armature
        ]
        concept = "no armature concept" if all(v.armature is None for v in views.values()) else "armature is zero"
        findings += problems_or_pass(backend, injected, concept)
    return findings


@check(
    "import.mimic",
    Tier.IMPORT,
    "Mimic joints",
    requires=[CAP_IMPORT],
    problem="Mimic couplings are enforced by some engines and ignored by others",
)
def mimic(ctx: Context) -> list[Finding]:
    """How each engine implements <mimic> couplings.

    MuJoCo turns them into equality constraints; PyBullet's loadURDF and Pinocchio (by default)
    leave the joint independent, so grippers and hands move differently. A simulator that drops
    the coupling is a WARN; the dynamics reference computing unconstrained dynamics is expected
    (INFO).
    """
    mimics = [ctx.declared.joints[n] for n in ctx.joints if ctx.declared.joints[n].mimic is not None]
    if not mimics:
        return [Finding(Severity.SKIP, "no <mimic> joints declared")]
    backends = ctx.with_cap(CAP_IMPORT)
    views = {b.label: b.joint_views() for b in backends}
    handling = {
        joint.name: {label: (v[joint.name].mimic if joint.name in v else "missing") for label, v in views.items()}
        for joint in mimics
    }
    findings = []
    for backend in backends:
        for joint in mimics:
            how = handling[joint.name][backend.label]
            lost = how in (MIMIC_IGNORED, "missing", None) and backend.is_simulator
            hint = backend.hints.get(HINT_MIMIC) or "The coupling is silently lost in this engine."
            coupling = f"follows '{joint.mimic.joint}' (multiplier {joint.mimic.multiplier:g}"
            coupling += f", offset {joint.mimic.offset:g})" if joint.mimic.offset else ")"
            findings.append(
                Finding(
                    Severity.WARN if lost else Severity.INFO,
                    f"{coupling}; built as: {how}",
                    backend=backend.label,
                    subject=joint.name,
                    data={"handling": handling[joint.name]},
                    hint=hint if lost else None,
                )
            )
    return findings
