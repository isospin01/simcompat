"""simcompat: does every physics engine simulate what your robot file declares?

>>> import simcompat
>>> report = simcompat.check("robot.urdf")        # or "robot.urdf.xacro", a directory, "rd:go2"
>>> report.ok, report.verdict
>>> for problem in report.problems: print(problem.headline)
>>> report.save("report.html")                     # also .md, .json, .txt
"""

from __future__ import annotations

__version__ = "0.2.0"


def check(
    robot,
    *,
    backends=None,
    reference="auto",
    package_dirs=None,
    xacro_args=None,
    config=None,
    only=None,
    skip=None,
    log=None,
):
    """Check one robot and return a :class:`~simcompat.report.model.Report`.

    Args:
        robot: a ``.urdf``, a ``.urdf.xacro``, a directory containing one of them, or ``"rd:<name>"``
            for any model in the ``robot_descriptions`` package.
        backends: engine spec or list of specs, e.g. ``["mujoco", "pybullet:inertia_from_file=1"]``.
            Default: every installed engine (``simcompat backends`` lists them and their options).
        reference: dynamics reference: ``"auto"`` (Pinocchio if installed), ``"none"`` (the first
            engine) or an engine label.
        package_dirs: ``{"my_robot_description": "/path/to/it"}`` to resolve ``package://`` URIs.
        xacro_args: ``{"name": "value"}`` arguments for xacro expansion.
        config: a :class:`~simcompat.config.RunConfig` (tolerances, samples, severity overrides),
            e.g. ``RunConfig.from_toml("simcompat.toml")``.
        only, skip: glob patterns over check ids, e.g. ``["dynamics.*"]`` (``simcompat explain``
            lists the checks).
        log: called with progress messages (e.g. ``print``).

    Raises ``ValueError`` (with a message saying what to do) for an unknown robot, engine, option or
    check; engines that fail to load are reported in the returned report instead.
    """
    from .runner import run

    return run(
        robot,
        backends=backends,
        reference=reference,
        package_dirs=package_dirs,
        xacro_args=xacro_args,
        config=config,
        only=only,
        skip=skip,
        log=log,
    )


def __getattr__(name):  # lazy public names, so `import simcompat` stays cheap
    if name == "Report":
        from .report.model import Report

        return Report
    if name == "Severity":
        from .checks.base import Severity

        return Severity
    if name == "RunConfig":
        from .config import RunConfig

        return RunConfig
    raise AttributeError(name)


__all__ = ["Report", "RunConfig", "Severity", "__version__", "check"]
