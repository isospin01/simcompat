"""Command-line interface. Run ``simcompat --help`` or ``simcompat <command> --help``.

This module only parses arguments and prints; every decision is made in the library.
"""

from __future__ import annotations

import argparse
import os
import sys
import textwrap
import traceback
from pathlib import Path

from . import __version__
from .errors import SimCompatError, UsageError
from .report.wording import pluralize
from .scaffold import DEFAULT_INSTALL

EXIT_OK, EXIT_FAIL, EXIT_USAGE, EXIT_BUG = 0, 1, 2, 3

EXAMPLES = """\
examples:
  simcompat check robot.urdf                     every installed engine; Pinocchio as reference
  simcompat check robot.urdf.xacro               xacro is expanded for you (pip install xacrodoc)
  simcompat check rd:go2                         any model from robot_descriptions
  simcompat check robot.urdf -o report.html      also .md, .json, .txt
  simcompat check robot.urdf -b mujoco -b pybullet:inertia_from_file=1
  simcompat check robot.urdf --only dynamics     a tier, a check id, or a glob such as 'rollout.*'
  simcompat matrix rd:go2 rd:g1 rd:panda -o matrix.html
  simcompat robots unitree                       find models to try (needs robot_descriptions)
  simcompat explain inertia.clusters             what a check compares and how to fix it
  simcompat init                                 add a profile and a GitHub Actions workflow

exit codes: 0 = ok, 1 = an engine differs from the file (FAIL/ERROR; with --strict also WARN),
            2 = usage or input error, 3 = internal error (a bug in simcompat)
"""


def _stderr(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def _use_color(args: argparse.Namespace) -> bool:
    return not args.no_color and sys.stdout.isatty() and "NO_COLOR" not in os.environ


def _pairs(values: list[str] | None, separator: str, option: str, placeholder: str) -> dict[str, str]:
    pairs = {}
    for value in values or []:
        key, found, rest = value.partition(separator)
        if not found or not key:
            raise UsageError(f"{option} expects {placeholder}, got {value!r}")
        pairs[key] = rest
    return pairs


def _run_options(args: argparse.Namespace) -> dict:
    """Keyword arguments for :func:`simcompat.runner.run`, shared by ``check`` and ``matrix``."""
    from .config import RunConfig

    config = RunConfig.from_toml(args.profile)
    for key in ("samples", "seed", "dt", "duration"):
        if getattr(args, key) is not None:
            setattr(config, key, getattr(args, key))
    return {
        "backends": args.backend,
        "reference": args.reference,
        "config": config,
        "only": args.only,
        "skip": args.skip,
        "package_dirs": _pairs(args.package_dir, "=", "--package-dir", "NAME=PATH"),
        "xacro_args": _pairs(args.xacro_arg, ":=", "--xacro-arg", "NAME:=VALUE"),
    }


def _check_outputs(outputs: list[str] | None, formats: tuple[str, ...]) -> None:
    """Reject an unknown output format before any work is done."""
    for output in outputs or []:
        if Path(output).suffix.lower() not in formats:
            raise UsageError(f"-o {output}: unknown format; use one of {', '.join(formats)}")


def _write(outputs: list[str] | None, save, quiet: bool) -> bool:
    """Write every output (one failure does not stop the others); True if all were written."""
    ok = True
    for output in outputs or []:
        try:
            path = save(output)
        except OSError as e:
            _stderr(f"simcompat: error: could not write {output}: {e.strerror or e}")
            ok = False
            continue
        if not quiet:
            _stderr(f"wrote {path}")
    return ok


# ------------------------------------------------------------------------------------- commands
def cmd_check(args: argparse.Namespace) -> int:
    from .report.model import REPORT_FORMATS
    from .runner import run

    _check_outputs(args.output, REPORT_FORMATS)
    report = run(args.robot, log=None if args.quiet else _stderr, **_run_options(args))
    if not args.quiet:
        print(report.to_text(color=_use_color(args), verbose=args.verbose))
    written = _write(args.output, report.save, args.quiet)
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary and not args.no_step_summary:
        with open(summary, "a", encoding="utf-8") as fh:
            fh.write(report.to_markdown())
    if not any(b.ok for b in report.backends):
        _stderr("simcompat: error: no engine could load the model")
        return EXIT_USAGE
    return report.exit_code(strict=args.strict) if written else EXIT_USAGE


def cmd_matrix(args: argparse.Namespace) -> int:
    from .checks.base import Severity
    from .matrix import MATRIX_FORMATS, build_matrix, render_markdown, save

    _check_outputs(args.output, MATRIX_FORMATS)
    options = _run_options(args)
    progress = None if args.quiet else _stderr
    rows, titles = build_matrix(args.robots, json_dir=args.json_dir, progress=progress, **options)
    if not args.quiet:
        print(render_markdown(rows, titles))
    written = _write(args.output, lambda output: save(rows, titles, output), args.quiet)
    if all(row.error for row in rows) or not written:
        return EXIT_USAGE
    failed = any(row.error or row.verdict >= (Severity.WARN if args.strict else Severity.FAIL) for row in rows)
    return EXIT_FAIL if failed else EXIT_OK


def cmd_explain(args: argparse.Namespace) -> int:
    from .checks import REGISTRY, ordered_checks
    from .checks.base import did_you_mean, matching_checks

    if not args.check:
        current = None
        for spec in ordered_checks():
            if spec.tier != current:
                current = spec.tier
                print(f"\n{spec.tier.label}")
            print(f"  {spec.id:26} {spec.title}")
        print("\nsimcompat explain <check> shows what a check compares, why it matters and how to fix it.")
        return EXIT_OK
    matches = matching_checks(args.check)
    if not matches:
        raise UsageError(f"no check matches {args.check!r}.{did_you_mean(args.check)} Run `simcompat explain`.")
    for spec in (REGISTRY[check_id] for check_id in matches):
        print(f"{spec.id}  ({spec.tier.label} tier)\n{'=' * len(spec.id)}\n")
        print(f"{spec.title}. When it fails: {spec.problem[0].lower() + spec.problem[1:]}.\n")
        print(textwrap.indent(spec.doc, "  ") + "\n")
    return EXIT_OK


def cmd_backends(_args: argparse.Namespace) -> int:
    from . import backends

    for name, installed, detail in backends.installed():
        cls = backends.REGISTRY[name]
        role = "reference library" if not cls.is_simulator else "simulator"
        status = f"installed {detail}" if installed else f"not installed ({detail})"
        print(f"{name:10} {role:18} {status}")
        for key, option in cls.options_spec.items():
            print(f"{'':12}-b {name}:{key}={option.placeholder:<10} {option.help}")
    return EXIT_OK


def cmd_robots(args: argparse.Namespace) -> int:
    from .inputs import ROBOT_DESCRIPTIONS_PREFIX, short_name

    try:
        from robot_descriptions._descriptions import DESCRIPTIONS
    except ModuleNotFoundError:
        raise UsageError("listing models needs robot_descriptions: pip install robot_descriptions") from None
    words = [w.lower() for w in args.filter]
    rows = []
    for name, description in sorted(DESCRIPTIONS.items()):
        tags = ", ".join(sorted(description.tags))
        text = " ".join([name, description.robot or "", description.maker or "", tags]).lower()
        if description.has_urdf and all(word in text for word in words):
            dof = "-" if description.dof is None else str(description.dof)
            spec = ROBOT_DESCRIPTIONS_PREFIX + short_name(name)
            rows.append((spec, description.robot or "", description.maker or "", dof, tags))
    if rows:
        rows.insert(0, ("SPEC", "ROBOT", "MAKER", "DOF", "TAGS"))
        widths = [max(len(row[i]) for row in rows) for i in range(4)]
        for row in rows:
            print("  ".join(cell.ljust(width) for cell, width in zip(row[:4], widths, strict=True)) + "  " + row[4])
    matching = f" matching {' '.join(args.filter)!r}" if words else ""
    _stderr(f"{pluralize(max(len(rows) - 1, 0), 'model')} with a URDF{matching}. Check one: simcompat check rd:<name>")
    return EXIT_OK


def cmd_init(args: argparse.Namespace) -> int:
    from .scaffold import guess_robot, init_project

    project = Path(args.dir)
    if args.robot:  # as typed (relative to here); the workflow runs from the project directory
        robot_here, robot_in_project = args.robot, os.path.relpath(args.robot, project)
    else:
        robot_in_project = guess_robot(project)
        if robot_in_project is None:
            raise UsageError(f"could not find exactly one URDF/xacro file in {project}; pass it: simcompat init ROBOT")
        robot_here = str(project / robot_in_project)
    for path, status in init_project(project, robot_in_project, force=args.force, install=args.install):
        print(f"{status:8} {path}")
    print(f"\nNext: simcompat check {robot_here} --profile {project / 'simcompat.toml'}")
    print(f'The workflow installs simcompat with `pip install "{args.install}"` (change it with --install).')
    return EXIT_OK


# --------------------------------------------------------------------------------------- parser
def _add_run_options(parser: argparse.ArgumentParser) -> None:
    engines = parser.add_argument_group("engines")
    engines.add_argument(
        "-b",
        "--backend",
        action="append",
        metavar="SPEC",
        help="engine to compare, repeatable: NAME or NAME:option=value "
        "(default: every installed one; see `simcompat backends`)",
    )
    engines.add_argument(
        "--reference",
        default="auto",
        metavar="LABEL",
        help="dynamics reference ('auto' = pinocchio if installed, 'none' = first engine)",
    )
    inputs = parser.add_argument_group("input")
    inputs.add_argument("--package-dir", action="append", metavar="NAME=PATH", help="resolve package://NAME to PATH")
    inputs.add_argument("--xacro-arg", action="append", metavar="NAME:=VALUE", help="argument for .xacro expansion")
    tuning = parser.add_argument_group("tuning")
    tuning.add_argument(
        "--profile", metavar="TOML", help="tolerances and severity overrides (`simcompat init` writes one)"
    )
    tuning.add_argument(
        "--only", action="append", metavar="PATTERN", help="run only matching checks: 'dynamics', 'rollout.*', an id"
    )
    tuning.add_argument("--skip", action="append", metavar="PATTERN", help="skip matching checks")
    tuning.add_argument("--samples", type=int, help="sampled configurations (default 6)")
    tuning.add_argument("--seed", type=int, help="random seed (default 0)")
    tuning.add_argument("--dt", type=float, help="rollout timestep in s (default 0.001)")
    tuning.add_argument("--duration", type=float, help="rollout duration in s (default 0.5)")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="simcompat",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EXAMPLES,
        description="Does every physics engine simulate what your robot file declares?",
    )
    parser.add_argument("--version", action="version", version=f"simcompat {__version__}")
    commands = parser.add_subparsers(dest="command", metavar="<command>")

    check = commands.add_parser(
        "check",
        help="check one robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Load one robot into every engine and compare what each one built.",
        epilog=EXAMPLES,
    )
    check.add_argument("robot", help="a .urdf, a .urdf.xacro, a directory with one of them, or rd:<name>")
    _add_run_options(check)
    out = check.add_argument_group("output")
    out.add_argument(
        "-o",
        "--output",
        action="append",
        metavar="FILE",
        help="write a report; format from the extension: .html, .md, .json, .txt (repeatable)",
    )
    out.add_argument(
        "-v", "--verbose", action="store_true", help="print every finding, the check matrix, joint ordering and notes"
    )
    out.add_argument("-q", "--quiet", action="store_true", help="print nothing; use the exit code")
    out.add_argument("--strict", action="store_true", help="exit 1 on WARN as well as FAIL")
    out.add_argument("--no-color", action="store_true", help="disable colors (also honours NO_COLOR)")
    out.add_argument("--no-step-summary", action="store_true", help="do not append to $GITHUB_STEP_SUMMARY")
    check.set_defaults(fn=cmd_check)

    matrix = commands.add_parser("matrix", help="check many robots and build a compatibility matrix")
    matrix.add_argument("robots", nargs="+", help="robots, as for `check`")
    _add_run_options(matrix)
    matrix.add_argument(
        "-o", "--output", action="append", metavar="FILE", help="also write the matrix as .md, .html or .json"
    )
    matrix.add_argument("--json-dir", metavar="DIR", help="also write one JSON report per robot")
    matrix.add_argument("--strict", action="store_true", help="exit 1 if any robot has a WARN")
    matrix.add_argument("-q", "--quiet", action="store_true", help="print nothing; use the exit code")
    matrix.set_defaults(fn=cmd_matrix)

    explain = commands.add_parser("explain", help="list checks, or explain one")
    explain.add_argument("check", nargs="?", help="a check id (inertia.clusters), a tier (dynamics) or a glob")
    explain.set_defaults(fn=cmd_explain)

    commands.add_parser("backends", help="installed engines and their options").set_defaults(fn=cmd_backends)

    robots = commands.add_parser("robots", help="list robot_descriptions models usable as rd:<name>")
    robots.add_argument(
        "filter", nargs="*", help="words to match in the name, maker or tags, e.g. 'unitree', 'humanoid', 'hand'"
    )
    robots.set_defaults(fn=cmd_robots)

    init = commands.add_parser("init", help="add a simcompat.toml profile and a GitHub Actions workflow")
    init.add_argument("robot", nargs="?", help="robot file (default: the single URDF/xacro found in --dir)")
    init.add_argument("--dir", default=".", help="project directory (default: current)")
    init.add_argument("--install", default=DEFAULT_INSTALL, metavar="PIP_SPEC", help="what the workflow installs")
    init.add_argument("--force", action="store_true", help="overwrite existing files")
    init.set_defaults(fn=cmd_init)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return EXIT_USAGE
    try:
        return args.fn(args)
    except SimCompatError as e:  # a problem with the input or options: the message says what to do
        _stderr(f"simcompat: error: {e}")
        return EXIT_USAGE
    except BrokenPipeError:  # the reader went away, e.g. `simcompat robots | head`
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())  # silence the final flush
        return EXIT_OK
    except OSError as e:  # a file could not be read or written
        _stderr(f"simcompat: error: {e}")
        return EXIT_USAGE
    except KeyboardInterrupt:
        _stderr("simcompat: interrupted")
        return 130
    except Exception:
        _stderr(traceback.format_exc())
        _stderr("simcompat: internal error. This is a bug in simcompat; please report it with the traceback above.")
        return EXIT_BUG


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
