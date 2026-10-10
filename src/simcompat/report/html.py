"""Self-contained HTML report: one file, no external resources, light and dark themes.

Everything is expandable in place (``<details>``); a small script only powers the
"hide passing checks" toggle.
"""

from __future__ import annotations

import html

from ..checks.base import REGISTRY, Finding, Severity
from ..diagnosis import Problem
from .model import FILE_COLUMN, Report
from .wording import (
    MAX_LINES_PER_PROBLEM,
    NO_PROBLEMS,
    SEVERITY_MEANING,
    VERDICT_SENTENCE,
    counts_line,
    explained_summary,
    method_note,
    ordering_columns,
    problem_suffix,
    regraded_suffix,
)

STYLE = """
:root{--bg:#fff;--fg:#1d1f23;--muted:#5d6470;--line:#e3e6eb;--card:#f7f8fa;--code:#eef0f3;
--pass:#1a7f37;--skip:#8b949e;--info:#0969da;--warn:#9a6700;--fail:#cf222e;--error:#8250df}
@media (prefers-color-scheme:dark){:root{--bg:#0f1115;--fg:#e6e8eb;--muted:#9aa3ad;--line:#2a2f37;
--card:#171a20;--code:#20242c;--pass:#3fb950;--skip:#8b949e;--info:#58a6ff;--warn:#d29922;--fail:#f85149;
--error:#bc8cff}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);
font:15px/1.5 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:1100px;margin:0 auto;padding:24px 16px 64px}
h1{font-size:24px;margin:0 0 4px}h2{font-size:18px;margin:32px 0 12px}
code,.mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:13px}
code{background:var(--code);padding:1px 5px;border-radius:4px}
.muted{color:var(--muted)}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.badge{display:inline-block;padding:2px 10px;border-radius:999px;font-weight:600;font-size:13px;color:#fff}
.chip{display:inline-block;padding:1px 8px;border-radius:999px;border:1px solid var(--line);font-size:12px}
.s-PASS{background:var(--pass)}.s-SKIP{background:var(--skip)}.s-INFO{background:var(--info)}
.s-WARN{background:var(--warn)}.s-FAIL{background:var(--fail)}.s-ERROR{background:var(--error)}
.t-PASS{color:var(--pass)}.t-SKIP{color:var(--skip)}.t-INFO{color:var(--info)}.t-WARN{color:var(--warn)}
.t-FAIL{color:var(--fail)}.t-ERROR{color:var(--error)}
.verdict{margin:16px 0;padding:14px 16px;border-radius:10px;background:var(--card);border:1px solid var(--line)}
.card{border:1px solid var(--line);border-left:4px solid var(--line);border-radius:10px;padding:14px 16px;
margin:12px 0;background:var(--card)}
.card.b-WARN{border-left-color:var(--warn)}.card.b-FAIL{border-left-color:var(--fail)}
.card.b-ERROR{border-left-color:var(--error)}
.card h3{margin:0 0 6px;font-size:16px}
table{border-collapse:collapse;width:100%;margin:8px 0}th,td{text-align:left;padding:5px 8px;
border-bottom:1px solid var(--line);vertical-align:top}th{font-size:12px;color:var(--muted);font-weight:600}
.fix{margin-top:8px}.fix b{color:var(--pass)}
details{border:1px solid var(--line);border-radius:8px;margin:6px 0;background:var(--bg)}
details>summary{cursor:pointer;padding:8px 12px;list-style:none;display:flex;gap:10px;align-items:center;
flex-wrap:wrap}details>summary::-webkit-details-marker{display:none}
details>summary::before{content:"▸";color:var(--muted)}details[open]>summary::before{content:"▾"}
details .body{padding:0 12px 10px}
.tier{font-size:12px;color:var(--muted);width:78px}.title{flex:1;min-width:220px}
.doc{white-space:pre-wrap;color:var(--muted);font-size:13px;margin:6px 0}
.scroll{overflow-x:auto}label{font-size:13px;color:var(--muted);cursor:pointer}
.hide-pass details.all-pass{display:none}
"""

_SCRIPT = """
document.getElementById('hide-pass').addEventListener('change', e =>
  document.getElementById('checks').classList.toggle('hide-pass', e.target.checked));
"""


def _e(text) -> str:
    return html.escape(str(text))


def _badge(severity: Severity, text: str | None = None) -> str:
    return f'<span class="badge s-{severity.name}">{_e(text or severity.name)}</span>'


def render(report: Report) -> str:
    title = f"SimCompat · {report.title}"
    body = "".join(
        [_header(report), _problems(report), _checks(report), _ordering(report), _engines(report), _legend(report)]
    )
    return (
        f'<!doctype html><html lang="en"><head><meta charset="utf-8">'
        f'<meta name="viewport" content="width=device-width,initial-scale=1"><title>{_e(title)}</title>'
        f"<style>{STYLE}</style></head><body><main>{body}</main><script>{_SCRIPT}</script></body></html>"
    )


def _header(report: Report) -> str:
    model, verdict = report.model, report.verdict
    engines = "".join(
        f'<span class="chip">{_e(b.title)}</span>'
        if b.ok
        else f'<span class="chip t-ERROR">{_e(b.label)}: failed</span>'
        for b in report.backends
    )
    notes = "".join(f"<li>{_e(n)}</li>" for n in model.get("notes", []))
    return (
        f"<h1>SimCompat report: {_e(report.title)}</h1>"
        f'<div class="muted mono">{_e(model.get("input", model["path"]))} · {model["links"]} links · '
        f"{model['joints']} joints ({len(model['movable_joints'])} movable) · simcompat {_e(report.tool_version)} · "
        f"{_e(report.created)}</div>"
        f'<div class="verdict"><div class="row">{_badge(verdict)} <b>{_e(VERDICT_SENTENCE[verdict])}</b></div>'
        f'<div class="row muted" style="margin-top:6px">{_e(counts_line(report))} · engines: {engines}</div>'
        + (f'<ul class="muted">{notes}</ul>' if notes else "")
        + "</div>"
    )


def _findings_table(findings: list[Finding], limit: int | None = None) -> str:
    rows = []
    for i, f in enumerate(findings):
        hidden = ' style="display:none" class="more"' if limit is not None and i >= limit else ""
        rows.append(
            f'<tr{hidden}><td><span class="t-{f.severity.name}">{f.severity.name}</span></td>'
            f"<td>{_e(f.backend or FILE_COLUMN)}</td><td class='mono'>{_e(f.subject or '')}</td>"
            f"<td>{_e(f.message + regraded_suffix(f))}</td></tr>"
        )
    more = ""
    if limit is not None and len(findings) > limit:
        more = (
            f'<button class="chip" onclick="this.previousElementSibling.querySelectorAll(\'.more\')'
            f".forEach(r=>r.style.display='');this.remove()\">show {len(findings) - limit} more</button>"
        )
    return (
        '<div class="scroll"><table><tr><th>Severity</th><th>Engine</th><th>Joint / link</th><th>Finding</th></tr>'
        + "".join(rows)
        + "</table>"
        + more
        + "</div>"
    )


def _problem_card(number: int, problem: Problem) -> str:
    spec = REGISTRY.get(problem.check)
    suffix = f" <span class='muted'>{_e(problem_suffix(problem))}</span>" if problem_suffix(problem) else ""
    hints = "".join(f'<div class="fix"><b>Fix:</b> {_e(h)}</div>' for h in problem.hints[:2])
    explains = (
        f'<div class="muted" style="margin-top:6px">Also explains: {_e(explained_summary(problem.explains))}</div>'
        if problem.explains
        else ""
    )
    doc = (
        f"<details><summary>What does <code>{_e(problem.check)}</code> check?</summary>"
        f'<div class="body doc">{_e(spec.doc)}</div></details>'
        if spec
        else ""
    )
    engines = "".join(f'<span class="chip">{_e(e)}</span>' for e in problem.engines)
    check_code = f"<code>{_e(problem.check)}</code>" if spec else ""
    return (
        f'<div class="card b-{problem.severity.name}"><h3>{number}. {_e(problem.headline)}{suffix}</h3>'
        f'<div class="row">{_badge(problem.severity)}{check_code}{engines}</div>'
        f"{_findings_table(problem.findings, limit=MAX_LINES_PER_PROBLEM)}{hints}{explains}{doc}</div>"
    )


def _problems(report: Report) -> str:
    problems = report.problems
    if not problems:
        return f"<h2>Problems</h2><p>{_e(NO_PROBLEMS)}</p>"
    cards = "".join(_problem_card(i, p) for i, p in enumerate(problems, 1))
    return f"<h2>Problems ({len(problems)})</h2>{cards}"


def _checks(report: Report) -> str:
    matrix = report.matrix()
    columns = report.columns()
    blocks = []
    for check_id in report.check_ids():
        spec = REGISTRY.get(check_id)
        row = matrix.get(check_id, {})
        chips = "".join(
            f'<span class="chip"><span class="t-{row[c].name}">●</span> {_e(c)}</span>' for c in columns if c in row
        )
        all_pass = all(sev in (Severity.PASS, Severity.SKIP) for sev in row.values())
        doc = f'<div class="doc">{_e(spec.doc)}</div>' if spec else ""
        blocks.append(
            f'<details class="{"all-pass" if all_pass else ""}"><summary>'
            f'<span class="tier">{_e(spec.tier.label if spec else "")}</span>'
            f'<span class="title">{_e(spec.title if spec else check_id)} <code>{_e(check_id)}</code></span>'
            f'{chips}</summary><div class="body">{doc}{_findings_table(report.findings_for(check_id))}</div></details>'
        )
    return (
        '<h2>All checks</h2><div class="row"><label><input type="checkbox" id="hide-pass"> '
        f'hide checks where every engine passes</label></div><div id="checks">{"".join(blocks)}</div>'
    )


def _ordering(report: Report) -> str:
    table = report.ordering_table()
    if not table:
        return ""
    shown = ordering_columns(table)
    head = "".join(f"<th>{_e(k)}</th>" for k in shown)
    rows = "".join(
        f"<tr><td>{i}</td>"
        + "".join(f"<td class='mono'>{_e(o[i]) if i < len(o) else ''}</td>" for o in shown.values())
        + "</tr>"
        for i in range(max(len(o) for o in shown.values()))
    )
    follows = "; ".join(f"<b>{_e(k)}</b> follows {_e(v)}" for k, v in table["follows"].items())
    return (
        f"<h2>Joint ordering</h2><p>Index policy and controller vectors by joint <b>name</b>. {follows}.</p>"
        f'<details><summary>Show the table</summary><div class="body scroll"><table><tr><th>#</th>{head}</tr>'
        f"{rows}</table></div></details>"
    )


def _engines(report: Report) -> str:
    blocks = []
    for result in report.backends:
        if not result.ok:
            hint = f"<p><b>Hint:</b> {_e(result.hint)}</p>" if result.hint else ""
            blocks.append(
                f"<details open><summary>{_badge(Severity.ERROR, 'failed')} {_e(result.label)}</summary>"
                f'<div class="body"><p class="mono">{_e(result.error)}</p>{hint}</div></details>'
            )
            continue
        info = result.info
        items = "".join(f"<li>simcompat set: {_e(s)}</li>" for s in info.settings)
        items += "".join(f"<li>engine default, <b>{_e(k)}</b>: {_e(v)}</li>" for k, v in info.defaults.items())
        items += "".join(f"<li class='mono'>{_e(w)}</li>" for w in info.warnings[:10])
        blocks.append(
            f"<details><summary>{_e(result.title)}</summary><div class='body'><ul>{items}</ul></div></details>"
        )
    return "<h2>Engines and settings</h2>" + "".join(blocks)


def _legend(report: Report) -> str:
    legend = " ".join(f"{s.name}: {meaning}." for s, meaning in SEVERITY_MEANING.items())
    return f'<p class="muted" style="margin-top:32px;font-size:13px">{_e(method_note(report))} {_e(legend)}</p>'
