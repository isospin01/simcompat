# Examples

Real output of simcompat 0.2.0 (MuJoCo 3.15.0, PyBullet 3.2.7, Pinocchio 4.1.0), regenerated with the commands below.

| File | What it shows | Command |
|---|---|---|
| [`reports/go2.txt`](reports/go2.txt) | The terminal report for the Unitree Go2: two problems, each with the fix | `simcompat check rd:go2` |
| [`reports/go2.html`](reports/go2.html) | The same run as a self-contained HTML page (open it in a browser) | `simcompat check rd:go2 -o go2.html` |
| [`reports/go2.md`](reports/go2.md) | The same run as Markdown, as it appears in a GitHub job summary | `-o go2.md` |
| [`reports/go2.json`](reports/go2.json) | The same run as JSON (`simcompat.report/v1`), for scripts | `-o go2.json` |
| [`reports/divergent.md`](reports/divergent.md) | A small test robot built to diverge in many ways at once (missing inertial, friction, mimic) | `simcompat check tests/fixtures/divergent.urdf -o divergent.md` |
| [`matrix.md`](matrix.md), [`matrix.html`](matrix.html) | A compatibility matrix of twelve popular robots in four engine configurations | see below |

```bash
simcompat matrix rd:go2 rd:go1 rd:a1 rd:anymal_c rd:g1 rd:h1 rd:talos rd:panda rd:ur10e rd:iiwa14 \
  rd:allegro_hand rd:fetch -b pinocchio -b mujoco -b pybullet -b pybullet:inertia_from_file=1 \
  -o matrix.md -o matrix.html
```

How to read the matrix: each cell names the causes behind that engine's differences (`simcompat explain <check>` describes each one). For example, PyBullet's default column fails almost everywhere because PyBullet recomputes inertia from collision shapes unless it is told not to; the `inertia_from_file=1` column shows what is left once it is.
