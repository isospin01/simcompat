# Reproducing the validation

These scripts produce every number in [VALIDATION.md](../VALIDATION.md). They use the public
[robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py) corpus, which
downloads each robot's repository on first use (a few GB in `~/.cache/robot_descriptions`).

```bash
pip install -e ".[all]" yourdfpy           # Python 3.10-3.12 (PyBullet has no 3.13 wheels yet)

python validation/corpus.py    --out validation/results/corpus            # 1. every URDF model, 4 engine configs
python validation/oracle.py    --out validation/results/oracle.jsonl      # 2. simcompat's parser vs. yourdfpy
python validation/confirm.py   --corpus validation/results/corpus \
                               --out validation/results/confirm.jsonl     # 3. counterfactual re-runs
python validation/mutations.py --out validation/results/mutations.jsonl   # 4. seeded importer bugs
python validation/analyze.py                                              # -> validation/RESULTS.md
```

Each script takes `--only go2 g1 panda` to run a subset, and `corpus.py` / `mutations.py` take
`--jobs N`. On a 2-core machine the whole pipeline takes about 15 minutes after the first download
(the corpus run about 3 minutes, the seeded-bug benchmark about 7).

| Script | Question it answers | Output |
|---|---|---|
| `corpus.py` | Does simcompat run on every public model, and what does it find? Each robot runs in its own process, so an engine crash on one model cannot stop the run. | one JSON report per robot + `index.json` |
| `oracle.py` | Is simcompat's ground truth (its own URDF parser) right? Every link's mass and pose is compared with yourdfpy, an independent implementation. | `oracle.jsonl` |
| `confirm.py` | Are simcompat's attributions right? Each kind of cause it reports on the corpus is tested by changing what it blames (an engine setting, or the file) and re-running, or arithmetically. | `confirm.jsonl` |
| `mutations.py` | Would simcompat catch a real importer bug? Twelve bug classes taken from real issues are injected, one at a time, into MuJoCo's compiled model of every robot, then checked against Pinocchio. | `mutations.jsonl` |
| `analyze.py` | Aggregates all of the above. | `RESULTS.md` |

`results/` holds the runs used for the current VALIDATION.md, so `analyze.py` can be re-run (or
extended) without repeating them. Re-run the scripts when a check's logic changes, and update
VALIDATION.md if the numbers move.
