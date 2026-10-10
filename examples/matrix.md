# SimCompat compatibility matrix

Generated 2026-10-09 by simcompat 0.2.0. Each cell shows the worst result for that engine and the causes behind it. A problem in the file itself is listed in the file column, and also in an engine's cell when it changes that engine's results. `simcompat explain <check>` describes each cause.

| Robot | DoF | file | pinocchio 4.1.0 | mujoco 3.15.0 | pybullet 3.2.7 | pybullet[inertia_from_file=1] 3.2.7 |
|---|---|---|---|---|---|---|
| `go2` | 12 | ⚠️ file.inertial | ✅ | ❌ inertia.clusters | ❌ inertia.clusters | ❌ inertia.clusters |
| `go1` | 12 | ✅ | ✅ | ⚠️ import.joint_order | ❌ inertia.clusters, import.joint_order | ⚠️ import.joint_order |
| `a1` | 12 | ⚠️ file.inertial | ✅ | ❌ inertia.clusters, import.joint_order | ❌ inertia.clusters, rollout.passive_effect, import.joint_order | ❌ inertia.clusters, rollout.passive_effect, import.joint_order |
| `anymal_c` | 12 | ⚠️ file.inertia_validity | ✅ | 💥 failed to load: Error: inertia must satisfy A + B &gt;= C; use 'balanceinertia' to fix | ❌ inertia.clusters, import.joint_order | ⚠️ import.joint_order |
| `g1` | 29 | ⚠️ file.inertial | ✅ | ✅ | ❌ inertia.clusters | ❌ inertia.clusters |
| `h1` | 19 | ⚠️ file.inertial | ✅ | ✅ | ❌ inertia.clusters | ❌ inertia.clusters |
| `talos` | 44 | ⚠️ file.inertia_validity, file.inertial | ✅ | 💥 failed to load: Error: inertia must satisfy A + B &gt;= C; use 'balanceinertia' to fix | ❌ inertia.clusters, rollout.passive_effect, import.joint_order, import.mimic | ❌ inertia.clusters, rollout.passive_effect, import.joint_order, import.mimic |
| `panda` | 9 | ✅ | ✅ | ✅ | ❌ inertia.clusters, import.mimic | ⚠️ import.mimic |
| `ur10e` | 6 | ⚠️ file.inertial | ✅ | ✅ | ❌ inertia.clusters | ❌ inertia.clusters |
| `iiwa14` | 7 | ⚠️ file.inertial | ✅ | ✅ | ❌ inertia.clusters | ❌ inertia.clusters |
| `allegro_hand` | 16 | ✅ | ✅ | ⚠️ import.joint_order | ❌ inertia.clusters, rollout.passive_effect, import.joint_order | ❌ rollout.passive_effect, import.joint_order |
| `fetch` | 14 | ⚠️ file.inertia_validity, file.inertial | ✅ | 💥 failed to load: Error: error 'inertia must have positive eigenvalues' in fullinertia | ❌ inertia.clusters, import.joint_order | ❌ inertia.clusters, import.joint_order |
