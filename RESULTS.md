# Results

Simulation only (MuJoCo Franka Panda, no hardware). Every number below is copied from a file the code wrote;
the source file is named on each line. Run date 2026-11-09, Ubuntu, `colcon test` via `scripts/build_and_test.sh`.

## Test summary
- `colcon test-result --all`: **`Summary: 26 tests, 0 errors, 0 failures, 0 skipped`** (`results/colcon_test_result.txt`).
  The 26 count = 22 GoogleTest cases + 4 CTest entries (one per test binary).
- GoogleTest cases (`results/gtest_xml/*.xml`): `test_trajectory` 8, `test_watchdog` 5, `test_fault_injection` 6,
  `test_integration` 3 → 22 passed, 0 failed. Full log: `results/colcon_test_log/arm_motion_control/stdout_stderr.log`.
- First run before the fixes: `Summary: 26 tests, 0 errors, 9 failures, 0 skipped` (`results/before-fix/`), see `docs/rca/`.

## Integration test (real rclcpp node + client node, in-process executor, ROS topics only)
`results/metrics.json` (run 2; run 1 in `results/metrics_run1.json`). 7-joint move (joints 1, 2, 4, 6 change, up to 0.6 rad) in 2.0 s,
re-sent every 100 ms, then 1 s settle.

| Metric | Run 2 (`metrics.json`) | Run 1 (`metrics_run1.json`) |
|---|---|---|
| Final joint error, max over joints (from last `/joint_states`) | 6.17946e-08 rad | 6.00586e-08 rad |
| RMS tracking error while tracking (all joints, every 1 ms step) | 0.00132503 rad | 0.00132504 rad |
| Max tracking error | 0.00345585 rad | 0.00345587 rad |
| Sim steps per wall second (1 ms physics + control step) | 999.813 | 1000.22 |
| Timer callbacks per wall second (1 ms wall timer) | 999.813 | 990.876 |
| Lag resets (fell > 20 steps behind wall time) | 0 | 0 |
| `/joint_states` messages received by client (250 Hz publish) | 780 | 781 |
| Watchdog trips / goals accepted | 0 / 21 | 0 / 21 |

Control rate: the controller runs once per physics step (`sim_timestep` 0.001 s → 1 kHz in sim time); sim time
tracked wall time at 999.813 steps per wall second in run 2. Not a hard real-time guarantee (normal Linux, no RT kernel).

Watchdog over ROS (`metrics.json` → `watchdog`): timeout 0.3 s, trips 1, drift over 1 s after the trip 0.000394 rad.
Bad goals over ROS (`bad_goal_over_ros`): NaN + over-limit goal → rejected 2, drift 0.000000 rad.

## Fault injection (controller core + MuJoCo, sim time, deterministic) — `results/fault_injection.json`
| Fault | Outcome after fixes | Before fixes (`results/before-fix/fault_injection.json`) |
|---|---|---|
| Baseline: re-send every 100 ms | rms 0.00132506 rad, max ref jump 0.0005625 rad/tick, 0 saturated ticks, final err 6.43347e-08 rad | max ref jump 0.165277, 3 saturated ticks, rms 0.0268768 (RCA-001) |
| 30 % dropped messages | max gap 0.3 s, 0 watchdog trips, rms 0.00132506, final err 6.43347e-08 | same ref-jump bug (0.172595) |
| Commands stop at 0.8 s | 1 watchdog trip, hold drift 0.0123103 rad, trip step 0.00176209 rad (= tracking error then) | ref jump 0.165277 |
| 300 ms late messages | 1 re-anchor, max ref jump 0.0005625, 0 saturated, final err 6.43347e-08 | ref jump 0.215161, 6 saturated ticks (RCA-002) |
| One message 5 s stale | 1 re-anchor, max ref jump 0.00031514, 0 saturated; watchdog stops it (1 trip) | ref jump 0.6 rad, 167 saturated ticks (RCA-002) |
| Over-limit + NaN goal mid-motion | 2 rejected, motion continued, final err 6.43347e-08 | rejected 2 (ok), ref jump from RCA-001 |

## Unit tests (`test_trajectory`, `test_watchdog`)
Quintic endpoint pos/vel/acc match (1e-9), rest-to-rest midpoint + peak velocity 1.875·Δq/T, multi-waypoint pass-through,
rejects: position over limit, peak velocity over limit (2 rad in 0.5 s), NaN/Inf positions and times, empty / zero-time /
non-increasing / wrong-size goals; watchdog trip timing, 0 = disabled, holds measured position, refresh keeps tracking,
finished trajectory holds the goal without tripping.
