# BUILD-LOG

Session 2026-11-09, one Claude Code session. Times are wall clock from the shell (`date`), 13:57 → ~14:10 EST.

## Done
| Step | Clock | Result |
|---|---|---|
| Repo + git, pixi standalone binary (`~/workspace/.tools/bin/pixi`, 0.84.1), pixi env (robostack-jazzy + conda-forge) | 13:57–13:59 | `pixi install` 21 s; rclcpp 28.1.22, gcc 15.3.0 (conda-forge) |
| MuJoCo C library | 13:59 | conda-forge `libmujoco` 3.16.0 ships headers + `mujocoConfig.cmake`; no tarball needed |
| Panda MJCF | 13:59 | Menagerie sparse clone → `assets/franka_emika_panda/` (`panda_nohand.xml` + used meshes + Apache-2.0 LICENSE) |
| Package `arm_motion_control` (header lib, sim wrapper, controller core, rclcpp node) | 13:59–14:02 | first `colcon build` passed (39 s, `--parallel-workers 1`, `MAKEFLAGS=-j2`) |
| GoogleTest unit + fault-injection + in-process integration tests | 14:02–14:03 | first `colcon test`: 9 failures (real bugs) — saved in `results/before-fix/` |
| Debug + fix (RCA-001 re-send jump, RCA-002 late/stale stamps) | 14:03–14:06 | `Summary: 26 tests, 0 errors, 0 failures, 0 skipped` |
| Repeat run, save logs, README / RESULTS / RCA docs, commit | 14:06–14:10 | run 2 numbers ≈ run 1 (see RESULTS.md) |

The rclcpp node compiled and ran about 5 minutes in, so the fallback plan without ROS 2 was not needed.

## Cuts and honest limits
- No Docker, no GitHub Actions, no Cartesian service, no clang-tidy.
- No `launch_testing` / `ros2` CLI: the integration test runs both nodes in one process with a MultiThreadedExecutor
  and talks over real ROS topics. `ros2cli` is not in the env, so there is no `ros2 topic pub` demo.
- Watchdog design needs the client to re-send the goal (same stamp) faster than `cmd_timeout`.
- Stale goals are re-anchored and executed late, not refused (no max-latency parameter).
- Timing is soft real time (normal kernel, wall timer); the measured step rate is in RESULTS.md, no RT guarantee claimed.
- No viewer / video (headless; `libmujoco` only).

## Remaining (optional)
- Add CI and a Dockerfile.
- `ros2cli` + a launch file demo; jerk logging; inertia feed-forward.

## Reusing the environment
The pixi env (ROS 2 Jazzy rclcpp + colcon + conda-forge compilers + `libmujoco` 3.16.0, `pixi.toml` + `pixi.lock`) can be copied into other ROS 2 + MuJoCo C++ projects as is. `.pixi/` is about 1.5 GB.
