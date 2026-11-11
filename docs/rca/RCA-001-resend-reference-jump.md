# RCA-001: re-sending the active goal made the reference jump

**Status:** fixed · **Found by:** `FaultInjection.BaselineStreaming` (and every streaming fault test) + `Integration.GoalReachedWithLowTrackingError` · **Severity:** high (torque saturation on a normal command pattern)

## Symptom
First `colcon test` run (`results/before-fix/`): 9 of 26 results failed. Even the baseline (no fault) streaming case had
`max_ref_jump_rad_per_tick = 0.165277` (the bound is 2.61 rad/s × 1 ms = 0.0026 rad), `saturated_ticks = 3`,
`rms_tracking_err_rad = 0.0268768`. The ROS integration test failed `rms < 0.02` with `0.026880708512136777`.

## Reproduce
Client re-sends the same `JointTrajectory` (same `header.stamp`) every 100 ms to feed the 0.5 s command watchdog.
`test/test_fault_injection.cpp` → `BaselineStreaming`.

## Root cause
`ControllerCore::accept_goal` built the new trajectory from `desired(now)` but anchored its t = 0 at the message
stamp (`start_time`, 100 ms+ in the past). So every re-send started the quintic at the *current* mid-motion point
while the clock said it was already 0.1 s, 0.2 s, ... into it: the sampled reference skipped ahead each time.
Start state and start time came from two different instants.

## Fix
`include/arm_motion_control/controller.hpp`: start state = `desired(start_time)` (the current plan evaluated at the
new trajectory's own t = 0). A re-send of the same goal now rebuilds an identical trajectory.

## Verification (after fix, `results/fault_injection.json`, `results/metrics.json`)
- baseline: `max_ref_jump_rad_per_tick 0.0005625`, `saturated_ticks 0`, `rms_tracking_err_rad 0.00132506`
- ROS integration: `rms_tracking_err_rad 0.00132503`, final joint error `6.17946e-08` rad

## Guard
Every fault test asserts the reference never moves more than the velocity limit per 1 ms tick.
