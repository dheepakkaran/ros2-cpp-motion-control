# RCA-003: faults that found no bug (handled by the design)

These faults were injected and passed on the first run of the fault tests' assertions that are about them
(their only first-run failures were the RCA-001 reference jump, which affected every streaming test). No bug was invented.

## Dropped messages (30 % random drop of 100 ms re-sends, seed 42)
`dropped_30pct`: `max_cmd_gap_s 0.3` < `cmd_timeout 0.5`, `watchdog_trips 0`, `final_err_rad 6.43347e-08`,
tracking identical to the baseline (`rms_tracking_err_rad 0.00132506`). Why: a re-send carries the full goal, so a
drop only delays the watchdog feed; the trajectory itself does not depend on each message.

## Commands stop mid-motion (client silent from 0.8 s)
`commands_stop_mid_motion`: `watchdog_trips 1`, `hold_drift_rad 0.0123103` (max motion after the trip while braking).
The reference steps once at the trip by `trip_ref_jump_rad 0.00176209`, equal to the tracking error at that tick
(`err_before_trip_rad 0.00176209`), because the watchdog holds the *measured* position. That is intended
(stop where the arm is); the test asserts the step is no larger than that error. Over ROS
(`Integration.WatchdogHoldsWhenCommandsStop`, timeout 0.3 s): drift over 1 s after the trip `0.000394` rad.

## Over-limit and NaN goals mid-motion
`bad_goals_mid_motion`: `goals_rejected 2`, motion continued to the original goal (`final_err_rad 6.43347e-08`).
Over ROS (`Integration.BadGoalRejectedOverRos`): 2 rejected, 0 accepted, drift `0.000000` rad, reason
`point 0: joint 1 position 2.500000 outside [-1.762800, 1.762800]`. Why: `Trajectory::build` validates finiteness,
position range, waypoint velocity and densely sampled peak velocity before anything replaces the active plan.
