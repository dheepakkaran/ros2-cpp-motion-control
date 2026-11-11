# RCA-002: late or stale message stamps made the reference jump / step to the goal

**Status:** fixed · **Found by:** `FaultInjection.LateMessages300ms`, `FaultInjection.StaleMessageOlderThanTrajectory` · **Severity:** high

## Symptom (`results/before-fix/fault_injection.json`)
- 300 ms late commands: `max_ref_jump_rad_per_tick 0.215161`, `saturated_ticks 6`.
- One command stamped 5 s in the past (whole 2 s trajectory already "over"): `max_ref_jump_rad_per_tick 0.6`
  (the full joint-1 move in one tick), `saturated_ticks 167` (torque at its limit for 167 ms: a step response).

## Root cause
The controller trusted `header.stamp` as the trajectory start. When the stamp is in the past and the arm is
holding, sampling at `now - stamp` lands in the middle (late) or at the end (stale) of the trajectory, so the
desired position jumps. Limit checks validate the *trajectory*, not the jump created by where we start sampling it.

## Fix
`ControllerCore::accept_goal`: after building, compare the new trajectory's state at `now` with the current
desired state (pos + vel, 1e-6 tolerance). If they differ, re-anchor: rebuild from the current desired state with
t = 0 = now (the move runs in full, shifted by the latency) and count it in `reanchored_goals()`. Re-sends of the
same stamp reuse that shift, so a late stream is re-anchored once, not on every message.
(The first version of the fix re-anchored every re-send; the per-stamp offset was added before the tests were re-run.)

## Verification (`results/fault_injection.json`)
- late 300 ms: `goals_reanchored 1`, `max_ref_jump_rad_per_tick 0.0005625`, `saturated_ticks 0`, `final_err_rad 6.43347e-08`
- stale 5 s: `goals_reanchored 1`, `max_ref_jump_rad_per_tick 0.00031514`, `saturated_ticks 0`; the single message is not
  refreshed, so the watchdog stops it at 0.5 s (`watchdog_trips 1`) as designed.

## Not done (cut)
No max-latency reject parameter; a stale goal is executed late instead of refused. A real system may prefer refusing.
