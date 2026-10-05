# CODEX — PHASE B.1: controlled lie-down, leg output OFF, restart from SYSTEM_HOLD

Date: 2026-10-05

Repository: `RubSevian/workhop_rl`  
Branch: `ros2_go2_rars01_real`

This task is an approved follow-up to `CODEX_SYSTEM_FSM_PHASE_B_REPORT.md`.

Do not redesign the seven-state System FSM. Do not change the validated A → stand → hold → RL path.

Main rule:

> **FSM follow-up = change only controlled-stop terminal behavior and the corresponding restart path.**

## 1. New required X behavior

`L1+L2+X` remains a controlled normal shutdown.

Required sequence:

```text
ACTIVE
→ CONTROLLED_STOP

1. block new NAV / remote velocity commands
2. set locomotion command exactly to [0,0,0]
3. keep RL ACTIVE
4. cancel navigation
5. cancel search / grasp / manipulation
6. command ARM_RETURN_HOME
7. keep RL running while the arm moves HOME
8. wait for fresh confirmed arm_home_ready
9. optional short HOME settle
10. capture fresh measured leg q
11. invalidate/stop RL cleanly
12. first fixed-PD target = captured measured leg q
13. start smooth lie-down interpolation
14. interpolate to approved lie_down_q
15. verify target reached
16. hold approved lie-down pose for a short configurable fixed-PD settle
17. cleanly stop custom leg LowCmd output
18. destroy/release this process LowCmd publisher/lease
19. enter SYSTEM_HOLD
```

Critical invariants:

```text
RL must remain active until ARM HOME is confirmed.
RL → fixed PD handoff must use fresh measured_q.
No target jump to default_dof_pos or stored lie_down_q at the handoff.
```

## 2. Approved lie-down target

Hardware order:

```text
FR, FL, RR, RL
```

Approved target:

```text
[ 0.01, 1.30, -2.70,
 -0.01, 1.30, -2.70,
 -0.30, 1.30, -2.70,
  0.30, 1.30, -2.70 ]
```

The target is derived from the real robot in its physical lying configuration.

Treat the target pose itself as approved.

The automatic trajectory dynamics still require physical commissioning:
- duration;
- Kp/Kd during lie-down;
- reached tolerance;
- settle duration;
- timeout.

Do not silently mark those dynamics as validated.

## 3. Trial enable semantics

The operator wants to physically test automatic X → lie-down now.

Do not globally mark production validation flags as true merely to make the test run.

Provide an explicit operator commissioning/trial gate, for example:

```text
controlled_stop_lie_down_trial = true
```

or reuse an equivalent existing candidate/trial configuration mechanism.

Requirements:
- default production config remains fail-closed;
- trial activation is explicit at launch/config time;
- status exposes that lie-down is running in commissioning/trial mode;
- target pose remains the approved target above;
- no runtime service/button may silently upgrade this capability.

## 4. Final SYSTEM_HOLD semantics

After successful X:

```text
SYSTEM_HOLD

Sport = RELEASED
Arm = HOME
RL = OFF
NAV = OFF
custom leg LowCmd output = OFF
LowCmd publisher = OFF
LowCmd lease = released
robot is physically lying in approved lie-down pose
```

Do not automatically return to Sport Mode.

Do not keep Kp=40 indefinitely after the robot has successfully reached and settled in the lying pose.

Important: stopping the custom LowCmd publisher is the software action. Do not overclaim hardware motor power-off semantics if firmware behavior is not explicitly measured. Status should distinguish:

```text
custom_leg_output = OFF
```

from any stronger physical statement such as confirmed electrical motor disable.

## 5. Output-stop implementation

Reuse the existing proven output-stop lifecycle where possible.

After:

```text
LIE_DOWN_REACHED
→ fixed-PD settle complete
```

perform:

```text
stop publishing LowCmd
→ release/destroy our publisher
→ release output lease
→ confirm output stopped
→ SYSTEM_HOLD
```

Do not:
- send an instantaneous zero-gain packet as a substitute for clean shutdown unless already part of proven existing semantics;
- enable Sport automatically;
- start another controller;
- create a second ownership path.

Late policy results and late async callbacks must be rejected by generation/state checks.

## 6. Failure handling during lie-down

If HOME is not reached but RL inputs remain valid:

```text
CONTROLLED_STOP::ARM_HOME_BLOCKED
→ command remains [0,0,0]
→ RL remains active
→ no lie-down
→ explicit blocker/status
```

If a critical input/policy/ownership fault makes RL unsafe:

```text
→ explicit critical event
→ EMERGENCY_FAULT according to central safety logic
```

If lie-down trajectory times out or target is not reached:

```text
CONTROLLED_STOP::LIE_DOWN_BLOCKED
→ do NOT stop LowCmd output
→ retain safe fixed-PD control at the last safe controlled target
→ explicit lie_down_timeout/blocker
→ wait for operator action / B
```

Only a positively confirmed `LIE_DOWN_REACHED + settle complete` may lead to normal output OFF.

## 7. New A behavior from SYSTEM_HOLD

Because normal X now stops custom leg output, a later `L1+L2+A` must reacquire output rather than reuse a live publisher.

Required path:

```text
SYSTEM_HOLD
→ A
→ TAKEOVER / readiness checks

if fresh Sport == RELEASED:
    skip Sport release RPC

→ verify no foreign LowCmd owner/publisher
→ acquire this process output lease
→ create custom LowCmd publisher
→ read fresh measured leg q
→ first LowCmd packet target = fresh measured q
→ HOLD_CURRENT
→ same existing gradual STAND
→ same existing HOLD
→ ResetPolicyState()
→ RL_ZERO
→ ACTIVE
```

Critical invariant:

```text
q_start = fresh measured_q_now
```

Do not start from stored lie_down_q, default_dof_pos, or stale captured values from the previous X.

## 8. Preserve existing stand/RL baseline

Do not change:

```text
capture_hold_sec = 0.02
stand_duration_sec = 6.0
hold_transition_sec = 4.0

fixed Kp = 40
fixed Kd = 1

RL Kp = 25
RL Kd = 1
```

Preserve:
- measured-q capture;
- stand interpolation math;
- default stand target;
- policy mapping;
- 315-D observation contract;
- history/reset behavior;
- 50 Hz policy loop;
- 500 Hz LowCmd loop;
- ticket/generation/watchdogs;
- first accepted policy semantics;
- packet/CRC;
- Sport helper;
- RARS calibration and single serial owner;
- current NAV bounds/clamp behavior.

A → RL_ZERO parity must remain unchanged.

## 9. Remote-test commissioning remains available

Do not remove or weaken `remote_test`.

Expected commissioning flow after RL_ZERO smoke:

```text
operation_profile:=remote_test
→ A
→ stand
→ hold
→ RL
→ neutral sticks = [0,0,0]
→ bounded remote vx/vy/wz commands
```

This is the intended profile for free RL walking from the Unitree remote during commissioning.

Do not mix NAV command source into REMOTE_TEST.

## 10. IMU calibration for autonomous navigation

The navigation repository already contains a standalone IMU calibration utility:

```text
RubSevian/autonomy_nav_go2
branch: ros2_Jazzy
package: src/utilities/calibrate_imu
executable: calibrate_imu
```

Existing command:

```bash
source install/setup.bash
ros2 run calibrate_imu calibrate_imu
```

Current utility behavior:
- subscribes `/utlidar/imu`;
- first ~2 s sends zero motion;
- keeps the robot static and collects bias data;
- from approximately 15 s to 35 s commands positive Z rotation at about 1.396 rad/s (~80 deg/s);
- estimates accelerometer/gyro bias and Z-axis projection terms;
- writes:

```text
~/Desktop/imu_calib_data.yaml
```

The original navigation stack expects that exact file name/path and loads it at startup.

Important integration note:

The existing `calibrate_imu` node directly uses the Unitree Sport API (`/api/sport/request`) and also publishes `/cmd_vel`.

Therefore it is not the same as the new `REMOTE_TEST` or `NAV_TEST` System FSM profile.

Recommended use:

```text
custom leg controller = STANDBY / LowCmd OFF
Go2 = normal Sport Mode
Arm owner may stay HOME
→ run standalone calibrate_imu
→ let the Sport-mode rotation sequence finish
→ confirm ~/Desktop/imu_calib_data.yaml exists
→ only then start/restart the autonomous navigation stack
```

Do not run the original Sport-mode calibration while an ACTIVE custom LowCmd/RL session owns the legs.

A later task may port IMU calibration rotation to the central FSM/RL command path, but that is not part of PHASE B.1.

## 11. Mandatory tests

Add/update tests for:

```text
X success:
ACTIVE
→ zero RL
→ ARM HOME
→ measured-q PD handoff
→ smooth lie-down
→ reached
→ settle
→ clean output stop
→ lease/publisher released
→ SYSTEM_HOLD
```

```text
X lie-down failure:
timeout/not reached
→ output remains ON
→ safe fixed-PD hold
→ explicit blocker
```

```text
A from SYSTEM_HOLD:
no live publisher
→ fresh RELEASED
→ verify foreign owner absent
→ reacquire lease
→ create publisher
→ first packet = fresh measured_q
→ same 6 s stand / 4 s hold / one reset
→ RL
```

Also verify:
- repeated X in SYSTEM_HOLD is idempotent;
- B has priority over X/A in every phase;
- no late policy/arm callback reactivates output after shutdown;
- no automatic Sport enable;
- baseline initial A differential remains byte-identical;
- REMOTE_TEST still accepts bounded remote commands after RL entry;
- NAV semantics remain unchanged.

## 12. Physical commissioning sequence

After offline tests pass:

```text
1. RL_ZERO_TEST
   A → stand → hold → RL zero

2. REMOTE_TEST
   small vx / vy / yaw commands from remote

3. controlled X trial
   zero RL → ARM HOME → lie-down → reached → settle → output OFF

4. A from lying SYSTEM_HOLD
   reacquire output → fresh measured-q first packet → stand → RL

5. repeat A/X cycle several times

6. only after that continue NAV/FULL commissioning
```

Physical observations must be logged separately from synthetic test PASS.

## Final instruction

Implement this as a small PHASE B.1 follow-up.

Do not reopen the whole FSM refactor.

The only intentional behavior changes are:

```text
normal X:
lie-down success
→ short settle
→ custom leg output OFF
→ release publisher/lease
→ SYSTEM_HOLD

A from SYSTEM_HOLD:
reacquire output
→ first packet fresh measured_q
→ existing stand/hold/reset/RL path
```

Keep all other validated behavior unchanged.
