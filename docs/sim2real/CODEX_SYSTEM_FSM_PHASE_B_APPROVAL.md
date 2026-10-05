# CODEX — PHASE B approval and final constraints

Date: 2026-10-05

Repository: `RubSevian/workhop_rl`
Branch: `ros2_go2_rars01_real`
PHASE A baseline: `39b78c9f998f8c86368afb91774096e66418fc93`

Input plan:
- `CODEX_SYSTEM_FSM_REFACTOR_PLAN.md`

PHASE A is approved with the corrections below. After applying them to the plan/documentation, proceed with PHASE B in small reviewable commits.

---

## 1. Main rule

> **FSM refactor = orchestration refactor, not locomotion/control refactor.**

Do not rewrite working low-level behavior as part of the FSM migration.

---

## 2. Preserve current NAV bounds behavior

The PHASE A plan proposed changing NAV out-of-bounds handling from the current clamp behavior to reject/zero.

Do **not** make that change in this FSM refactor.

For PHASE B:

```text
current NAV bounds semantics
→ PRESERVE AS-IS
```

Specifically:
- preserve current `/cmd_vel` finite/freshness logic;
- preserve current bounds/clamp behavior;
- do not change clamp → reject/zero;
- do not alter planner/pathFollower algorithms;
- do not modify `autonomy_nav_go2` as part of this refactor.

Any future clamp → reject/zero change must be a separate explicitly approved change after the System FSM refactor.

Update the PHASE A plan so it no longer treats clamp → reject/zero as part of PHASE B.

---

## 3. Lie-down target is approved from the real lying pose

The lie-down target is derived from the physical robot in its lying configuration and is approved as the target pose.

Hardware order:

```text
FR, FL, RR, RL
```

Approved target:

```text
FR: [ 0.01, 1.30, -2.70]
FL: [-0.01, 1.30, -2.70]
RR: [-0.30, 1.30, -2.70]
RL: [ 0.30, 1.30, -2.70]
```

Full 12-joint hardware-order array:

```text
[ 0.01, 1.30, -2.70,
 -0.01, 1.30, -2.70,
 -0.30, 1.30, -2.70,
  0.30, 1.30, -2.70 ]
```

Treat this as:

```text
lie_down_q = APPROVED TARGET
source = physical lying-pose measurement / operator-derived real-robot pose
```

Do not mark the **joint target itself** as `NOT VERIFIED`.

What still requires physical commissioning is the automatic transition into this pose:
- trajectory duration;
- fixed PD gains during lie-down;
- reached tolerance;
- timeout;
- settle time;
- smoothness/stability under the real manipulator/payload configuration.

Correct the PHASE A wording accordingly.

---

## 4. Final L1 + L2 + X semantics

`L1+L2+X` is a controlled normal shutdown of the autonomous/custom-control session, not an emergency.

Required sequence:

```text
ACTIVE
→ CONTROLLED_STOP

1. block new NAV and remote velocity commands
2. set locomotion command exactly to [0,0,0]
3. keep RL ACTIVE
4. cancel navigation
5. cancel search / grasp / current manipulation
6. command ARM_RETURN_HOME
7. keep RL running while the arm moves HOME
8. RL continues receiving valid:
      q_arm
      dq_arm
      accepted q_arm_des → HOME
9. wait for confirmed arm_home_ready
10. optional short settle confirmation
11. capture fresh measured leg q
12. invalidate/stop RL cleanly
13. first fixed-PD target = captured measured leg q
14. begin smooth lie-down interpolation from captured measured q
15. interpolate to approved lie_down_q
16. verify lie-down reached
17. keep fixed PD hold at lie_down_q
18. enter SYSTEM_HOLD
```

Critical invariants:

```text
RL must NOT be disabled immediately on X.
```

```text
RL → fixed PD handoff must not create a q target jump.
```

Therefore:

```text
q_fixed_initial = measured_q_at_handoff
```

Do not substitute `default_dof_pos` at this handoff.

`LIE_DOWN` is an internal `CONTROLLED_STOP` phase, not a new top-level `SystemState`.

---

## 5. Failure behavior during X

### ARM HOME timeout

If the arm owner is healthy and arm observations remain valid, but HOME is not reached in time:

```text
CONTROLLED_STOP::ARM_HOME_BLOCKED
→ locomotion command stays [0,0,0]
→ RL remains active
→ publish explicit arm_home_timeout/blocker
→ do not silently disable RL
→ do not start lie-down
```

Do not hide this as an automatic emergency transition.

### Critical arm/policy/input failure

If RL can no longer safely run because required observations/control health become stale or invalid:

```text
explicit critical control fault
→ central FSM decides EMERGENCY_FAULT
```

Preserve existing fail-closed watchdog semantics.

### Lie-down timeout / not reached

If the automatic lie-down path does not reach the target in the allowed time:

```text
CONTROLLED_STOP::LIE_DOWN_BLOCKED
→ retain custom LowCmd ownership
→ retain safe fixed-PD control
→ publish explicit lie_down_timeout/blocker
→ no automatic Sport enable
→ no silent LowCmd shutdown
```

The fallback target must be explicit and covered by tests.

---

## 6. Final SYSTEM_HOLD semantics

After successful X:

```text
SYSTEM_HOLD

Sport = RELEASED
LowCmd ownership = THIS_PROCESS
LowCmd publisher = ON
RL = OFF
NAV = OFF
Arm = HOME
Legs = fixed PD hold at approved lie_down_q
```

Do not automatically return to Sport Mode.

Do not automatically destroy custom LowCmd ownership after normal X.

---

## 7. L1 + L2 + A from SYSTEM_HOLD

A second `L1+L2+A` must restart the robot normally from the lying custom-control state.

Required sequence:

```text
SYSTEM_HOLD
→ A
→ TAKEOVER / readiness checks

if fresh Sport == RELEASED:
    skip Sport release RPC

if LowCmd owner == THIS_PROCESS and lease/publisher are healthy:
    reuse existing ownership

→ capture fresh measured leg q
→ HOLD_CURRENT
→ same existing gradual STAND transition
→ same existing HOLD
→ ResetPolicyState()
→ RL_ZERO
→ ACTIVE
```

Stand-up must start from:

```text
q_start = fresh measured_q
```

not from the stored lie-down target.

---

## 8. Preserve validated A → RL behavior exactly

Preserve:

```text
Sport release
→ graph/ownership verification
→ first LowCmd at measured q
→ HOLD_CURRENT
→ STAND_TRANSITION
→ HOLDING
→ ResetPolicyState()
→ RL_ZERO
→ ACTIVE
```

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

Also preserve:
- measured-q capture behavior;
- stand interpolation math;
- `default_dof_pos` stand target;
- policy-to-hardware mapping;
- 315-D actor observation contract;
- policy history/reset behavior;
- Torch inference path;
- 50 Hz policy scheduling;
- 500 Hz LowCmd IO path;
- policy ticket/generation logic;
- policy watchdog thresholds;
- first-policy-result semantics;
- `MakeLowCmd` layout;
- CRC;
- Sport helper behavior;
- output lease;
- remote chord decoding;
- RARS calibration;
- RARS single serial owner;
- RARS HOME stream behavior.

For the same inputs/timestamps/profile, A → RL_ZERO must remain equivalent to baseline except for approved orchestration/status changes.

---

## 9. Central System FSM remains seven top-level states

Use exactly:

```text
INIT
STANDBY
TAKEOVER
ACTIVE
CONTROLLED_STOP
SYSTEM_HOLD
EMERGENCY_FAULT
```

Internal phases are not top-level states.

Suggested internal phases:

```text
TAKEOVER:
  PRECHECK
  SPORT_RELEASE
  SPORT_VERIFIED
  LOW_LEVEL_CAPTURE
  HOLD_CURRENT
  STAND
  HOLD
  RL_ZERO_ENTRY

CONTROLLED_STOP:
  ZERO_RL
  ARM_RETURN_HOME
  ARM_HOME_SETTLE
  PD_CAPTURE
  LIE_DOWN
  LIE_DOWN_VERIFY
  LIE_DOWN_HOLD
```

There must be only one authoritative mutable global SystemState owner.

Do not create a second competing FSM in `RealControllerCore` or elsewhere.

---

## 10. OperationProfile remains immutable

Supported launch-time profiles:

```text
read_only
arm_test
leg_safety_test
rl_zero_test
remote_test
nav_test
full_mission
```

Unknown profile:

```text
startup failure
→ no Sport release
→ no LowCmd
→ no physical side effects
```

No remote button, ROS topic, service, or runtime parameter may upgrade capabilities.

---

## 11. Arm readiness split is mandatory

Do not keep `arm_home_ready` as a continuous ACTIVE blocker.

Use `arm_home_ready` for:
- takeover;
- controlled-stop HOME confirmation;
- SYSTEM_HOLD expected posture.

Use during active manipulation/RL:

```text
arm_control_ready
fresh q_arm
fresh dq_arm
fresh accepted q_arm_des
owner healthy
no arm fault
```

In FULL_MISSION:

```text
arm away from HOME + healthy control
!= fault
```

The gripper remains excluded from actor observation, but may remain part of HOME readiness.

---

## 12. B remains highest priority

Priority:

```text
B > X > A > normal mission events
```

`L1+L2+B`:

```text
→ latch EMERGENCY_FAULT
→ block new mission/velocity/start events
→ invalidate current policy work
→ stop new inference
→ cancel navigation/manipulation
→ no HOME wait
→ Go2 leg emergency damping when eligible:
     Kp = 0
     Kd = 3
     dq = 0
     tau = 0
→ request arm emergency disable/relax through the single RARS owner
```

Do not copy Go2 leg damping gains to the arm.

Arm emergency disable/relax remains subject to separate bench/physical validation.

No automatic recovery from `EMERGENCY_FAULT`.

---

## 13. Required PHASE B commit order

Use small reviewable commits:

```text
1. freeze baseline fixtures / differential oracle
2. OperationProfile parser + immutable capabilities + tests
3. SystemReadiness split + tests
4. central SystemState/Event/Dispatch ownership
5. migrate A / takeover path with baseline parity
6. implement approved X orchestration
7. implement A restart from SYSTEM_HOLD
8. implement B orchestration / arm emergency port
9. remove continuous HOME blocker from ACTIVE runtime readiness
10. status / launch / config / compatibility cleanup
11. full regression + differential + timing tests
```

Do not combine X/B behavior changes with unrelated navigation, planner, actor, or transport refactors.

Each commit must compile and pass relevant tests before the next step.

---

## 14. Mandatory tests

### A baseline parity

Verify:

```text
STANDBY + A
→ exact baseline capture/stand/hold/reset/RL behavior
```

Compare:
- q targets over time;
- Kp/Kd;
- dq/tau;
- serialized LowCmd/CRC where practical;
- transition timestamps;
- policy reset timing;
- first accepted policy output timing.

### X

Verify:

```text
ACTIVE + X
→ command [0,0,0]
→ RL remains active before HOME
→ HOME request only once/idempotently
→ no PD handoff before HOME
→ capture measured q after HOME
→ no target jump
→ gradual lie-down
→ approved exact 12-joint target
→ final fixed hold
→ SYSTEM_HOLD
```

### A from SYSTEM_HOLD

Verify:

```text
SYSTEM_HOLD + A
→ no Sport release if already fresh RELEASED
→ reuse own healthy LowCmd ownership
→ fresh measured q capture
→ same 6 s stand
→ same 4 s hold
→ policy reset
→ RL_ZERO / ACTIVE
```

### X failures

Cover:
- HOME timeout;
- recoverable arm blocker;
- stale/invalid arm observations;
- policy failure while waiting HOME;
- lie-down timeout;
- B during every controlled-stop phase.

### Profiles

Verify services cannot bypass profile capability limits.

### NAV

During this refactor, assert current NAV bounds behavior remains unchanged.

Do **not** introduce clamp → reject/zero in PHASE B.

---

## 15. Documentation correction

Update:

```text
docs/sim2real/CODEX_SYSTEM_FSM_REFACTOR_PLAN.md
```

so that it reflects:

1. current NAV bounds/clamp behavior is preserved during this refactor;
2. lie-down target pose is approved from the physical lying configuration;
3. only automatic lie-down trajectory dynamics/criteria remain to be physically commissioned;
4. final X sequence is zero-RL stabilization → ARM HOME → RL handoff → lie-down → SYSTEM_HOLD;
5. A from SYSTEM_HOLD reuses custom ownership and starts stand from fresh measured q.

Do not rewrite the audit history; mark these as operator-approved corrections after PHASE A review.

---

## 16. Stop conditions

Stop and report instead of guessing if implementation would require changing any of these without explicit need:

- stand interpolation/control behavior;
- baseline A timing;
- policy observation layout;
- policy mapping;
- watchdog thresholds;
- LowCmd packet format/CRC;
- RARS calibration;
- RARS serial ownership model;
- planner algorithms;
- NAV command semantics;
- physical emergency arm behavior beyond an explicitly gated interface.

If a concrete bug requires one of these changes, document it separately and wait for operator approval before changing behavior.

---

## Final instruction

PHASE A is approved with the corrections in this file.

Proceed with PHASE B.

Keep the validated locomotion/control baseline intact and migrate ownership/orchestration first.

The intentional behavior changes approved for PHASE B are:

```text
X:
RL zero stabilization
→ ARM HOME
→ RL-to-PD measured-q handoff
→ smooth lie-down
→ SYSTEM_HOLD

A from SYSTEM_HOLD:
fresh measured-q capture
→ existing stand/hold/reset/RL path
without unnecessary Sport release

B:
central highest-priority emergency orchestration
with arm emergency interface gated by validation
```

No other control-behavior change is implicitly approved.
