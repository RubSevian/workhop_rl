# CODEX TASK — R3 RARS01 Auto-Home Owner

## Уточнение оператора: feedback после enable (04.10.2026)

Для этой STM motor feedback появляется после enable. До enable нельзя требовать motor feedback для начала startup countdown. Usable startup communication означает успешный SDK connect и работающий receiver. Порядок: connect → 10 с → enable once → немедленный HOME stream → ожидание реального feedback в ограниченном initial grace → HOME readiness. Во время grace readiness false; после timeout fault без повторного enable. LowCmd ног этим уточнением не разрешается.

## Goal

Implement the real Jetson runtime behavior for RARS01 without modifying `rars_arm_sdk`.

SDK source of truth:

```text
RubSevian/rars_arm_sdk
branch: main
SHA: f90278b46125f2b311e4555173321e80a6c7be3f
```

Verified SDK behavior:
- `connect()` only opens communication.
- `enable()` sends the enable sequence but does not start a hidden hold loop.
- `command_rate_hz` must be honored by the external caller.
- `sendPositionTargets()` must be streamed continuously.
- SDK already applies `direction` and `zero_offset`.
- `setZero()` must not be called at startup.
- Motors 0..5 are arm joints; motor 6 is the gripper.
- For this deployment, HOME for all 7 motors is zero.

```text
HOME_TARGET = [0,0,0,0,0,0,0]
```

## Required runtime

One persistent RARS owner process runs on Jetson and is the only process that owns the arm serial device.

```text
Jetson runtime starts
→ RARS owner starts
→ connect to STM32 through existing rars_arm_sdk
→ wait for valid communication
→ wait ~10 s
→ enable motors once
→ continuously stream HOME_TARGET at configured command_rate_hz
→ continuously read feedback
→ compute and publish arm_home_ready
```

The arm lifecycle is independent from Go2 Sport Mode.

The leg controller must not enable or command the arm when `L1+L2+A` is pressed.

## Startup delay

Add configurable startup behavior, e.g.

```yaml
real_deployment:
  rars01:
    auto_home:
      enabled: true
      startup_delay_s: 10.0
      home_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      home_tolerance_rad: 0.15
      command_rate_hz: 100.0
      enable_once_on_boot: true
    require_home_ready_for_leg_takeover: true
```

The 10 s countdown starts only after the owner has connected and communication is usable. Do not infer Jetson physical power-button time.

If the serial device is not ready:
- keep retrying connection safely;
- do not enable;
- do not start the enable countdown before communication exists.

## SDK usage

Use existing API only:

```cpp
rars_arm::RarsArm arm(config);

arm.connect();

// wait startup delay

arm.enable();

rars_arm::RarsArm::MotorValues target{};
target.fill(0.0F);

while (running) {
    arm.sendPositionTargets(target);
    arm.tryReadJointState(state);
    auto status = arm.communicationStatus();
}
```

Equivalent asynchronous implementation is fine.

Do not:
- call `setZero()`;
- rewrite `direction`;
- rewrite `zero_offset`;
- change saved calibration;
- invent new zeros.

The operator already calibrated/saved zero positions through the SDK GUI.

The SDK coordinate conversion remains authoritative.

## HOME target

Use zero for all seven motors:

```text
joint1  = 0
joint2  = 0
joint3  = 0
joint4  = 0
joint5  = 0
joint6  = 0
gripper = 0
```

Do not capture the current gripper position for startup HOME.

## Continuous hold

Do not assume:

```text
enable()
→ arm automatically holds zero forever
```

The current SDK does not provide that guarantee.

After enable, continuously call:

```text
sendPositionTargets([0,0,0,0,0,0,0])
```

using `ArmConfiguration.command_rate_hz` (normally 100 Hz).

If target streaming fails or becomes stale, `arm_home_ready` must become false.

## arm_home_ready

Expose a clear readiness status.

It should require at least:

```text
connected
SDK local enabled == true
fresh feedback
all 7 expected motor IDs valid
all 7 q/dq finite
all 7 motor statuses non-fault
no SDK feedback watchdog trip
no STM32 watchdog trip
HOME target stream fresh
all 7 measured positions within HOME tolerance
```

Use real feedback; never fabricate q=0.

Do not treat only `isEnabled()` as proof that the motors are physically healthy.

Expose diagnostics:
- connected
- enabled_local
- feedback_age
- motor_id[7]
- motor_status[7]
- q[7]
- dq[7]
- home_error[7]
- target age/freshness
- watchdog flags
- arm_home_ready
- last_error

## Enable/fault behavior

Enable exactly once during a normal successful boot.

Do not create an infinite automatic enable retry loop after a runtime hardware fault.

Separate:
- first boot startup;
- fault recovery.

After a runtime fault:
```text
FAULT_LATCHED
arm_home_ready = false
```

Do not let `systemd Restart=always` silently create repeated motor enables after hardware faults.

## Interaction with Go2 startup

Normal state before takeover:

```text
Go2:
Sport Mode active

RARS01:
connected
enabled
streaming [0,0,0,0,0,0,0]
holding HOME
arm_home_ready = true
```

Then the existing R3 remote sequence is:

```text
L1+L2+A
→ leg PRECHECK
→ check arm_home_ready
→ if true:
     continue automatic Sport release
     → STANDUP
     → startup DAMPING/HOLD
     → RL_ZERO
→ if false:
     keep Sport Mode unchanged
     do not create LowCmd ownership
```

The leg controller must not call:
- `arm.enable()`;
- `arm.setZero()`;
- `arm.sendPositionTargets()`.

It only consumes arm readiness/state.

## RL observation

Locomotion actor uses only arm joints 1..6:

```text
q_arm
dq_arm
q_arm_des
```

During HOME hold:

```text
q_arm_des = [0,0,0,0,0,0]
```

Gripper is also physically held at zero by the owner but remains excluded from the locomotion actor.

Accepted `q_arm_des` should be refreshed only after successful SDK target send.

If the HOME send fails/stales:

```text
arm_target_ready = false
arm_home_ready = false
```

## Single serial owner

Preserve exactly one serial owner:

```text
RARS OWNER
  ├─ owns serial device
  ├─ calls rars_arm_sdk
  ├─ reads feedback
  ├─ streams HOME and later manipulation targets
  └─ publishes state/readiness/accepted target

GraspNet / IK / mission code
  └─ sends desired arm targets to the owner through IPC/ROS
     and never opens the serial device directly
```

The same owner must later become the backend for real IK/grasp trajectories. Do not create a second arm-control process for later phases.

## Tests

Add offline/mock tests for:

1. startup delay:
```text
connected
→ before 10 s: no enable
→ after delay: enable exactly once
```

2. HOME target:
```text
sendPositionTargets([0,0,0,0,0,0,0])
```
including gripper zero.

3. continuous target streaming.

4. readiness:
- valid fresh 7-motor feedback near zero -> true;
- stale feedback -> false;
- disabled motor -> false;
- fault status -> false;
- bad ID -> false;
- NaN/Inf -> false;
- target stale/send failure -> false;
- watchdog trip -> false;
- position outside tolerance -> false.

5. SDK isolation:
- `setZero()` never called;
- calibration/direction/offset not modified.

6. leg integration:
```text
arm_home_ready=false + L1+L2+A
→ Sport release NOT requested

arm_home_ready=true + L1+L2+A
→ normal R3 startup may proceed
```

## Boot integration

Prepare a persistent Jetson service, for example:

```text
rars01-owner.service
```

Requirements:
- starts with the normal Jetson deployment;
- waits for the device safely;
- does not call `setZero`;
- does not erase latched hardware faults automatically;
- does not repeatedly re-enable motors after runtime faults;
- logs startup state.

Use the configured/stable device path; do not blindly hardcode `/dev/ttyACM0` if a better existing path/config is available.

## Out of scope

Do not implement in this task:
- GraspNet trajectory execution;
- IK execution;
- object search;
- grasp;
- payload;
- navigation changes;
- automatic return to Sport Mode.

This task is only:

```text
Jetson runtime
→ connect RARS
→ wait ~10 s
→ enable once
→ continuously hold all 7 joints at zero
→ expose honest arm_home_ready
→ let Go2 takeover only check readiness
```

## Report

Create:

```text
~/go2_diploma/sim2real/R3_ARM_AUTO_HOME_IMPLEMENTATION_REPORT.md
```

Include:
- changed files;
- SDK SHA;
- confirmation SDK itself was not modified;
- startup FSM;
- exact delay semantics;
- HOME target `[0,0,0,0,0,0,0]`;
- command rate source;
- proof `setZero()` is never called;
- readiness definition;
- fault behavior;
- accepted `q_arm_des`;
- single-owner design;
- boot/systemd design;
- offline test results;
- anything still requiring physical verification.

Do not claim physical HOME hold until tested on the real manipulator.

## Definition of done

Complete when:

```text
Jetson owner starts
+ connects to RARS
+ waits configured ~10 s
+ enables once
+ continuously streams [0,0,0,0,0,0,0]
+ reads feedback
+ computes arm_home_ready honestly
+ publishes measured q/dq and accepted q_des
+ Go2 L1+L2+A only checks arm_home_ready
+ no setZero
+ no calibration modification
+ single serial owner preserved
```

and all offline/mock regression tests pass.
