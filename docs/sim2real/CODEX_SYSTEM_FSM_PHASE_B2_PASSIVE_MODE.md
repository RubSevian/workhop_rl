# CODEX — PHASE B.2: explicit Go2 passive mode after lie-down

Date: 2026-10-05

Repository: `RubSevian/workhop_rl`
Branch: `ros2_go2_rars01_real`

This is a small follow-up to PHASE B.1.

Do not reopen the seven-state FSM architecture.
Do not change the validated A → stand → hold → RL path.

## 1. Purpose

Official Unitree low-level examples distinguish:

```text
mode = 0x01  → active servo/PMSM control
mode = 0x00  → passive mode
```

Therefore a successful normal X should explicitly command PASSIVE mode before the LowCmd publisher and lease are released.

## 2. Final X sequence

```text
ACTIVE
→ CONTROLLED_STOP
→ block NAV/remote velocity
→ command [0,0,0]
→ keep RL active
→ cancel NAV/manipulation
→ ARM_RETURN_HOME
→ wait fresh arm_home_ready
→ optional HOME settle
→ capture fresh measured leg q
→ RL → fixed-PD handoff at captured measured q
→ smooth lie-down interpolation
→ verify approved lie_down_q reached
→ short fixed-PD settle
→ explicit PASSIVE transition for all 12 leg motors
→ short bounded sequence of valid passive LowCmd packets
→ stop LowCmd publication
→ destroy publisher
→ release lease
→ confirm output stopped
→ SYSTEM_HOLD
```

Critical invariants:

```text
RL remains active until ARM HOME is confirmed.
First fixed-PD target after RL = fresh measured_q.
No jump to default_dof_pos or lie_down_q at the handoff.
```

## 3. PASSIVE packet

For all 12 leg motors:

```text
mode = 0x00
kp   = 0
kd   = 0
tau  = 0
```

Use the existing LowCmd/CRC path.

For q/dq, use the existing Unitree non-commanding sentinel convention already used by this codebase if available; do not invent a position-control target for passive mode.

The passive sequence must be short and bounded and sent through the existing 500 Hz output path.

Do not leave an infinite passive publisher running.

## 4. Approved lying target

Hardware order FR, FL, RR, RL:

```text
[ 0.01, 1.30, -2.70,
 -0.01, 1.30, -2.70,
 -0.30, 1.30, -2.70,
  0.30, 1.30, -2.70 ]
```

The target is approved because it was derived from the physical lying pose.

Trajectory dynamics still remain commissioning parameters.

## 5. SYSTEM_HOLD after successful X

```text
state = SYSTEM_HOLD
Sport = RELEASED
RL = OFF
Arm = HOME
custom LowCmd publisher = OFF
LowCmd lease = released
last commanded leg mode = PASSIVE (0x00)
```

Status should distinguish:

```text
custom_leg_output = OFF
passive_command_sent = true
motor_power_off_confirmed = false
```

Do not claim electrical power-off from software alone.

## 6. A after X

Yes: after successful X, the operator presses `L1+L2+A` again to raise the robot and return to RL.

Required path:

```text
SYSTEM_HOLD
→ L1+L2+A
→ readiness checks

if Sport is still fresh RELEASED:
    skip Sport release RPC

→ verify no foreign LowCmd owner/publisher
→ reacquire LowCmd lease
→ create LowCmd publisher
→ get fresh LowState / measured q
→ restore active leg control:
     mode = 0x01 for all 12 motors
→ first active q target = fresh measured_q
→ HOLD_CURRENT
→ same 6 s STAND interpolation
→ same 4 s HOLD
→ ResetPolicyState()
→ RL_ZERO
→ ACTIVE
```

Critical requirements:

```text
mode = 0x01 before/with the first active LowCmd packet
q_first = fresh measured_q
```

Do not use stored lie_down_q or default_dof_pos as the first active target.

## 7. X vs B

Normal X:

```text
zero RL
→ ARM HOME
→ lie-down
→ settle
→ mode=0x00 PASSIVE
→ output OFF
→ SYSTEM_HOLD
```

Emergency B stays unchanged:

```text
EMERGENCY_FAULT
→ no HOME wait
→ eligible leg damping Kp=0, Kd=3, dq=0, tau=0
→ separately gated arm emergency action
```

Do not replace B damping with passive mode in this task.

## 8. Preserve baseline

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

Also preserve actor 315-D contract, motor mapping, policy reset/history, watchdogs, 50 Hz policy, 500 Hz IO, packet CRC/layout except the intentional motor mode change, Sport helper, leases, RARS calibration/owner, and current NAV semantics.

Initial A → RL baseline parity must remain unchanged.

## 9. Mandatory tests

### Passive transition after successful X

Verify all 12 motors:

```text
mode == 0x00
kp == 0
kd == 0
tau == 0
CRC valid
```

Verify passive packets are sent before publisher/lease shutdown.

### No false passive success

On HOME timeout, lie-down timeout, reached dropout, or critical fault:
- do not report successful passive shutdown;
- keep the relevant safe failure behavior.

### Restart after passive

Verify:

```text
SYSTEM_HOLD + A
→ reacquire lease/publisher
→ first active packet has mode=0x01 on all 12 motors
→ q = fresh measured_q
→ same stand/hold/reset/RL timing
```

Also verify stale callbacks cannot recreate output after shutdown, repeated X in SYSTEM_HOLD is idempotent, B has priority, and Sport is not auto-enabled.

## 10. Physical test tomorrow

```text
1. RL_ZERO_TEST
   A → stand → hold → RL zero

2. REMOTE_TEST
   small bounded vx/vy/yaw

3. X trial
   zero RL
   → ARM HOME
   → lie-down
   → settle
   → mode=0x00 PASSIVE
   → output OFF

4. physically verify legs are passive/relaxed

5. press L1+L2+A again
   → reacquire output
   → mode=0x01
   → first target = fresh measured_q
   → stand
   → hold
   → RL

6. repeat A/X a few times
```

Log software status and physical behavior separately.

## Final instruction

Implement only:

```text
successful X:
lie-down
→ settle
→ explicit mode=0x00 PASSIVE
→ short bounded valid passive packet sequence
→ output OFF
→ SYSTEM_HOLD

A after X:
reacquire output
→ restore mode=0x01
→ first target fresh measured_q
→ existing stand/hold/RL path
```

No other control behavior changes are approved.
