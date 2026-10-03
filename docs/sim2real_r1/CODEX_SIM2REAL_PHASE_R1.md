# CODEX TASK — Sim2Real Phase R1: Jetson / ROS 2 Jazzy

## Context

We are moving the diploma project from the working Sim2Sim stack to a real Unitree Go2 + RARS01 system.

Target runtime:

- Jetson Orin Nano
- Ubuntu 24.04
- ROS 2 Jazzy
- aarch64
- Unitree Go2
- RARS01 manipulator
- new TorchScript locomotion policy with unified actor contract `315 -> 12`

Working root:

```text
~/go2_diploma/sim2real
```

Read-only Sim2Sim reference:

```text
~/go2_diploma/sim2sim_reference/go2_diploma_sim2sim
```

The reference is for comparison only. Do not modify it, do not source its Humble build/install, and do not use it as the Jetson runtime workspace.

## Repositories

Expected working repositories:

```text
~/go2_diploma/sim2real/repos/workhop_rl
  branch: ros2_go2_rars01_real
  base: ros2_go2_rars01_sim

~/go2_diploma/sim2real/repos/autonomy_nav_go2
  branch: ros2_Jazzy

~/go2_diploma/sim2real/repos/rars01_graspnet
  branch: jetson

~/go2_diploma/sim2real/repos/rars01_description
  branch: go2_arm

~/go2_diploma/sim2real/repos/rars_arm_sdk
  branch: main
```

`workhop_rl:origin/go2_nav` is a reference source for old real-robot/Jazzy/safety solutions. Do not merge it wholesale.

## Safety restriction

During Phase R1 do NOT:

- publish Unitree `LowCmd`;
- switch or release `sport_mode`;
- send commands to Go2 motors;
- open/send RARS serial motor commands;
- run any launch that can actuate the real robot;
- run automatic stand-up;
- run navigation that reaches the actuator output path.

All tests in this phase must be offline/unit/build tests only.

## Goal of Phase R1

Prepare `workhop_rl:ros2_go2_rars01_real` for a clean ROS 2 Jazzy build on the Jetson while preserving the validated Sim2Sim RL observation/action semantics.

The result must start DISARMED and must not produce real actuator output by default.

## Preserve from the current unified Sim2Sim controller

Keep the behavior/math of:

- `unified_observation_contract.hpp/.cpp`;
- unified `rl_agent.cpp`;
- actor input `315`;
- actor output `12`;
- 5 history frames × 63 observations;
- previous-action semantics;
- reset/history semantics;
- projected gravity convention;
- action scaling;
- default leg pose;
- `navigation_command_adapter` semantic behavior.

Do not redesign the RL observation contract.

Policy order:

```text
FL, FR, RL, RR
```

Each leg:

```text
hip, thigh, calf
```

Real Go2 motor order:

```text
FR, FL, RR, RL
```

Required motor -> policy mapping:

```text
[3,4,5, 0,1,2, 9,10,11, 6,7,8]
```

Audit all stand-up/hold/transition code so positions, gains and limits use the correct mapping.

## Phase R1 implementation work

### 1. Jazzy / Jetson build layer

Selectively port the useful build approach from `origin/go2_nav`:

- ROS 2 Jazzy underlay;
- C++20 where required by installed Torch;
- external `Torch_DIR`;
- correct aarch64 SDK2 handling;
- imported SDK2 target rather than x86_64 hardcoded paths;
- real-only targets by default;
- legacy MuJoCo / LCM / simulator targets disabled by default;
- do not use a vendored Humble RMW overlay;
- use the installed Jazzy CycloneDDS packages;
- keep SDK2-only utilities separate from the ROS controller process.

Do not blindly copy the old CMake files. Preserve current unified sources/tests.

### 2. Real controller configuration

Create a dedicated real profile, proposed path:

```text
src/unitree_ros2_to_real/config/go2_rars01_real.yaml
```

It must preserve the unified policy contract and expose real deployment thresholds separately from simulation defaults.

Do not silently fall back to legacy `weights/go2/config.yaml`.

The new policy must be selected explicitly.

### 3. Startup/output safety

Real controller startup must be:

```text
DISARMED
```

No actuator output is allowed merely because the node has started.

Introduce/test a clear output gate so an offline node can load configuration/model and execute non-hardware logic without publishing real motor commands.

Do not implement automatic stand-up as the default startup behavior.

### 4. Policy loading and offline inference

Verify the new TorchScript policy with the C++ runtime used by the real controller:

```text
input:  [1,315]
output: [1,12]
```

Run CPU inference timing measurements.

Target control period:

```text
20 ms / 50 Hz
```

Do not move RL inference to CUDA in Phase R1 unless required for diagnosis. CPU is the baseline.

### 5. Policy-state reset

Ensure entering RL mode resets policy state correctly using the current unified reset API.

The reset must:

- rebuild history from the current real observation;
- repeat the first current frame for the full history as defined by the unified contract;
- zero previous action;
- work on every re-entry to RL, not only once at process startup.

### 6. Stand/hold mapping fix

Fix the discovered index-order problem in real stand-up/hold/transition code.

Never apply policy-ordered defaults directly to hardware motor index `i`.

Add deterministic unit tests for:

- policy -> motor mapping;
- motor -> policy mapping;
- asymmetric hip signs;
- stand target mapping;
- hold target mapping.

### 7. SDK2 mode-switch source

Selectively bring in the SDK2-only sources from `origin/go2_nav`:

```text
go2_motion_mode.hpp
go2_motion_mode.cpp
go2_mode_switch.cpp
```

Build them if possible, but DO NOT run Sport Mode switching in Phase R1.

The ROS RL controller and SDK2 mode switch must remain separate processes.

### 8. Offline tests

Add or extend tests covering at minimum:

- observation dimension = 315;
- action dimension = 12;
- history order;
- reset semantics;
- previous-action reset;
- model loading;
- finite output from a zero/test observation;
- policy/motor index mapping;
- stand/hold mapping;
- startup DISARMED behavior;
- actuator-output gate;
- transition logic that can be tested without transport.

Do not require Go2 hardware for these tests.

## RARS01

The `rars_arm_sdk` repository is now available under:

```text
~/go2_diploma/sim2real/repos/rars_arm_sdk
```

For Phase R1:

- inspect it;
- record its current branch and SHA;
- identify how the Python/C++ API is exposed;
- identify dependencies needed for the later real bridge;
- do not open serial;
- do not send motor commands;
- do not yet implement the complete RARS ROS bridge.

The actual RARS bridge belongs to the next phase after the Jazzy/offline controller is stable.

## Navigation

Do not perform the FAR/localPlanner/Point-LIO behavioral backport in Phase R1.

Only inspect interfaces if needed to keep the controller API compatible with the future real navigation chain:

```text
Point-LIO
  -> terrain analysis
  -> FAR
  -> localPlanner
  -> pathFollower
  -> /cmd_vel (TwistStamped)
  -> navigation gate/watchdog
  -> unified RL
```

`autonomy_nav_go2:ros2_Jazzy` remains the Jazzy navigation base.

The later navigation phase will selectively restore diploma behavioral changes from the read-only Sim2Sim reference while preserving Jazzy API changes.

## Build isolation

The real build must not accidentally discover:

- Humble overlays from the reference;
- nested old `rmw_cyclonedds`;
- duplicate `unitree_go` / `unitree_api` packages;
- old x86_64 SDK libraries;
- simulation-only packages unless explicitly enabled.

Prefer the Jazzy `unitree_go` / `unitree_api` packages from `autonomy_nav_go2` as the single interface owner if consistent with the audit.

Document the final overlay order.

## Commands allowed in this phase

Allowed examples:

```bash
source /opt/ros/jazzy/setup.bash
colcon list
colcon build ...
colcon test ...
colcon test-result --verbose
cmake ...
python ...   # offline checks only
git status
git diff
git show
git rev-parse
```

Do not run hardware-actuating executables or launch files.

## Deliverables

After implementation, report:

1. exact changed files;
2. purpose of each change;
3. branch + SHA of every working repository, including `rars_arm_sdk`;
4. final Jazzy environment/overlay order;
5. exact build commands;
6. exact test commands;
7. test results;
8. C++ policy load result;
9. CPU inference timing at `[1,315] -> [1,12]`;
10. confirmation that startup is DISARMED and no LowCmd path is active by default;
11. remaining blockers for Phase R2;
12. any assumptions that still need verification on the physical Go2.

## Stop conditions

Stop and report instead of guessing if any of these are ambiguous:

- joint ordering;
- LowCmd packet layout;
- CRC behavior;
- Go2 firmware ownership semantics;
- policy/config identity;
- Torch ABI/linking;
- duplicate ROS interface ownership;
- RARS measured-state semantics;
- whether a command path can actuate hardware.

Do not work around a safety uncertainty by enabling output.
