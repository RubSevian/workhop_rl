# RARS01 AUTO HOME deployment

Owner alone opens serial and uses the existing saved SDK configuration. SDK source is unchanged. Physical profile holds **all seven motors at zero**, including closing the calibrated gripper to zero. Actor receives only joints 1..6.

The service is prepared, not installed/enabled by the implementation task. It starts at normal Jetson boot after deployment by the operator. It never launches Go2 LowCmd or switches Sport. The standalone owner defaults to `read_only=true`, `connect_serial=false`; the service explicitly selects the physical profile.

## Installation after separate arm commissioning

1. Build the Jazzy workspace with `scripts/build_r3.sh` and run `scripts/test_r3.sh`.
2. Copy `rars01-owner.env.example` to `/etc/rars01-owner.env`; set the actual existing SDK/calibration config. Optionally set the verified `/dev/serial/by-id/...` device; otherwise the existing config port is used. No guessed by-id path is shipped.
3. Copy `rars01-owner.service` to `/etc/systemd/system/`; adapt User/Group and absolute workspace path if deploying under a different account. Run daemon-reload and enable the unit for boot only after physical arm verification. Starting the unit performs real enable after usable communication plus 10 s.
4. Keep every manual SDK owner on the same `RARS_OWNER_LOCK_DIRECTORY`. Stop the SDK GUI and direct GraspNet hardware backend before this service takes serial ownership. Advisory leases cannot constrain unrelated programs that ignore them. No second owner is started by the Go2 launch.
5. Observe `/rars01/commissioning/state` and `/go2/locomotion_status`. HOME readiness is required before the existing gated A sequence may release Sport.

## Fault and restart

`Restart=no`. A hardware/runtime failure latches FAULT in the still-running diagnostic process and stops sending targets. A durable `/var/lib/rars01-owner/enable-journal` records the enable attempt before calling SDK. Same-boot restart after an attempt is blocked; a recorded FAULT remains blocked even across boot. There is no automatic recovery service and no automatic clearing of the journal.

Operator recovery must stop the unit, inspect/correct the fault and safely support/position the arm. Only then may the journal be explicitly cleared as a separate recovery action. Clearing it permits a new physical enable; do not automate that operation. A normally recorded enable attempt from a previous boot permits the next normal boot; interruption/restart within a boot conservatively becomes a persistent fault.

SDK destruction on intentional owner shutdown calls its existing best-effort disable. Stopping/restarting the leg controller does not stop this arm service. After runtime faults, actual mechanical behavior depends on the verified STM watchdog and motor firmware.

Communication must include valid feedback from all seven motors before the countdown. If a deployed STM only sends feedback after enable, startup remains WAIT_COMMUNICATION; no fabricated zeros bypass it. After SDK enable resets receiver statistics, the configured initial feedback grace allows waiting for the first new frame without claiming readiness.

## Versioned workspace helpers

`workspace_scripts/` preserves the currently used root `sim2real/scripts` helpers, including the AUTO HOME verifier update. To reproduce that workspace layout, copy these files to `sim2real/scripts/`. Their relative paths assume the existing sibling `repos/`, `weights/`, `build_r1/` and `install_r1/` layout; do not run them directly from this snapshot directory.

No trajectories, IK, grasp execution, navigation changes or automatic return to Sport are included.
