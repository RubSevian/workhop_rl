# Sim2Real R1 workspace artifacts

This directory versions the audit, task, implementation report, scripts and
policy manifest that accompany the R1 source commit. The execution workspace
is the sibling layout `sim2real/repos/{workhop_rl,autonomy_nav_go2,...}`.

To restore the workspace artifacts from a fresh checkout, copy `scripts/` to
`sim2real/scripts/`, copy `weights/POLICY_MANIFEST.yaml` to
`sim2real/weights/POLICY_MANIFEST.yaml`, and copy the three reports/task files to
`sim2real/`. Run the scripts from that workspace location, as recorded in the
implementation report. The scripts in this archive are exact workspace copies.

`sim2real/weights/policy_2.pt` is a separately supplied binary. It is not included
in this commit; verify its SHA against the tracked manifest before building.
Build/install/log artifacts are local and are not committed. No actuator
executables should be run during R1.
