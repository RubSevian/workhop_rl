#!/usr/bin/env python3
"""Independent reference for the Phase-2 Go2-RARS01 63D actor frame.

Provenance: workshop_legged_gym, branch: walk_diplom_arm_unified_eval_curriculum_symmetric_loss.
Relevant source paths: go2.py::_build_actor_frame, go2_arm_static_random.py::Go2WalkArmStaticRandomAware::_build_actor_frame,
and go2_arm_unified_curriculum.py::_build_actor_frame. This file does not import that repository or runtime code; it compares the result with
the C++ fixture emitted by unified_observation_contract_test.
"""

import argparse
import math
import subprocess
import sys
from pathlib import Path

FRAME = 63
HISTORY = 5
CLIP = 100.0
DEFAULT_LEGS = [0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
                0.1, 0.8, -1.5, -0.1, 0.8, -1.5]


def projected_gravity(quaternion):
    """Inverse-rotate world gravity [0, 0, -1] for xyzw quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("base_quat must be a non-zero finite xyzw quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [-2.0 * x * z + 2.0 * w * y,
            -2.0 * y * z - 2.0 * w * x,
            -2.0 * z * z - (2.0 * w * w - 1.0)]


def clip(values):
    return [max(-CLIP, min(CLIP, value)) for value in values]


def build_frame():
    ang_vel = [4.0, 8.0, 12.0]
    command = [1.0, 2.0, 4.0]
    leg_pos = [10.0 + index for index in range(12)]
    leg_vel = [30.0 + index for index in range(12)]
    previous_action = [50.0 + index for index in range(12)]
    arm_pos = [70.0 + index for index in range(6)]
    arm_vel = [80.0 + index for index in range(6)]
    arm_target = [90.0 + index for index in range(6)]
    frame = (
        [value * 0.25 for value in ang_vel]
        + projected_gravity([0.0, 0.0, 0.0, 1.0])
        + [command[0] * 2.0, command[1] * 2.0, command[2] * 0.25]
        + [value - default for value, default in zip(leg_pos, DEFAULT_LEGS)]
        + [value * 0.05 for value in leg_vel]
        + previous_action
        + arm_pos
        + [value * 0.05 for value in arm_vel]
        + arm_target
    )
    if len(frame) != FRAME:
        raise AssertionError(f"reference frame has {len(frame)} values, expected {FRAME}")
    return clip(frame)


def parse_fixture(stdout):
    parsed = {}
    for line in stdout.splitlines():
        values = line.split()
        if values:
            parsed[values[0]] = [float(value) for value in values[1:]]
    if set(parsed) != {"FRAME", "HISTORY"}:
        raise AssertionError("C++ fixture must emit FRAME and HISTORY rows")
    return parsed["FRAME"], parsed["HISTORY"]


def assert_close(name, actual, expected, tolerance=1e-6):
    if len(actual) != len(expected):
        raise AssertionError(f"{name}: length {len(actual)}, expected {len(expected)}")
    error = max(abs(left - right) for left, right in zip(actual, expected))
    if error > tolerance:
        raise AssertionError(f"{name}: max abs error {error} exceeds {tolerance}")
    print(f"{name}: max_abs_error={error:.3g}")


def main():
    parser = argparse.ArgumentParser()
    default_binary = Path(__file__).resolve().parents[1] / "build" / "unitree_rl_controller" / "unified_observation_contract_test"
    parser.add_argument("--cpp-test", type=Path, default=default_binary)
    args = parser.parse_args()
    if not args.cpp_test.is_file():
        raise FileNotFoundError(f"C++ parity fixture not found: {args.cpp_test}")

    fixture = subprocess.run([str(args.cpp_test), "--dump-parity-fixture"], check=True,
                             text=True, capture_output=True)
    cpp_frame, cpp_history = parse_fixture(fixture.stdout)
    frame = build_frame()
    history = frame * 3 + [1.0] * FRAME + [2.0] * FRAME
    assert_close("frame", cpp_frame, frame)
    assert_close("history", cpp_history, history)
    print("reference_unified_observation: PASS")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"reference_unified_observation: FAIL: {error}", file=sys.stderr)
        sys.exit(1)
