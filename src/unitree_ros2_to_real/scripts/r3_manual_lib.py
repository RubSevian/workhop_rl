"""Pure offline helpers; imports no ROS or robot SDK."""
import math

def validate(command, duration, bounds=(.20, .10, .10), max_duration=1.0):
    if len(command) != 3 or any(not math.isfinite(x) for x in command):
        raise ValueError('three finite values required')
    if sum(x != 0 for x in command) > 1:
        raise ValueError('one axis at a time')
    if any(abs(x) > b for x, b in zip(command, bounds)):
        raise ValueError('commissioning bounds exceeded')
    if not math.isfinite(duration) or not 0 < duration <= max_duration:
        raise ValueError('explicit bounded duration required')
    return tuple(command)

def eligible(status, received, now):
    return (0 <= now-received <= .25 and
            status.get('state') in ('RL_ZERO', 'RL_ACTIVE') and
            status.get('read_only') is False and status.get('output_enabled') is True and
            status.get('fault_latched') is False and not status.get('blockers', ['unknown']))
