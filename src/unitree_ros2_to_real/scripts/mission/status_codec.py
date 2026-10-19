"""Decode status mappings with the same safe YAML schema and native parser.

The 100 Hz arm status is flow YAML, not JSON. PyYAML's Python parser can
consume almost an entire CPU core in each subscriber. CSafeLoader preserves
the safe constructors, including nonfinite feedback values during startup.
"""
import yaml

SAFE_STATUS_LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)


def decode_status(text):
    if len(text) > 2_000_000:
        raise ValueError('oversized input')
    value = yaml.load(text, Loader=SAFE_STATUS_LOADER)
    if not isinstance(value, dict):
        raise ValueError('status must be a mapping')
    return value
