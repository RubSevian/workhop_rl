"""JSON plan with a bounded first-line identity for rejection before decoding arrays."""
import json

IDENTITY_KEYS = ('id', 'session_id', 'owner_session_id', 'navigation_session_id', 'goal_revision')


def encode_plan(plan):
    identity = {key: plan[key] for key in IDENTITY_KEYS}
    header = json.dumps({'request_identity': identity}, allow_nan=False, separators=(',', ':'))
    if len(header.encode('utf-8')) > 2048:
        raise ValueError('PLAN_IDENTITY_TOO_LARGE')
    # Still ordinary JSON/YAML. The owner reads only this first line on its
    # control callback; expensive array parsing remains asynchronous.
    body = {key: value for key, value in plan.items() if key != 'request_identity'}
    return header[:-1] + ',\n' + json.dumps(body, allow_nan=False, separators=(',', ':'))[1:]
