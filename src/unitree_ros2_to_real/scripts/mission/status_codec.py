"""Decode status mappings with the same safe YAML schema and native parser.

The 100 Hz arm status is flow YAML, not JSON. PyYAML's Python parser can
consume almost an entire CPU core in each subscriber. CSafeLoader preserves
the safe constructors, including nonfinite feedback values during startup.
"""
import json
import re
from functools import lru_cache
import yaml

SAFE_STATUS_LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)
_TOKENS = re.compile(r'"(?:[^"\\]|\\.)*"|[^{}\[\],:\s]+')
_INTEGER = re.compile(r'-?(?:0|[1-9][0-9]*)\Z')
_FLOAT = re.compile(r'-?(?:0|[1-9][0-9]*)\.[0-9]+(?:[eE][+-][0-9]+)?\Z')
_SCI_STRING = re.compile(r'[-+]?[0-9]+(?:\.[0-9]+)?[eE][+-]?[0-9]+\Z')
_PLAIN = re.compile(r'[A-Za-z_][A-Za-z0-9_./-]*\Z')
_BOOL = {value: literal for literal, values in (
    ('true', ('true','True','TRUE','yes','Yes','YES','on','On','ON')),
    ('false', ('false','False','FALSE','no','No','NO','off','Off','OFF')))
    for value in values}


@lru_cache(maxsize=512)
def _scalar_json(token):
    if token in _BOOL:return _BOOL[token]
    if token in ('~','null','Null','NULL'):return 'null'
    if _INTEGER.fullmatch(token) or _FLOAT.fullmatch(token):return token
    if _PLAIN.fullmatch(token) or _SCI_STRING.fullmatch(token):
        # Both whitelists contain only ASCII without quotes/control characters.
        return '"'+token+'"'
    raise ValueError('unsupported YAML scalar')


def _flow_mapping(text):
    """Fast, deliberately restricted YAML-flow subset emitted by our C++ nodes.

    JSON's C decoder constructs scalars/containers. Preserve YAML 1.1 types,
    including scientific notation that PyYAML regards as a string. Exotic
    YAML, nonfinite values and unusual quoting use the existing safe loader.
    """
    if not text.startswith('{') or not text.rstrip().endswith('}'):
        raise ValueError('not a flow mapping')
    def scalar(match):
        token=match.group()
        if token.startswith('"'):return token
        end=match.end()
        if end<len(text) and text[end]==':' and (end+1==len(text) or not text[end+1].isspace()):
            raise ValueError('noncanonical plain key separator')
        return (_scalar_json(token) if len(token)<=128
                else _scalar_json.__wrapped__(token))
    return json.loads(_TOKENS.sub(scalar,text))


def decode_status(text):
    if len(text) > 2_000_000:
        raise ValueError('oversized input')
    try:
        value = _flow_mapping(text)
    except (ValueError,OverflowError):
        value = yaml.load(text, Loader=SAFE_STATUS_LOADER)
    if not isinstance(value, dict):
        raise ValueError('status must be a mapping')
    return value
