"""Shared source-age contracts; facts here never grant actuator authority."""
STATUS_TOPICS = {
    'r3': '/go2/locomotion_status', 'owner': '/rars01/commissioning/state',
    'nav': '/go2/navigation/status', 'perception': '/rars01/mission/perception_health',
}


def source_ttl(cfg, name):
    if name in ('base', 'scene'):
        return cfg['limits']['sensor_age_s']
    execution = cfg['execution']
    key = {'r3': 'r3_status_ttl_s', 'nav': 'nav_status_ttl_s', 'perception': 'perception_status_ttl_s'}.get(name)
    return execution.get(key, execution['heartbeat_ttl_s']) if key else execution['heartbeat_ttl_s']


def input_diagnostic(cfg, name, data, now, *, phase, reason, action):
    stamp = data.get('observed_ns')
    return dict(subsystem=name, source_topic=STATUS_TOPICS.get(name, name), reason=reason,
                source_stamp=stamp, receive_mono_ns=int(now * 1e9),
                status_age_ms=(now - stamp / 1e9) * 1000 if type(stamp) is int else None,
                ttl_ms=source_ttl(cfg, name) * 1000, phase=phase, action_taken=action,
                session=data.get('session_id', data.get('controller_session_id', data.get('owner_session_id'))),
                operator_next_step='Wait for fresh valid input; inspect the named source if unavailable.')


CONTROLLED_HOME_PHASES=frozenset(('ARM_RETURN_HOME','ARM_HOME_BLOCKED','ARM_HOME_SETTLE'))
