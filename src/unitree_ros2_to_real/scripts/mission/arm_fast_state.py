"""Versioned ROS owner snapshot adapter. No expiry/session checks are bypassed."""
FIELDS = ('observed_ns', 'owner_session_id', 'mission_capable', 'mission_id', 'mission_state', 'mission_error', 'mission_blocker', 'home_requested', 'grasp_verification', 'grasp_assessment', 'grasp_assessment_reason', 'home_joint_count', 'gripper_command_active', 'gripper_hold_confirmed', 'gripper_command_torque_nm', 'gripper_hold_target', 'owner_state', 'connected', 'enabled_local', 'enable_attempted', 'command_rate_hz', 'successful_sends', 'first_accepted_ns', 'last_send_gap_ms', 'max_send_gap_ms', 'frame_received_ns', 'common_feedback_age_s', 'feedback_age_s', 'measured_q', 'measured_dq', 'motor_ids', 'valid', 'motor_id', 'motor_status', 'valid7', 'q', 'dq', 'torque', 'home_error', 'home_target', 'feedback_ready', 'motors_enabled', 'static_hold', 'arm_emergency_disable_enabled', 'arm_emergency_validated', 'arm_home_ready', 'watchdog_armed', 'watchdog_tripped', 'stm32_watchdog_tripped', 'protocol_v2_detected', 'per_joint_freshness_proven', 'target_age_s', 'target_valid', 'last_error')
ARRAYS = ('measured_q', 'measured_dq', 'motor_ids', 'valid', 'motor_id', 'motor_status', 'valid7', 'q', 'dq', 'torque', 'home_error', 'home_target')


def to_mapping(msg):
    if msg.schema_version != 1:
        raise ValueError('ARM_STATE_SCHEMA_VERSION')
    if sum(len(getattr(msg,key)) for key in ('owner_session_id','mission_state',
            'mission_error','mission_blocker','grasp_verification','grasp_assessment','grasp_assessment_reason','owner_state','last_error'))>2_000_000:
        raise ValueError('oversized input')
    value = {key: getattr(msg,key) for key in FIELDS}
    for key in ARRAYS:
        value[key] = list(value[key])
    value['object_grasped'] = msg.object_grasped if msg.object_grasped_known else None
    if msg.accepted_target_valid:
        value.update(accepted_target=list(msg.accepted_target),accepted_ns=msg.accepted_ns)
    return value


def from_mapping(value):
    """Synthetic fixture/parity helper, not a hardware publisher."""
    from rars01_state_interfaces.msg import ArmFastState
    msg=ArmFastState()
    for key in FIELDS:
        item=value.get(key,'UNKNOWN' if key=='grasp_assessment' else '') if key in ('grasp_assessment','grasp_assessment_reason') else value[key]
        if key in ARRAYS:
            item=[float(x) for x in item] if key not in ('motor_ids','motor_id','motor_status','valid','valid7') else item
        elif key in ('gripper_hold_target', 'target_age_s', 'last_send_gap_ms', 'max_send_gap_ms', 'feedback_age_s', 'common_feedback_age_s', 'gripper_command_torque_nm', 'command_rate_hz'):item=float(item)
        setattr(msg,key,item)
    msg.object_grasped_known=value.get('object_grasped') is not None
    msg.object_grasped=bool(value.get('object_grasped'))
    msg.accepted_target_valid='accepted_ns' in value
    if msg.accepted_target_valid:
        msg.accepted_ns=value['accepted_ns'];msg.accepted_target=[float(x) for x in value['accepted_target']]
    return msg
