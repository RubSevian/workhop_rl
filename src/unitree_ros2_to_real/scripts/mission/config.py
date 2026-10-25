from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml

def environment_checks_enabled(collision):
    enabled = collision.get('environment_enabled', True)
    if type(enabled) is not bool:
        raise ValueError('collision.environment_enabled must be a boolean')
    return enabled

def payload_checks_enabled(collision):
    enabled = collision.get('payload_enabled', True)
    if type(enabled) is not bool:
        raise ValueError('collision.payload_enabled must be a boolean')
    return enabled

def apply_operator_trial(cfg, enabled):
    """Explicit staged commissioning, never physical qualification evidence."""
    if type(enabled) is not bool:raise ValueError('operator_trial must be boolean')
    if enabled and cfg.get('test_mode','full')!='full':raise ValueError('operator_trial requires test_mode=full')
    cfg['operator_trial']=enabled
    return cfg


def physical_retreat_qualified(collision):
    return (collision.get('retreat_physical_clearance_validated') is True
            and isinstance(collision.get('retreat_physical_clearance_evidence'),str)
            and bool(collision['retreat_physical_clearance_evidence'].strip()))

def self_collision_pairs(collision, name):
    pairs = collision.get(name, [])
    if not isinstance(pairs, list):
        raise ValueError(name+' must be a list of link pairs')
    result = set()
    for pair in pairs:
        if (not isinstance(pair, list) or len(pair) != 2
                or any(not isinstance(link, str) or not link for link in pair)
                or pair[0] == pair[1]):
            raise ValueError('invalid '+name+' link pair')
        key = frozenset(pair)
        if key in result:
            raise ValueError('duplicate '+name+' link pair')
        result.add(key)
    return result

def ignored_self_collision_pairs(collision):
    return self_collision_pairs(collision, 'ignored_self_collision_pairs')

def allowed_zero_contact_pairs(collision):
    return self_collision_pairs(collision, 'allowed_zero_contact_pairs')

def search_angles(search):
    count=round((search['joint1_max_rad']-search['joint1_min_rad'])/search['joint1_step_rad'])+1
    entry=round((search['joint1_start_rad']-search['joint1_min_rad'])/search['joint1_step_rad'])
    return ([search['joint1_start_rad']-i*search['joint1_step_rad'] for i in range(entry)]
            +[search['joint1_min_rad']+i*search['joint1_step_rad'] for i in range(count)])

def load(path, workspace):
    cfg = yaml.safe_load(Path(path).read_text())['go2_mission']
    if type(cfg.get('local_height_filters_enabled',True)) is not bool:
        raise ValueError('local_height_filters_enabled must be boolean')
    if cfg.get('test_mode','full') not in ('full','ready_observe'):
        raise ValueError('invalid test_mode')
    aliases=cfg.get('object_class_aliases',[])
    if not isinstance(aliases,list) or any(not isinstance(name,str) or not name.strip() for name in aliases):
        raise ValueError('invalid object_class_aliases')
    if cfg['version'] != 1:
        raise ValueError('unsupported mission config')
    for key, value in cfg['paths'].items():
        cfg['paths'][key] = str((Path(workspace) / value).absolute())
    deployment=yaml.safe_load(Path(cfg['paths']['deployment']).read_text())['real_deployment']
    navigation=yaml.safe_load(Path(cfg['paths']['navigation']).read_text())
    # This module is also loaded directly by launch via importlib (no package).
    for key, maximum in (('r3_status_ttl_s', .50), ('nav_status_ttl_s', .60), ('perception_status_ttl_s', 1.50)):
        value=cfg['execution'].get(key,cfg['execution']['heartbeat_ttl_s'])
        if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=maximum:
            raise ValueError('invalid execution.'+key)
    legs_age=cfg['frames'].get('leg_feedback_max_age_s',deployment['rars01']['feedback_timeout_s'])
    if type(legs_age) not in (int,float) or not math.isfinite(legs_age) or not 0<legs_age<=.5:
        raise ValueError('invalid frames.leg_feedback_max_age_s (maximum 0.5 s)')
    arm_age=cfg['frames'].get('arm_feedback_max_age_s',deployment['rars01']['feedback_timeout_s'])
    if type(arm_age) not in (int,float) or not math.isfinite(arm_age) or not 0<arm_age<=.5:
        raise ValueError('invalid frames.arm_feedback_max_age_s (maximum 0.5 s)')
    cfg['limits']={'legs_age_s':legs_age,'sensor_age_s':navigation['bridge']['sensor_timeout_s'],
                   'arm_age_s':arm_age,
                   'tf_position_tolerance_m':navigation['bridge']['tf_pair_position_tolerance_m'],
                   'tf_quaternion_tolerance':navigation['bridge']['tf_pair_quaternion_tolerance']}
    # Never feed malformed limits to search/timers, even in read-only mode.
    for group in ('search', 'refinement', 'settle', 'execution'):
        for key, value in cfg[group].items():
            if group == 'refinement' and key == 'enabled':
                if type(value) is not bool:raise ValueError('invalid refinement.enabled')
                continue
            if group == 'execution' and key in ('detection_debug_enabled','diagnostic_recording_enabled','short_journal_enabled'):
                if type(value) is not bool:raise ValueError('invalid execution.'+key)
                continue
            if group == 'execution' and key == 'arm_state_transport':
                if value not in ('yaml','typed'):
                    raise ValueError('invalid execution.arm_state_transport')
                continue
            if group == 'search' and key == 'mode':
                continue
            if value is None and (group,key) in {('search','timeout_s'),('execution','state_timeout_s'),('execution','mission_timeout_s'),('execution','navigation_timeout_s')}:
                continue
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'invalid {group}.{key}')
            if key not in ('joint1_min_rad','joint1_start_rad') and value <= 0:
                raise ValueError(f'nonpositive {group}.{key}')
    nice=cfg['compute'].get('inference_nice',0)
    if type(nice) is not int or not 0<=nice<=19:raise ValueError('compute.inference_nice must be in [0,19]')
    if type(cfg['compute']['cpu_threads']) is not int or cfg['compute']['cpu_threads']<1:
        raise ValueError('compute.cpu_threads must be a positive integer')
    if type(cfg['compute'].get('grasp_full_diagnostics',True)) is not bool:
        raise ValueError('compute.grasp_full_diagnostics must be boolean')
    if type(cfg['compute'].get('grasp_debug_enabled',False)) is not bool:
        raise ValueError('compute.grasp_debug_enabled must be boolean')
    startup=cfg['startup']
    if type(startup['camera_probe_frames']) is not int or startup['camera_probe_frames']<1:
        raise ValueError('startup.camera_probe_frames must be positive integer')
    for key in ('camera_probe_timeout_s','camera_stop_timeout_s','model_timeout_s','shutdown_timeout_s'):
        if type(startup[key]) not in (int,float) or not math.isfinite(startup[key]) or startup[key]<=0:
            raise ValueError('invalid startup.'+key)
    scan = cfg['search']
    mode = scan.get('mode', 'stop_and_observe')
    if mode == 'continuous':
        raise ValueError('search.mode continuous is reserved but not implemented; use stop_and_observe')
    if mode != 'stop_and_observe':
        raise ValueError('invalid search.mode')
    if scan['joint1_min_rad'] >= scan['joint1_max_rad'] or type(scan['max_attempts']) is not int:
        raise ValueError('invalid bounded scan')
    intervals = (scan['joint1_max_rad']-scan['joint1_min_rad'])/scan['joint1_step_rad']
    entry=(scan['joint1_start_rad']-scan['joint1_min_rad'])/scan['joint1_step_rad']
    if (not math.isclose(intervals,round(intervals),abs_tol=1e-9)
            or not math.isclose(entry,round(entry),abs_tol=1e-9)
            or len(search_angles(scan))!=scan['max_attempts']):
        raise ValueError('scan limits/step/attempts disagree')
    if not scan['joint1_min_rad']<=scan['joint1_start_rad']<=scan['joint1_max_rad']:
        raise ValueError('search start outside sweep bounds')
    for path in ('arm_urdf', 'full_urdf'):
        limit = ET.parse(cfg['paths'][path]).getroot().find("joint[@name='joint1']/limit")
        if limit is None or not (float(limit.get('lower')) <= scan['joint1_min_rad']
                                 < scan['joint1_max_rad'] <= float(limit.get('upper'))):
            raise ValueError('search exceeds joint1 limits in '+path)
    observations=scan['observations'];confirmations=scan.get('confirmation_frames',observations)
    if (type(observations) is not int or type(confirmations) is not int
            or not 1<=confirmations<=observations<=5):
        raise ValueError('search requires 1 <= confirmation_frames <= observations <= 5')
    if not 3 <= cfg['refinement']['fresh_frames'] <= 5:
        raise ValueError('refinement requires 3-5 fresh frames')
    if cfg['settle']['min_stable_s'] < 5 or cfg['settle']['timeout_s'] <= cfg['settle']['min_stable_s']:
        raise ValueError('settle requires >=5 seconds and a longer finite timeout')
    for key in ('max_state_skew_s', 'continuity_translation_m', 'continuity_rotation_rad'):
        value=cfg['frames'][key]
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid frames.'+key)
    for key in ('odometry_buffer_size','tf_lookup_attempts_per_tick'):
        if type(cfg['frames'][key]) is not int or cfg['frames'][key]<1:
            raise ValueError('invalid frames.'+key)
    history=cfg['frames']['odometry_history_s']
    depth=cfg['frames'].get('arm_history_depth',1)
    if type(depth) is not int or not 1<=depth<=400:
        raise ValueError('frames.arm_history_depth must be in [1,400]')
    arm_tolerance=cfg['collision'].get('arm_joint_measurement_tolerance_rad',0.)
    if type(arm_tolerance) not in (int,float) or not math.isfinite(arm_tolerance) or not 0<=arm_tolerance<=math.radians(2)+1e-12:
        raise ValueError('invalid collision.arm_joint_measurement_tolerance_rad')
    grip_tolerance=cfg['collision'].get('gripper_closed_measurement_tolerance_rad',0.)
    if type(grip_tolerance) not in (int,float) or not math.isfinite(grip_tolerance) or not 0<=grip_tolerance<=math.radians(5)+1e-12:
        raise ValueError('invalid collision.gripper_closed_measurement_tolerance_rad')
    if type(cfg['collision'].get('retreat_physical_clearance_validated',False)) is not bool:
        raise ValueError('collision.retreat_physical_clearance_validated must be boolean')
    if not isinstance(cfg['collision'].get('retreat_physical_clearance_evidence',''),str):
        raise ValueError('collision.retreat_physical_clearance_evidence must be string')
    for section,key in (('execution','state_timeout_s'),('execution','mission_timeout_s'),
                        ('execution','navigation_timeout_s'),('search','timeout_s'),('collision','check_timeout_s')):
        value=cfg[section][key]
        if value is not None and (type(value) not in (int,float) or not math.isfinite(value) or value<=0):
            raise ValueError('invalid optional timeout '+section+'.'+key)
    state_timeout=cfg['execution']['state_timeout_s']
    recovery=cfg['execution'].get('observation_recovery_timeout_s',30.)
    retry=cfg['execution'].get('observation_retry_delay_s',.25)
    if not 0<retry<recovery or (state_timeout is not None and recovery>state_timeout):
        raise ValueError('observation recovery must be bounded by state_timeout_s')
    candidates=cfg['execution'].get('camera_admission_candidates',1)
    if type(candidates) is not int or not 1<=candidates<=8:
        raise ValueError('execution.camera_admission_candidates must be in [1,8]')
    attempts=cfg['execution'].get('camera_admission_attempts',1)
    if type(attempts) is not int or not 1<=attempts<=10:
        raise ValueError('execution.camera_admission_attempts must be in [1,10]')
    if cfg['frames'].get('leg_state_transport','raw') not in ('raw','native_batch'):
        raise ValueError('frames.leg_state_transport must be raw/native_batch')
    for key,default,upper in [('leg_collector_period_ms',20,100),('leg_collector_capacity',512,1000)]:
        value=cfg['frames'].get(key,default)
        if type(value) is not int or not 1<=value<=upper:raise ValueError('Invalid frames.'+key)
    depth=cfg['frames'].get('scene_receive_depth',32)
    if type(depth) is not int or not 1<=depth<=400:
        raise ValueError('frames.scene_receive_depth must be in [1,400]')
    if type(cfg['frames'].get('tf_index_enabled',False)) is not bool:
        raise ValueError('frames.tf_index_enabled must be boolean')
    if type(history) not in (int,float) or not math.isfinite(history) or history<cfg['limits']['sensor_age_s']:
        raise ValueError('frames.odometry_history_s must cover sensor_age_s')
    margin = cfg['collision']['margin_m']
    environment_checks_enabled(cfg['collision'])
    payload_checks_enabled(cfg['collision'])
    verification=cfg['grasp_verification']
    if type(verification.get('max_attempts',2)) is not int or verification.get('max_attempts',2)!=2:
        raise ValueError('grasp_verification.max_attempts must be 2')
    if verification.get('mode') not in ('stub','torque_width'):raise ValueError('unsupported grasp_verification.mode')
    control=cfg.get('gripper_torque_control',{})
    if control.get('mode')!='measured_contact':raise ValueError('unsupported gripper_torque_control.mode')
    for key in ('kp','kd','close_speed_rad_s','contact_torque_nm','hold_gain_rad_s_per_nm','hold_deadband_nm','max_target_lead_rad'):
        v=control.get(key)
        if type(v) not in (int,float) or not math.isfinite(v) or v<=0:raise ValueError('invalid gripper_torque_control.'+key)
    if control['hold_deadband_nm']>=control['contact_torque_nm']:raise ValueError('invalid gripper hold deadband')
    a=cfg.get('grasp_assessment',{})
    if verification.get('mode')=='torque_width':
        for key in ('contact_torque_nm','min_grasp_width_m','confirmation_duration_s'):
            v=a.get(key)
            if type(v) not in (int,float) or not math.isfinite(v) or v<=0:raise ValueError('invalid grasp_assessment.'+key)
    duration=verification.get('close_duration_s')
    if type(duration) not in (int,float) or not math.isfinite(duration) or duration<=0 or (state_timeout is not None and duration>=state_timeout):
        raise ValueError('invalid grasp_verification.close_duration_s')
    if type(cfg['collision'].get('broadphase_enabled',True)) is not bool:
        raise ValueError('collision.broadphase_enabled must be a boolean')
    for key,default in (('broadphase_padding_m',1e-9),):
        value=cfg['collision'].get(key,default)
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid collision.'+key)
    if type(margin) not in (int, float) or not math.isfinite(margin) or margin <= 0:
        raise ValueError('collision margin must be a finite positive distance in metres')
    if ignored_self_collision_pairs(cfg['collision']) & allowed_zero_contact_pairs(cfg['collision']):
        raise ValueError('conflicting self-collision pair rules')
    if (not math.isfinite(cfg['collision']['sample_max_joint_step_rad'])
            or cfg['collision']['sample_max_joint_step_rad']<=0
            or (state_timeout is not None and cfg['execution']['max_plan_duration_s']>state_timeout)):
        raise ValueError('invalid collision sampling/owner duration cap')
    return cfg

def blockers(cfg):
    environment = environment_checks_enabled(cfg['collision'])
    result = []
    if cfg.get('test_mode','full')=='full' and not physical_retreat_qualified(cfg['collision']) and cfg.get('operator_trial') is not True:
        result.append('physical_retreat_clearance_unqualified')
    if cfg['frames']['vehicle_to_mount'] is None:
        result.append('vehicle_to_mount_missing')
    for key in (('floor_z_in_map_m',) if environment else ()) + (('payload_box_m', 'payload_tcp_xyz_m') if payload_checks_enabled(cfg['collision']) else ()):
        if cfg['collision'][key] is None:
            result.append(key + '_missing')
    for name in ('grasp_root', 'grasp_config', 'sdk_config', 'deployment', 'arm_urdf',
                 'full_urdf', 'description_root', 'hand_eye', 'python', 'graspnet_root'):
        if not Path(cfg['paths'][name]).exists():
            result.append('missing_' + name)
    if environment and not cfg['collision'].get('scene_revision'):result.append('scene_revision_missing')
    return result
