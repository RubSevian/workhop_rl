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
    return [search['joint1_start_rad']]+[search['joint1_min_rad']+i*search['joint1_step_rad'] for i in range(count)]

def load(path, workspace):
    cfg = yaml.safe_load(Path(path).read_text())['go2_mission']
    if cfg.get('test_mode','full') not in ('full','ready_observe'):
        raise ValueError('invalid test_mode')
    if cfg['version'] != 1:
        raise ValueError('unsupported mission config')
    for key, value in cfg['paths'].items():
        cfg['paths'][key] = str((Path(workspace) / value).absolute())
    deployment=yaml.safe_load(Path(cfg['paths']['deployment']).read_text())['real_deployment']
    navigation=yaml.safe_load(Path(cfg['paths']['navigation']).read_text())
    cfg['limits']={'sensor_age_s':navigation['bridge']['sensor_timeout_s'],
                   'arm_age_s':deployment['rars01']['feedback_timeout_s'],
                   'tf_position_tolerance_m':navigation['bridge']['tf_pair_position_tolerance_m'],
                   'tf_quaternion_tolerance':navigation['bridge']['tf_pair_quaternion_tolerance']}
    # Never feed malformed limits to search/timers, even in read-only mode.
    for group in ('search', 'refinement', 'settle', 'execution'):
        for key, value in cfg[group].items():
            if group == 'search' and key == 'mode':
                continue
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'invalid {group}.{key}')
            if key not in ('joint1_min_rad','joint1_start_rad') and value <= 0:
                raise ValueError(f'nonpositive {group}.{key}')
    if type(cfg['compute']['cpu_threads']) is not int or cfg['compute']['cpu_threads']<1:
        raise ValueError('compute.cpu_threads must be a positive integer')
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
    if not math.isclose(intervals, scan['max_attempts']-2, rel_tol=0, abs_tol=1e-9):
        raise ValueError('scan limits/step/attempts disagree')
    if not scan['joint1_min_rad']<=scan['joint1_start_rad']<=scan['joint1_max_rad']:
        raise ValueError('search start outside sweep bounds')
    for path in ('arm_urdf', 'full_urdf'):
        limit = ET.parse(cfg['paths'][path]).getroot().find("joint[@name='joint1']/limit")
        if limit is None or not (float(limit.get('lower')) <= scan['joint1_min_rad']
                                 < scan['joint1_max_rad'] <= float(limit.get('upper'))):
            raise ValueError('search exceeds joint1 limits in '+path)
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
    if type(history) not in (int,float) or not math.isfinite(history) or history<cfg['limits']['sensor_age_s']:
        raise ValueError('frames.odometry_history_s must cover sensor_age_s')
    margin = cfg['collision']['margin_m']
    environment_checks_enabled(cfg['collision'])
    payload_checks_enabled(cfg['collision'])
    verification=cfg['grasp_verification']
    if verification.get('mode')!='stub':raise ValueError('unsupported grasp_verification.mode')
    duration=verification.get('close_duration_s')
    if type(duration) not in (int,float) or not math.isfinite(duration) or duration<=0 or duration>=cfg['execution']['state_timeout_s']:
        raise ValueError('invalid grasp_verification.close_duration_s')
    if type(cfg['collision'].get('broadphase_enabled',True)) is not bool:
        raise ValueError('collision.broadphase_enabled must be a boolean')
    for key,default in (('broadphase_padding_m',1e-9),('check_timeout_s',20.)):
        value=cfg['collision'].get(key,default)
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid collision.'+key)
    if type(margin) not in (int, float) or not math.isfinite(margin) or margin <= 0:
        raise ValueError('collision margin must be a finite positive distance in metres')
    if ignored_self_collision_pairs(cfg['collision']) & allowed_zero_contact_pairs(cfg['collision']):
        raise ValueError('conflicting self-collision pair rules')
    if (not math.isfinite(cfg['collision']['sample_max_joint_step_rad'])
            or cfg['collision']['sample_max_joint_step_rad']<=0
            or cfg['execution']['max_plan_duration_s']>cfg['execution']['state_timeout_s']):
        raise ValueError('invalid collision sampling/owner duration cap')
    return cfg

def blockers(cfg):
    environment = environment_checks_enabled(cfg['collision'])
    result = []
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
