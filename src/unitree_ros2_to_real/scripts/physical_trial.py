#!/usr/bin/env python3
"""Operator inspection/evidence and explicit one-shot RETREAT approval; no SDK."""
import argparse
import json
import math
from pathlib import Path
import re
import time
import yaml
from mission.config import load
from mission.health_contract import STATUS_TOPICS,source_ttl
from mission.status_codec import decode_status


def safe(value):
    if isinstance(value,dict):return {k:safe(v) for k,v in value.items()}
    if isinstance(value,list):return [safe(v) for v in value]
    return str(value) if isinstance(value,float) and not math.isfinite(value) else value


def approval_identity(owner,mission,r3):
    preview=owner.get('retreat_trial') or {}
    if (owner.get('operator_trial') is not True or mission.get('operator_trial') is not True
            or mission.get('state')!='WAIT_RETREAT_APPROVAL' or preview.get('state')!='WAITING_OPERATOR'
            or not preview.get('approval_service','').startswith('/rars01/mission/approve_retreat/t')
            or preview.get('mission_session_id')!=mission.get('session_id')
            or preview.get('goal_revision')!=mission.get('goal_revision')
            or r3.get('state')!='ACTIVE' or r3.get('phase')!='RL_ZERO' or r3.get('fault_latched')
            or not owner.get('feedback_ready') or not owner.get('target_valid')
            or owner.get('gripper_command_active') is not True
            or owner.get('grasp_assessment') not in ('GRASPED','NOT_GRASPED')):
        raise ValueError('WAITING_OPERATOR_WITH_FRESH_HOLD_AND_VALID_ASSESSMENT_REQUIRED')
    return (owner['owner_session_id'],preview['mission_session_id'],preview['goal_revision'],preview['plan_id'],preview['approval_service'])


def preflight_failures(data):
    r3,nav,owner,vision,mission=(data[n] for n in ('r3','nav','owner','perception','mission'))
    failures=list(r3.get('remote_sequence_blockers',['R3_PREFLIGHT_NOT_REPORTED']))
    if r3.get('read_only') is not False:failures.append('R3_READ_ONLY')
    if r3.get('fault_latched'):failures.append('R3_FAULT_LATCHED')
    if not r3.get('policy_rt_ready'):failures.append('FIFO20_WORKER_NOT_READY')
    if not nav.get('navigation_ready'):failures.append('NAV:'+nav.get('block_reason','NOT_READY'))
    if not vision.get('ready') or not vision.get('preload_ready'):failures.append('PERCEPTION:'+vision.get('reason','NOT_READY'))
    if owner.get('owner_state')!='HOLD_HOME' or not all(owner.get(k) is True for k in ('feedback_ready','target_valid','arm_home_ready','mission_capable')):failures.append('RARS01_HOME_FEEDBACK_TARGET_REQUIRED')
    failures+=mission.get('blockers',[])
    if r3.get('output_enabled'):failures.append('OUTPUT_ALREADY_ACTIVE_USE_STATUS_NOT_PREFLIGHT')
    return failures


def descent_record_allowed(r3):
    return (r3.get('state')=='SYSTEM_HOLD' and r3.get('phase')=='LIE_DOWN_HOLD'
            and r3.get('output_enabled') is False and r3.get('output_stop_confirmed') is True
            and r3.get('passive_sequence_complete') is True and not r3.get('fault_latched'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('preflight','gripper','approve-retreat','record','record-descent'))
    parser.add_argument('--workspace',type=Path,default=Path.home()/'go2_diploma/sim2real')
    parser.add_argument('--seconds',type=float,default=180.)
    parser.add_argument('--result',choices=('pass','fail'))
    parser.add_argument('--notes',default='')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 0<args.seconds<=600:parser.error('seconds must be in (0,600]')
    if args.action in ('record','record-descent') and (args.result is None or not args.notes.strip()):parser.error('record requires --result and nonempty --notes describing tested conditions')
    cfg=load(args.workspace/'repos/rars01_graspnet/config/jetson_orin_nano.yaml',args.workspace)
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('go2_explicit_physical_trial_operator')
    topics=dict(STATUS_TOPICS,mission='/go2/mission/status');data={}
    def receive(name,msg):
        try:
            d=decode_status(msg.data) if name in ('r3','owner') else json.loads(msg.data)
            if isinstance(d,dict) and type(d.get('observed_ns')) is int and d['observed_ns']>data.get(name,{}).get('observed_ns',0):data[name]=d
        except (ValueError,TypeError,yaml.YAMLError):pass
    subscriptions=[node.create_subscription(String,topic,lambda msg,n=name:receive(n,msg),1) for name,topic in topics.items()]
    def fresh(after=0):
        now=time.monotonic_ns()
        return all(name in data and after<data[name]['observed_ns']<=now and (now-data[name]['observed_ns'])/1e9<=source_ttl(cfg,name)
                   and node.count_publishers(topic)==1 for name,topic in topics.items())
    def collect(after=0):
        until=time.monotonic()+5
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
            if fresh(after):return
        raise RuntimeError('FRESH_UNIQUE_COMPONENT_STATUSES_REQUIRED: inspect architecture monitor; no authorization sent')
    def save(update):
        session=re.sub('[^A-Za-z0-9_-]','_',data['mission']['session_id'])
        path=args.workspace/'runtime/operator_trials'/(session+'.json');path.parent.mkdir(parents=True,exist_ok=True)
        record=json.loads(path.read_text()) if path.exists() else {}
        record.update(update);record['qualification_flags_changed']=False
        temporary=path.with_suffix('.json.tmp');temporary.write_text(json.dumps(safe(record),ensure_ascii=False,indent=2)+'\n');temporary.replace(path)
        print('Evidence:',path,flush=True)
    try:
        collect()
        if args.action=='preflight':
            failures=preflight_failures(data)
            print(json.dumps(dict(preflight='BLOCKED' if failures else 'READY_FOR_OPERATOR_A',failures=failures,
                state=data['r3'].get('state'),phase=data['r3'].get('phase'),output_enabled=data['r3'].get('output_enabled')),ensure_ascii=False,indent=2))
            if failures:raise RuntimeError('Preflight did not pass; no motion request sent')
        elif args.action=='gripper':
            until=time.monotonic()+args.seconds;last=0.;peak=0.
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.02)
                if not fresh():continue
                owner=data['owner'];torque=owner.get('torque',[])
                tau=abs(float(torque[6])) if len(torque)==7 and math.isfinite(float(torque[6])) else None
                if tau is not None:peak=max(peak,tau)
                if time.monotonic()-last>=.2:
                    print(json.dumps(dict(phase=data['mission'].get('state'),torque_abs_nm=tau,observed_peak_nm=peak,
                        width_m=owner.get('grasp_width_m'),confirmation_s=owner.get('grasp_confirmation_s'),
                        assessment=owner.get('grasp_assessment'),feedback_ready=owner.get('feedback_ready'),
                        tau_ff_nm=owner.get('gripper_command_torque_nm')),ensure_ascii=False),flush=True);last=time.monotonic()
                if data['mission'].get('state')=='WAIT_RETREAT_APPROVAL' and owner.get('retreat_trial',{}).get('state')=='WAITING_OPERATOR':
                    save(dict(gripper_observation=dict(observed_peak_nm=peak,snapshot=data,time_ns=time.time_ns(),physical_torque_cap_qualified=False)))
                    print('Stopped before RETREAT. No approval sent. Inspect actual clearances, then use approve-retreat.',flush=True);break
            else:print('Observation window ended; no motion or approval was sent.',flush=True)
        elif args.action=='approve-retreat':
            identity=approval_identity(data['owner'],data['mission'],data['r3'])
            preview=data['owner']['retreat_trial'];print(json.dumps(safe(dict(trial=preview,owner_session_id=identity[0],grasp_width_m=data['owner'].get('grasp_width_m'),grasp_assessment=data['owner'].get('grasp_assessment'))),ensure_ascii=False,indent=2),flush=True)
            save(dict(preview=dict(ticket=list(identity),snapshot=data,time_ns=time.time_ns(),operator_approval_granted=False)))
            expected='RETREAT '+str(preview['plan_id'])
            print('Approval starts physical RETREAT (25% original sampled velocity), then the saved scan pose and HOME. NOT_GRASPED means grasp unconfirmed. It does not qualify clearance.',flush=True)
            if input('After inspecting the complete real path and object, type exactly '+expected+': ').strip()!=expected:raise RuntimeError('Operator did not confirm; no request sent')
            collect(time.monotonic_ns())
            if approval_identity(data['owner'],data['mission'],data['r3'])!=identity:raise RuntimeError('TRIAL_IDENTITY_CHANGED: inspect again; no request sent')
            save(dict(approval=dict(ticket=list(identity),snapshot=data,time_ns=time.time_ns(),operator_confirmation=expected)))
            client=node.create_client(Trigger,identity[-1])
            if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Exact ticket service absent; no request sent')
            collect(time.monotonic_ns())
            if approval_identity(data['owner'],data['mission'],data['r3'])!=identity:raise RuntimeError('TRIAL_IDENTITY_CHANGED before request')
            future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=3)
            if not future.done() or future.result() is None:raise RuntimeError('Approval response unknown: inspect monitor; do not automatically retry')
            print(json.dumps(dict(success=future.result().success,message=future.result().message),ensure_ascii=False),flush=True)
            if not future.result().success:raise RuntimeError('Approval rejected; inspect monitor')
        elif args.action=='record-descent':
            if args.result=='pass' and not descent_record_allowed(data['r3']):
                raise RuntimeError('A descent pass record requires healthy lying/passive completion and confirmed output OFF')
            save(dict(descent_result=dict(operator_verdict=args.result,conditions=args.notes,snapshot=data,time_ns=time.time_ns(),operator_validated_changed=False)))
        else:
            if args.result=='pass' and (data['owner'].get('retreat_trial',{}).get('state')!='SUCCEEDED' or data['mission'].get('state')!='GRASP_PRESUMED_SUCCESS'):
                raise RuntimeError('A pass record requires observed RETREAT SUCCEEDED and completed measured HOME; no flags changed')
            save(dict(result=dict(operator_verdict=args.result,conditions_and_clearances=args.notes,snapshot=data,time_ns=time.time_ns(),scope='this plan/pose/opening/object only')))
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
