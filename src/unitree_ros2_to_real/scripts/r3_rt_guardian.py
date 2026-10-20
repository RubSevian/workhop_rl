#!/usr/bin/env python3
"""Internal ros2 launch prefix. Own child/TID only; no actuator RPCs.

The launch shell must grant RLIMIT_RTPRIO=20. No sudo, root controller,
system-wide scheduler settings, or silent OTHER fallback inside this process.
"""
import ctypes
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import signal
import struct
import subprocess
import sys
import tempfile
import time


def identity(pid, tid=None):
    tid = pid if tid is None else tid
    base = Path('/proc') / str(pid) / 'task' / str(tid)
    stat = (base / 'stat').read_text().rsplit(')', 1)[1].split()
    status = dict(line.split(':', 1) for line in (base / 'status').read_text().splitlines() if ':' in line)
    if int(status['Tgid']) != pid or any(int(x) != os.getuid() for x in status['Uid'].split()):
        raise RuntimeError('Target TGID/UID changed')
    return dict(pid=pid, tid=tid, start=int(stat[19]), parent=int(stat[1]),
                cpu_ns=(int(stat[11])+int(stat[12]))*1_000_000_000//os.sysconf('SC_CLK_TCK'),
                policy=os.sched_getscheduler(tid), priority=os.sched_getparam(tid).sched_priority,
                comm=(base / 'comm').read_text().strip())


def safe_exit(status, age, stable, config):
    timeout, samples = config["safe_exit_timeout_s"], config["safe_exit_stable_samples"]
    return (bool(status) and 0 <= age < timeout and stable >= samples
            and status.get('output_enabled') is False
            and status.get('lowcmd_publisher_present') is False
            and status.get('lowcmd_lease_present') is False
            and status.get('custom_leg_output') == 'OFF')


def sha(path):
    with open(path, 'rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def change_scheduler(proc, ready, lease, executable_sha, priority):
    """Same lease serialization and identity/readback contract as tested setter."""
    fcntl.flock(lease, fcntl.LOCK_EX)
    try:
        main = identity(proc.pid)
        worker = identity(proc.pid, ready['tid'])
        state = struct.unpack('<Q', os.pread(lease, 8, 8))[0]
        if (proc.poll() is not None or ready['pid'] != proc.pid or ready['uid'] != os.getuid()
                or ready['tid'] == proc.pid or main['parent'] != os.getpid()
                or main['start'] != ready['pid_start'] or worker['start'] != ready['tid_start']
                or worker['comm'] != 'r3_rt_policy' or sha(f'/proc/{proc.pid}/exe') != executable_sha
                or os.pread(lease, 8, 0) != b'R3OFF001' or state not in (0, 1)):
            raise RuntimeError('Exact policy identity/lease validation refused')
        if priority and (state != 0 or worker['policy'] != os.SCHED_OTHER or main['policy'] != os.SCHED_OTHER):
            raise RuntimeError('Unexpected initial scheduler/lease')
        if not priority:
            os.pwrite(lease, struct.pack('<Q', 1), 88)  # close future takeover admission
        os.sched_setscheduler(ready['tid'], os.SCHED_FIFO if priority else os.SCHED_OTHER,
                              os.sched_param(priority))
        readback = identity(proc.pid, ready['tid'])
        if (readback['policy'], readback['priority']) != ((os.SCHED_FIFO, 20) if priority else (os.SCHED_OTHER, 0)):
            raise RuntimeError('Scheduler readback failed')
        return readback
    finally:
        fcntl.flock(lease, fcntl.LOCK_UN)


def load_guardian_config(path):
    import yaml
    with open(path) as stream:
        config = yaml.safe_load(stream)['real_deployment']['r3_commissioning']['guardian']
    bounds = dict(status_source_timeout_s=.25, safe_exit_timeout_s=.5, discovery_s=1.5,
        registration_timeout_s=12., policy_progress_timeout_s=.2, io_progress_timeout_s=.2,
        status_timeout_s=.5, initial_status_timeout_s=2., lowstate_timeout_s=.5,
        cpu_window_s=.5, save_period_s=.1, poll_period_s=.01, spin_period_s=.02,
        terminal_wait_s=8., shutdown_poll_s=.02)
    for key, maximum in bounds.items():
        value = config[key]
        if type(value) not in (int,float) or not math.isfinite(value) or not 0 < value <= maximum:
            raise RuntimeError('Guardian setting exceeds existing contract: '+key)
    if (config['priority'] != 20 or type(config['safe_exit_stable_samples']) is not int
            or config['safe_exit_stable_samples'] < 2 or not 0 < config['cpu_max_fraction'] <= .8
            or not 0 < config['rttime_soft_us'] <= config['rttime_hard_us'] <= 200000):
        raise RuntimeError('Invalid guardian RT/safe-exit limits')
    return config


def main():
    command = sys.argv[1:]
    live_shadow = command[:1] == ['--live-compute-probe']
    compute_probe = live_shadow or command[:1] == ['--compute-probe']
    if compute_probe:
        command = command[1:]
        # Diagnostic-only admission uses the same registration/lease/rollback
        # as normal launch, on the compile-time no-actuator executable.
        dds = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'
        if live_shadow:
            dds = dds.replace('name="lo"', 'name="enP8p1s0"')
            if (os.environ.get('ROS_DOMAIN_ID') != '0' or os.environ.get('CYCLONEDDS_URI') != dds
                    or os.environ.get('GO2_NETWORK_INTERFACE') or os.environ.get('ROS_STATIC_PEERS')):
                raise RuntimeError('Live shadow requires real domain0/enP8p1s0 without SDK interface')
        elif (os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
                or os.environ.get('CYCLONEDDS_URI') != dds or os.environ.get('GO2_NETWORK_INTERFACE')
                or os.environ.get('ROS_STATIC_PEERS')):
            raise RuntimeError('Compute probe requires isolated domain228/loopback')
    if command[:1] == ['--']:
        command = command[1:]
    expected = 'r3_live_shadow' if live_shadow else ('r3_runtime_timing' if compute_probe else 'go2_r3_commissioning')
    if not command or Path(command[0]).name != expected:
        raise RuntimeError('Expected installed R3 executable')
    if os.getuid() == 0 or os.sched_getscheduler(0) != os.SCHED_OTHER:
        raise RuntimeError('Guardian must run as the ordinary user in OTHER0')
    if resource.getrlimit(resource.RLIMIT_RTPRIO)[0] < 20:
        raise RuntimeError('FIFO20 unavailable: before launch run sudo prlimit --pid $$ --rtprio=20:20 in this terminal; no controller started')
    # Imports happen only after admission. rclpy never receives the child ROS args.
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    import yaml
    config = load_guardian_config(os.environ['R3_RT_CONFIG_PATH'])
    libc = ctypes.CDLL(None)
    if libc.prctl(15, b'r3_rt_guardian', 0, 0, 0):
        raise RuntimeError('Cannot name guardian')
    affinity = os.sched_getaffinity(0)
    os.sched_setaffinity(0, {min(affinity)})
    root = Path(os.environ.get('R3_RT_LOG_ROOT', config['log_root']))
    root.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix='session_', dir=root))
    print('R3 RT evidence: ' + str(out), flush=True)
    events = (out / 'events.jsonl').open('w')
    def event(**values):
        values['mono_ns'] = time.monotonic_ns()
        events.write(json.dumps(values) + '\n');events.flush()
    event(configuration=config)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('r3_rt_guardian', namespace='/diagnostics', enable_rosout=False, start_parameter_services=False)
    status = None;status_ns = 0;stable = 0;previous_sent = None;status_error = None
    status_log = (out / 'status.jsonl').open('w')
    def receive(message, info):
        nonlocal status, status_ns, stable, previous_sent, status_error
        try:
            age = time.time_ns() - info['source_timestamp']
            value = yaml.load(message.data, Loader=getattr(yaml, 'CSafeLoader', yaml.SafeLoader))
            if not 0 <= age <= round(config["status_source_timeout_s"]*1e9) or not isinstance(value, dict):
                raise RuntimeError('Stale/future/invalid native status')
            sent = value.get('sent_packets')
            if type(sent) is not int or sent < 0:
                raise RuntimeError('Invalid packet counter')
            stable = stable + 1 if sent == previous_sent else 0;previous_sent = sent
            status = value;status_ns = time.monotonic_ns()
            status_log.write(json.dumps(dict(mono_ns=status_ns, status=value)) + '\n');status_log.flush()
        except Exception as exc:
            status_error = str(exc)
    status_topic = '/timing/probe_status' if compute_probe else '/go2/locomotion_status'
    sub = node.create_subscription(String, status_topic, receive, 1)
    until = time.monotonic()+config["discovery_s"]
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=config["spin_period_s"])
    if node.count_publishers(status_topic):
        raise RuntimeError('Existing R3 status publisher; refusing second controller')
    stop = False
    def interrupted(sig, frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, interrupted);signal.signal(signal.SIGTERM, interrupted)
    parent = os.getpid()
    def child_setup():
        os.sched_setaffinity(0, affinity)
        resource.setrlimit(resource.RLIMIT_RTTIME, (config["rttime_soft_us"], config["rttime_hard_us"]))
        if libc.prctl(1, signal.SIGTERM, 0, 0, 0) or os.getppid() != parent:
            os._exit(125)
    executable_sha = sha(command[0])
    log = (out / 'controller.log').open('wb');tail = (out / 'controller.log').open('rb')
    # Child is outside the launch process group: terminal Ctrl+C reaches guardian.
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, preexec_fn=child_setup)
    pfd = os.pidfd_open(proc.pid)
    ready = None;lease = None;armed = False;demoted = False;failure = None;buffer = b'';terminal = None
    def pump():
        nonlocal ready, buffer
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0)
        buffer += tail.read(65536)
        if len(buffer) > 262144:
            raise RuntimeError('Controller line capacity exceeded')
        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)
            print(line.decode(errors='replace'), flush=True)
            try:
                value = json.loads(line)
                if value.get('ready') is True:
                    ready = value
            except (ValueError, AttributeError):
                pass
    try:
        deadline = time.monotonic()+config["registration_timeout_s"]
        while (ready is None or (compute_probe and status is None)) and proc.poll() is None and time.monotonic() < deadline and not stop:
            pump();time.sleep(config["poll_period_s"])
        if not ready or ready['pid'] != proc.pid:
            raise RuntimeError('Native policy registration absent')
        if compute_probe and (status is None or status.get('compute_probe') is not True
                or status.get('read_only') is not True or status.get('output_enabled') is not False
                or status.get('sent_packets') != 0
                or (live_shadow and (status.get('live_robot_inputs') is not True
                    or status.get('lowcmd_publisher_present') is not False))
                or (not live_shadow and (node.count_publishers('/lowcmd')
                    or node.count_publishers('/api/sport/request')))):
            raise RuntimeError('Read-only compute probe admission refused')
        lease = os.open(f'/proc/{proc.pid}/fd/{ready["lease_fd"]}', os.O_RDWR|os.O_CLOEXEC)
        event(admission=change_scheduler(proc, ready, lease, executable_sha, config["priority"]), argv=command, exe_sha256=executable_sha)
        # Guard readiness separately: after this write native A/output may run.
        armed = True
        os.pwrite(lease, struct.pack('<Q', 1), 8)
        began = time.monotonic();cpu_window = None;last_save = 0
        print('R3 policy FIFO20 confirmed; IO/inputs/DDS remain OTHER0. Startup disarmed.', flush=True)
        while proc.poll() is None:
            pump();now = time.monotonic_ns()
            terminal = struct.unpack('<8s11Q', os.pread(lease, 96, 0))
            if terminal[1] == 2:
                break
            try:
                current = identity(proc.pid, ready['tid'])
                if current['start'] != ready['tid_start']:
                    raise RuntimeError('Policy TID identity changed')
                if not demoted and (current['policy'], current['priority']) != (os.SCHED_FIFO, config["priority"]):
                    raise RuntimeError('Policy scheduler changed')
                if terminal[6] and now-terminal[2] > round(config["policy_progress_timeout_s"]*1e9):
                    raise RuntimeError('policy_progress_timeout')
                if terminal[3] and now-terminal[3] > round(config["io_progress_timeout_s"]*1e9):
                    raise RuntimeError('IO_progress_timeout')
                if status_error:
                    raise RuntimeError(status_error)
                if node.count_publishers(status_topic) != 1:
                    raise RuntimeError('Native status absent/ambiguous')
                if (status_ns and now-status_ns > round(config["status_timeout_s"]*1e9)) or (not status_ns and time.monotonic()-began > config["initial_status_timeout_s"]):
                    raise RuntimeError('Native status stale/missing')
                if status and not status.get('read_only') and now-terminal[4] > round(config["lowstate_timeout_s"]*1e9):
                    raise RuntimeError('LowState receipt stale')
                for task in Path('/proc', str(proc.pid), 'task').iterdir():
                    if int(task.name) == ready['tid']:
                        continue
                    try:
                        if os.sched_getscheduler(int(task.name)) != os.SCHED_OTHER or os.sched_getparam(int(task.name)).sched_priority:
                            raise RuntimeError('Non-policy thread RT')
                    except ProcessLookupError:
                        pass
                if cpu_window is None:
                    cpu_window = (now, current['cpu_ns'])
                elif now-cpu_window[0] > round(config["cpu_window_s"]*1e9):
                    if current['cpu_ns']-cpu_window[1] > (now-cpu_window[0])*config["cpu_max_fraction"]:
                        raise RuntimeError('Policy CPU runaway')
                    cpu_window = (now, current['cpu_ns'])
                if status and status.get('fault_latched'):
                    raise RuntimeError('Native FSM fault: '+str(status.get('fault')))
            except Exception as exc:
                if failure is None:
                    failure = str(exc);event(failure=failure)
                    print('RT guardian: '+failure+'. Operator stop required; no automatic motion RPC.', flush=True)
            if failure or stop:
                # A terminal interrupt requests shutdown; keep policy scheduled
                # while the operator performs the native stop sequence.
                if failure and not demoted:
                    event(demotion=change_scheduler(proc, ready, lease, executable_sha, 0));demoted = True
                if safe_exit(status, (now-status_ns)/1e9, stable, config):
                    break
            if now-last_save > round(config["save_period_s"]*1e9):
                (out / 'state.json').write_text(json.dumps(dict(pid=proc.pid, ready=ready, failure=failure,
                    stop_requested=stop, policy_fifo20=not demoted, status=status, mono_ns=now))+'\n')
                last_save = now
            time.sleep(config["poll_period_s"])
    except BaseException as exc:
        failure = failure or repr(exc);event(exception=repr(exc))
    finally:
        if lease is not None:
            terminal = struct.unpack('<8s11Q', os.pread(lease, 96, 0))
        if failure and armed and not demoted and proc.poll() is None and terminal[1] != 2:
            try:
                event(demotion=change_scheduler(proc, ready, lease, executable_sha, 0));demoted = True
            except Exception as exc:
                failure = failure or repr(exc);event(demotion_error=repr(exc))
        if armed and proc.poll() is None and terminal[1] != 2:
            warned = False
            # Exceptions and SIGTERM obey the same output-OFF guard as Ctrl+C.
            while proc.poll() is None and not safe_exit(status, (time.monotonic_ns()-status_ns)/1e9, stable, config):
                if not warned:
                    print('Controller kept alive: perform native operator stop; output OFF is not confirmed.', flush=True);warned = True
                try:
                    pump()
                except Exception as exc:
                    print('Guardian cleanup observation error: '+repr(exc), flush=True)
                time.sleep(config["shutdown_poll_s"])
        if armed and not demoted and proc.poll() is None and terminal[1] != 2:
            try:
                event(demotion=change_scheduler(proc, ready, lease, executable_sha, 0));demoted = True
            except Exception as exc:
                failure = failure or repr(exc);event(demotion_error=repr(exc))
        if proc.poll() is None and terminal is not None and terminal[1] == 2:
            try:
                proc.wait(timeout=config["terminal_wait_s"])
            except subprocess.TimeoutExpired:
                pass
        if proc.poll() is None:
            signal.pidfd_send_signal(pfd, signal.SIGINT)
            # No escalation to kill; native output was confirmed OFF first.
            while proc.poll() is None:
                pump();time.sleep(config["shutdown_poll_s"])
        if lease is not None:
            terminal = struct.unpack('<8s11Q', os.pread(lease, 96, 0));os.close(lease)
        proof = terminal is not None and (terminal[1], terminal[8], terminal[9], terminal[10]) == (2, 0, 0, 0)
        result = dict(failure=failure, terminal_demoted=proof, returncode=proc.returncode,
                      own_actuator_commands=0, independent_physical_stop_confirmed=False)
        (out / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
        event(result=result);print('R3 RT result: '+json.dumps(result), flush=True)
        tail.close();log.close();events.close();status_log.close();os.close(pfd)
        node.destroy_node();rclpy.try_shutdown();os.sched_setaffinity(0, affinity)
    return 0 if failure is None and proof and proc.returncode == 0 else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print('R3 RT startup refused: '+str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1)
