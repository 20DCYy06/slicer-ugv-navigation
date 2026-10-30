#!/usr/bin/env python3
"""TEST_ONLY real ROS GNSS node + released dependencies + PTY/caster drill.

Run only in a network-none container without device mappings, with its own
master. No receiver, public caster, real account, drive or TF.
"""
import argparse
import base64
import io
import json
import math
import os
from pathlib import Path
import pty
import secrets
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid

PACKAGE = Path(__file__).absolute().parents[1]
sys.path[:0] = [str(PACKAGE/'src')]
import rosbag
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from gnss_process_helpers import graph, safe_path, capture_process, stop_process, group_members
from cup_gnss.ntrip import crc24q
from cup_gnss.upstream_nmea import parse_nmea

MASTER = 'http://127.0.0.1:11331'
if os.environ.get('CUP_NAV_ISOLATED_TEST') != '1' or os.environ.get('ROS_MASTER_URI') != MASTER:
    raise ValueError('Requires isolated GNSS test: network none, no devices, independent master11331')


def packet(message, suffix):
    payload = bytes((message >> 4, (message & 15) << 4)) + suffix
    body = b'\xd3' + bytes((len(payload) >> 8, len(payload) & 255)) + payload
    return body + crc24q(body).to_bytes(3, 'big')


def raw_gga(quality, diff_age):
    # Deliberate TEST_ONLY receiver time-of-day offset proves public stamp uses
    # ROS receipt, not GGA's undated/fabricated host-date measurement timestamp.
    epoch = (time.time() - 1.25) % 86400
    hours, remain = divmod(epoch, 3600); minutes, seconds = divmod(remain, 60)
    stamp = '%02d%02d%06.3f' % (hours, minutes, seconds)
    body = ('GNGGA,' + stamp + ',3112.123456,N,12130.654321,E,%d,27,0.7,15.0,M,5.0,M,%.1f,0001'
            % (quality, diff_age))
    checksum = 0
    for value in body.encode('ascii'): checksum ^= value
    return '$' + body + '*%02X' % checksum


class Caster:
    def __init__(self, expected_auth):
        self.expected_auth = expected_auth
        self.stop = threading.Event(); self.bad = threading.Event()
        self.accepted = 0; self.uploads = []; self.frames = []; self.errors = []
        self.bad_sent = 0; self.upstream_requests = 0
        self.server = socket.socket(); self.server.bind(('127.0.0.1', 0))
        self.server.listen(4); self.server.settimeout(.1)
        self.port = self.server.getsockname()[1]
        self.thread = threading.Thread(target=self.run, daemon=True); self.thread.start()

    def run(self):
        try:
            while not self.stop.is_set():
                try: connection, _ = self.server.accept()
                except socket.timeout: continue
                self.accepted += 1
                try:
                    with connection:
                        connection.settimeout(.1)
                        request = b''
                        deadline = time.monotonic() + 2
                        while b'\r\n\r\n' not in request:
                            if time.monotonic() > deadline or self.stop.is_set(): raise OSError('test request timeout')
                            try: data = connection.recv(4096)
                            except socket.timeout: continue
                            if not data: raise OSError('test peer closed')
                            request += data
                        if self.expected_auth not in request: raise AssertionError('synthetic authorization mismatch')
                        if b'User-Agent: NTRIP ntrip_client_ros\r\n' not in request:
                            raise AssertionError('released upstream request was not used')
                        self.upstream_requests += 1
                        connection.sendall(b'I'); time.sleep(.005)
                        connection.sendall(b'CY 200 OK\r\n')
                        pending = b''; got_gga = False; next_frame = time.monotonic()
                        while not self.stop.is_set():
                            try:
                                data = connection.recv(4096)
                                if not data: break
                                pending += data
                                while b'\r\n' in pending:
                                    line, pending = pending.split(b'\r\n', 1)
                                    if line:
                                        self.uploads.append(line.decode('ascii'))
                                        got_gga = True
                            except socket.timeout: pass
                            if not got_gga: continue
                            if self.bad.is_set():
                                self.bad.clear()
                                corrupt = bytearray(packet(1074, b'TEST_ONLY_BAD_CRC'))
                                corrupt[-1] ^= 1
                                connection.sendall(bytes(corrupt)); self.bad_sent += 1
                                # The real node should close and establish a new connection.
                                while not self.stop.is_set():
                                    try:
                                        if not connection.recv(4096): break
                                    except socket.timeout: continue
                                break
                            if time.monotonic() >= next_frame:
                                frame = packet(1074, b'TEST_ONLY_' + len(self.frames).to_bytes(4, 'big'))
                                self.frames.append(frame)
                                # Split inside the RTCM header, not just between messages.
                                connection.sendall(frame[:1]); time.sleep(.005)
                                connection.sendall(frame[1:])
                                next_frame = time.monotonic() + .15
                except (OSError, socket.timeout):
                    continue
        except BaseException as exc:
            self.errors.append(type(exc).__name__)
        finally:
            self.server.close()

    def close(self):
        self.stop.set(); self.thread.join(3)
        if self.thread.is_alive(): raise AssertionError('owned fake caster did not stop')


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', required=True)
    root = safe_path(parser.parse_args().output)
    if graph(MASTER)['reachable']: raise ValueError('Refuse a pre-existing master')
    root.mkdir(parents=True, exist_ok=False)
    run_id = 'TEST_ONLY-mature-gnss-' + uuid.uuid4().hex
    env = dict(os.environ, NAV_RUN_ID=run_id, ROS_LOG_DIR=str(root/'roslogs'))
    logs = []; roles = []; caster = None; master_fd = None
    feed_stop = threading.Event(); reader_stop = threading.Event()
    feeder = reader = None; bag = None; subscribers = []
    lock = threading.RLock(); events = []; fixes = []; diagnostics = []; raw_messages = []
    sent = []; serial_received = bytearray(); wire_counts = {}
    phase = {'quality':4, 'diff_age':.5}
    phases = []
    started = time.monotonic()
    temporary = tempfile.TemporaryDirectory(prefix='cup_TEST_ONLY_credentials_')
    username = 'TEST_ONLY_' + uuid.uuid4().hex
    password = secrets.token_hex(16)
    auth = base64.b64encode((username+':'+password).encode('ascii'))
    secret_markers = [username.encode(), password.encode(), auth]
    failure = None

    def mark(name, **values):
        item = {'phase':name, 'mono_s':time.monotonic(), 'wall_s':time.time(), **values}
        phases.append(item)

    def wait(predicate, label, timeout=9):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate(): return
            if node is not None and node.poll() is not None:
                raise AssertionError('real gnss_node exited during '+label)
            time.sleep(.05)
        latest = diagnostics[-1]['values'] if diagnostics else {}
        raise AssertionError(label+'; latest diagnostic='+json.dumps(latest, sort_keys=True))

    def latest(**expected):
        with lock:
            return bool(diagnostics) and all(diagnostics[-1]['values'].get(k)==v for k,v in expected.items())

    def receive(topic, message):
        wall = time.time(); stamp = rospy.Time.now()
        with lock:
            bag.write(topic, message, t=stamp)
            wire_counts[topic] = wire_counts.get(topic, 0) + 1
            if topic == '/gnss/nmea':
                value = {'raw':message.data, 'receipt_wall_s':wall}; raw_messages.append(value)
            elif topic == '/gnss/fix':
                value = {'stamp':message.header.stamp.to_sec(), 'frame_id':message.header.frame_id,
                         'status':message.status.status, 'service':message.status.service,
                         'latitude':message.latitude, 'longitude':message.longitude, 'altitude':message.altitude,
                         'covariance':list(message.position_covariance),
                         'covariance_type':message.position_covariance_type, 'receipt_wall_s':wall}
                fixes.append(value)
            else:
                status = message.status[0]
                value = {'stamp':message.header.stamp.to_sec(), 'level':status.level, 'message':status.message,
                         'values':{pair.key:pair.value for pair in status.values}, 'receipt_wall_s':wall}
                diagnostics.append(value)
            events.append({'topic':topic, 'message':value, 'test_only':True})

    node = None
    try:
        credentials = Path(temporary.name)/'ntrip.json'
        credentials.write_text(json.dumps({'username':username, 'password':password})); credentials.chmod(0o600)
        caster = Caster(b'Authorization: Basic '+auth+b'\r\n')
        master_fd, slave_fd = pty.openpty(); slave_path = os.ttyname(slave_fd); os.close(slave_fd)
        os.set_blocking(master_fd, False)
        log = (root/'core.log').open('w'); logs.append(log)
        core = subprocess.Popen(['roscore','-p','11331'], stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, env=env)
        roles.append(capture_process(core, run_id, ['roscore']))
        deadline = time.monotonic()+15
        while not graph(MASTER)['reachable']:
            if time.monotonic()>deadline: raise AssertionError('isolated master startup timeout')
            time.sleep(.1)
        rospy.init_node('TEST_ONLY_mature_gnss_observer', disable_rosout=True, disable_signals=True)
        bag = rosbag.Bag(str(root/'raw.bag'), 'w')
        for topic, cls in (('/gnss/nmea',String),('/gnss/fix',NavSatFix),('/gnss/diagnostics',DiagnosticArray)):
            subscribers.append(rospy.Subscriber(topic,cls,lambda m,t=topic:receive(t,m),queue_size=200))
        params = {'source':'serial', 'serial_port':slave_path, 'baud':115200, 'expected_usb_serial':'',
                  'frame_id':'gnss_link', 'ntrip_enabled':True, 'ntrip_host':'127.0.0.1',
                  'ntrip_port':caster.port, 'ntrip_mountpoint':'AUTO',
                  'ntrip_credentials_file':str(credentials), 'rtcm_pause_file':str(root/'rtcm.paused'),
                  'rtcm_evidence_dir':str(root/'rtcm-evidence')}
        rospy.set_param('/cup_gnss',params)
        (root/'parameters.json').write_text(json.dumps({'test_only':True,'parameters':params},indent=2)+'\n')
        log = (root/'gnss-node.log').open('w'); logs.append(log)
        command = [sys.executable,'-B',str(PACKAGE/'scripts/gnss_node.py')]
        assert not any(token.decode() in json.dumps(command) for token in secret_markers)
        node = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        roles.append(capture_process(node,run_id,['gnss_node.py']))
        wait(lambda: latest(serial_connected='true'), 'serial PTY open')
        fds = []
        for entry in Path('/proc/%d/fd'%node.pid).iterdir():
            try:
                if os.readlink(entry)==slave_path: fds.append(entry.name)
            except FileNotFoundError: pass
        assert len(fds)==1, 'The node must hold exactly one serial descriptor'
        mark('sole_serial_owner',pid=node.pid,fd=fds[0],path=slave_path)

        def feed():
            while not feed_stop.is_set():
                with lock: raw = raw_gga(phase['quality'],phase['diff_age']); sent.append(raw)
                encoded = (raw+'\r\n').encode('ascii')
                try:
                    os.write(master_fd,encoded[:7]); feed_stop.wait(.005); os.write(master_fd,encoded[7:])
                except OSError: return
                feed_stop.wait(.195)

        def read_serial():
            while not reader_stop.is_set():
                try:
                    ready,_,_ = select.select([master_fd],[],[],.05)
                    if ready:
                        data = os.read(master_fd,4096)
                        with lock: serial_received.extend(data)
                except (OSError,ValueError):
                    if not reader_stop.is_set(): reader_stop.wait(.05)

        reader = threading.Thread(target=read_serial,daemon=True); reader.start()
        feeder = threading.Thread(target=feed,daemon=True); feeder.start()
        wait(lambda: latest(gga_quality='4',fresh_rtk_usable='true'), 'q4 + real RTCM write qualification')
        assert len(serial_received)>0
        with lock: first = next(value for value in reversed(fixes) if value['status']==2)
        assert first['frame_id']=='gnss_link' and first['covariance_type']==0 and first['covariance']==[0.]*9
        assert abs(first['receipt_wall_s']-first['stamp'])<.5
        assert math.isclose(first['latitude'],31+12.123456/60,abs_tol=1e-10)
        assert math.isclose(first['longitude'],121+30.654321/60,abs_tol=1e-10)
        assert first['altitude']==20., 'MSL15 + geoid5 must be ellipsoid20'
        # Match by original GGA UTC rather than trusting the latest asynchronous diagnostic.
        nearest = min(raw_messages,key=lambda value:abs(value['receipt_wall_s']-first['receipt_wall_s']))
        undated_epoch = parse_nmea(nearest['raw'])['epoch']
        delta = (first['stamp']%86400-undated_epoch+43200)%86400-43200
        assert .9<delta<1.7, 'GGA time-of-day must not silently replace receipt stamp'
        mark('q4_fresh_upstream_fix',covariance='UNKNOWN',receipt_minus_gga_tod_s=delta)
        assert latest(nmea_backend='nmea_navsat_driver/0.6.1',ntrip_backend='ntrip_client')

        with lock: phase['quality']=5
        wait(lambda: latest(gga_quality='5',fresh_rtk_usable='false'), 'Float remains distinct from Fixed')
        assert any(value['status']==2 for value in fixes[-5:])
        mark('q5_float_not_fixed')
        with lock: phase['quality']=0
        wait(lambda: latest(gga_quality='0',fresh_rtk_usable='false'), 'quality0 invalid')
        wait(lambda: fixes[-1]['status']==-1, 'no-fix NavSatFix heartbeat')
        assert math.isnan(fixes[-1]['latitude'])
        mark('q0_no_fix')
        before = caster.accepted
        with lock: phase['quality']=4
        wait(lambda: latest(gga_quality='4',fresh_rtk_usable='true'), 'quality recovery')
        assert caster.accepted>before
        mark('q4_recovered_after_no_fix')

        with lock: phase['diff_age']=8.
        wait(lambda: latest(gga_quality='4',fresh_rtk_usable='false',
                             invalid_reason='differential_age_missing_or_stale'), 'stale differential age')
        mark('q4_stale_differential_rejected')
        with lock: phase['diff_age']=.5
        wait(lambda: latest(fresh_rtk_usable='true'), 'differential age recovery')
        before = caster.accepted; caster.bad.set()
        wait(lambda: diagnostics and int(diagnostics[-1]['values'].get('ntrip_crc_failures','0'))>=1,
             'bad CRC diagnosis')
        wait(lambda: caster.accepted>before and latest(fresh_rtk_usable='true'), 'upstream reconnect after CRC')
        assert b'TEST_ONLY_BAD_CRC' not in serial_received
        mark('bad_crc_reconnected_without_forwarding',caster_connections=caster.accepted)

        pause = root/'rtcm.paused'; pause.touch()
        wait(lambda: latest(ntrip_state='paused'), 'pause closes transport')
        with lock: paused_bytes=len(serial_received)
        wait(lambda: latest(gga_quality='4',fresh_rtk_usable='false',
                             fresh_rtk_reason='rtcm_serial_write_missing_or_stale'),
             'quality4 alone cannot keep stale RTCM usable',timeout=9)
        with lock: assert len(serial_received)==paused_bytes
        mark('paused_q4_stale_write_rejected')
        before = caster.accepted; pause.unlink()
        wait(lambda: latest(fresh_rtk_usable='true') and caster.accepted>before,'pause/resume restores live writes')
        mark('resumed_live_rtcm_and_fresh_q4')
        current_graph = graph(MASTER)
        publishers=dict(current_graph['state'][0])
        for topic in ('/gnss/nmea','/gnss/fix','/gnss/diagnostics'):
            assert publishers.get(topic)==['/cup_gnss'], 'GNSS must have one publisher'
        assert not any(topic in publishers for topic in ('/cmd_vel','/cmd_vel/nav','/cmd_vel/safe','/tf'))
        (root/'graph.json').write_text(json.dumps(current_graph,indent=2)+'\n')
        all_params=json.dumps(rospy.get_param('/')).encode()
        assert not any(secret in all_params for secret in secret_markers), 'credential content reached ROS parameters'
        feed_stop.set(); feeder.join(2)
        # Stop the owned node while its caster remains connected: shutdown must
        # close NTRIP and serial workers, not require the caster to exit first.
        stop_process(roles[-1],timeout=6)
        wait(lambda: node.poll() is not None,'node shutdown',timeout=2)
        assert node.returncode==0, 'GNSS node must exit cleanly'
        reader_stop.set(); reader.join(2)
        captured=bytes(serial_received)
        written=(root/'rtcm-evidence/serial-written.rtcm3').read_bytes()
        assert captured==written and captured, 'PTY received bytes must equal acknowledged UART evidence'
        (root/'pty-received.rtcm3').write_bytes(captured)
        assert all(upload in sent for upload in caster.uploads), 'Only original receiver GGA may be uploaded'
        assert caster.upstream_requests==caster.accepted and not caster.errors
        mark('clean_stop',exit_code=node.returncode,pty_bytes=len(captured),uploads=len(caster.uploads))
        for subscriber in subscribers: subscriber.unregister()
        subscribers=[]
        with lock: bag.close();bag=None
        # Scan actual serialized messages, not compressed file substrings only.
        with rosbag.Bag(str(root/'raw.bag')) as raw_bag:
            for topic,message,_stamp in raw_bag.read_messages():
                stream=io.BytesIO();message.serialize(stream)
                assert not any(secret in stream.getvalue() for secret in secret_markers)
        for path in (root/'gnss-node.log',root/'core.log',root/'parameters.json'):
            assert not any(secret in path.read_bytes() for secret in secret_markers)
        result={'test_only':True,'result':'PASS','duration_s':time.monotonic()-started,
                'nmea_backend':'nmea_navsat_driver/0.6.1','ntrip_backend':'ntrip_client/1.4.1',
                'phases':phases,'wire_counts':wire_counts,'raw_receiver_gga_uploads':len(caster.uploads),
                'caster_connections':caster.accepted,'bad_crc_injected':caster.bad_sent,
                'pty_bytes_equal_acknowledged_serial_write':len(captured),
                'serial_owner_descriptors':1,'credential_scan':'PASS',
                'hardware_acceptance':False,'rtcm_receiver_use_not_claimed':True}
        (root/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,sort_keys=True))
    except BaseException as exc:
        failure=type(exc).__name__
        (root/'failure.json').write_text(json.dumps({'test_only':True,'result':'FAIL',
                                                   'reason':str(exc),'phases':phases},indent=2)+'\n')
        print((root/'gnss-node.log').read_text()[-5000:] if (root/'gnss-node.log').exists() else '',file=sys.stderr)
        raise
    finally:
        feed_stop.set();reader_stop.set()
        for thread in (feeder,reader):
            if thread is not None:thread.join(2)
        if caster is not None:caster.close()
        for subscriber in subscribers: subscriber.unregister()
        for role in reversed(roles):stop_process(role,timeout=5)
        residue=[member for role in roles for member in group_members(role)]
        if bag is not None:
            with lock:bag.close()
        if master_fd is not None:os.close(master_fd)
        for log in logs:log.close()
        (root/'events.jsonl').write_text(''.join(json.dumps(value,sort_keys=True)+'\n' for value in events))
        (root/'uploads.json').write_text(json.dumps({'test_only':True,'raw_gga':caster.uploads if caster else []},indent=2)+'\n')
        (root/'cleanup.json').write_text(json.dumps({'test_only':True,'owned_process_residue':residue,
                                                    'failed':failure,'credentials_removed':True},indent=2)+'\n')
        temporary.cleanup()
        rospy.signal_shutdown('TEST_ONLY complete')
        if residue:raise AssertionError('owned process residue remains')


if __name__=='__main__':main()
