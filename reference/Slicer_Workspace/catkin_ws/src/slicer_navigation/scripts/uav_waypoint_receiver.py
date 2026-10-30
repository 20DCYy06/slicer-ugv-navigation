#!/usr/bin/env python3
"""FKIE-imported UAV waypoint batches -> YAML only; never send a navigation goal."""
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

import rospy
from std_msgs.msg import String
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from navigation_core import validate_waypoints


def save_batch(filename, data):
    validate_waypoints(data)
    data['waypoints'] = [dict(name=p.get('name', 'point_%03d' % i),
                              **{k: v for k, v in p.items() if k != 'name'})
                         for i, p in enumerate(data['waypoints'], 1)]
    filename = Path(filename).resolve()
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(str(filename)+'.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if filename.exists():
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            backup = filename.parent/'.backups'/(filename.name+'.fkie_uav.'+stamp+'.bak')
            backup.parent.mkdir(exist_ok=True)
            shutil.copy2(filename, backup)
        fd, temporary = tempfile.mkstemp(prefix=filename.name+'.', dir=str(filename.parent))
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, filename)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    return data


def main():
    rospy.init_node('ugv_waypoint_receiver')
    topic = rospy.get_param('~topic', '/uav/mission/waypoints')
    filename = rospy.get_param('~file', str(Path(__file__).resolve().parents[1]/'config/waypoints.yaml'))

    def receive(message):
        try:
            data = save_batch(filename, json.loads(message.data))
            sender = message._connection_header.get('callerid', 'unknown')
            for point in data['waypoints']:
                rospy.loginfo('[UAV POINT] %s x=%.4f y=%.4f z=%.4f yaw=%s wait_s=%s',
                              point['name'], point['x'], point['y'], point['z'],
                              point.get('yaw'), point.get('wait_s'))
            rospy.loginfo('[UAV SAVED] %d points frame=%s sender=%s file=%s; navigation NOT started',
                          len(data['waypoints']), data['frame_id'], sender, filename)
        except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as exc:
            rospy.logerr('[UAV NOT SAVED] %s; existing file preserved', exc)

    subscriber = rospy.Subscriber(topic, String, receive, queue_size=1)
    rospy.loginfo('[UAV LISTEN] %s -> %s; save only, no automatic patrol', topic, filename)
    rospy.spin()


if __name__ == '__main__':
    main()
