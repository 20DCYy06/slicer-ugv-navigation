#!/usr/bin/env python3
"""Low-bandwidth external state export; internal navigation topics are untouched.

Keep serialized messages, timestamps and frame IDs unchanged. Rate caps apply
only to external telemetry; latched discrete states publish only when changed.
"""
import argparse
from pathlib import Path
import threading
import time
import rospy
from roslib.message import get_message_class
import yaml


class StateRelay:
    def __init__(self, publisher, route):
        self.publisher = publisher
        hz = float(route.get('max_hz', 0))
        self.period = 1. / hz if hz > 0 else 0.
        self.on_change = route.get('on_change', False)
        self.last_at, self.last_payload = -float('inf'), None
        self.lock = threading.Lock()

    def forward(self, message):
        payload = bytes(message._buff)
        with self.lock:
            now = time.monotonic()
            if self.on_change and payload == self.last_payload:
                return
            if now - self.last_at < self.period:
                return
            self.publisher.publish(message)
            self.last_at, self.last_payload = now, payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).parent/'config/relay.yaml'))
    args = parser.parse_args(rospy.myargv()[1:])
    with open(args.config, encoding='utf-8') as stream:
        routes = yaml.safe_load(stream)['routes']
    rospy.init_node('state_relay', anonymous=False)
    publishers, subscribers = [], []
    for route in routes:
        message_type = get_message_class(route['type'])
        if message_type is None:
            raise ValueError('ROS message type unavailable: '+route['type'])
        publisher = rospy.Publisher(route['target'], message_type, queue_size=1,
                                    latch=route.get('latch', False), tcp_nodelay=True)
        publishers.append(publisher)
        relay = StateRelay(publisher, route)
        subscribers.append(rospy.Subscriber(route['source'], rospy.AnyMsg,
                                             relay.forward, queue_size=1,
                                             tcp_nodelay=True))
        rospy.loginfo('State export: %s -> %s [%s] max_hz=%s on_change=%s',
                      route['source'], route['target'], route['type'],
                      route.get('max_hz', 0), route.get('on_change', False))
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
