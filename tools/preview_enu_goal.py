#!/usr/bin/env python3
"""Preview one caller-supplied ENU target in map using planar rigid transforms.

This offline calculator does not measure or validate a registration. A transform
named target_from_source means p_target = R(yaw) * p_source + translation.
Positions are metres; yaw is radians, counterclockwise from the frame's +x axis.
For ENU, +x is East and +y is North. The optional yaw is the target orientation,
not the UAV's attitude. No ROS, network, plan request or motion command is used.
"""
import argparse
import json
import math
import sys


def finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError('must be finite')
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--east', required=True, type=finite, help='target East coordinate in metres')
    parser.add_argument('--north', required=True, type=finite, help='target North coordinate in metres')
    parser.add_argument('--target-yaw', type=finite,
                        help='optional target yaw in ENU, radians; omitted means unspecified')
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--map-from-enu', nargs=3, type=finite, metavar=('TX', 'TY', 'YAW'),
                        help='direct map <- ENU transform: metres, metres, radians')
    choice.add_argument('--odom-from-enu', nargs=3, type=finite, metavar=('TX', 'TY', 'YAW'),
                        help='odom <- ENU transform; requires --map-from-odom')
    parser.add_argument('--map-from-odom', nargs=3, type=finite, metavar=('TX', 'TY', 'YAW'),
                        help='map <- odom transform from the same evaluation snapshot')
    parser.add_argument('--origin-label', help='optional caller-provided origin note; not verified')
    parser.add_argument('--alignment-source', help='optional registration measurement/source note')
    args = parser.parse_args(argv)
    if args.odom_from_enu is not None and args.map_from_odom is None:
        parser.error('--odom-from-enu requires --map-from-odom from the same evaluation snapshot')
    if args.map_from_enu is not None and args.map_from_odom is not None:
        parser.error('--map-from-odom is only used with --odom-from-enu')
    return args


def normalized_yaw(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def transform_target(x, y, yaw, transform):
    """Apply one SE(2) transform; preserve an unspecified target orientation."""
    tx, ty, angle = transform
    cosine, sine = math.cos(angle), math.sin(angle)
    result_x = tx + cosine * x - sine * y
    result_y = ty + sine * x + cosine * y
    # Normalize before addition so large, finite input angles cannot overflow.
    result_yaw = (None if yaw is None else
                  normalized_yaw(normalized_yaw(yaw) + normalized_yaw(angle)))
    if not math.isfinite(result_x) or not math.isfinite(result_y):
        raise ValueError('transformed coordinates are not finite; no preview was produced')
    return result_x, result_y, result_yaw


def preview(args):
    transforms = ([('map_from_enu', args.map_from_enu)] if args.map_from_enu is not None
                  else [('odom_from_enu', args.odom_from_enu),
                        ('map_from_odom', args.map_from_odom)])
    x, y, yaw = args.east, args.north, args.target_yaw
    input_target = {'frame_id': 'enu', 'east': x, 'north': y}
    if yaw is not None:
        input_target['target_yaw'] = yaw
    transform_records = []
    for direction, values in transforms:
        x, y, yaw = transform_target(x, y, yaw, values)
        tx, ty, angle = values
        transform_records.append({'direction': direction,
                                  'translation': {'x': tx, 'y': ty}, 'yaw': angle})
    target = {'frame_id': 'map', 'x': x, 'y': y}
    if yaw is not None:
        target['yaw'] = yaw
    registration = {
        'verified': False,
        'assumption': ('The caller-provided registration is valid for this target and '
                       'transform snapshot. Origin, drift, map/odom resets and '
                       'measurement accuracy are not checked.'),
    }
    if args.origin_label is not None:
        registration['origin_label'] = args.origin_label
    if args.alignment_source is not None:
        registration['source'] = args.alignment_source
    return {'preview_only': True, 'units': {'position': 'm', 'yaw': 'rad'},
            'yaw_convention': 'counterclockwise from +x; ENU +x is East',
            'input': input_target, 'transforms': transform_records,
            'map_target': target, 'registration': registration}


def main(argv=None):
    args = arguments(argv)
    try:
        record = preview(args)
        output = json.dumps(record, indent=2, sort_keys=True, allow_nan=False)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2
    print(output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
