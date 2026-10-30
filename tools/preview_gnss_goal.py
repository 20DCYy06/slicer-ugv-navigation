#!/usr/bin/env python3
"""Preview a WGS84 antenna point through an explicit common ENU origin into map.

Latitude/longitude are degrees and altitude is WGS84 ellipsoid height in metres.
This offline calculator reuses cup_gnss.enu and preview_enu_goal; it does not
measure a map registration, convert datums, compensate a lever arm, or use ROS.
Inputs and origin must both actually be WGS84. Relabelling CGCS2000 is not a
datum conversion. Optional target yaw is radians counterclockwise from East.
"""
import argparse
import json
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
# Resolve only this project's sources, including under python -I -S.
sys.path[:0] = [str(WORKSPACE / 'tools'), str(WORKSPACE / 'src/cup_gnss/src')]
from cup_gnss.enu import to_enu
from preview_enu_goal import finite, preview as enu_preview


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--latitude', required=True, type=finite, help='WGS84 latitude in degrees')
    parser.add_argument('--longitude', required=True, type=finite, help='WGS84 longitude in degrees')
    parser.add_argument('--altitude-ellipsoid', required=True, type=finite,
                        help='WGS84 ellipsoid height in metres, not MSL')
    origin = parser.add_mutually_exclusive_group()
    origin.add_argument('--origin-file', type=Path,
                        help='legacy common origin JSON; provide a transform on the CLI')
    origin.add_argument('--site-config', type=Path,
                        help='commented origin/registration YAML; default without CLI transform: gnss_site.yaml')
    choice = parser.add_mutually_exclusive_group()
    choice.add_argument('--map-from-enu', nargs=3, type=finite, metavar=('TX', 'TY', 'YAW'),
                        help='caller-supplied map <- ENU transform: metres, metres, radians')
    choice.add_argument('--odom-from-enu', nargs=3, type=finite, metavar=('TX', 'TY', 'YAW'),
                        help='caller-supplied odom <- ENU; requires --map-from-odom')
    parser.add_argument('--map-from-odom', nargs=3, type=finite, metavar=('TX', 'TY', 'YAW'),
                        help='map <- odom from the same evaluation snapshot')
    parser.add_argument('--target-yaw', type=finite,
                        help='optional target yaw in ENU radians; omitted means unspecified')
    parser.add_argument('--alignment-source', help='optional caller-provided registration source note')
    args = parser.parse_args(argv)
    if args.odom_from_enu is not None and args.map_from_odom is None:
        parser.error('--odom-from-enu requires --map-from-odom from the same evaluation snapshot')
    if args.map_from_enu is not None and args.map_from_odom is not None:
        parser.error('--map-from-odom is only used with --odom-from-enu')
    if args.map_from_odom is not None and args.odom_from_enu is None:
        parser.error('--map-from-odom requires --odom-from-enu')
    if args.origin_file is not None and args.map_from_enu is None and args.odom_from_enu is None:
        parser.error('--origin-file requires --map-from-enu or --odom-from-enu')
    return args


def origin_record(record):
    if not isinstance(record, dict) or record.get('datum') != 'WGS84':
        raise ValueError('origin datum must explicitly be WGS84; unknown/CGCS2000 cannot be relabelled')
    if not isinstance(record.get('origin'), dict):
        raise ValueError('origin must contain latitude, longitude and altitude_ellipsoid_m')
    origin_id = record.get('origin_id')
    if origin_id is not None and not isinstance(origin_id, str):
        raise ValueError('origin_id must be a string or null')
    return {'datum': 'WGS84', 'origin_id': origin_id,
            'origin': {key: record['origin'].get(key) for key in
                       ('latitude', 'longitude', 'altitude_ellipsoid_m')}}


def load_origin(path):
    with path.open(encoding='utf-8') as stream:
        return origin_record(json.load(stream))


def site_vector(record, key, allow_unknown=False):
    values = record.get(key)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(key + ' must contain three numbers')
    result = []
    for value in values:
        if value is None and allow_unknown:
            result.append(None)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            result.append(finite(value))
        else:
            raise ValueError(key + ' must contain finite numbers' + (' or null' if allow_unknown else ''))
    return result


def load_site(path):
    # Legacy JSON/explicit-transform calls remain usable under python -I -S.
    try:
        import yaml
    except ImportError:
        raise ValueError('site YAML requires the existing python3-yaml dependency') from None
    try:
        with path.open(encoding='utf-8') as stream:
            record = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise ValueError('invalid site YAML: ' + str(error)) from None
    return record, origin_record(record)


def preview(args):
    use_site = args.site_config is not None or (
        args.origin_file is None and args.map_from_enu is None and args.odom_from_enu is None)
    antenna = None
    if use_site:
        path = args.site_config or WORKSPACE / 'src/ugv_bringup/config/gnss_site.yaml'
        record, origin = load_site(path)
        antenna = site_vector(record, 'antenna_in_base_link_m', allow_unknown=True)
        if args.map_from_enu is None and args.odom_from_enu is None:
            args.map_from_enu = site_vector(record, 'map_from_enu')
    else:
        origin = load_origin(args.origin_file or
                             WORKSPACE / 'src/ugv_bringup/config/gnss_enu_origin.json')
    east, north, up = to_enu(args.latitude, args.longitude, args.altitude_ellipsoid,
                             origin['origin'])
    args.east, args.north = east, north
    args.origin_label = origin['origin_id']
    result = enu_preview(args)
    result['input']['up'] = up
    result['gnss_input'] = {'datum': 'WGS84', 'latitude': args.latitude,
                            'longitude': args.longitude,
                            'altitude_ellipsoid_m': args.altitude_ellipsoid}
    result['enu_origin'] = origin
    result['units']['latitude_longitude'] = 'deg'
    result['physical_point'] = 'gnss_antenna'
    result['lever_arm_compensation_applied'] = False
    if antenna is not None:
        result['antenna_in_base_link_m'] = antenna  # Recorded only; no body pose/yaw is available here.
    result['conversion_note'] = ('WGS84 inputs and common origin only; no datum/epoch conversion. '
                                 'Relabelling CGCS2000 is not a conversion. '
                                 'Map registration and antenna-to-base offset are not measured here.')
    return result


def main(argv=None):
    args = arguments(argv)
    try:
        result = preview(args)
        output = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
    except (OSError, ValueError, TypeError, OverflowError) as error:
        print('GNSS preview rejected: ' + str(error), file=sys.stderr)
        return 2
    print(output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
