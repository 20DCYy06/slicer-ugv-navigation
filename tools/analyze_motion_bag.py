#!/usr/bin/env python3
"""Read-only ROS bag motion evidence. All comparisons use bag receive time.

Importing this module needs no ROS. MCU acceptance, software duty references and wheel
odometry are observations, never proof of actuator execution or ground stop.
"""
import argparse
from collections import Counter, defaultdict
import json
import math

POSE_TOPICS = ('/odom', '/lio/odom', '/wheel/odom', '/Odometry')
CHAIN_TOPICS = ('/cmd_vel/nav_raw', '/cmd_vel/nav', '/cmd_vel')
DIAGNOSTICS = '/slicer_c30d_driver/diagnostics'
MOTION_COMMANDS = {16: ('wheel_speed', 'mm/s', 'speed_mm_s_abcd'),
                   17: ('pwm', 'permille', 'command_pwm_permille_abcd'),
                   22: ('body_speed', 'mm/s and mrad/s', None)}


def get(value, path, default=None):
    for part in path.split('.'):
        value = value.get(part, default) if isinstance(value, dict) else getattr(value, part, default)
        if value is None:
            return default
    return value


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stamp(value):
    if hasattr(value, 'to_sec'):
        return value.to_sec()
    if number(value):
        return value
    seconds = get(value, 'secs')
    return seconds + get(value, 'nsecs', 0)*1e-9 if number(seconds) else None


def unknown(reason):
    return dict(state='unknown', reason=reason)


def point(message, at, **fields):
    return dict(receive_s=at, header_stamp_s=stamp(get(message, 'header.stamp')), **fields)


def values(row, key):
    items = row.get(key)
    return items if isinstance(items, (list, tuple)) and len(items) == 4 and all(map(number, items)) else None


def motion_values(row):
    if row.get('kind') == 22:
        body = [row.get('body_vx_mm_s'), row.get('body_wz_mrad_s')]
        return body if all(type(value) is int and -32768 <= value <= 32767 for value in body) else None
    specification = MOTION_COMMANDS.get(row.get('kind'))
    return values(row, specification[2]) if specification else None


def advancing_frames(rows, tick_key):
    advancing = []
    for item in rows:
        tick = item.get(tick_key)
        if type(tick) is int and 0 <= tick < 2**32 and (not advancing or
                0 < (tick-advancing[-1][tick_key]) % 2**32 < 2**31):
            advancing.append(item)
    return advancing


def body_feedback_summary(rows, max_age):
    advancing = advancing_frames(rows, 'mcu_us')
    pairs = [(a, b) for a, b in zip(advancing, advancing[1:])
             if b['receive_s']-a['receive_s'] <= max_age]
    invalid_imu, faults = 0, 0
    for item in rows:
        flags, age, status = item.get('imu_flags'), item.get('imu_age_ms'), item.get('status_flags')
        invalid_imu += not (item.get('version') == 1 and item.get('who_am_i') == 0xea
                            and type(flags) is int and flags & 7 == 7 and not flags & 8
                            and number(age) and 0 <= age <= 100)
        faults += type(status) is int and status >> 8 != 0

    def maximum(key):
        return max((abs(item[key]) for item in rows if number(item.get(key))), default=None)

    return dict(count=len(rows), advancing_mcu_frame_count=len(advancing),
                rejected_or_missing_mcu_tick_count=len(rows)-len(advancing),
                mcu_tick_progression=dict(state='observed', through_receive_s=pairs[-1][1]['receive_s'])
                if pairs else unknown('need a fresh pair of advancing body MCU ticks'),
                invalid_imu_feedback_count=invalid_imu, fault_feedback_count=faults,
                last_values=rows[-1] if rows else None,
                imu_flags_observed=sorted({item['imu_flags'] for item in rows if type(item.get('imu_flags')) is int}),
                sources_observed=sorted({item['source'] for item in rows if type(item.get('source')) is int}),
                status_flags_observed=sorted({item['status_flags'] for item in rows if type(item.get('status_flags')) is int}),
                max_abs_vx_m_s=maximum('vx_m_s'), max_abs_wz_rad_s=maximum('wz_rad_s'),
                max_abs_target_vx_m_s=maximum('target_vx_m_s'), max_abs_target_wz_rad_s=maximum('target_wz_rad_s'))


def velocity_summary(rows, start, end, linear_limit, angular_limit, max_age, duration):
    samples = []
    for at, message in rows:
        if start <= at <= end:
            root = get(message, 'twist.twist', message)
            v, w = get(root, 'linear.x'), get(root, 'angular.z')
            if number(v) and number(w):
                samples.append(point(message, at, linear_x_m_s=v, angular_z_rad_s=w,
                                     valid=True, nonzero=abs(v) > linear_limit or abs(w) > angular_limit))
            else:
                samples.append(point(message, at, valid=False, nonzero=None))
    last = next((item for item in reversed(samples) if item['nonzero'] is True), None)
    invalid = [item for item in samples if not item['valid']]
    barrier = max(last['receive_s'] if last else -math.inf,
                  invalid[-1]['receive_s'] if invalid else -math.inf)
    tail = [item for item in samples if item['receive_s'] > barrier and item['valid']]
    first = tail[0] if tail else None
    gaps = [b['receive_s']-a['receive_s'] for a, b in zip(tail, tail[1:])]
    continuous = (len(tail) >= 2 and tail[-1]['receive_s']-tail[0]['receive_s'] >= duration
                  and max(gaps, default=math.inf) <= max_age and end-tail[-1]['receive_s'] <= max_age)
    state = (dict(state='observed', first_receive_s=first['receive_s'],
                  through_receive_s=tail[-1]['receive_s']) if continuous and last is not None
             else unknown('need preceding nonzero and a fresh continuous zero tail'))
    return dict(sample_count=len(samples), invalid_sample_count=len(invalid), last_nonzero=last,
                first_zero_after_last_nonzero=next((item for item in samples if item['nonzero'] is False
                                                    and last is not None and item['receive_s'] > last['receive_s']), None),
                first_observed_zero_tail=first, last_sample=samples[-1] if samples else None,
                maximum_zero_tail_gap_s=max(gaps, default=None), sustained_zero=state)


def planar_pose(pose, frame_id):
    x, y = get(pose, 'position.x'), get(pose, 'position.y')
    q = [get(pose, 'orientation.'+key, 0.) for key in ('x', 'y', 'z', 'w')]
    if not number(x) or not number(y) or not all(map(number, q)):
        return unknown('pose or quaternion is unavailable')
    norm = math.hypot(*q)
    if not number(norm) or norm < 1e-6:
        return unknown('pose or quaternion is unavailable')
    qx, qy, qz, qw = [v/norm for v in q]
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return dict(state='observed', frame_id=frame_id, x_m=x, y_m=y, yaw_rad=yaw)


def pose_sample(rows, at, max_age, bag_end):
    candidates = [(time, message) for time, message in rows if time <= at]
    if not candidates or at > bag_end:
        return unknown('requested observation time is not covered')
    time, message = candidates[-1]
    if at-time > max_age:
        return dict(unknown('latest pose is stale'), latest_receive_s=time, age_s=at-time)
    pose = planar_pose(get(message, 'pose.pose'), get(message, 'header.frame_id'))
    return point(message, time, age_s=at-time, **pose) if pose['state'] == 'observed' else pose


def pose_summary(rows, begin, terminal, max_age, bag_end, next_goal=None, target=None):
    before = pose_sample(rows, begin, max_age, bag_end)
    output = dict(before_goal=before)
    for label, at in (('terminal', terminal), ('terminal_plus_5s', terminal+5 if terminal is not None else None)):
        sample = pose_sample(rows, at, max_age, bag_end) if at is not None else unknown('no matching terminal result')
        if at is not None and next_goal is not None and at >= next_goal:
            sample = unknown('observation interrupted by a later goal')
        if before.get('state') == sample.get('state') == 'observed' and before['frame_id'] == sample['frame_id']:
            dx, dy, yaw = sample['x_m']-before['x_m'], sample['y_m']-before['y_m'], before['yaw_rad']
            da = sample['yaw_rad']-yaw
            sample['relative'] = dict(forward_m=dx*math.cos(yaw)+dy*math.sin(yaw),
                                      left_m=-dx*math.sin(yaw)+dy*math.cos(yaw),
                                      displacement_m=math.hypot(dx, dy), yaw_rad=math.atan2(math.sin(da), math.cos(da)))
        else:
            sample['relative'] = unknown('need fresh start/end poses in the same frame')
        if target is not None:
            residual = unknown('need a valid goal and fresh /odom pose in the same goal frame')
            if (target.get('state') == sample.get('state') == 'observed'
                    and target.get('frame_id') and target['frame_id'] == sample['frame_id']):
                dx, dy = target['x_m']-sample['x_m'], target['y_m']-sample['y_m']
                da = target['yaw_rad']-sample['yaw_rad']
                residual = dict(state='observed', frame_id=target['frame_id'], x_m=dx, y_m=dy,
                                position_norm_m=math.hypot(dx, dy), yaw_rad=math.atan2(math.sin(da), math.cos(da)))
            sample['goal_frame_odom_residual'] = residual
        output[label] = sample
    return output


def diagnostic_events(rows):
    output = []
    for at, message in rows:
        for status in get(message, 'status', []):
            fields = {}
            for item in get(status, 'values', []):
                text = get(item, 'value')
                try:
                    fields[get(item, 'key')] = json.loads(text)
                except (ValueError, TypeError):
                    fields[get(item, 'key')] = text
            fields.update(receive_s=at, header_stamp_s=stamp(get(message, 'header.stamp')),
                          event=get(status, 'message'))
            output.append(fields)
    return output


def driver_summary(instance, rows, max_age):
    sequences = [item['event_seq'] for item in rows if type(item.get('event_seq')) is int]
    unique = sorted(set(sequences))
    gaps = [[a+1, b-1] for a, b in zip(unique, unique[1:]) if b > a+1]
    tx, sent_by_sequence = [], defaultdict(list)
    for item in rows:
        if item.get('event') != 'serial_tx' or item.get('success') is not True:
            continue
        if motion_values(item) is not None:
            mode, unit, _ = MOTION_COMMANDS[item['kind']]
            item = dict(item, motion_mode=mode, command_unit=unit)
            tx.append(item)
        # Keep malformed/non-motion transmissions in the index as well, so
        # sequence reuse cannot falsely select an earlier valid zero command.
        sequence = item.get('command_seq')
        if type(sequence) is int and 0 <= sequence <= 65535:
            sent_by_sequence[sequence].append(item)
    nonzero = [item for item in tx if any(motion_values(item))]
    last = nonzero[-1] if nonzero else None
    zeros = [item for item in tx if not any(motion_values(item))
             and (last is None or item['receive_s'] > last['receive_s'])]
    rx = [item for item in rows if item.get('event') == 'command_rx' and item.get('valid') is True
          and number(item.get('linear_x_m_s')) and number(item.get('angular_z_rad_s'))]
    last_rx_nonzero = next((item for item in reversed(rx)
                            if item['linear_x_m_s'] != 0 or item['angular_z_rad_s'] != 0), None)
    rx_zero = next((item for item in rx if item['linear_x_m_s'] == item['angular_z_rad_s'] == 0
                    and (last_rx_nonzero is None or item['receive_s'] > last_rx_nonzero['receive_s'])), None)
    telemetry = [item for item in rows if item.get('event') == 'telemetry']
    body_telemetry = [item for item in rows if item.get('event') == 'body_telemetry']
    advancing = advancing_frames(telemetry, 'mcu_tick_us')
    body_advancing = advancing_frames(body_telemetry, 'mcu_us')

    def matched_zero(item, allow_body_from_legacy=False):
        # Exact lookup handles skipped acknowledgements and 65535 -> 0.
        # Reuse of the same sequence inside the observation window is ambiguous.
        body = item.get('event') == 'body_telemetry'
        ack = item.get('ack_seq' if body else 'accepted_command_seq')
        matches = [sent for sent in sent_by_sequence.get(ack, [])
                   if sent['receive_s'] <= item['receive_s']] if type(ack) is int and 0 <= ack <= 65535 else []
        return (matches[0] if len(matches) == 1 and matches[0] in zeros
                and ((matches[0].get('kind') == 22) == body
                     or allow_body_from_legacy and not body) else None)

    def freshness_confirmation(index, samples=advancing, allow_body_from_legacy=False):
        current = samples[index]
        if index and current['receive_s']-samples[index-1]['receive_s'] <= max_age:
            return current['receive_s']
        if index+1 < len(samples):
            following = samples[index+1]
            if (following['receive_s']-current['receive_s'] <= max_age
                    and matched_zero(following, allow_body_from_legacy) is not None):
                return following['receive_s']
        return None

    all_telemetry = sorted(telemetry+body_telemetry, key=lambda item: item['receive_s'])
    raw_candidate = next((dict(item, state='observed', matched_zero_tx=matched_zero(item)) for item in all_telemetry
                          if instance is not None and matched_zero(item) is not None),
                         unknown('no matching accepted-command observation'))
    accepted = unknown('no unambiguous successful zero transmission matched within this driver instance')
    pwm_zero = unknown('no software duty-reference zero correlated with a matched accepted zero')
    if instance is not None:
        candidates = []
        for samples in (advancing, body_advancing):
            for index, item in enumerate(samples):
                matched = matched_zero(item)
                confirmed = freshness_confirmation(index, samples)
                if matched is not None and confirmed is not None:
                    candidates.append(dict(item, state='observed', matched_zero_tx=matched,
                                           freshness_confirmed_through_receive_s=confirmed))
        if candidates:
            accepted = min(candidates, key=lambda item: item['receive_s'])
    if accepted['state'] == 'observed':
        for index, item in enumerate(advancing):
            if (values(item, 'pwm_timer_ticks_abcd') is None
                    or any(values(item, 'pwm_timer_ticks_abcd'))
                    or freshness_confirmation(index, allow_body_from_legacy=True) is None):
                continue
            matched = matched_zero(item, allow_body_from_legacy=True)
            if matched is None:
                continue
            confirmation_fields = {}
            if matched['kind'] == 22:
                # Firmware emits 0x90 before 0x93 while the host keeps sending
                # new zero sequences. Correlate each duty sample to its own
                # independently fresh body ACK in either receive order.
                confirmations = [candidate for candidate in candidates
                                 if candidate['matched_zero_tx'] == matched
                                 and abs(candidate['receive_s']-item['receive_s']) <= max_age]
                if not confirmations:
                    continue
                confirmation = min(confirmations, key=lambda candidate: abs(candidate['receive_s']-item['receive_s']))
                confirmation_fields = dict(body_zero_confirmation_receive_s=confirmation['receive_s'],
                    body_zero_freshness_confirmed_through_receive_s=confirmation['freshness_confirmed_through_receive_s'])
            elif item['receive_s'] < accepted['receive_s']:
                continue
            # These are evidence receive times, never physical execution latency.
            pwm_zero = dict(item, state='observed', **confirmation_fields)
            break
    encoder = unknown('need two telemetry encoder observations after software duty-reference zero')
    if pwm_zero['state'] == 'observed':
        raw = [item for item in advancing if item['receive_s'] >= pwm_zero['receive_s']
               and values(item, 'raw_encoder_abcd') is not None]
        pairs = [(a, b) for a, b in zip(raw, raw[1:])
                 if type(a.get('mcu_tick_us')) is type(b.get('mcu_tick_us')) is int
                 and 0 < (b['mcu_tick_us']-a['mcu_tick_us']) % 2**32 < 2**31
                 and b['receive_s']-a['receive_s'] <= max_age]
        if pairs:
            deltas = [[((b-a+2**31) % 2**32)-2**31 for a, b in zip(
                first['raw_encoder_abcd'], second['raw_encoder_abcd'])] for first, second in pairs]
            encoder = dict(state='observed', sample_pairs=len(pairs), changed_pairs=sum(any(d) for d in deltas),
                           sum_abs_delta_counts_abcd=[sum(abs(d[i]) for d in deltas) for i in range(4)],
                           through_receive_s=pairs[-1][1]['receive_s'])
    voltage = [item['voltage_v'] for item in telemetry if number(item.get('voltage_v'))]
    fresh_pairs = [(a, b) for a, b in zip(advancing, advancing[1:]) if b['receive_s']-a['receive_s'] <= max_age]

    def maximum(items, key):
        numbers = [abs(v) for item in items for v in (values(item, key) or [])]
        return max(numbers, default=None)

    return dict(driver_instance_id=instance, first_receive_s=rows[0]['receive_s'], last_receive_s=rows[-1]['receive_s'],
                event_count=len(rows), event_seq_gap_ranges=gaps,
                unobserved_prefix_events=max(0, unique[0]-1) if unique else None,
                missing_event_seq_count=len(rows)-len(sequences), duplicate_event_seq_count=len(sequences)-len(unique),
                event_seq_out_of_order_count=sum(b < a for a, b in zip(sequences, sequences[1:])),
                advancing_mcu_frame_count=len(advancing),
                rejected_or_missing_mcu_tick_count=len(telemetry)-len(advancing),
                mcu_tick_progression=dict(state='observed', through_receive_s=fresh_pairs[-1][1]['receive_s'])
                if fresh_pairs
                else unknown('need a fresh pair of advancing MCU ticks'),
                sent_motion_command_kinds=sorted({item['kind'] for item in tx}),
                last_nonzero_motion_tx=last,
                last_nonzero_pwm_tx=next((item for item in reversed(nonzero) if item['kind'] == 17), None),
                last_nonzero_wheel_speed_tx=next((item for item in reversed(nonzero) if item['kind'] == 16), None),
                last_nonzero_body_speed_tx=next((item for item in reversed(nonzero) if item['kind'] == 22), None),
                body_command_units=dict(body_vx_mm_s='mm/s', body_wz_mrad_s='mrad/s'),
                body_command_targets=[item for item in rows if item.get('event') == 'command_tx'
                                      and item.get('control_mode') == 'body_speed'],
                body_feedback=body_feedback_summary(body_telemetry, max_age),
                first_zero_rx=rx_zero,
                first_zero_tx_after_last_nonzero=zeros[0] if zeros and last else unknown('no observed nonzero-to-zero transmission transition'),
                raw_zero_acceptance_candidate=raw_candidate, accepted_zero_observation=accepted,
                telemetry_duty_reference_zero=pwm_zero, encoder_after_duty_reference_zero=encoder,
                # Preserve existing report consumers; both names denote the
                # same software reference, never an actual CCR readback.
                telemetry_pwm_zero=pwm_zero, encoder_after_pwm_zero=encoder,
                voltage_v=dict(min=min(voltage), max=max(voltage)) if voltage else unknown('voltage missing'),
                flags_observed=sorted({item['flags'] for item in telemetry if type(item.get('flags')) is int}),
                max_abs_sent_pwm_permille=maximum([item for item in tx if item['kind'] == 17], 'command_pwm_permille_abcd'),
                max_abs_sent_wheel_speed_mm_s=maximum([item for item in tx if item['kind'] == 16], 'speed_mm_s_abcd'),
                max_abs_sent_body_vx_mm_s=max((abs(item['body_vx_mm_s']) for item in tx if item['kind'] == 22), default=None),
                max_abs_sent_body_wz_mrad_s=max((abs(item['body_wz_mrad_s']) for item in tx if item['kind'] == 22), default=None),
                max_abs_telemetry_duty_reference_timer_ticks=maximum(telemetry, 'pwm_timer_ticks_abcd'),
                max_abs_telemetry_pwm_timer_ticks=maximum(telemetry, 'pwm_timer_ticks_abcd'),
                max_abs_measured_mm_s=maximum(telemetry, 'measured_mm_s_abcd'))


def driver_groups(events, max_age=.5):
    groups = defaultdict(list)
    for item in events:
        groups[item.get('driver_instance_id')].append(item)
    return [driver_summary(instance, rows, max_age) for instance, rows in groups.items()]


def analyze(records, zero_epsilon=1e-9, max_age_s=.5, continuous_s=1.):
    rows = sorted(records, key=lambda item: item[2])
    topics = defaultdict(list)
    for topic, message, at in rows:
        topics[topic].append((at, message))
    begin, end = (rows[0][2], rows[-1][2]) if rows else (None, None)
    events = diagnostic_events(topics[DIAGNOSTICS])
    goals, results = [], []
    for topic, message, at in rows:
        if topic.endswith('/move_base/goal'):
            goals.append((at, message))
        elif topic.endswith('/move_base/result'):
            results.append((at, message))
    output = []
    for goal_index, (at, message) in enumerate(goals):
        identifier = get(message, 'goal_id.id')
        matched = [(time, item) for time, item in results if time >= at
                   and identifier and get(item, 'status.goal_id.id') == identifier
                   and get(item, 'status.status') in (2, 3, 4, 5, 8, 9)]
        terminal, terminal_message = matched[0] if matched else (None, None)
        next_goal = goals[goal_index+1][0] if goal_index+1 < len(goals) else None
        until = min(end, terminal+5 if terminal is not None else end, next_goal if next_goal is not None else end)
        scoped = defaultdict(list, {topic: [(time, item) for time, item in topic_rows if next_goal is None or time < next_goal]
                                    for topic, topic_rows in topics.items()})
        chain = {topic: velocity_summary(scoped[topic], at, until, zero_epsilon, zero_epsilon,
                                         max_age_s, continuous_s) for topic in CHAIN_TOPICS}
        wheel = velocity_summary(scoped['/wheel/odom'], at, until, .03, .05, max_age_s, continuous_s)
        if terminal is None:
            for stream in [*chain.values(), wheel]:
                stream['sustained_zero'] = unknown('no matching terminal result')
        target = get(message, 'goal.target_pose')
        target = planar_pose(get(target, 'pose'), get(target, 'header.frame_id'))
        output.append(dict(goal_id=identifier, receive_s=at,
                           target=target,
                           result=point(terminal_message, terminal, status=get(terminal_message, 'status.status'))
                           if terminal is not None else None,
                           observation_end_s=until, next_goal_receive_s=next_goal,
                           requested_post_result_5s_covered=terminal is not None and end >= terminal+5
                           and (next_goal is None or next_goal > terminal+5),
                           poses={topic: pose_summary(scoped[topic], at, terminal, max_age_s, end, next_goal,
                                                      target if topic == '/odom' else None) for topic in POSE_TOPICS},
                           velocity_chain=chain, wheel_stop=wheel,
                           driver_observations=driver_groups([item for item in events if at <= item['receive_s'] <= until
                                                            and (next_goal is None or item['receive_s'] < next_goal)], max_age_s)))
    return dict(schema_version=1, time_basis='bag_receive_s', bag_start_s=begin, bag_end_s=end,
                topic_counts=dict(Counter(topic for topic, message, at in rows)),
                thresholds=dict(zero_epsilon=zero_epsilon, sample_max_age_s=max_age_s,
                                maximum_continuous_sample_gap_s=max_age_s, continuous_duration_s=continuous_s,
                                wheel_linear_m_s=.03, wheel_angular_rad_s=.05,
                                driver_rx_zero='exact linear.x=angular.z=0',
                                driver_motion_zero='all four kind-matched PWM/wheel-speed references or both 0x16 body references exactly zero',
                                driver_pwm_zero='all four reported or transmitted values exactly zero'),
                telemetry_pwm_semantics='0x90 pwm_timer_ticks_abcd is firmware duty[] software reference in timer-tick units; not an actual CCR readback, measured bridge output or current.',
                interpretation=['Header stamps and host monotonic clocks are retained separately; receive-time differences are not physical execution latency.',
                                'accepted_command_seq means MCU acceptance, never actuator execution or ground-stop acknowledgement.',
                                'Successful 0x10 references use mm/s; 0x11 uses permille; 0x16 body vx uses mm/s and wz uses mrad/s. Acceptance does not verify actual closed-loop response or physical stopping.',
                                '0x93 body feedback uses m/s and rad/s. Its MCU ticks and IMU health are reported independently of 0x90; a fresh ACK pair does not establish valid IMU response or physical stopping.',
                                'A final zero alone cannot establish timely stopping; missing, stale or interrupted observations remain unknown.',
                                'Event sequence gaps include missing evidence, not a diagnosis of where loss occurred; unobserved prefixes may predate recording.',
                                'Only /odom has goal-frame residuals: target minus observed pose in the matching goal frame, with yaw wrapped to +/-pi; these are odom residuals, not external accuracy.',
                                'Relative poses use each topic own initial frame and heading; provisional wheel geometry is not ground truth.'],
                goals=output, driver_instances=driver_groups(events, max_age_s))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='existing local ROS1 bag; opened read-only')
    args = parser.parse_args()
    import rosbag
    with rosbag.Bag(args.bag, 'r') as bag:
        result = analyze((topic, message, time.to_sec()) for topic, message, time in bag.read_messages())
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))


if __name__ == '__main__':
    main()
