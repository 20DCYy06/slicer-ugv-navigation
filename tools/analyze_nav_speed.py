#!/usr/bin/env python3
"""Offline receive-time speed budget for explicitly supplied current navigation bags.

No ROS master or robot connection is used. MCU targets/encoders/gyro are sensor
observations, not independent ground truth or proof of actuator execution.
"""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

from analyze_motion_bag import diagnostic_events, get

CHAIN = ('/cmd_vel/nav_raw', '/cmd_vel/nav', '/cmd_vel')
DIAG = '/slicer_c30d_driver/diagnostics'
REASON = '/slicer_nav_arc_guard/hold_reason'
ADJUST = '/slicer_nav_arc_guard/adjustment'
AGE = .2


def stats(values):
    values = sorted(values)
    if not values:
        return dict(n=0)
    return dict(n=len(values), min=values[0], max=values[-1],
                median=statistics.median(values), mean=statistics.mean(values),
                p95=values[min(len(values)-1, math.ceil(.95*len(values))-1)],
                rms=math.sqrt(statistics.mean(x*x for x in values)))


def index(rows):
    times = [r[0] for r in rows]
    def latest(at, age=AGE):
        i = bisect_right(times, at)-1
        return rows[i][1] if i >= 0 and at-times[i] <= age else None
    return latest


def segments(rows, start, end, age=AGE):
    """Zero-order hold, clipped to Action and freshness; absent intervals unknown."""
    latest = index(rows)
    cuts = sorted({start, end} | {t for t, _ in rows if start < t < end}
                  | {t+age for t, _ in rows if start < t+age < end})
    return [(a, b, latest((a+b)/2, age)) for a, b in zip(cuts, cuts[1:])]


def mode(value, v_epsilon, w_epsilon):
    if value is None:
        return 'unknown'
    v, w = value
    return ('translation' if abs(v) > v_epsilon else
            'pure_turn_request' if abs(w) > w_epsilon else 'full_zero_wait_request')


def weighted_percentile(rows, fraction):
    rows = sorted(rows)
    total = sum(weight for _, weight in rows)
    accumulated = 0.
    for value, weight in rows:
        accumulated += weight
        if accumulated >= fraction*total:
            return value
    return None


def velocity(rows, start, end, max_v, max_w):
    scoped = [(t, x) for t, x in rows if start <= t < end]
    held = segments(rows, start, end)
    exact, practical = Counter(), Counter()
    cap_v = cap_w = active_v = active_w = integral_v = integral_w = 0.
    exact_runs = []
    for a, b, value in held:
        dt = b-a
        exact[mode(value, 1e-6, 1e-6)] += dt
        practical[mode(value, .005, .01)] += dt
        if value is None:
            continue
        v, w = value
        integral_v += v*dt
        integral_w += abs(w)*dt
        active_v += dt*(abs(v) > .005)
        active_w += dt*(abs(w) > .01)
        cap_v += dt*(abs(v) >= .95*max_v)
        cap_w += dt*(abs(w) >= .95*max_w)
        if mode(value, 1e-6, 1e-6) == 'full_zero_wait_request':
            if exact_runs and abs(a-exact_runs[-1]['end_s']) < 1e-6:
                exact_runs[-1]['end_s'] = b
                exact_runs[-1]['duration_s'] += dt
            else:
                exact_runs.append(dict(begin_s=a, end_s=b, duration_s=dt))
    return dict(sample_count=len(scoped),
                max_receive_gap_s=max((b[0]-a[0] for a, b in zip(scoped, scoped[1:])), default=None),
                linear_m_s=stats([x[0] for _, x in scoped]),
                abs_angular_rad_s=stats([abs(x[1]) for _, x in scoped]),
                abs_linear_when_translating_m_s=stats([abs(x[0]) for _, x in scoped if abs(x[0]) > .005]),
                abs_angular_when_turning_rad_s=stats([abs(x[1]) for _, x in scoped if abs(x[1]) > .01]),
                time_weighted_abs_linear_translation_median_m_s=weighted_percentile(
                    [(abs(x[0]), b-a) for a, b, x in held if x is not None and abs(x[0]) > .005], .5),
                exact_command_budget_s=dict(exact), practical_command_budget_s=dict(practical),
                exact_full_zero_runs=exact_runs,
                near_max_linear_s=cap_v, near_max_linear_action_fraction=cap_v/(end-start),
                near_max_linear_translation_fraction=cap_v/active_v if active_v else None,
                near_max_angular_s=cap_w, near_max_angular_action_fraction=cap_w/(end-start),
                near_max_angular_turning_fraction=cap_w/active_w if active_w else None,
                held_integral_v_m=integral_v, held_integral_abs_w_rad=integral_w,
                command_distance_equivalent_at_max_v_s=integral_v/max_v,
                practical_translation_time_minus_command_distance_at_max_v_s=practical['translation']-integral_v/max_v)


def text_budget(rows, start, end):
    # Reasons/adjustments are latched change events, not periodically fresh data.
    total = Counter()
    for a, b, text in segments(rows, start, end, math.inf):
        total['<unknown>' if text is None else text] += b-a
    return dict(total)


def chain_comparison(upstream, downstream, reasons, start, end):
    up, down, reason = index(upstream), index(downstream), index(reasons)
    counters = Counter()
    dv, dw = [], []
    cuts = sorted({start, end} | {t for t, _ in upstream+downstream+reasons if start < t < end}
                  | {t+AGE for t, _ in upstream+downstream if start < t+AGE < end})
    for a, b in zip(cuts, cuts[1:]):
        before, value = up((a+b)/2), down((a+b)/2)
        if before is None or value is None:
            counters['unknown_s'] += b-a
            continue
        dt = b-a
        counters['paired_s'] += dt
        v, w = value
        dv.append(v-before[0])
        dw.append(abs(w)-abs(before[1]))
        counters['vx_reduced_more_than_005_s'] += dt*(before[0]-v > .005)
        counters['abs_wz_reduced_more_than_01_s'] += dt*(abs(before[1])-abs(w) > .01)
        if mode(before, 1e-6, 1e-6) != 'full_zero_wait_request' and mode(value, 1e-6, 1e-6) == 'full_zero_wait_request':
            counters['upstream_nonzero_downstream_exact_zero_s'] += dt
            counters['upstream_nonzero_downstream_exact_zero_reason:'+str(reason(a, math.inf))] += dt
    return dict(duration_s=dict(counters), downstream_minus_latest_upstream_v_m_s=stats(dv),
                downstream_minus_latest_upstream_abs_w_rad_s=stats(dw),
                caveat='Receive-time alignment; control publication/transport lag and Guard smoothing contribute to instantaneous differences.')


def error_summary(rows, target, measured):
    errors = [r[measured]-r[target] for r in rows]
    return dict(error=stats(errors), absolute_error=stats([abs(x) for x in errors]),
                target=stats([r[target] for r in rows]), measured=stats([r[measured] for r in rows]))


def tracking(events, start, end):
    body = [r for r in events if r['event'] == 'body_telemetry' and start <= r['receive_s'] < end]
    for r in body:
        r['gyro_z_rad_s'] = r['gyro_rad_s'][2]
    tx = [r for r in events if r['event'] == 'serial_tx' and r.get('kind') == 22 and r.get('success')]
    sent = defaultdict(list)
    for r in tx:
        sent[(r.get('driver_instance_id'), r.get('command_seq'))].append(r)
    accepted, mismatches, latency = 0, [], []
    for r in body:
        choices = [t for t in sent[(r.get('driver_instance_id'), r.get('ack_seq'))] if t['receive_s'] <= r['receive_s']]
        if len(choices) != 1:
            continue
        t = choices[0]
        accepted += 1
        latency.append(r['receive_s']-t['receive_s'])
        if abs(r['target_vx_m_s']-t['body_vx_mm_s']/1000) > 1e-9 or abs(r['target_wz_rad_s']-t['body_wz_mrad_s']/1000) > 1e-9:
            mismatches.append(dict(receive_s=r['receive_s'], ack_seq=r['ack_seq']))
    groups = {'all': body,
              'translation_target_gt005': [r for r in body if abs(r['target_vx_m_s']) > .005],
              'pure_turn_target': [r for r in body if abs(r['target_vx_m_s']) <= .005 and abs(r['target_wz_rad_s']) > .01],
              'near_max_vx_target': [r for r in body if abs(r['target_vx_m_s']) >= .95*.125]}
    steady_v, steady_w = [], []
    for r in body:
        previous = [p for p in body if r['receive_s']-.4 <= p['receive_s'] <= r['receive_s']]
        if not previous or r['receive_s']-previous[0]['receive_s'] < .35:
            continue
        if max(p['target_vx_m_s'] for p in previous)-min(p['target_vx_m_s'] for p in previous) <= .002 and abs(r['target_vx_m_s']) > .005:
            steady_v.append(r)
        if max(p['target_wz_rad_s'] for p in previous)-min(p['target_wz_rad_s'] for p in previous) <= .005 and abs(r['target_wz_rad_s']) > .01:
            steady_w.append(r)
    summaries = {key: dict(vx=error_summary(rows, 'target_vx_m_s', 'vx_m_s'),
                           wz=error_summary(rows, 'target_wz_rad_s', 'wz_rad_s'),
                           gyro_z=error_summary(rows, 'target_wz_rad_s', 'gyro_z_rad_s')) for key, rows in groups.items()}
    summaries['steady_vx_target_04s'] = error_summary(steady_v, 'target_vx_m_s', 'vx_m_s')
    summaries['steady_wz_target_04s'] = error_summary(steady_w, 'target_wz_rad_s', 'wz_rad_s')
    turn_bins = {}
    for label, low, high in [('01_to_075', .01, .075), ('075_to_125', .075, .125),
                             ('125_to_2', .125, .2), ('2_and_above', .2, math.inf)]:
        samples = [r for r in groups['pure_turn_target'] if low < abs(r['target_wz_rad_s']) <= high]
        turn_bins[label] = dict(samples=len(samples),
                               target_abs_wz_rad_s=stats([abs(r['target_wz_rad_s']) for r in samples]),
                               gyro_abs_z_rad_s=stats([abs(r['gyro_z_rad_s']) for r in samples]),
                               gyro_quiet_sample_fraction=sum(abs(r['gyro_z_rad_s']) <= .01 for r in samples)/len(samples) if samples else None)
    quiet_turn_runs = []
    turn_requested_s = turn_gyro_quiet_s = 0.
    for i, r in enumerate(body):
        until = min(end, body[i+1]['receive_s'] if i+1 < len(body) else end, r['receive_s']+AGE)
        if abs(r['target_vx_m_s']) <= .005 and abs(r['target_wz_rad_s']) > .01:
            turn_requested_s += until-r['receive_s']
            turn_gyro_quiet_s += (until-r['receive_s'])*(abs(r['gyro_z_rad_s']) <= .01)
        if abs(r['target_vx_m_s']) <= .005 and abs(r['target_wz_rad_s']) > .01 and abs(r['gyro_z_rad_s']) <= .01:
            if quiet_turn_runs and abs(quiet_turn_runs[-1]['end_s']-r['receive_s']) < 1e-6:
                run = quiet_turn_runs[-1]
                run['end_s'] = until
            else:
                run = dict(begin_s=r['receive_s'], end_s=until, target_wz=[], gyro_z=[], wheel_abs_max=[])
                quiet_turn_runs.append(run)
            run['target_wz'].append(r['target_wz_rad_s'])
            run['gyro_z'].append(r['gyro_z_rad_s'])
            run['wheel_abs_max'].append(max(abs(x) for x in r['wheel_mm_s']))
    quiet_turn_runs = [dict(begin_s=r['begin_s'], end_s=r['end_s'], duration_s=r['end_s']-r['begin_s'],
                            target_wz_rad_s=stats(r['target_wz']), gyro_z_rad_s=stats(r['gyro_z']),
                            wheel_abs_max_mm_s=stats(r['wheel_abs_max'])) for r in quiet_turn_runs if r['end_s']-r['begin_s'] >= .3]
    telemetry = [r for r in events if r['event'] == 'telemetry' and start <= r['receive_s'] < end]
    return dict(body_samples=len(body), ack_matched_samples=accepted, ack_target_mismatch_count=len(mismatches),
                ack_target_mismatch_examples=mismatches[:3], serial_tx_to_ack_receive_s=stats(latency),
                body_imu_flags=dict(Counter(r.get('imu_flags') for r in body)),
                body_status_flags=dict(Counter(r.get('status_flags') for r in body)),
                gyro_z_rad_s=stats([r['gyro_rad_s'][2] for r in body]),
                body_wz_minus_gyro_z_rad_s=stats([r['wz_rad_s']-r['gyro_rad_s'][2] for r in body]),
                command_stop_reasons=dict(Counter(r.get('stop_reason') for r in events if r['event'] == 'command_tx' and start <= r['receive_s'] < end)),
                tracking=summaries, pure_turn_target_bins=turn_bins,
                pure_turn_target_duration_s=turn_requested_s,
                pure_turn_target_gyro_quiet_duration_s=turn_gyro_quiet_s,
                pure_turn_target_gyro_quiet_runs_ge03s=quiet_turn_runs,
                wheels={letter: dict(measured_speed_mm_s=stats([r['measured_mm_s_abcd'][i] for r in telemetry]),
                                     body_wheel_mm_s=stats([r['wheel_mm_s'][i] for r in body]),
                                     software_duty_timer_ticks=stats([r['pwm_timer_ticks_abcd'][i] for r in telemetry]))
                        for i, letter in enumerate('ABCD')})


def analyze(path):
    import rosbag
    rows = defaultdict(list)
    selected = list(CHAIN)+[DIAG, REASON, ADJUST, '/cup_nav/manual_state', '/ugv/move_base/goal', '/ugv/move_base/result']
    with rosbag.Bag(str(path), 'r') as bag:
        for topic, message, timestamp in bag.read_messages(topics=selected):
            rows[topic].append((timestamp.to_sec(), message))
    events = diagnostic_events(rows[DIAG])
    manifest = json.loads(path.with_name('manifest.json').read_text())
    before = manifest['before']['parameters']
    after = manifest['after']['parameters']
    dwa = before['/ugv/move_base']['DWAPlannerROS']
    vlimit, wlimit = dwa['max_vel_x'], dwa['max_vel_theta']
    chain = {topic: [(t, (m.linear.x, m.angular.z)) for t, m in rows[topic]] for topic in CHAIN}
    reasons = [(t, m.data) for t, m in rows[REASON]]
    result = dict(bag=str(path), input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  observed_publishers={topic: manifest['observed_publishers'][topic] for topic in CHAIN},
                  configuration=dict(max_vel_x_m_s=vlimit, max_vel_theta_rad_s=wlimit,
                                     Qwz=before['/slicer_ekf']['process_noise_covariance'][176],
                                     dwa_before_after_equal=dwa == after['/ugv/move_base']['DWAPlannerROS'],
                                     xy_goal_tolerance_m=dwa['xy_goal_tolerance'], yaw_goal_tolerance_rad=dwa['yaw_goal_tolerance'],
                                     driver_control_mode=manifest['driver_reported_control_mode']), actions=[])
    for start, goal in rows['/ugv/move_base/goal']:
        matches = [(t, m) for t, m in rows['/ugv/move_base/result'] if m.status.goal_id.id == goal.goal_id.id and t >= start]
        if not matches:
            continue
        end, terminal = matches[0]
        body = [r for r in events if r['event'] == 'body_telemetry']
        action = dict(goal_id=goal.goal_id.id, begin_s=start, terminal_s=end, duration_s=end-start,
                      result_status=terminal.status.status,
                      command_chain={topic: velocity(chain[topic], start, end, vlimit, wlimit) for topic in CHAIN},
                      mcu_target=velocity([(r['receive_s'], (r['target_vx_m_s'], r['target_wz_rad_s'])) for r in body], start, end, vlimit, wlimit),
                      mcu_feedback=velocity([(r['receive_s'], (r['vx_m_s'], r['wz_rad_s'])) for r in body], start, end, vlimit, wlimit),
                      mcu_gyro_feedback=velocity([(r['receive_s'], (r['vx_m_s'], r['gyro_rad_s'][2])) for r in body], start, end, vlimit, wlimit),
                      guard_reason_duration_s=text_budget(reasons, start, end),
                      guard_adjustment_duration_s=text_budget([(t, m.data) for t, m in rows[ADJUST]], start, end),
                      raw_to_guard=chain_comparison(chain[CHAIN[0]], chain[CHAIN[1]], reasons, start, end),
                      guard_to_mux=chain_comparison(chain[CHAIN[1]], chain[CHAIN[2]], reasons, start, end),
                      driver=tracking(events, start, end))
        manual = [json.loads(m.data) for t, m in rows['/cup_nav/manual_state'] if start <= t < end]
        action['manual_states'] = dict(samples=len(manual), source_counts=dict(Counter(m.get('selected_source') for m in manual)),
                                       manual_hold_true=sum(bool(m.get('manual_hold')) for m in manual),
                                       emergency_true=sum(bool(m.get('emergency_stop_latched')) for m in manual))
        result['actions'].append(action)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = dict(method=dict(time='bag receive time; [Action goal, matching terminal result)',
                              integration='Zero-order hold with .2s max age. Latched Guard reason/adjustment changes have no age timeout.',
                              exact_command_bins='translation |vx|>1e-6 m/s; else pure_turn_request |wz|>1e-6 rad/s; else full_zero_wait_request. These describe commands, not physical motion.',
                              practical_bins='translation |vx|>.005 m/s; else pure_turn_request |wz|>.01 rad/s; else full_zero_wait_request (near-zero bin). MCU feedback practical bins describe recorded sensors.',
                              near_max='>=95% recorded DWA max; fractions time weighted, active denominators use practical thresholds.',
                              tracking='MCU target versus feedback in same body frame; steady means target range <=.002m/s or .005rad/s for trailing .4s (>=.35s covered).',
                              ack='Match same driver instance and exact 0x16 serial sequence; acceptance is not actuator-execution proof.',
                              limitations='Two current field runs; wheel-derived vx and onboard gyro are not independent ground truth. Raising caps or changing Q is not evaluated.'),
                  trials=[analyze(path) for path in args.bags])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(output=str(args.output), actions=sum(len(t['actions']) for t in output['trials']))))


if __name__ == '__main__':
    main()
