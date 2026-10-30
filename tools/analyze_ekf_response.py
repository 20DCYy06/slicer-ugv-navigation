#!/usr/bin/env python3
"""Analyze an explicit field bag and offline EKF replay; no ROS master or devices.

LIO is a shared-input reference, never independent pose ground truth. Sensor quiet
means recorded MCU gyro/body vx and wheel odometry vx satisfy the stated limits.
"""
import argparse
from bisect import bisect_right
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_motion_bag import diagnostic_events


def stats(values):
    v=sorted(float(x) for x in values if math.isfinite(float(x)))
    return dict(n=len(v),mean=statistics.mean(v) if v else None,
                rms=math.sqrt(statistics.mean(x*x for x in v)) if v else None,
                p95_abs=sorted(abs(x) for x in v)[int(.95*(len(v)-1))] if v else None,
                max_abs=max(map(abs,v),default=None))


def angle(v):
    return math.atan2(math.sin(v),math.cos(v))


class Series:
    def __init__(self, rows, max_gap=.15):
        self.input_count=len(rows)
        by_time={float(row[0]):list(map(float,row)) for row in rows}
        self.rows=sorted(by_time.values())
        self.times=[r[0] for r in self.rows]
        self.time_set=set(self.times)
        self.max_gap=max_gap
        previous=None
        unwrapped=None
        for row in self.rows:
            wrapped=row[3]
            unwrapped=wrapped if previous is None else unwrapped+angle(wrapped-previous)
            row[3]=unwrapped
            previous=wrapped

    def at(self, at):
        i=bisect_right(self.times,at)
        if i and self.times[i-1]==at:
            return self.rows[i-1]
        if not i or i==len(self.rows):
            return None
        a,b=self.rows[i-1],self.rows[i]
        if b[0]-a[0]>self.max_gap:
            return None
        q=(at-a[0])/(b[0]-a[0])
        return [at]+[x+(y-x)*q for x,y in zip(a[1:],b[1:])]

    def window(self,begin,end):
        return [r for r in self.rows if begin<=r[0]<=end]


def dwell(rows,begin,end,predicate,duration,max_gap):
    selected=[r for r in rows if begin<=r[0]<=end]
    first=None
    previous=None
    first_confirmed=None
    start_confirmed=None
    gaps=[]
    for row in selected:
        if previous is not None:
            gaps.append(row[0]-previous)
        if not predicate(row) or (previous is not None and row[0]-previous>max_gap):
            first=None
        if predicate(row) and first is None:
            first=row[0]
        if first is not None and row[0]-first>=duration and first_confirmed is None:
            first_confirmed=row[0]
            start_confirmed=first
        previous=row[0]
    missing=not selected or selected[0][0]-begin>max_gap or end-selected[-1][0]>max_gap or any(g>max_gap for g in gaps)
    return dict(state='observed' if first_confirmed is not None else ('missing_data' if missing else 'threshold_not_reached'),
                first_under_s=start_confirmed,confirmed_s=first_confirmed,
                delay_s=start_confirmed-begin if start_confirmed is not None else None,
                dwell_s=duration,samples=len(selected),missing_data=missing,
                max_gap_s=max(gaps,default=None))


def quiet_metrics(series,begin,end,increment):
    rows=series.window(begin,end)
    if len(rows)<3:
        return dict(state='missing_data',samples=len(rows))
    times=[r[0]-rows[0][0] for r in rows]
    mx,my=statistics.mean(times),statistics.mean(r[3] for r in rows)
    den=sum((t-mx)**2 for t in times)
    slope=sum((t-mx)*(r[3]-my) for t,r in zip(times,rows))/den if den else 0.
    yaw_noise=[r[3]-(my+slope*(t-mx)) for t,r in zip(times,rows)]
    increments=[series.at(r[0]+increment)[3]-r[3] for r in rows
                if r[0]+increment<=end and series.at(r[0]+increment) is not None]
    return dict(state='observed',samples=len(rows),begin=begin,end=end,
                yaw_detrended_jitter_rad=stats(yaw_noise),yaw_linear_drift_rad_s=slope,
                yaw_change_over_interval_rad=stats(increments),interval_s=increment,
                vx_m_s=stats(r[4] for r in rows),wz_rad_s=stats(r[6] for r in rows))


def sensor_quiet_evidence(rows,begin,end,threshold,max_gap):
    selected=[r for r in rows if begin<=r[0]<=end]
    gaps=[b[0]-a[0] for a,b in zip(selected,selected[1:])]
    valid=sum(bool(r[4]) for r in selected)
    quiet=sum(bool(r[4]) and max(abs(v) for v in r[1:4])<=threshold for r in selected)
    missing=(not selected or selected[0][0]-begin>max_gap or end-selected[-1][0]>max_gap
             or any(g>max_gap for g in gaps))
    return dict(state='observed' if selected and valid==quiet==len(selected) and not missing else 'incomplete_or_not_quiet',
                samples=len(selected),feedback_valid_fraction=valid/len(selected) if selected else None,
                valid_quiet_fraction=quiet/len(selected) if selected else None,
                missing_data=missing,max_gap_s=max(gaps,default=None),
                max_abs_mcu_vx_m_s=max((abs(r[1]) for r in selected),default=None),
                max_abs_mcu_gyro_z_rad_s=max((abs(r[2]) for r in selected),default=None),
                max_abs_wheel_vx_m_s=max((abs(r[3]) for r in selected),default=None))


def yaw_jumps(series,begin,end,increment):
    rows=series.window(begin,end)
    adjacent=[b[3]-a[3] for a,b in zip(rows,rows[1:]) if 0<b[0]-a[0]<=series.max_gap]
    fixed=[series.at(r[0]+increment)[3]-r[3] for r in rows
           if r[0]+increment<=end and series.at(r[0]+increment) is not None]
    return dict(interval_s=increment,change_over_interval_rad=stats(fixed),
                adjacent_change_rad=stats(adjacent),
                excluded_large_gaps=sum(b[0]-a[0]>series.max_gap for a,b in zip(rows,rows[1:])),
                method='Unwrap yaw with wrapped successive differences, then interpolate only across covered gaps.')


def compare(a,b,begin,end):
    errors=defaultdict(list)
    largest=defaultdict(list)
    exact=0
    for r in a.window(begin,end):
        other=b.at(r[0])
        if other is None:
            continue
        exact+=r[0] in b.time_set
        position_error=math.hypot(r[1]-other[1],r[2]-other[2])
        errors['position_m'].append(position_error)
        largest['position_m'].append((position_error,r[0],position_error))
        for label,i in [('yaw_rad',3),('vx_m_s',4),('vy_m_s',5),('wz_rad_s',6),('P_yaw',7),('P_wz',8)]:
            value=angle(r[i]-other[i]) if i==3 else r[i]-other[i]
            errors[label].append(value)
            largest[label].append((abs(value),r[0],value))
    return dict(reference_samples=len(a.window(begin,end)),matched_samples=len(errors['position_m']),
                exact_stamp_matches=exact,errors={k:stats(v) for k,v in errors.items()},
                largest_errors={key:[dict(header_s=t,error=v) for magnitude,t,v in sorted(rows,reverse=True)[:3]]
                                for key,rows in largest.items()})


def lio_increments(series,lio,begin,end):
    pairs=[(r,lio.at(r[0])) for r in series.window(begin,end)]
    pairs=[(a,b) for a,b in pairs if b is not None]
    if not pairs:
        return dict(state='missing_data')
    a0,b0=pairs[0]
    errors=defaultdict(list)
    def delta(r,start):
        dx,dy=r[1]-start[1],r[2]-start[2]
        c,s=math.cos(start[3]),math.sin(start[3])
        return dx*c+dy*s,-dx*s+dy*c,r[3]-start[3]
    for a,b in pairs:
        da,db=delta(a,a0),delta(b,b0)
        errors['displacement_increment_m'].append(math.hypot(da[0]-db[0],da[1]-db[1]))
        errors['yaw_increment_rad'].append(angle(da[2]-db[2]))
    return dict(state='observed',matched_samples=len(pairs),anchor_header_s=a0[0],
                label='LIO reference; each series uses displacement and yaw increments relative to its own first synchronized pose. No independent pose ground truth.',
                errors={k:stats(v) for k,v in errors.items()})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('replay',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--dwell',type=float,default=.5)
    parser.add_argument('--velocity-threshold',type=float,default=.01)
    parser.add_argument('--max-gap',type=float,default=.15)
    parser.add_argument('--yaw-interval',type=float,default=.1)
    args=parser.parse_args()
    import rosbag
    topics=defaultdict(list)
    selected=['/lio/odom','/wheel/odom','/odom','/cmd_vel','/slicer_c30d_driver/diagnostics','/ugv/move_base/goal','/ugv/move_base/result']
    with rosbag.Bag(str(args.bag)) as bag:
        begin,end=bag.get_start_time(),bag.get_end_time()
        for topic,message,time in bag.read_messages(topics=selected):
            topics[topic].append((time.to_sec(),message))
    def odom_rows(topic):
        rows=[]
        for receive,m in topics[topic]:
            p,q,v=m.pose.pose.position,m.pose.pose.orientation,m.twist.twist
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            rows.append([m.header.stamp.to_sec(),p.x,p.y,yaw,v.linear.x,v.linear.y,v.angular.z,m.pose.covariance[35],m.twist.covariance[35]])
        return rows
    recorded=Series(odom_rows('/odom'),args.max_gap)
    lio=Series(odom_rows('/lio/odom'),args.max_gap)
    wheel=topics['/wheel/odom']
    wheel_times=[t for t,m in wheel]
    events=diagnostic_events(topics['/slicer_c30d_driver/diagnostics'])
    body=[r for r in events if r.get('event')=='body_telemetry']
    sensor=[]
    for r in body:
        i=bisect_right(wheel_times,r['receive_s'])
        fresh=i and r['receive_s']-wheel_times[i-1]<=args.max_gap
        valid=(fresh and r.get('imu_flags',0)&7==7 and not r.get('imu_flags',0)&8 and r.get('imu_age_ms',999)>-1 and r.get('imu_age_ms',999)<=100)
        sensor.append([r['receive_s'],r['vx_m_s'],r['gyro_rad_s'][2],wheel[i-1][1].twist.twist.linear.x if fresh else math.inf,valid])
    profiles=json.loads((args.replay/'effective-parameters.json').read_text())
    series={'recorded':recorded}
    series.update({name:Series(json.loads((args.replay/(name+'.json')).read_text()),args.max_gap) for name in profiles})
    goals={m.goal_id.id:t for t,m in topics['/ugv/move_base/goal']}
    successes=[(t,m) for t,m in topics['/ugv/move_base/result'] if m.status.status==3]
    output=dict(OFFLINE_ONLY=True,bag=str(args.bag),replay=str(args.replay),
                thresholds=dict(absolute_vx_m_s=args.velocity_threshold,absolute_wz_rad_s=args.velocity_threshold,dwell_s=args.dwell,max_sample_gap_s=args.max_gap),
                stationary_window='Fixed terminal+5s..terminal+12s for all profiles, bounded by next goal and bag end; residual motion and stationary noise are reported separately.',
                clocks='Command and MCU quiet anchors use bag receive time. Odom/replay curves use their unchanged header stamps. Sensor ages use receive time.',
                boundary='Sensor/software observations only. LIO is a shared-input reference, not physical stop or independent pose ground truth.',
                profiles={},actions=[])
    warmup=min(goals.values()) if goals else begin+20
    for name,s in series.items():
        output['profiles'][name]=dict(input_rows=s.input_count,unique_header_stamps=len(s.rows),
            reproduction_vs_recorded=compare(recorded,s,warmup,end),
            comparison_vs_baseline=compare(series['baseline'],s,warmup,end) if 'baseline' in series else None,
            parameters=profiles.get(name))
    csvrows=[]
    for number,(terminal,message) in enumerate(successes,1):
        start=goals[message.status.goal_id.id]
        next_goal=min((t for t in goals.values() if t>start),default=end+1)
        observation_end=min(terminal+12,next_goal-.2,end)
        commands=[(t,m) for t,m in topics['/cmd_vel'] if start<=t<=terminal]
        moving=[t for t,m in commands if m.linear.x!=0 or m.angular.z!=0]
        last=moving[-1] if moving else None
        zero=next((t for t,m in commands if last is not None and t>last and m.linear.x==0 and m.angular.z==0),None)
        action=dict(number=number,id=message.status.goal_id.id,goal_receive_s=start,success_receive_s=terminal,
                    last_nonzero_final_cmd_receive_s=last,permanent_final_zero_receive_s=zero,
                    observation_end_s=observation_end,
                    settled_sensor_quiet_evidence=sensor_quiet_evidence(sensor,terminal+5,observation_end,
                                                                      args.velocity_threshold,args.max_gap),profiles={})
        if zero is None:
            action['sensor_quiet']=dict(state='missing_data',reason='No nonzero followed by permanent zero final /cmd_vel in this Action')
            output['actions'].append(action)
            continue
        quiet=dwell(sensor,zero,observation_end,
                    lambda r:r[4] and max(abs(v) for v in r[1:4])<=args.velocity_threshold,args.dwell,args.max_gap)
        action['sensor_quiet']=quiet
        anchor=quiet['first_under_s']
        for name,s in series.items():
            metrics=dict(yaw_jumps=yaw_jumps(s,start,terminal,args.yaw_interval),
                         lio_reference_increments=lio_increments(s,lio,start,terminal),
                         settled_sensor_quiet=quiet_metrics(s,terminal+5,observation_end,args.yaw_interval))
            if anchor is not None:
                metrics['convergence_from_sensor_quiet']={label:dwell(s.rows,anchor,observation_end,predicate,args.dwell,args.max_gap)
                    for label,predicate in [('vx',lambda r:abs(r[4])<=args.velocity_threshold),
                                            ('wz',lambda r:abs(r[6])<=args.velocity_threshold),
                                            ('both',lambda r:max(abs(r[4]),abs(r[6]))<=args.velocity_threshold)]}
                metrics['early_sensor_quiet']=quiet_metrics(s,anchor,min(anchor+2,observation_end),args.yaw_interval)
                metrics['stop_yaw_drift_rad']=(s.at(min(terminal,observation_end))[3]-s.at(anchor)[3]) if s.at(anchor) and s.at(min(terminal,observation_end)) else None
                both=metrics['convergence_from_sensor_quiet']['both']
                csvrows.append(dict(action=number,profile=name,final_zero_s=zero,sensor_quiet_s=anchor,
                    quiet_after_final_zero_s=anchor-zero,convergence_both_s=both['delay_s'],
                    convergence_state=both['state'],success_after_sensor_quiet_s=terminal-anchor,
                    stop_yaw_drift_rad=metrics['stop_yaw_drift_rad'],
                    stationary_yaw_jitter_rms_rad=metrics['settled_sensor_quiet'].get('yaw_detrended_jitter_rad',{}).get('rms'),
                    stationary_wz_rms_rad_s=metrics['settled_sensor_quiet'].get('wz_rad_s',{}).get('rms'),
                    lio_increment_position_rms_m=metrics['lio_reference_increments'].get('errors',{}).get('displacement_increment_m',{}).get('rms'),
                    lio_increment_yaw_rms_rad=metrics['lio_reference_increments'].get('errors',{}).get('yaw_increment_rad',{}).get('rms')))
            action['profiles'][name]=metrics
        output['actions'].append(action)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(output,indent=2)+'\n')
    if csvrows:
        with (args.output/'actions.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(csvrows[0]))
            writer.writeheader();writer.writerows(csvrows)
    print(json.dumps(dict(summary=str(args.output/'summary.json'),csv=str(args.output/'actions.csv'),profiles=list(series),actions=len(output['actions'])),indent=2))


if __name__=='__main__':
    main()
