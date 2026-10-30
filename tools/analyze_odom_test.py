#!/usr/bin/env python3
"""Read-only odom test analysis. Bag timestamps and header clocks stay distinct.

Only ROS1 bag reading needs ROS; analyze() accepts pure offline record fixtures.
Coordinates are compared through each stream's own start-frame displacement,
never by subtracting unrelated absolute origins. No robot command is published.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics
import sys

from analyze_motion_bag import diagnostic_events, get, number, stamp

POSE_TOPICS=('/odom','/lio/odom','/wheel/odom','/Odometry')
COMMAND_TOPICS=('/cmd_vel/nav_raw','/cmd_vel/nav','/cmd_vel')
TOPIC_KEYS={'odom':'/odom','lio':'/lio/odom','wheel':'/wheel/odom','lio_raw':'/Odometry',
            'scan':'/scan','tf':'/tf','tf_static':'/tf_static','driver_diagnostics':'/slicer_c30d_driver/diagnostics',
            'diagnostics':'/diagnostics','raw':'/cmd_vel/nav_raw','nav':'/cmd_vel/nav','cmd':'/cmd_vel'}
SELECTED_TOPICS=tuple(TOPIC_KEYS.values())
MANUAL_FIELDS=('real_displacement_m','real_forward_m','real_lateral_m','real_stop_distance_m')


def unknown(reason):
    return {'state':'unknown','reason':reason}


def stats(values):
    values=[float(v) for v in values if number(v)]
    return ({'count':len(values),'min':min(values),'max':max(values),
             'mean':statistics.mean(values),'rms':math.sqrt(statistics.mean(v*v for v in values))}
            if values else unknown('no finite samples'))


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def timing(rows,future_tolerance=.05):
    times=[t for t,_ in rows]
    gaps=[b-a for a,b in zip(times,times[1:])]
    stamps=[(t,stamp(get(m,'header.stamp'))) for t,m in rows]
    known=[(t,h) for t,h in stamps if number(h)]
    deltas=[b[1]-a[1] for a,b in zip(known,known[1:])]
    positive=[d for d in deltas if d>0]
    return {'sample_count':len(rows),'record_start_s':min(times) if times else None,
            'record_end_s':max(times) if times else None,
            'bag_record_rate_hz':(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None,
            'record_max_gap_s':max(gaps,default=None),
            'record_nonadvancing_count':sum(d<=0 for d in gaps),
            'header':{'sample_count':len(known),'missing_count':len(rows)-len(known),
                      'zero_count':sum(h==0 for _,h in known),
                      'advancing_rate_hz':1/statistics.mean(positive) if positive else None,
                      'duplicate_count':sum(d==0 for d in deltas),'backward_count':sum(d<0 for d in deltas),
                      'max_advancing_gap_s':max(positive,default=None),
                      'future_vs_record_count':sum(h-t>future_tolerance for t,h in known),
                      'min_record_minus_header_s':min((t-h for t,h in known),default=None),
                      'max_record_minus_header_s':max((t-h for t,h in known),default=None)},
            'clock_note':'header minus bag-record offsets are clock/data-age observations, not physical latency'}


def pose(message,at):
    p=get(message,'pose.pose');x=get(p,'position.x');y=get(p,'position.y')
    q=[get(p,'orientation.'+k) for k in ('x','y','z','w')]
    if not number(x) or not number(y) or not all(number(v) for v in q) or sum(v*v for v in q)<1e-12:
        return None
    norm=math.sqrt(sum(v*v for v in q));qx,qy,qz,qw=[v/norm for v in q]
    frame=get(message,'header.frame_id','');child=get(message,'child_frame_id','')
    return {'bag_record_s':at,'header_stamp_s':stamp(get(message,'header.stamp')),
            'frame_id':frame,'child_frame_id':child,'x_m':x,'y_m':y,
            'yaw_rad':math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))}


def parse_poses(rows):
    return [(t,pose(m,t)) for t,m in sorted(rows,key=lambda item:item[0])]


def at_time(rows,at,max_gap):
    before=[p for t,p in rows if t<=at and p is not None]
    after=[p for t,p in rows if t>=at and p is not None]
    if not before:return None
    a=before[-1]
    if at-a['bag_record_s']>max_gap:return None
    if a['bag_record_s']==at or not after:return a
    b=after[0];dt=b['bag_record_s']-a['bag_record_s']
    if dt<=0 or dt>max_gap or (a['frame_id'],a['child_frame_id'])!=(b['frame_id'],b['child_frame_id']):return None
    fraction=(at-a['bag_record_s'])/dt
    sample=dict(a,bag_record_s=at,x_m=a['x_m']+fraction*(b['x_m']-a['x_m']),
                y_m=a['y_m']+fraction*(b['y_m']-a['y_m']),
                yaw_rad=wrap(a['yaw_rad']+fraction*wrap(b['yaw_rad']-a['yaw_rad'])),
                support_start_s=a['bag_record_s'],support_end_s=b['bag_record_s'])
    if number(a['header_stamp_s']) and number(b['header_stamp_s']):
        sample['header_stamp_s']=a['header_stamp_s']+fraction*(b['header_stamp_s']-a['header_stamp_s'])
    return sample


def interval_displacement(rows,start,end,max_gap,reset_jump,future_tolerance):
    if start is None or end is None or end<=start:return unknown('need a nonempty recorded interval')
    a,b=at_time(rows,start,max_gap),at_time(rows,end,max_gap)
    if a is None or b is None:return unknown('start/end pose missing or stale in bag-record time')
    support_start=a.get('support_start_s',a['bag_record_s']);support_end=b.get('support_end_s',b['bag_record_s'])
    selected=[p for t,p in rows if support_start<=t<=support_end]
    if len(selected)<2 or any(p is None for p in selected):return unknown('invalid or insufficient pose samples')
    frames={(p['frame_id'].lstrip('/'),p['child_frame_id'].lstrip('/')) for p in selected}
    if len(frames)!=1 or not a['frame_id']:return unknown('pose frame/child changed or is unspecified')
    for p in selected:
        h=p['header_stamp_s']
        if not number(h) or p['bag_record_s']-h>max_gap or h-p['bag_record_s']>future_tolerance:
            return unknown('header clock/data age not consistent with a fresh bag-record sample')
    for left,right in zip(selected,selected[1:]):
        if right['bag_record_s']-left['bag_record_s']>max_gap:return unknown('pose interval contains a receive gap')
        if right['header_stamp_s']<=left['header_stamp_s']:return unknown('header stamp did not advance')
        if math.hypot(right['x_m']-left['x_m'],right['y_m']-left['y_m'])>reset_jump:
            return unknown('suspected pose reset/jump; not treated as physical movement')
    path_points=[a]+[p for p in selected if a['bag_record_s']<p['bag_record_s']<b['bag_record_s']]+[b]
    dx,dy=b['x_m']-a['x_m'],b['y_m']-a['y_m'];yaw=a['yaw_rad']
    return {'state':'estimated','source':'odometry_pose_increment',
            'requested_start_s':start,'requested_end_s':end,
            'sample_start_s':a['bag_record_s'],'sample_end_s':b['bag_record_s'],
            'start_support_s':[a.get('support_start_s',a['bag_record_s']),a.get('support_end_s',a['bag_record_s'])],
            'end_support_s':[b.get('support_start_s',b['bag_record_s']),b.get('support_end_s',b['bag_record_s'])],
            'endpoint_interpolation':'linear in bag-record time when bracketed; shortest yaw arc; no extrapolated motion',
            'frame_id':a['frame_id'],'child_frame_id':a['child_frame_id'],
            'forward_m':dx*math.cos(yaw)+dy*math.sin(yaw),
            'left_m':-dx*math.sin(yaw)+dy*math.cos(yaw),
            'displacement_m':math.hypot(dx,dy),'yaw_rad':wrap(b['yaw_rad']-yaw),
            'yaw_unwrapped_delta_rad':sum(wrap(r['yaw_rad']-l['yaw_rad']) for l,r in zip(path_points,path_points[1:])),
            'path_length_m':sum(math.hypot(r['x_m']-l['x_m'],r['y_m']-l['y_m']) for l,r in zip(path_points,path_points[1:]))}


def pose_speeds(points,max_gap,reset_jump):
    header,record=[],[]
    for a,b in zip(points,points[1:]):
        d=math.hypot(b['x_m']-a['x_m'],b['y_m']-a['y_m'])
        dt=b['bag_record_s']-a['bag_record_s']
        if (a['frame_id'],a['child_frame_id'])!=(b['frame_id'],b['child_frame_id']) or d>reset_jump or not 0<dt<=max_gap:continue
        record.append(d/dt)
        ha,hb=a['header_stamp_s'],b['header_stamp_s']
        if number(ha) and number(hb) and 0<hb-ha<=max_gap:header.append(d/(hb-ha))
    return {'by_header_time_m_s':stats(header),'by_record_time_m_s':stats(record),
            'note':'pose-increment estimates; neither is external measured speed'}


def manual_measurements(data):
    data=data or {}
    if not isinstance(data,dict):raise ValueError('measurements must be a JSON object')
    result={}
    for key in MANUAL_FIELDS:
        value=data.get(key)
        if value is None:result[key]=unknown('not supplied by the operator');continue
        if not number(value) or (key in ('real_displacement_m','real_stop_distance_m') and value<0):
            raise ValueError(key+' must be a finite value; distances cannot be negative')
        result[key]={'state':'supplied','source':'manual_input','value':value,'unit':'m',
                     'reference':data.get('reference','operator reference point not specified')}
    for key,lo,hi in [('window','start_s','end_s'),('stop','command_zero_s','settled_s')]:
        val=data.get(key,{})
        if not isinstance(val,dict):raise ValueError(key+' must be an object')
        for field in (lo,hi):
            if val.get(field) is not None and not number(val[field]):raise ValueError(key+'.'+field+' must be finite')
        if val.get(lo) is not None and val.get(hi) is not None and val[hi]<=val[lo]:raise ValueError(key+' end must exceed start')
    return result


def command_zero(rows):
    parsed=[]
    for at,message in rows:
        vx,wz=get(message,'linear.x'),get(message,'angular.z')
        if not number(vx) or not number(wz):continue
        parsed.append((at,abs(vx)>1e-6 or abs(wz)>1e-6))
    nonzero=[t for t,moving in parsed if moving]
    if not nonzero:return unknown('no preceding nonzero command observed')
    final_nonzero=max(nonzero)
    zero=min((t for t,moving in parsed if t>final_nonzero and not moving),default=None)
    return ({'state':'observed','bag_record_s':zero,'source':'final_Twist_first_zero_after_last_nonzero'}
            if zero is not None else unknown('no observed final-command transition to zero'))


def mcu_instances(telemetry):
    grouped=defaultdict(list)
    for row in telemetry:grouped[row.get('driver_instance_id') or 'unspecified'].append(row)
    output=[]
    for instance,rows in grouped.items():
        ticks=[r['mcu_tick_us'] for r in rows if type(r.get('mcu_tick_us')) is int and 0<=r['mcu_tick_us']<2**32]
        intervals=[(b-a)%(2**32) for a,b in zip(ticks,ticks[1:])]
        reset_count=sum(not 0<dt<2**31 for dt in intervals)
        arrays={key:[[r[key][wheel] for r in rows if isinstance(r.get(key),(list,tuple)) and len(r[key])==4 and all(number(v) for v in r[key])] for wheel in range(4)] for key in ('measured_mm_s_abcd','pwm_timer_ticks_abcd')}
        output.append({'driver_instance_id':instance,'telemetry_count':len(rows),
            'mcu_tick_advance_count':sum(0<dt<2**31 for dt in intervals),
            'mcu_tick_wrap_count':sum(b<a and 0<(b-a)%2**32<2**31 for a,b in zip(ticks,ticks[1:])),
            'mcu_tick_nonadvancing_or_reset_count':reset_count,
            'measured_mm_s_abcd':[stats(values) for values in arrays['measured_mm_s_abcd']],
            'software_duty_ticks_abcd':[stats(values) for values in arrays['pwm_timer_ticks_abcd']],
            'flags_observed':sorted({r['flags'] for r in rows if type(r.get('flags')) is int}),
            'voltage_v':stats(r.get('voltage_v') for r in rows)})
    return output


def analyze(records,measurements=None,max_gap=.25,reset_jump_m=1.,future_tolerance=.05):
    if (not number(max_gap) or max_gap<=0 or not number(reset_jump_m) or reset_jump_m<=0
            or not number(future_tolerance) or future_tolerance<0):
        raise ValueError('max-gap and suspected-jump threshold must be positive')
    manual=manual_measurements(measurements);measurements=measurements or {};topics=defaultdict(list)
    for topic,message,at in records:
        if number(at):topics[topic].append((float(at),message))
    all_times=[t for rows in topics.values() for t,_ in rows]
    begin,end=(min(all_times),max(all_times)) if all_times else (None,None)
    report={'time_basis':'bag_record_s','record_start_s':begin,'record_end_s':end,
            'interpretation':['Bag record time, ROS header time, host monotonic time and MCU ticks are separate clocks.',
                              'Independent absolute origins are never subtracted. Increment disagreement is not ground truth.',
                              'Reported velocity, encoder motion and software duty do not prove physical stopping.'],
            'thresholds':{'max_gap_s':max_gap,'suspected_jump_m':reset_jump_m,'future_header_tolerance_s':future_tolerance},
            'topics':{topic:timing(rows,future_tolerance) for topic,rows in topics.items()},
            'missing_topics':[topic for topic in SELECTED_TOPICS if not topics.get(topic)],
            'manual_measurements':manual,'odometry':{},'increment_comparisons':{},'manual_comparisons':{},'tf':{'dynamic':{},'static':{}}}
    parsed={topic:parse_poses(topics[topic]) for topic in POSE_TOPICS}
    window=measurements.get('window',{})
    def delta(topic,start,finish):return interval_displacement(parsed[topic],start,finish,max_gap,reset_jump_m,future_tolerance)
    for topic in POSE_TOPICS:
        rows=topics[topic];points=parsed[topic];valid=[p for _,p in points if p is not None]
        start=window.get('start_s') if window.get('start_s') is not None else (points[0][0] if points else None)
        finish=window.get('end_s') if window.get('end_s') is not None else (points[-1][0] if points else None)
        velocity_rows=[(t,m) for t,m in rows if start is not None and finish is not None and start<=t<=finish]
        velocity_poses=[p for t,p in points if start is not None and finish is not None and start<=t<=finish and p is not None]
        jumps=sum(math.hypot(b['x_m']-a['x_m'],b['y_m']-a['y_m'])>reset_jump_m for a,b in zip(valid,valid[1:]))
        report['odometry'][topic]={'sample_count':len(rows),'invalid_pose_count':len(rows)-len(valid),
            'suspected_pose_jump_count':jumps,'displacement':delta(topic,start,finish),
            'analysis_window_s':[start,finish],'pose_derived_speed':pose_speeds(velocity_poses,max_gap,reset_jump_m),
            'reported_velocity':{'linear_x_m_s':stats(get(m,'twist.twist.linear.x') for _,m in velocity_rows),
                                 'angular_z_rad_s':stats(get(m,'twist.twist.angular.z') for _,m in velocity_rows)}}
        d=report['odometry'][topic]['displacement'];comparison={}
        for measured,estimated,label in [('real_displacement_m','displacement_m','displacement_minus_manual_m'),
                                         ('real_forward_m','forward_m','forward_minus_manual_m'),
                                         ('real_lateral_m','left_m','lateral_minus_manual_m')]:
            if manual[measured]['state']=='supplied' and d.get('state')=='estimated':comparison[label]=d[estimated]-manual[measured]['value']
        report['manual_comparisons'][topic]=dict(comparison,state='estimated_difference',note='odometry estimate minus operator measurement; no pass/fail implied') if comparison else unknown('need matching manual measurement and valid pose interval')
    for source in ('/lio/odom','/wheel/odom','/Odometry'):
        key=source+' -> /odom';a,b=parsed[source],parsed['/odom']
        if not a or not b:report['increment_comparisons'][key]=unknown('one or both streams absent');continue
        start=window.get('start_s') if window.get('start_s') is not None else max(a[0][0],b[0][0])
        finish=window.get('end_s') if window.get('end_s') is not None else min(a[-1][0],b[-1][0])
        da,db=delta(source,start,finish),delta('/odom',start,finish)
        if da.get('state')!=db.get('state') or da.get('state')!='estimated':
            report['increment_comparisons'][key]=dict(unknown('fresh common-window increments unavailable'),source_increment=da,ekf_increment=db);continue
        report['increment_comparisons'][key]={'state':'estimated','source_increment':da,'ekf_increment':db,
            'forward_difference_m':da['forward_m']-db['forward_m'],'left_difference_m':da['left_m']-db['left_m'],
            'yaw_difference_rad':wrap(da['yaw_rad']-db['yaw_rad']),
            'endpoint_receive_skew_s':max(abs(da['sample_start_s']-db['sample_start_s']),abs(da['sample_end_s']-db['sample_end_s'])),
            'physical_localization_error':unknown('no ground truth/extrinsic alignment; these are independently start-frame increments')}
    for topic,label in [('/tf','dynamic'),('/tf_static','static')]:
        edges=defaultdict(list)
        for at,m in topics[topic]:
            for t in get(m,'transforms',[]):edges[str(get(t,'header.frame_id',''))+' -> '+str(get(t,'child_frame_id',''))].append((at,t))
        report['tf'][label]={edge:timing(rows,future_tolerance) for edge,rows in edges.items()}
    events=[]
    for topic in ('/diagnostics','/slicer_c30d_driver/diagnostics'):events.extend(diagnostic_events(topics[topic]))
    events.sort(key=lambda e:e['receive_s'])
    telemetry=[e for e in events if e.get('event')=='telemetry'];tx=[e for e in events if e.get('event')=='serial_tx']
    report['mcu']={'diagnostic_events':len(events),'telemetry_events':len(telemetry),
        'events_by_name':dict(Counter(str(e.get('event')) for e in events)),
        'instances':mcu_instances(telemetry),
        'observed_tx_kinds':sorted({e['kind'] for e in tx if type(e.get('kind')) is int}),
        'voltage_v':stats(e.get('voltage_v') for e in telemetry),
        'last_telemetry':telemetry[-1] if telemetry else unknown('MCU telemetry diagnostics unavailable'),
        'semantics':'ACK is command acceptance; pwm_timer_ticks is software duty, not actual bridge output/current/physical stop.'}
    stop=measurements.get('stop',{});zero=command_zero(topics['/cmd_vel'])
    if stop.get('command_zero_s') is not None:zero={'state':'supplied','bag_record_s':stop['command_zero_s'],'source':'manual_bag_record_timestamp'}
    finish=stop.get('settled_s') if stop.get('settled_s') is not None else end
    report['stopping']={'command_zero':zero,'end_s':finish,
        'end_source':'operator_supplied_bag_record_timestamp' if stop.get('settled_s') is not None else 'record_end_not_physical_settled_time',
        'physical_stop_confirmed':unknown('operator/video observation required; zero feedback alone is insufficient'),
        'estimated_after_zero':{topic:delta(topic,zero['bag_record_s'],finish) if zero.get('bag_record_s') is not None else unknown('no zero-command reference') for topic in POSE_TOPICS},
        'manual_stop_distance':manual['real_stop_distance_m']}
    return report


def resolve_bag(path):
    path=Path(path)
    if path.is_file():return path
    if not path.is_dir():raise ValueError('input must be a bag file or capture directory')
    preferred=path/'motion.bag'
    if preferred.is_file():return preferred
    bags=list(path.glob('*.bag'))
    if len(bags)!=1:raise ValueError('capture directory must contain one completed .bag; specify a file if multiple')
    return bags[0]


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',help='existing local .bag or capture directory; opened read-only')
    p.add_argument('--measurements',type=Path,help='optional operator measurements JSON; absence stays unknown')
    p.add_argument('--real-displacement-m',type=float,help='manual net distance of the same vehicle reference point')
    p.add_argument('--real-stop-distance-m',type=float,help='manual distance from chosen stop reference to physical rest')
    p.add_argument('--max-gap',type=float,default=.25,help='maximum endpoint age and internal receive gap, seconds')
    p.add_argument('--suspected-jump-m',type=float,default=1.,help='pose-step warning threshold, not an automatic hardware diagnosis')
    p.add_argument('--output',type=Path,help='default report.json beside input bag')
    args=p.parse_args(argv)
    try:
        bag_path=resolve_bag(args.capture)
        measurements=json.loads(args.measurements.read_text()) if args.measurements else {}
        manual_measurements(measurements)
        if args.real_displacement_m is not None:measurements['real_displacement_m']=args.real_displacement_m
        if args.real_stop_distance_m is not None:measurements['real_stop_distance_m']=args.real_stop_distance_m
        manual_measurements(measurements)
        names={topic:topic for topic in SELECTED_TOPICS}
        manifest=bag_path.parent/'manifest.json'
        if manifest.is_file():
            mapping=json.loads(manifest.read_text()).get('topic_map',{})
            if not isinstance(mapping,dict):raise ValueError('manifest topic_map must be an object')
            for key,canonical in TOPIC_KEYS.items():
                if isinstance(mapping.get(key),str):names[canonical]=mapping[key]
        actual_to_canonical={actual:canonical for canonical,actual in names.items()}
        import rosbag
        with rosbag.Bag(str(bag_path),'r') as bag:
            report=analyze(((actual_to_canonical[topic],message,t.to_sec()) for topic,message,t in bag.read_messages(topics=list(actual_to_canonical))),measurements,max_gap=args.max_gap,reset_jump_m=args.suspected_jump_m)
        report['input_bag']=str(bag_path);report['topic_names']=names;output=args.output or bag_path.parent/'report.json'
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print('Report: '+str(output))
        for topic in POSE_TOPICS:
            d=report['odometry'][topic]['displacement']
            print(topic+(': %.3fm forward, %.3fm left (odometry estimate)' % (d['forward_m'],d['left_m']) if d.get('state')=='estimated' else ': unknown — '+d['reason']))
        print('Physical stopping remains operator/video evidence; manual values are labelled separately.')
        return 0
    except (OSError,ValueError,ImportError) as exc:
        p.error(str(exc))


if __name__=='__main__':main()
