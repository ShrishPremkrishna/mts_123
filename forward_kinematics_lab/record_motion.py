"""Record existing joint/foot topics without publishing commands; export one leg.

record: stores raw asynchronous samples, including invalid samples, for inspection.
export: requires fresh named joints, consistent FK, and bounded reachable segments.
The course foot topics have no header: receipt alignment is approximate, not a
claim of synchronized acquisition. Positions are already in base_link metres.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

LEGS = ('front_r', 'front_l', 'back_r', 'back_l')


def finite(values):
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in values)


def clean_nonfinite(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,list):return [clean_nonfinite(v) for v in value]
    if isinstance(value,dict):return {k:clean_nonfinite(v) for k,v in value.items()}
    return value


def import_mcap(source, destination):
    """Decode the class topics offline; never play the bag or initialize ROS."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    source=Path(source)
    topics={'/joint_states':None,**{f'/leg_{leg}_end_effector_position':leg for leg in LEGS}}
    counts={topic:0 for topic in topics};origin=None
    with source.open('rb') as bag,Path(destination).open('x') as stream:
        def save(row):stream.write(json.dumps(clean_nonfinite(row),allow_nan=False)+'\n')
        save(dict(schema='pupper_topic_recording_v1',frame='base_link',units='metres',
                  source_mcap_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  note='Offline CDR decoding of class foot/joint topics. Foot frame follows the class publisher; export independently checks FK. MCAP log timestamps approximate receipt, not acquisition.'))
        reader=make_reader(bag,validate_crcs=True,decoder_factories=[DecoderFactory()])
        for schema,channel,message,msg in reader.iter_decoded_messages(topics=list(topics),log_time_order=True):
            leg=topics[channel.topic]
            expected='sensor_msgs/msg/JointState' if leg is None else 'std_msgs/msg/Float64MultiArray'
            if schema.name!=expected:raise ValueError(f'Unexpected schema on {channel.topic}: {schema.name}')
            if origin is None:origin=message.log_time
            row=dict(receipt_s=(message.log_time-origin)*1e-9)
            if leg is None:
                row.update(kind='joints',header_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                           names=list(msg.name),positions=list(msg.position),velocities=list(msg.velocity))
            else:row.update(kind='foot',leg=leg,position=list(msg.data))
            save(row);counts[channel.topic]+=1
    if not counts['/joint_states'] or not sum(counts[t] for t in topics if t!='/joint_states'):
        raise ValueError('Bag lacks required foot/joint messages; inspect the preserved partial import')
    return counts


def export_motion(source, destination, leg, max_gap=.1, max_skew=.05, start_s=None, end_s=None):
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ik_heuristic_walking_lab'))
    from kinematics import LEG_FK
    from recorded_motion import RecordedMotion
    if leg not in LEGS:raise ValueError('Unknown leg')
    if not finite([max_gap,max_skew]) or min(max_gap,max_skew)<=0:raise ValueError('Invalid alignment limits')
    if any(v is not None and (not finite([v]) or v<0) for v in (start_s,end_s)):
        raise ValueError('Invalid explicit recording window')
    if start_s is not None and end_s is not None and end_s<=start_s:raise ValueError('Empty recording window')
    rows = [json.loads(line) for line in Path(source).read_text().splitlines()]
    if not rows or rows[0].get('schema') != 'pupper_topic_recording_v1':
        raise ValueError('Unknown recording schema')
    if (rows[0].get('frame'),rows[0].get('units'))!=('base_link','metres'):
        raise ValueError('Recording must declare body-frame foot coordinates in metres')
    names = [f'leg_{leg}_{i}' for i in (1,2,3)]
    joints = [r for r in rows[1:] if r.get('kind') == 'joints']
    feet = [r for r in rows[1:] if r.get('kind') == 'foot' and r.get('leg') == leg]
    # Never silently discard unaligned or invalid samples inside the selected interval.
    original_count=len(feet)
    if start_s is not None:feet=[r for r in feet if r['receipt_s']>=start_s]
    if end_s is not None:feet=[r for r in feet if r['receipt_s']<=end_s]
    if len(feet)<2 or not joints:raise ValueError('Need at least two foot samples and joint feedback')
    times=np.asarray([r['receipt_s'] for r in feet],float)
    jt=np.asarray([r['receipt_s'] for r in joints],float)
    headers=np.asarray([r.get('header_s',float('nan')) for r in joints],float)
    if not np.isfinite(headers).all() or (headers<=0).any() or (np.diff(headers)<=0).any():
        raise ValueError('Joint acquisition stamps are absent or not advancing')
    if not np.isfinite(times).all() or not np.isfinite(jt).all() or (np.diff(times)<=0).any() or (np.diff(jt)<=0).any():
        raise ValueError('Receipt timestamps must be finite and strictly increasing per channel')
    if np.max(abs((headers-headers[0])-(jt-jt[0])))>2*max_skew:
        raise ValueError('Joint acquisition and receipt clock progress disagree')
    positions=[];angles=[];skews=[];matched=[]
    fk=LEG_FK[LEGS.index(leg)]
    joint_angles=[]
    for joint in joints:
        labels=joint['names'];values=joint['positions']
        if len(labels)!=len(values) or len(labels)!=len(set(labels)) or any(name not in labels for name in names):
            raise ValueError('Missing or duplicate joint names or mismatched joint array')
        q=[values[labels.index(name)] for name in names]
        if not finite(q):raise ValueError('Invalid recorded joints')
        joint_angles.append(q)
    joint_angles=np.asarray(joint_angles);joint_feet=np.asarray([fk(q) for q in joint_angles])
    for foot in feet:
        p=foot['position']
        if len(p)!=3 or not finite(p):raise ValueError('Invalid recorded coordinates')
        skew=abs(jt-foot['receipt_s']);candidates=np.flatnonzero(skew<=max_skew)
        if not len(candidates):raise ValueError('Missing fresh joint sample; cannot align foot sample')
        errors=np.linalg.norm(joint_feet[candidates]-p,axis=1)
        if errors.min()>.002:raise ValueError('Foot sample disagrees with fresh named-joint FK by more than2mm')
        # The foot is computed from one earlier joint message, but the topics
        # arrive asynchronously. Match its FK within the bounded time window;
        # use nearest receipt only to break geometrically indistinguishable ties.
        tied=candidates[errors<=errors.min()+1e-12]
        nearest=int(tied[np.argmin(skew[tied])])
        if matched and nearest<matched[-1]:raise ValueError('Aligned joint acquisition order runs backward')
        positions.append(p);angles.append(joint_angles[nearest].tolist());skews.append(float(skew[nearest]));matched.append(nearest)
    payload=dict(schema='pupper_recorded_foot_v1',leg=leg,frame='base_link',units='metres',
                 times_s=(times-times[0]).tolist(),positions=positions,joint_positions=angles,
                 source_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),
                 timing=dict(method='minimum FK error within receipt window; nearest receipt tie-break; foot topic is unstamped',
                             allowed_skew_s=max_skew,maximum_skew_s=max(skews),maximum_gap_s=max_gap,
                             matched_joint_receipt_s=jt[matched].tolist(),matched_joint_header_s=headers[matched].tolist()),
                 source_window=dict(requested_start_s=start_s,requested_end_s=end_s,
                                    first_receipt_s=float(times[0]),last_receipt_s=float(times[-1]),
                                    selected_foot_samples=len(feet),excluded_foot_samples=original_count-len(feet)),
                 hardware_validated=False)
    RecordedMotion(payload, max_gap=max_gap)  # Validate the full interpolated path before writing.
    with Path(destination).open('x') as stream:json.dump(payload,stream,indent=2,allow_nan=False)
    return payload


def record(destination, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    start=time.monotonic()
    with Path(destination).open('x') as stream:
        def save(row):
            # Preserve invalid numeric samples explicitly; export refuses them.
            stream.write(json.dumps(clean_nonfinite(row),allow_nan=False)+'\n');stream.flush()
        save(dict(schema='pupper_topic_recording_v1',frame='base_link',units='metres',
                  note='Subscription only. Foot topics are unstamped; receipt time is not acquisition time.'))
        rclpy.init();node=Node('pupper_motion_recorder',enable_rosout=False)
        subscriptions=[]
        try:
            subscriptions.append(node.create_subscription(JointState,'/joint_states',lambda msg:save(dict(
                kind='joints',receipt_s=time.monotonic()-start,
                header_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                names=list(msg.name),positions=list(msg.position),velocities=list(msg.velocity))),qos_profile_sensor_data))
            for leg in LEGS:
                subscriptions.append(node.create_subscription(Float64MultiArray,f'/leg_{leg}_end_effector_position',
                    lambda msg,leg=leg:save(dict(kind='foot',leg=leg,receipt_s=time.monotonic()-start,position=list(msg.data))),qos_profile_sensor_data))
            while time.monotonic()-start<seconds:rclpy.spin_once(node,timeout_sec=.1)
        except KeyboardInterrupt:pass
        finally:node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='mode',required=True)
    rec=sub.add_parser('record');rec.add_argument('--out',type=Path,required=True);rec.add_argument('--seconds',type=float,default=30)
    bag=sub.add_parser('import-mcap');bag.add_argument('source',type=Path);bag.add_argument('--out',type=Path,required=True)
    exp=sub.add_parser('export');exp.add_argument('source',type=Path);exp.add_argument('--out',type=Path,required=True);exp.add_argument('--leg',choices=LEGS,default='front_r')
    exp.add_argument('--start',type=float,help='Explicit inclusive start in recording receipt seconds')
    exp.add_argument('--end',type=float,help='Explicit inclusive end in recording receipt seconds')
    args=parser.parse_args()
    if args.mode=='record':
        if not math.isfinite(args.seconds) or not 0<args.seconds<=600:parser.error('seconds must be within(0,600]')
        record(args.out,args.seconds)
    elif args.mode=='import-mcap':print(json.dumps(import_mcap(args.source,args.out),indent=2))
    else:export_motion(args.source,args.out,args.leg,start_s=args.start,end_s=args.end)


if __name__=='__main__':main()
