"""Read-only ROS contact inference logger. Never starts or commands controllers.

Run on a ROS host with an already-running joint-state publisher:
  python -m implicit_compliance.shadow --model EXPORT_DIR --out NEW_DIR --seconds 60

Only main() imports ROS. ShadowProcessor can be replayed/tested without ROS,
hardware, model training, or any motor publisher/service.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import time

import numpy as np

from .data import CHANNELS, sha256
from .streaming import ContactPredictor, motor_channels


def json_safe(value):
    """Preserve invalid numerical evidence as null, never as a fabricated zero."""
    if isinstance(value, dict): return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)): return [json_safe(v) for v in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(value): return None
    if isinstance(value, np.ndarray): return json_safe(value.tolist())
    return value


class IntervalStats:
    def __init__(self):
        self.count=0;self.total=0.;self.minimum=float('inf');self.maximum=-float('inf')
        self.recent=deque(maxlen=10000)

    def add(self, value):
        if value is not None and math.isfinite(value):
            self.count+=1;self.total+=value
            self.minimum=min(self.minimum,value);self.maximum=max(self.maximum,value)
            self.recent.append(value)

    def report(self):
        if not self.count:return {'count':0}
        return dict(count=self.count,mean=self.total/self.count,minimum=self.minimum,
                    maximum=self.maximum,recent_median=float(np.median(self.recent)),
                    recent_p95=float(np.percentile(self.recent,95)),
                    quantile_scope=f'last {len(self.recent)} observations (bounded memory)')


class ShadowProcessor:
    """Inspect timestamps and named motor channels, then optionally infer.

    Header age assumes the publisher and this ROS node use the same ROS clock.
    It measures publication/receipt age, not unprovided hardware sensing latency.
    """
    def __init__(self, predictor, *, max_header_age_s=.25, max_future_skew_s=.05):
        if not all(math.isfinite(t) for t in (max_header_age_s,max_future_skew_s)) or max_header_age_s<=0 or max_future_skew_s<0:
            raise ValueError('Invalid header age bounds')
        self.predictor=predictor
        self.max_header_age_s=max_header_age_s;self.max_future_skew_s=max_future_skew_s
        self.count=0;self.outcomes=Counter();self.first_receipt=None;self.last_receipt=None
        self.last_header=None;self.first_header=None
        self.receipt_intervals=IntervalStats();self.header_intervals=IntervalStats()
        self.header_ages=IntervalStats();self.processing_ms=IntervalStats();self.inference_ms=IntervalStats()
        self.channel_count=0;self.channel_sum=np.zeros(9);self.channel_sumsq=np.zeros(9)
        self.channel_min=np.full(9,np.inf);self.channel_max=np.full(9,-np.inf)

    def process(self, *, names, positions, velocities, efforts, header_sec, header_nanosec,
                receipt_monotonic_s, receipt_ros_s):
        begin=time.perf_counter()
        record=dict(sequence=self.count,receipt_monotonic_s=receipt_monotonic_s,
                    receipt_ros_s=receipt_ros_s,header_sec=header_sec,header_nanosec=header_nanosec,
                    joint_names=list(names),position=list(positions),velocity=list(velocities),
                    published_effort=list(efforts),channels=None,prediction=None,
                    input_valid=False,window_valid=False,prediction_available=False)
        self.count+=1
        delta=None if self.last_receipt is None else receipt_monotonic_s-self.last_receipt
        record['receipt_interval_s']=delta;self.receipt_intervals.add(delta)
        if self.first_receipt is None:self.first_receipt=receipt_monotonic_s
        self.last_receipt=receipt_monotonic_s
        status=None
        try:
            header=float(header_sec)+float(header_nanosec)*1e-9
            if not all(math.isfinite(t) for t in (header,receipt_monotonic_s,receipt_ros_s)):
                raise ValueError('nonfinite clock value')
            if not 0<=header_nanosec<1000000000 or header<=0:
                raise ValueError('header stamp missing or malformed')
            age=receipt_ros_s-header
            header_delta=None if self.last_header is None else header-self.last_header
            record.update(header_s=header,header_age_s=age,header_interval_s=header_delta)
            self.header_intervals.add(header_delta);self.header_ages.add(age)
            if self.first_header is None:self.first_header=header
            self.last_header=header
            if delta is not None and delta<=0: status='receipt_clock_not_increasing'
            elif header_delta is not None and header_delta<=0:
                status='duplicate_timestamp' if header_delta==0 else 'time_rewind'
            elif age>self.max_header_age_s:status='stale_header'
            elif age < -self.max_future_skew_s:status='future_header'
        except (TypeError, ValueError, OverflowError) as exc:
            status='invalid_timestamp';record['detail']=str(exc)
        try:
            channels=motor_channels(names,positions,velocities,efforts)
            if not np.isfinite(channels).all():raise ValueError('Nonfinite motor channel')
            record['channels']=channels.tolist()
            self.channel_count+=1;self.channel_sum+=channels;self.channel_sumsq+=channels.astype(float)**2
            self.channel_min=np.minimum(self.channel_min,channels);self.channel_max=np.maximum(self.channel_max,channels)
        except (TypeError, ValueError, IndexError, OverflowError) as exc:
            status=status or 'invalid_channels';record['channel_error']=str(exc)
        if status:
            self.predictor.invalidate(status)
        else:
            record['input_valid']=True
            try:
                prediction=self.predictor.push(header,channels)
                status=self.predictor.last_status
                record['window_valid']=self.predictor.window.last_status=='ready'
                if prediction is not None:
                    record['prediction']=prediction;record['prediction_available']=True
                    self.inference_ms.add(float(prediction['inference_ms']))
            except (ValueError, RuntimeError, TypeError) as exc:
                status='inference_error';record['detail']=str(exc)
                self.predictor.invalidate(status)
        record['status']=status;self.outcomes[status]+=1
        elapsed=(time.perf_counter()-begin)*1000
        record['processing_latency_ms']=elapsed;self.processing_ms.add(elapsed)
        return json_safe(record)

    def summary(self):
        duration=0 if self.first_receipt is None else self.last_receipt-self.first_receipt
        mean=self.channel_sum/max(1,self.channel_count)
        std=np.sqrt(np.maximum(self.channel_sumsq/max(1,self.channel_count)-mean**2,0))
        return json_safe(dict(messages=self.count,observed_receipt_span_s=duration,
            received_message_rate_hz=(self.count-1)/duration if duration>0 else None,
            model_resampling_hz=self.predictor.window.sample_hz,
            interpolation_caveat='Resampling cannot create independent sensor measurements; message rate also does not prove each motor sample is new.',
            outcome_counts=dict(self.outcomes),receipt_intervals_s=self.receipt_intervals.report(),
            header_intervals_s=self.header_intervals.report(),header_age_s=self.header_ages.report(),
            processing_latency_ms=self.processing_ms.report(),inference_latency_ms=self.inference_ms.report(),
            max_header_age_s=self.max_header_age_s,max_future_skew_s=self.max_future_skew_s,
            channel_statistics=dict(order=CHANNELS,finite_messages=self.channel_count,
                                    mean=mean.tolist() if self.channel_count else None,
                                    std=std.tolist() if self.channel_count else None,
                                    minimum=self.channel_min.tolist() if self.channel_count else None,
                                    maximum=self.channel_max.tolist() if self.channel_count else None),
            scope='Read-only source/stream calibration evidence. No ground-truth contacts or hardware torque measurements are supplied by this logger.'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=60.)
    parser.add_argument('--topic',default='/joint_states')
    parser.add_argument('--inference-hz',type=float,default=20.)
    parser.add_argument('--max-header-age',type=float,default=.25)
    parser.add_argument('--max-future-skew',type=float,default=.05)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0:parser.error('--seconds must be finite and positive')
    predictor=ContactPredictor(args.model,inference_hz=args.inference_hz)
    processor=ShadowProcessor(predictor,max_header_age_s=args.max_header_age,max_future_skew_s=args.max_future_skew)
    # No ROS imports occur in the pure processor or when displaying --help.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    args.out.mkdir(parents=True,exist_ok=False)
    manifest=dict(started_utc=datetime.now(timezone.utc).isoformat(),status='recording',
                  model_directory=str(args.model.resolve()),model_sha256=sha256(args.model/'contact.pt'),
                  metadata_sha256=sha256(args.model/'metadata.json'),metadata=predictor.metadata,
                  topic=args.topic,requested_seconds=args.seconds,inference_hz=args.inference_hz,
                  python=platform.python_version(),source_sha256={p.name:sha256(p) for p in (Path(__file__),Path(__file__).with_name('streaming.py'))},
                  interface='subscription only; no publishers, services, controller launch, or motor commands',
                  effort_semantics='Published values are preserved. Current local class driver uses a PD estimate; installed driver and historical dataset provenance require verification.',
                  timestamp_semantics='Header age assumes a shared ROS clock; header timestamps are not asserted to be hardware acquisition timestamps.')
    manifest_path=args.out/'manifest.json'
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    node=None
    status='complete'
    try:
        rclpy.init()
        node=Node('pupper_contact_shadow')
        with (args.out/'samples.jsonl').open('x') as log:
            def callback(msg):
                record=processor.process(names=msg.name,positions=msg.position,velocities=msg.velocity,
                    efforts=msg.effort,header_sec=msg.header.stamp.sec,header_nanosec=msg.header.stamp.nanosec,
                    receipt_monotonic_s=time.monotonic(),receipt_ros_s=node.get_clock().now().nanoseconds*1e-9)
                log.write(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n')
                if processor.count%1000==0:log.flush()
            subscription=node.create_subscription(JointState,args.topic,callback,qos_profile_sensor_data)
            deadline=time.monotonic()+args.seconds
            while rclpy.ok() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=min(.1,max(0,deadline-time.monotonic())))
            if not rclpy.ok():status='ros_shutdown'
            elif not processor.count:status='complete_no_messages'
            del subscription
    except KeyboardInterrupt:
        status='interrupted'
    except BaseException:
        status='failed'
        raise
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report=processor.summary();report['status']=status
        (args.out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        manifest.update(status=status,finished_utc=datetime.now(timezone.utc).isoformat(),summary='summary.json')
        manifest_path.write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
        print(f'Read-only shadow capture: {args.out} ({processor.count} messages; {status})')


if __name__=='__main__':main()
