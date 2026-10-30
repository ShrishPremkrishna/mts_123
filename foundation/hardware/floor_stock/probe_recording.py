"""Small JSON evidence boundary for the supervised probe; no ROS imports.

Records are appended with os.write, without per-sensor flush/fsync. Explicit
checkpoints sync before the watchdog is armed and after physical cleanup only.
Completed unsynced lines survive process failure, but not necessarily power loss.
"""
from array import array
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

STREAMS=('joint_states','imu','status','controller_states','commands','policy_output','position_command','imu_latency','policy_inference_latency','physical_joy','joy_publishers')

def json_value(value):
    if value is None or type(value) in (str,bool,int):return value
    if type(value) is float:
        return value if math.isfinite(value) else {'nonfinite_float':str(value)}
    if isinstance(value,(list,tuple,array)):return [json_value(x) for x in value]
    if isinstance(value,dict) and all(isinstance(k,str) for k in value):
        return {k:json_value(v) for k,v in value.items()}
    raise TypeError('Unsupported evidence value: '+type(value).__name__)

def encoded(value,indent=None):
    return (json.dumps(json_value(value),allow_nan=False,indent=indent,separators=None if indent else (',',':'))+'\n').encode('utf-8')

def atomic_json(path,value):
    path=Path(path);payload=encoded(value,indent=2) # serialize before touching a file
    temporary=path.with_name(path.name+'.tmp-'+uuid.uuid4().hex)
    fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    try:
        with os.fdopen(fd,'wb') as stream:
            stream.write(payload);stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,path)
        directory=os.open(path.parent,os.O_RDONLY)
        try:os.fsync(directory)
        finally:os.close(directory)
    except BaseException:
        # Preserve any partial temp file for inspection; existing final untouched.
        raise

def joy_receipt(message,begin,end):
    return {'monotonic':end,'publish_begin_s':begin,'publish_end_s':end,
            'topic':'/joy','buttons':list(message.buttons),'synthetic':True}

class Recorder:
    def __init__(self,path,initial):
        self.path=Path(path);self.fd=os.open(self.path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        self.sequence=0;self.bytes=0;self.synced_sequence=0;self.closed=False;self.finalized=False;self.write_error=None
        self.raw={kind:[] for kind in STREAMS};self.max_append_s=0.
        self.append('lifecycle',{'event':'created','initial':initial});self.sync()
    def append(self,kind,row):
        started=time.monotonic()
        if self.write_error is not None:raise RuntimeError('journal has an incomplete write; refusing later appends')
        safe=json_value(row)
        if kind not in (*STREAMS,'lifecycle'):raise ValueError('unexpected evidence stream')
        if kind in self.raw and len(self.raw[kind])>=16384:raise RuntimeError('bounded telemetry capacity exceeded')
        payload=encoded({'sequence':self.sequence+1,'kind':kind,'row':safe})
        if len(payload)>65536 or self.bytes+len(payload)>16*1024*1024:raise RuntimeError('bounded evidence byte capacity exceeded')
        offset=0
        try:
            while offset<len(payload):
                count=os.write(self.fd,payload[offset:])
                if count<=0:raise OSError('incomplete evidence write')
                offset+=count
        except BaseException as error:
            self.write_error={'error':repr(error),'known_partial_bytes':offset}
            raise
        self.max_append_s=max(self.max_append_s,time.monotonic()-started)
        self.sequence+=1;self.bytes+=len(payload)
        if kind in self.raw:self.raw[kind].append(safe)
        return safe
    def sync(self):
        os.fsync(self.fd);self.synced_sequence=self.sequence
    def checkpoint(self,event,data):
        self.append('lifecycle',{'event':event,'data':data});self.sync()
    def close(self):
        if self.closed:return
        try:
            if self.write_error is None:
                self.append('lifecycle',{'event':'journal_finalized','stream_counts':{k:len(v) for k,v in self.raw.items()}})
                self.sync();self.finalized=True
            else:self.sync() # preserve known prefix/torn tail without appending behind it
        finally:os.close(self.fd);self.closed=True
    def receipt(self):
        return {'path':str(self.path),'records':self.sequence,'complete_record_bytes':self.bytes,'file_bytes':self.path.stat().st_size,
                'synced_through_sequence':self.synced_sequence,'finalized':self.finalized,
                'max_append_s':self.max_append_s,'append_timing_scope':'normalization+encoding+completed os.write loop; excludes fsync','write_error':self.write_error,'sha256':hashlib.sha256(self.path.read_bytes()).hexdigest(),
                'durability':'No per-sensor fsync. Pre-release and final checkpoints synced; power-loss tail may be absent.'}

def finish_report(report,data,recorder):
    """After cleanup only. Journal closure precedes any final-report formatting."""
    errors=[]
    if recorder is not None:
        try:recorder.close()
        except Exception as error:errors.append('journal finalization: '+repr(error))
        if not recorder.finalized:errors.append('evidence journal is incomplete; valid prefix only')
        try:data['evidence_journal']=recorder.receipt()
        except Exception as error:errors.append('journal receipt: '+repr(error))
    if errors:data['passed_software_sequence']=False;data['recording_errors']=errors
    try:
        atomic_json(report,data)
        return {'report_written':True,'report':str(report),'passed_software_sequence':bool(data.get('passed_software_sequence'))}
    except Exception as error:
        data['passed_software_sequence']=False
        failure={'report_written':False,'passed_software_sequence':False,'error':repr(error),
                 'report':str(report),'journal':None if recorder is None else str(recorder.path)}
        fallback=Path(report).with_suffix('.report-error.json')
        try:atomic_json(fallback,failure);failure['failure_receipt']=str(fallback)
        except Exception as second:failure['failure_receipt_error']=repr(second)
        return failure

def read_journal(path):
    """Read completed prefix; a torn last line is explicit, never a final verdict."""
    data=Path(path).read_bytes();lines=data.splitlines(keepends=True);records=[];partial=False
    for index,line in enumerate(lines):
        if not line.endswith(b'\n'):
            if index!=len(lines)-1:raise ValueError('incomplete interior journal line')
            partial=True;break
        record=json.loads(line)
        if record['sequence']!=len(records)+1:raise ValueError('journal sequence gap')
        records.append(record)
    return {'records':records,'incomplete_tail':partial,
            'finalized':bool(records and records[-1]['kind']=='lifecycle' and records[-1]['row'].get('event')=='journal_finalized')}
