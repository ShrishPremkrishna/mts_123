"""Offline02-versus07 comparison; host-time matching, never a readiness decision."""
from pathlib import Path
import argparse,bisect,hashlib,json,math

def compare(rows):
    streams={2:[],7:[]}
    for packet in rows:
        if packet.get('parse_ok') is not True:continue
        for report in packet.get('reports',[]):
            if report.get('id') not in streams:continue
            values=report.get('gyro_sensor_rad_s',[])
            bias=report.get('estimated_bias_rad_s',[]) if report['id']==7 else [0.,0.,0.]
            if len(values)!=3 or len(bias)!=3 or not all(math.isfinite(x) for x in [*values,*bias]):continue
            streams[report['id']].append((packet['monotonic_end_s'],values,bias,report.get('accuracy')))
    for values in streams.values():values.sort(key=lambda x:x[0])
    times=[x[0] for x in streams[2]]
    results=[]
    for tolerance in [.005,.02]:
        residuals=[];ages=[]
        for time,velocity,bias,_ in streams[7]:
            insertion=bisect.bisect_left(times,time)
            possible=[i for i in [insertion-1,insertion] if 0<=i<len(times)]
            if not possible:continue
            i=min(possible,key=lambda n:abs(times[n]-time));age=times[i]-time
            if abs(age)>tolerance:continue
            residuals.append([velocity[k]-bias[k]-streams[2][i][1][k] for k in range(3)]);ages.append(age)
        n=len(residuals)
        results.append({'host_receipt_pair_tolerance_s':tolerance,'matched_07_reports':n,'unmatched_07_reports':len(streams[7])-n,'mean_uncal_minus_bias_minus_cal_rad_s':[sum(r[k] for r in residuals)/n for k in range(3)] if n else None,'rms_residual_rad_s':[math.sqrt(sum(r[k]**2 for r in residuals)/n) for k in range(3)] if n else None,'max_abs_host_receipt_offset_s':max(map(abs,ages)) if n else None})
    return {'report_counts':{str(k):len(v) for k,v in streams.items()},'streams_available':bool(streams[2] and streams[7]),'comparison_available':any(r['matched_07_reports'] for r in results),'comparisons':results,'limits':['02 is calibrated report output;07 supplies uncalibrated output and device-estimated bias. None is external physical truth.','Nearest host-receipt matching at5/20ms is an offline sensitivity check, not physical sensor-time synchronization; adjacent reports may be reused.','Residuals do not establish health, calibration accuracy or the cause of drift. No arbitrary zeroing or offset correction is applied.']}

def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('output',type=Path);a=p.parse_args();raw=a.capture/'packets.jsonl';data=raw.read_bytes();result=compare([json.loads(x) for x in data.splitlines()]);result.update({'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'input_sha256':hashlib.sha256(data).hexdigest()})
    with a.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
if __name__=='__main__':main()
