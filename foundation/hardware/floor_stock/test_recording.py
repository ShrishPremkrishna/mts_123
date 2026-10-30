from array import array
import ast,importlib.util,json,os,subprocess,sys
from pathlib import Path
from types import SimpleNamespace
import pytest
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('recording',ROOT/'probe_recording.py');r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)

def test_actual_array_boundary_snapshot_and_full_report(tmp_path):
    buttons=array('i',[0]*13);buttons[9]=1
    with pytest.raises(TypeError):json.dumps({'buttons':buttons})
    rec=r.Recorder(tmp_path/'evidence.jsonl',{'session':'test'})
    rec.append('commands',r.joy_receipt(SimpleNamespace(buttons=buttons),1.,1.01))
    rec.append('joint_states',{'position':array('d',[.1]*12),'velocity':array('d',[0.]*12)})
    buttons[9]=0
    data={'passed_software_sequence':True,'raw_receipts':rec.raw,'parameters':{'gains':array('d',[7.5]*12)}}
    outcome=r.finish_report(tmp_path/'report.json',data,rec)
    loaded=json.loads((tmp_path/'report.json').read_text())
    assert outcome['report_written'] and loaded['raw_receipts']['commands'][0]['buttons'][9]==1
    assert loaded['parameters']['gains']==[7.5]*12
    assert r.read_journal(rec.path)['finalized'] and loaded['evidence_journal']['finalized']

def test_no_fsync_per_sensor_or_command_append(tmp_path,monkeypatch):
    calls=[];original=r.os.fsync
    def sync(fd):calls.append(fd);original(fd)
    monkeypatch.setattr(r.os,'fsync',sync)
    rec=r.Recorder(tmp_path/'events.jsonl',{})
    rec.checkpoint('pre_release',{})
    before=len(calls)
    for _ in range(100):rec.append('joint_states',{'position':array('d',[0.]*12)})
    rec.append('commands',{'buttons':array('i',[0]*13)})
    assert len(calls)==before
    rec.close();assert len(calls)==before+1

def test_nonfinite_data_labeled_without_invalid_json(tmp_path):
    rec=r.Recorder(tmp_path/'events.jsonl',{})
    rec.append('imu',{'bad':[float('nan'),float('inf'),float('-inf')]})
    data={'passed_software_sequence':False,'raw_receipts':rec.raw}
    r.finish_report(tmp_path/'report.json',data,rec)
    loaded=json.loads((tmp_path/'report.json').read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
    assert loaded['raw_receipts']['imu'][0]['bad']==[{'nonfinite_float':'nan'},{'nonfinite_float':'inf'},{'nonfinite_float':'-inf'}]

def test_report_format_failure_leaves_finalized_journal(tmp_path):
    rec=r.Recorder(tmp_path/'events.jsonl',{});rec.append('status',{'payload':'saved'})
    outcome=r.finish_report(tmp_path/'report.json',{'passed_software_sequence':True,'bad':object()},rec)
    assert not outcome['report_written'] and not outcome['passed_software_sequence']
    assert r.read_journal(rec.path)['finalized']
    assert json.loads((tmp_path/'report.report-error.json').read_text())['report_written'] is False

def test_final_replace_failure_never_truncates_previous_report(tmp_path,monkeypatch):
    report=tmp_path/'report.json';report.write_text('{"previous":true}\n')
    rec=r.Recorder(tmp_path/'events.jsonl',{});rec.append('commands',{'buttons':array('i',[0]*13)})
    monkeypatch.setattr(r.os,'replace',lambda *args:(_ for _ in ()).throw(OSError('injected replace failure')))
    outcome=r.finish_report(report,{'passed_software_sequence':True},rec)
    assert not outcome['report_written'] and report.read_text()=='{"previous":true}\n'
    assert r.read_journal(rec.path)['finalized']

def test_process_exit_preserves_completed_unsynced_lines(tmp_path):
    code="""import importlib.util,os,sys
from pathlib import Path
s=importlib.util.spec_from_file_location('rec',sys.argv[1]);r=importlib.util.module_from_spec(s);s.loader.exec_module(r)
f=r.Recorder(Path(sys.argv[2]),{});f.append('commands',{'buttons':[0]*13});os._exit(17)
"""
    result=subprocess.run([sys.executable,'-B','-c',code,str(ROOT/'probe_recording.py'),str(tmp_path/'partial.jsonl')])
    assert result.returncode==17
    data=r.read_journal(tmp_path/'partial.jsonl');assert len(data['records'])==2 and not data['finalized']

def test_torn_tail_is_explicit(tmp_path):
    rec=r.Recorder(tmp_path/'events.jsonl',{});rec.append('status',{'payload':'x'});os.close(rec.fd)
    with rec.path.open('ab') as stream:stream.write(b'{"sequence":3')
    restored=r.read_journal(rec.path)
    assert len(restored['records'])==2 and restored['incomplete_tail'] and not restored['finalized']



def test_partial_write_failure_permanently_preserves_completed_prefix(tmp_path,monkeypatch):
    rec=r.Recorder(tmp_path/'events.jsonl',{});rec.append('commands',{'buttons':[0]*13})
    original=r.os.write;calls=[0]
    def torn(fd,payload):
        calls[0]+=1
        if calls[0]==1:return original(fd,payload[:12])
        raise OSError('injected after partial write')
    monkeypatch.setattr(r.os,'write',torn)
    with pytest.raises(OSError):rec.append('imu',{'quaternion':[0,0,0,1]})
    monkeypatch.setattr(r.os,'write',original)
    with pytest.raises(RuntimeError,match='refusing later'):rec.append('lifecycle',{'event':'error'})
    data={'passed_software_sequence':True};outcome=r.finish_report(tmp_path/'report.json',data,rec)
    assert outcome['report_written'] and not outcome['passed_software_sequence']
    restored=r.read_journal(rec.path)
    assert len(restored['records'])==2 and restored['incomplete_tail'] and not restored['finalized']
    assert data['evidence_journal']['write_error']['known_partial_bytes']==12
