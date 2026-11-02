from pathlib import Path
import json
p=Path(__file__).parent;d=json.loads((p/'protocol-fixtures.json').read_text());lines=['static void fixture_checks(){','std::vector<diagnostic::Report> reports;std::vector<Bytes> records;']
def emit(x):lines.append(x)
for v in d['request_fixtures']:
 b=v['payload_hex'];name=v['name'];expr='diagnostic::product_query()' if name=='product_id' else 'diagnostic::calibration_query(42)' if name.startswith('calibration') else 'diagnostic::feature_query('+str(v['expected_feature_id'])+')'
 emit(f'expect({expr}==bytes("{b}"),"{name}");')
for v in d['feature_restore_fixtures']:
 emit('{auto f=diagnostic::feature_response(bytes("'+v['response_hex']+'"));expect(f.set_request()==bytes("'+v['restore_payload_hex']+'"),"exact feature fields restored");}')
for v in d['actual_prior_probe2_controls']:
 emit('expect(diagnostic::control_records(bytes("'+v['payload_hex']+'"),records),"actual captured control batch");')
for v in d['report_fixtures']:
 b=v['payload_hex'];name=v['name'];expect=not(name.startswith('truncated')or name.startswith('unknown_report'))
 emit(f'expect(diagnostic::reports(bytes("{b}"),reports)=={str(expect).lower()},"{name}");')
 if name.startswith('07_signed'):
  z=v['expected'][0];emit(f'expect(reports.size()==1&&reports[0].values[0]==512&&reports[0].values[1]==-1024&&reports[0].values[2]==14&&reports[0].values[3]==2&&reports[0].values[4]==-3&&reports[0].values[5]==14,"signed Q9 velocity/bias");')
  emit('expect(reports[0].delay_ticks==259&&reports[0].reference_delta_ticks==-1000,"14bit delay and signed timestamp");')
 if name=='mixed07_rebase08_13':emit('expect(reports.size()==3&&reports[1].reference_delta_ticks==-1010&&reports[1].values[3]==16384&&reports[2].stability==1,"mixed report boundaries");')
 if name=='unknown_timebase':emit('expect(!reports[0].timing_valid,"sentinel timing remains unknown");')
for b in ['0100','f20107000000000000000000','f20206000000000000000000','f20303000000000000000000','fd02000000102700000000000000000000']:
 emit(f'expect(!diagnostic::allowed_request(bytes("{b}"),false),"inspect disallows reset/calibration/DCD/set");')
# Complete valid prefix followed by a truncated report must never return the prefix.
emit('expect(!diagnostic::reports(bytes("fbe80300000201030000000000000007"),reports)&&reports.empty(),"atomic partial report rejection");')
emit('expect(!diagnostic::control_records(bytes("fc02000000102700000000000000000000f8"),records)&&records.empty(),"atomic partial control rejection");')
# A direct log path ensures negativeQ12 is not printed as confident positive radians.
emit('''{Fake f;std::ostringstream r,w,p,s;IO io{[&](uint8_t*b,size_t n){return f.read(b,n);},[&](const uint8_t*b,size_t n){return f.write(b,n);},[&](){return f.time;},[&](){f.time+=.005;}};Capture c(io,r,w,p,s);c.deadline=200.;
f.channel=3;f.queue.push_back(bytes("fb00000000050100000000000000000040ffff"));c.poll();expect(p.str().find("\\\"rotation_accuracy_rad\\\":null")!=std::string::npos,"negativeQ12 accuracy unknown in actual log");
expect(diagnostic::reports(bytes("fb00000000050100000000000000000040ffff"),reports)&&reports[0].rotation_accuracy==65535,"RV negative Q12 raw preserved");}''')

source=Path('/Users/shrish/workspace/cs123-foundation/artifacts/hardware/2026-11-01/battery2-diagnostic/sensor-compare-probe/raw/stationary-compare-2-packets.jsonl')
import hashlib,struct
actual=[]
for lineno,line in enumerate(source.open(),1):
 row=json.loads(line)
 if row.get('channel')!=3 or not row.get('read_complete') or not row.get('payload_hex'):continue
 data=bytes.fromhex(row['payload_hex']);cursor=0;ids=[];values=[];okay=True
 while cursor<len(data):
  kind=data[cursor];n={251:5,250:5,2:10,5:14,8:12,19:6}.get(kind)
  if not n or cursor+n>len(data):okay=False;break
  if kind not in [251,250]:
   ids.append(kind);count={2:3,5:4,8:4,19:0}[kind];values.append(list(struct.unpack('<'+'h'*count,data[cursor+4:cursor+4+2*count])) if count else [])
  cursor+=n
 if okay and any(i in [2,5] for i in ids):
  actual.append({'line_1based':lineno,'payload_hex':data.hex(),'ids':ids,'signed_values':values})
  emit('expect(diagnostic::reports(bytes("'+data.hex()+'"),reports),"captured02/05 sensor batch");')
  emit('expect(reports.size()=='+str(len(ids))+',"captured report count");')
  for i,(kind,vs) in enumerate(zip(ids,values)):
   expr='reports['+str(i)+'].id=='+str(kind)
   for k,v in enumerate(vs):expr+='&&reports['+str(i)+'].values['+str(k)+']=='+str(v)
   emit('expect('+expr+',"independent signed captured values");')
  if len(actual)==8:break
assert any(5 in x['ids'] for x in actual) and any(2 in x['ids'] for x in actual)
(p/'captured-sensor-fixtures.json').write_text(json.dumps({'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'fixtures':actual},indent=2)+'\n')

lines.append('}')
(p/'generated_fixture_checks.hpp').write_text('\n'.join(lines)+'\n')
