"""Offline-only real FastAPI/ASGI Stop ordering and latency evaluation.

No listeners, network, API keys, subprocesses, real ROS, or physical hardware.
"""
import argparse,asyncio,hashlib,importlib.util,json,logging,os,pathlib,socket,statistics,subprocess,sys,time

def forbidden(*a,**k):raise AssertionError('offline Stop probe forbids network and subprocess access')

async def run(args):
    root=pathlib.Path(args.repo).resolve();sys.path.insert(0,str(root/'gemini_pupper/robot'))
    os.environ['PUPPER_SESSION_SKILLS']=''
    socket.create_connection=forbidden;socket.socket.connect=forbidden;socket.socket.connect_ex=forbidden;socket.socket.bind=forbidden
    subprocess.run=forbidden;subprocess.Popen=forbidden
    import test_tools as harness
    import robot_api
    assert robot_api.Node is harness._Node and robot_api.SingleThreadedExecutor is harness._Executor
    import httpx
    spec=importlib.util.spec_from_file_location('offline_stop_probe_server',root/'gemini_pupper/robot/server.py')
    server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)
    server.FAKE_MODE=True
    logging.getLogger('httpx').setLevel(logging.WARNING)
    report={'scope':'real FastAPI app and realtime fake ROS over in-process ASGI, no sockets or hardware','probe_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'sources':{p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in ['gemini_pupper/robot/robot_api.py','gemini_pupper/robot/server.py','gemini_pupper/robot/tool_registry.py']},'latency':[],'ordering':[]}
    async with server.app.router.lifespan_context(server.app):
      robot=harness.ROBOT
      async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app),base_url='http://in-process.invalid') as client:
       async def epoch_headers():
        health=(await client.get('/health')).json()
        assert health.get('command_epoch_required') is True and isinstance(health.get('command_epoch'),str)
        return {'X-Pupper-Command-Epoch':health['command_epoch']}
       for i in range(args.samples):
        accepted=(await client.post('/queue/wait',json={'duration':60},headers=await epoch_headers())).json();assert accepted['success']
        await asyncio.sleep(.01)
        before=time.monotonic();reply=await client.post('/stop/immediate');after=time.monotonic()
        firstzero=next(t for t,m in robot.twists() if t>=before and not any([m.linear.x,m.linear.y,m.angular.z]))
        await asyncio.sleep(.01)
        report['latency'].append({'sample':i+1,'first_zero_request_s':firstzero-before,'software_response_s':after-before,'response':reply.json(),'post_status':(await client.get('/queue/status')).json()})
       for mode in ['body_pending','before_app_entry']:
        entered=asyncio.Event();release=asyncio.Event()
        payload=json.dumps({'direction':'forward','speed':.2,'duration':.1}).encode()
        class DelayedBody(httpx.AsyncByteStream):
         async def __aiter__(self):
          entered.set();await release.wait();yield payload
        async def held_app(scope,receive,send):
         if scope['path']=='/tools/move':entered.set();await release.wait()
         await server.app(scope,receive,send)
        transport=httpx.ASGITransport(app=held_app if mode=='before_app_entry' else server.app)
        async with httpx.AsyncClient(transport=transport,base_url='http://in-process.invalid') as delayed:
         headers={**await epoch_headers(),'Content-Type':'application/json'}
         sent=time.monotonic();pending=asyncio.create_task(delayed.post('/tools/move',content=DelayedBody() if mode=='body_pending' else payload,headers=headers))
         await asyncio.wait_for(entered.wait(),2)
         stop=(await client.post('/stop/immediate')).json();stopped=time.monotonic();release.set()
         response=await asyncio.wait_for(pending,2);result=response.json();await asyncio.sleep(.15)
         nonzero=[{'after_stop_s':t-stopped,'vx':m.linear.x,'vy':m.linear.y,'wz':m.angular.z} for t,m in robot.twists() if t>=stopped and any([m.linear.x,m.linear.y,m.angular.z])]
         report['ordering'].append({'mode':mode,'request_began_before_stop':sent<stopped,'stop_response':stop,'late_motion_http_status':response.status_code,'late_motion_response':result,'nonzero_commands_after_stop':nonzero,'post_status':(await client.get('/queue/status')).json()})
         await client.post('/stop/immediate')
    vals=[x['software_response_s'] for x in report['latency']]
    report['latency_summary']={'samples':len(vals),'response_median_s':statistics.median(vals),'response_max_s':max(vals),'max_first_zero_request_s':max(x['first_zero_request_s'] for x in report['latency'])}
    report['passed']=all(x['late_motion_http_status']==409 and not x['late_motion_response']['success'] and not x['nonzero_commands_after_stop'] for x in report['ordering'])
    pathlib.Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'passed':report['passed'],'latency':report['latency_summary'],'ordering':[{'mode':x['mode'],'accepted':x['late_motion_response']['success'],'nonzero_after_stop':len(x['nonzero_commands_after_stop'])} for x in report['ordering']]}))
    return report['passed']
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--repo',default=str(pathlib.Path(__file__).resolve().parents[2]));p.add_argument('--output',required=True);p.add_argument('--samples',type=int,default=20)
 args=p.parse_args()
 if not 1<=args.samples<=100:p.error('--samples must be between 1 and 100')
 raise SystemExit(0 if asyncio.run(run(args)) else 1)
