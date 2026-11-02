"""In-process manual-arrow admission; no sockets, ROS hardware or credentials."""
import asyncio
import pytest
from test_server_transport import server, client_with_rig, harness, robot_api

LEFT={'forward_backward_velocity':0.,'right_left_velocity':0.,'turning_velocity':-20.,'duration':.5}
FORWARD={**LEFT,'forward_backward_velocity':.2,'turning_velocity':0.}

async def hold_motion(rig):
    gate=asyncio.Event();started=asyncio.Event();old=rig.ts.sleep
    async def held(seconds):
        started.set();await gate.wait();await old(seconds)
    rig.ts.sleep=held
    return gate,started

@pytest.mark.parametrize('bodies',[(LEFT,LEFT),(LEFT,FORWARD),(FORWARD,LEFT)])
def test_concurrent_manual_requests_admit_at_most_one(server,bodies):
    async def run():
        async with client_with_rig(server)as(client,rig):
            gate,_=await hold_motion(rig)
            replies=await asyncio.gather(*(client.post('/move/velocity',json=x)for x in bodies))
            payload=[x.json()for x in replies]
            assert sum(x['success']is True for x in payload)==1,payload
            refused=next(x for x in payload if not x['success'])
            assert refused['details']['refusal']=={'schema':'pupper_manual_refusal_v1','code':'busy'}
            assert rig.ts.commands.qsize()+(rig.ts.current is not None)==1
            gate.set();assert await rig.settle()
    asyncio.run(run())

def test_manual_refuses_existing_voice_job_without_cancelling_it(server):
    async def run():
        async with client_with_rig(server)as(client,rig):
            gate,started=await hold_motion(rig)
            voice=await client.post('/tools/move',json={'direction':'forward','speed':.2,'duration':.5});assert voice.json()['success']
            await started.wait();manual=await client.post('/move/velocity',json=LEFT)
            assert not manual.json()['success'];assert rig.ts.commands.empty()
            gate.set();assert await rig.settle();nonzero=harness.walk_twists(rig)
            assert nonzero and all(m.linear.x==.2 and m.angular.z==0 for m in nonzero)
    asyncio.run(run())

def test_intentional_voice_sequence_after_manual_still_queues(server):
    async def run():
        async with client_with_rig(server)as(client,rig):
            gate,started=await hold_motion(rig)
            assert(await client.post('/move/velocity',json=LEFT)).json()['success'];await started.wait()
            assert(await client.post('/tools/move',json={'direction':'forward','speed':.2,'duration':.5})).json()['success']
            assert rig.ts.commands.qsize()==1
            gate.set();assert await rig.settle()
            moving=harness.walk_twists(rig);assert any(m.angular.z>0 for m in moving)and any(m.linear.x>.1 for m in moving)
    asyncio.run(run())

def test_stop_clears_manual_then_fresh_explicit_manual_can_start(server):
    async def run():
        async with client_with_rig(server)as(client,rig):
            gate,started=await hold_motion(rig)
            assert(await client.post('/move/velocity',json=FORWARD)).json()['success'];await started.wait()
            stopped=await client.post('/stop/immediate');assert stopped.json()['success'];gate.set();assert await rig.settle()
            split=len(rig.robot.twists())
            assert(await client.post('/move/velocity',json=LEFT)).json()['success'];assert await rig.settle()
            subsequent=[m for _,m in rig.robot.twists()[split:]if m.linear.x or m.linear.y or m.angular.z]
            assert subsequent and all(m.linear.x==m.linear.y==0 and m.angular.z>0 for m in subsequent)
    asyncio.run(run())

def test_manual_waiting_for_admission_lock_is_invalidated_by_stop(server):
    async def run():
        async with client_with_rig(server)as(client,rig):
            await rig.ts.action_lock.acquire()
            old_epoch=rig.ts.command_epoch
            call=asyncio.create_task(client.post('/move/velocity',json=FORWARD,headers={'X-Pupper-Command-Epoch':old_epoch}))
            for _ in range(20):await harness._real_sleep(0)
            assert not call.done()
            assert(await client.post('/stop/immediate')).json()['success']
            rig.ts.action_lock.release();reply=await call
            assert reply.json()['success']is False;assert await rig.settle();assert not harness.walk_twists(rig)
    asyncio.run(run())

def test_left_arrow_http_maps_only_positive_ros_yaw_and_finishes_zero(server):
    async def run():
        async with client_with_rig(server)as(client,rig):
            assert(await client.post('/move/velocity',json=LEFT)).json()['success'];assert await rig.settle()
            moving=harness.walk_twists(rig);assert moving and all(m.linear.x==m.linear.y==0 and m.angular.z>0 for m in moving)
            last=rig.robot.twists()[-1][1];assert last.linear.x==last.linear.y==last.angular.z==0
    asyncio.run(run())
