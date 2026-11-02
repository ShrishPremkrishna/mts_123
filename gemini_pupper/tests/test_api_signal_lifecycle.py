"""Real Robot/ASGI lifecycle with fake ROS signals and transport; no OS signals/hardware."""
import asyncio
import signal
import sys
import types

import pytest
from test_server_transport import server, harness, robot_api


@pytest.fixture
def context_lifecycle(monkeypatch):
    original_robot = robot_api.Robot
    prior = {name:getattr(robot_api,name) for name in ('time','asyncio','load_tricks')}
    state = {'alive':False, 'signal_options':None, 'events':[], 'rclpy_signal_handlers':False}
    no_handlers=object()
    monkeypatch.setitem(sys.modules,'rclpy.signals',types.SimpleNamespace(
        SignalHandlerOptions=types.SimpleNamespace(NO=no_handlers)))
    def init(**kwargs):
        state['alive']=True
        state['signal_options']=kwargs
        state['rclpy_signal_handlers']=kwargs.get('signal_handler_options') is not no_handlers
        state['events'].append('init')
    def destroy_context():
        state['events'].append('context_shutdown');state['alive']=False
    monkeypatch.setattr(robot_api.rclpy,'ok',lambda:state['alive'])
    monkeypatch.setattr(robot_api.rclpy,'init',init)
    monkeypatch.setattr(robot_api.rclpy,'try_shutdown',destroy_context,raising=False)
    original_publish=harness._Pub.publish
    def publish(pub,msg):
        if not state['alive']:raise RuntimeError('publisher context invalid')
        if pub.topic=='/llm_cmd_vel':
            state['events'].append(('velocity',msg.linear.x,msg.linear.y,msg.angular.z))
        original_publish(pub,msg)
    monkeypatch.setattr(harness._Pub,'publish',publish)
    monkeypatch.setattr(harness._Executor,'shutdown',lambda self,**kwargs:state['events'].append('executor_shutdown'),raising=False)
    monkeypatch.setattr(harness._Node,'destroy_node',lambda self:state['events'].append('node_destroy'),raising=False)
    rigs=[]
    def factory(*,ros_signal_handlers=True):
        def construct(**kwargs):
            if not ros_signal_handlers:kwargs['ros_signal_handlers']=False
            return original_robot(**kwargs)
        monkeypatch.setattr(harness,'Robot',construct)
        rig=harness.Rig();rigs.append(rig);return rig.ts
    state.update(factory=factory,rigs=rigs,no_handlers=no_handlers)
    yield state
    for name,value in prior.items():setattr(robot_api,name,value)


@pytest.mark.parametrize('received_signal',[signal.SIGINT,signal.SIGTERM])
def test_api_signal_keeps_context_until_actual_stop_and_lifespan_cleanup(server,monkeypatch,context_lifecycle,received_signal):
    state=context_lifecycle
    async def case():
        monkeypatch.setattr(server,'Robot',state['factory'])
        monkeypatch.setattr(server,'follower_config',lambda:types.SimpleNamespace(mode='disabled'))
        async def detector():await asyncio.Event().wait()
        monkeypatch.setattr(server.detections,'run',detector)
        async with server.app.router.lifespan_context(server.app):
            rig=state['rigs'][0];assert await rig.settle()
            entered=asyncio.Event()
            async def running_job():
                rig.ts.send_speed(.2,0,0);entered.set()
                try:await asyncio.Event().wait()
                finally:rig.ts.send_speed(0,0,0)
            assert (await rig.ts.queue('pending move',running_job,10,then=robot_api.UNCHANGED)).ok
            await entered.wait()
            state['events'].clear()
            state['events'].append(('uvicorn_exit_requested',int(received_signal)))
            # Jazzy's installed default handler invalidates context before ASGI
            # finally. NO leaves that job to the owning ASGI lifecycle instead.
            if state['rclpy_signal_handlers']:state['alive']=False
        events=state['events'];zero=('velocity',0.,0.,0.)
        assert zero in events and events.index(zero)<events.index('executor_shutdown')
        assert events.index('executor_shutdown')<events.index('node_destroy')<events.index('context_shutdown')
        assert state['signal_options']=={'signal_handler_options':state['no_handlers']}
        assert not state['alive'] and server.tools is None
        assert rig.ts.worker.done() and rig.ts.commands.empty()
    asyncio.run(case())


def test_standalone_robot_keeps_existing_ros_signal_default(context_lifecycle):
    async def case():
        state=context_lifecycle;r=state['factory']()
        assert await state['rigs'][0].settle()
        assert state['signal_options']=={} and state['rclpy_signal_handlers']
        assert (await r.shutdown()).ok
    asyncio.run(case())


def test_already_initialized_context_is_not_reinitialized(context_lifecycle):
    async def case():
        state=context_lifecycle;state['alive']=True
        r=state['factory'](ros_signal_handlers=False)
        assert await state['rigs'][0].settle()
        assert state['signal_options'] is None and 'init' not in state['events']
        assert (await r.shutdown()).ok
    asyncio.run(case())


def test_stop_failure_is_not_hidden_and_resources_still_close(server,monkeypatch,context_lifecycle):
    state=context_lifecycle
    async def case():
        monkeypatch.setattr(server,'Robot',state['factory'])
        monkeypatch.setattr(server,'follower_config',lambda:types.SimpleNamespace(mode='disabled'))
        async def detector():await asyncio.Event().wait()
        monkeypatch.setattr(server.detections,'run',detector)
        with pytest.raises(RuntimeError,match='injected failed zero publish'):
            async with server.app.router.lifespan_context(server.app):
                rig=state['rigs'][0];assert await rig.settle()
                state['events'].clear()
                def failed(*args):raise RuntimeError('injected failed zero publish')
                monkeypatch.setattr(rig.ts,'send_speed',failed)
        assert state['events']==['executor_shutdown','node_destroy','context_shutdown']
        assert server.tools is None and rig.ts.worker.done()
    asyncio.run(case())
