"""A controller selected outside Gemini must not be hidden by cached planning state."""
import pytest
from test_status_inventory import run_case
import test_tools as harness
import robot_api
from operator_feedback import receipt_feedback

@pytest.mark.parametrize("controllers,expected", [
    ({"neural_controller_default":"active"},"locomotion"),
    ({robot_api.LOCOMOTION_CONTROLLER:"active"},"locomotion"),
    ({robot_api.LOCOMOTION_CONTROLLER:"active","neural_controller_default":"active"},"unknown"),
    ({"neural_controller_student":"active"},"unknown"),
    ({"unfamiliar_joint_controller":"active"},"unknown"),
    ({"neural_controller_default":"inactive"},"none"),
    ({"joint_state_broadcaster":"active"},"unknown"),
])
def test_inventory_mode_is_independent_of_cached_planning(controllers,expected):
    async def case(rig):
        rig.robot.controllers=controllers.copy()
        rig.ts.state_now=robot_api.MOTORS_IDLE
        rig.ts.state_after_queue=robot_api.LOCOMOTION
        before=(rig.ts.state_now,rig.ts.state_after_queue,list(rig.robot.switches))
        result=await rig.ts.mode()
        assert result.ok
        assert result.details['observed_controller_mode']==expected
        assert result.details['state']==robot_api.MOTORS_IDLE
        assert (rig.ts.state_now,rig.ts.state_after_queue,rig.robot.switches)==before
        text=receipt_feedback('robot_status',result.as_dict())
        assert text.startswith('Controller:') and 'motors idle' not in text
        assert result.details['physical_observation']=={'pose':None,'speed':None,'stopped':None}
        # The cheap queue endpoint remains a planning snapshot, not a new ROS observation.
        assert 'observed_controller_mode' not in rig.ts.status()
    run_case(case)

@pytest.mark.parametrize('stock_active',[False,True])
def test_only_typed_feedback_broadcasters_are_ignored(stock_active):
    async def case(rig):
        rig.robot.controllers={'neural_controller_default':'active' if stock_active else 'inactive'}
        original=rig.ts._call
        async def with_feedback(client,request,timeout=5.0):
            response=await original(client,request,timeout)
            if client is rig.ts.list_client:
                response.controller.append(harness._Msg(name='feedback_any_name',state='active',type='joint_state_broadcaster/JointStateBroadcaster'))
                response.controller.append(harness._Msg(name='imu_any_name',state='active',type='imu_sensor_broadcaster/IMUSensorBroadcaster'))
            return response
        rig.ts._call=with_feedback
        result=await rig.ts.mode()
        assert result.details['observed_controller_mode']==('locomotion' if stock_active else 'none')
    run_case(case)

@pytest.mark.parametrize('name,expected',[('sit_stay_shake','sit_stay_shake'),('secret_dance','secret_dance'),('custom','other_skill')])
def test_known_skill_controller_observation(name,expected):
    async def case(rig):
        trick=rig.ts.tricks['hold_pose'];trick.name=name
        rig.robot.controllers={trick.controller:'active'}
        result=await rig.ts.mode()
        assert result.details['observed_controller_mode']==expected
    run_case(case)


def test_no_inventory_is_unknown_not_idle():
    async def case(rig):
        rig.robot.controllers={}
        result=await rig.ts.mode()
        assert not result.ok and result.details['observed_controller_mode']=='unknown'
    run_case(case)
