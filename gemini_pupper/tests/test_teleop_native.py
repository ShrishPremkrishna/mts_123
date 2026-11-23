"""Check keyboard input through the native fake-ROS combined runtime."""
import asyncio

from final_project.behaviors.fc_executor_bridge import BridgeCore, DEADLINE_CONTRACT
from final_project.behaviors.operator_drive import OperatorDrive
from final_project.behaviors.test_command_path_evidence import CLOCK, SOURCE
from final_project.behaviors.test_operator_drive import axes, send
from final_project.pupper_fc.model import Command, Field
from test_fc_walk_kick import field_clock_rig
from fc_walk_kick import CombinedRuntime


def test_manual_and_autonomous_share_one_runtime_stop_and_wire(monkeypatch):
    async def case():
        async with field_clock_rig(monkeypatch) as rig:
            robot = rig.ts
            core = BridgeCore("pupper-2", "field", "test",
                              {"pupper-1": Field(), "pupper-2": Field(attack_sign=-1)},
                              clock=robot.now, monotonic_ns=lambda: round(robot.now()*1e9),
                              command_contract=DEADLINE_CONTRACT, source_instance=SOURCE,
                              command_clock_id=CLOCK, kick_side="left")
            out = []
            runtime = CombinedRuntime(robot, core.executor, core.robot_id, velocity_sink=out.append,
                                      path_problem=lambda _: None)
            await runtime.open()
            try:
                c = OperatorDrive(core)
                c.observe_path(True, 1)
                base = robot.now()
                send(c, base, "mode", mode="teleop")
                c.match_epoch = "keyboard-game"
                send(c, base+.12, "drive", **axes())
                send(c, base+.24, "drive", delay=.20, **axes(forward=1))
                rig.clock.now = base + .44
                step = runtime.tick(manual_command=c.command(robot.now()))
                assert step.request is None and len(out) == 1 and out[-1].vx == .22
                packet, actuating, _ = core.deadline.encode(out[-1])
                assert actuating
                assert core.deadline.protocol.decode(packet).expires_mono_ns <= round((base+.49)*1e9)
                rig.clock.now = base + .50
                runtime.tick(manual_command=c.command(robot.now()))
                assert out[-1].vx == 0 and runtime.stop_reason is None
                core.executor.arm("field-match", "field-clock", robot.now())
                runtime.tick(manual_command=Command(.1, 0, 0, robot.now()+.1, core.robot_id))
                assert "conflicts" in runtime.stop_reason and out[-1].vx == 0
                assert not robot.current and not rig.robot.switches
            finally:
                await runtime.close()
    asyncio.run(case())
