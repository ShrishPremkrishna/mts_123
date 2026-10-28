"""A published reference is not proof that the controller accepted or finished it."""
import asyncio
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot'))
import test_tools as harness


def run_case(case):
    async def run():
        rig=harness.Rig()
        try:
            await rig.settle()
            await case(rig)
        finally:
            rig.close();await asyncio.sleep(0)
    asyncio.run(run())


@pytest.mark.parametrize('receipt',['silent','playing'])
def test_missing_acceptance_or_completion_quarantines_and_relaxes(receipt):
    async def case(rig):
        rig.robot.reference_ack=receipt
        result=await rig.ts.play_reference_trick(rig.ts.tricks['hop'])
        assert not result.ok and rig.ts.controller_fault
        assert not rig.robot.active() and not rig.ts.reference_active
        assert not rig.ts.reference_request_id
    run_case(case)


@pytest.mark.parametrize('receipt',['rejected','interrupted'])
def test_explicit_refusal_or_interruption_is_not_success(receipt):
    async def case(rig):
        rig.robot.reference_ack=receipt
        result=await rig.ts.play_reference_trick(rig.ts.tricks['hop'])
        assert not result.ok
        assert not rig.ts.controller_fault # known outcome, unlike missing reply
        assert 'completed' not in result.message
    run_case(case)


def test_correlated_completion_describes_playback_without_claiming_landing():
    async def case(rig):
        started=rig.clock.now
        result=await rig.ts.play_reference_trick(rig.ts.tricks['hop'])
        assert rig.clock.now-started >= .5 + rig.ts.tricks['hop'].total_time
        assert result.ok and 'reference playback' in result.message
        assert 'standing' not in result.message and 'land' not in result.message
        published=[m for _,topic,m in rig.robot.published if topic.endswith('/reference_trick')]
        assert len(published)==1
        assert published[0].layout.dim[0].label.startswith('pupper_reference_v1:')
        assert len(published[0].layout.dim[0].label.split(':')[1])==32
        assert published[0].data[:3]==[1.,.8,.1]
    run_case(case)


def test_old_request_or_malformed_receipt_cannot_complete_new_motion():
    async def case(rig):
        rig.robot.reference_ack='silent'
        original=rig.ts.ref_pub.publish
        def stale(message):
            original(message)
            rig.ts._on_reference_status(harness._DataMsg('pupper_reference_v1 '+'0'*32+' completed ok'))
            rig.ts._on_reference_status(harness._DataMsg('malformed completed'))
        rig.ts.ref_pub.publish=stale
        result=await rig.ts.play_reference_trick(rig.ts.tricks['hop'])
        assert not result.ok and rig.ts.controller_fault
    run_case(case)


def test_late_playing_receipt_does_not_undo_correlated_terminal_state():
    async def case(rig):
        rig.ts.reference_request_id='a'*32
        rig.ts._on_reference_status(harness._DataMsg('pupper_reference_v1 '+'a'*32+' completed ok'))
        rig.ts._on_reference_status(harness._DataMsg('pupper_reference_v1 '+'a'*32+' playing ok'))
        assert rig.ts.reference_status==('completed','ok')
    run_case(case)


@pytest.mark.parametrize('when',['before','playing','completed','settle'])
def test_quarantine_dominates_reference_receipt(when):
    async def case(rig):
        fault='uncertain controller outcome'
        original=rig.robot.send
        completed_at=[]
        def send(topic,message):
            if topic.endswith('/reference_status'):
                phase=message.data.split()[2]
                if phase==when:
                    rig.ts.controller_fault=fault
                if phase=='completed':
                    completed_at.append(rig.clock.now)
            original(topic,message)
        rig.robot.send=send
        def settle_fault():
            if when=='settle' and completed_at and rig.clock.now>completed_at[0]:
                rig.ts.controller_fault=fault
        rig.clock.on_advance.append(settle_fault)
        if when=='before':rig.ts.controller_fault=fault
        result=await rig.ts.play_reference_trick(rig.ts.tricks['hop'])
        assert not result.ok and result.message==fault
        if when=='before':
            assert not any(topic.endswith('/reference_trick') for _,topic,_ in rig.robot.published)
    run_case(case)
