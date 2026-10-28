"""Slot telemetry must match live scheduler boundaries without changing its state."""
from types import SimpleNamespace
import pytest
import torch
from mjlab.scripts.pupper_locomotion import jump_schedule_diagnostics


def test_schedule_telemetry_tracks_distinct_windows_and_is_read_only():
    slots=torch.tensor([[1.,float('inf'),float('inf')]]*5)
    env=SimpleNamespace(num_envs=5,device='cpu',step_dt=.25,
                        episode_length_buf=torch.tensor([3,4,6,7,8]),
                        _pupper_mixed_jump_slots=slots)
    before_slots=slots.clone();before_clock=env.episode_length_buf.clone()
    result=jump_schedule_diagnostics(env,.75,1.)
    assert result['jump_reference_active'].tolist()==[False,True,True,False,False]
    assert result['jump_reward_active'].tolist()==[False,True,True,True,False]
    assert result['jump_reward_elapsed_s'].tolist()==[0.,0.,.5,.75,0.]
    assert result['episode_time_s'].tolist()==[.75,1.,1.5,1.75,2.]
    torch.testing.assert_close(env._pupper_mixed_jump_slots,before_slots)
    torch.testing.assert_close(env.episode_length_buf,before_clock)
    # Reset/re-scheduling is reflected in the next sample, not cached metadata.
    env.episode_length_buf.zero_();env._pupper_mixed_jump_slots.fill_(float('inf'))
    assert not jump_schedule_diagnostics(env,.75,1.)['jump_reward_active'].any()


def test_missing_schedule_is_not_silently_created_by_diagnostics():
    env=SimpleNamespace(num_envs=1,device='cpu',step_dt=.02,episode_length_buf=torch.zeros(1))
    with pytest.raises(ValueError,match='initialized jump schedule'):
        jump_schedule_diagnostics(env,.75,1.)
    assert not hasattr(env,'_pupper_mixed_jump_slots')
