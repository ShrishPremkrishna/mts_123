from types import SimpleNamespace
import json
import pytest
import torch
from mjlab.tasks.pupper.heightfield_contract import apply_perlin_resolution,PhysicsOverflowAudit
from mjlab.tasks.pupper_gait.pupper_gait_env_cfg import pupper_mixed_gaits_bumpy_env_cfg,pupper_mixed_gaits_env_cfg


def test_explicit_grid_retains_amplitude_and_does_not_mutate_shared_config():
    cfg=pupper_mixed_gaits_bumpy_env_cfg();original=cfg.scene.terrain.terrain_generator
    before=original.sub_terrains['bumpy']
    report=apply_perlin_resolution(cfg,.1);after=cfg.scene.terrain.terrain_generator.sub_terrains['bumpy']
    assert report['prior_resolution_m']=={'bumpy':.05}
    assert after.resolution==.1 and before.resolution==.05
    assert after.height_range==before.height_range and after.horizontal_scale==before.horizontal_scale and after.scale==before.scale
    assert cfg.scene.terrain.terrain_generator is not original
    with pytest.raises(ValueError,match='generated terrain'):apply_perlin_resolution(pupper_mixed_gaits_env_cfg(),.1)


def test_overflow_accumulates_before_reset_and_restores_step(tmp_path):
    state=SimpleNamespace(overflow=torch.zeros(2,dtype=torch.int32));calls=[]
    def step():
        calls.append(1);state.overflow[:]=torch.tensor([0,32]) if len(calls)==1 else 0
    sim=SimpleNamespace(data=state,step=step)
    audit=PhysicsOverflowAudit(sim,tmp_path/'audit.json',check_every=2)
    sim.step();state.overflow.zero_()  # autoreset would clear the simulator bit
    with pytest.raises(RuntimeError,match='Physics overflow'):sim.step()
    with pytest.raises(RuntimeError):audit.close()
    assert sim.step is step
    report=json.loads((tmp_path/'audit.json').read_text())
    assert report['world_bitmasks']==[0,32] and not report['zero_overflow']
    assert report['physics_steps_observed']==2


def test_clear_overflow_audit_has_no_simulator_side_effect(tmp_path):
    state=SimpleNamespace(overflow=torch.zeros(1,dtype=torch.int32));calls=[]
    def step():calls.append(1)
    sim=SimpleNamespace(data=state,step=step);audit=PhysicsOverflowAudit(sim,tmp_path/'audit.json',check_every=2)
    for _ in range(3):sim.step()
    assert audit.close()['zero_overflow'] and len(calls)==3 and sim.step is step
