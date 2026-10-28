"""Explicit heightfield discretization and overflow checks for torso-contact runs."""
from copy import deepcopy
import json
import math
from pathlib import Path


def apply_perlin_resolution(cfg, resolution):
    """Change only the Perlin sampling grid; retain amplitude and randomization."""
    from mjlab.terrains.heightfield_terrains import HfPerlinNoiseTerrainCfg
    terrain=cfg.scene.terrain
    if not math.isfinite(resolution) or resolution<=0:
        raise ValueError('Heightfield resolution must be positive and finite')
    if terrain is None or terrain.terrain_type!='generator' or terrain.terrain_generator is None:
        raise ValueError('Explicit Perlin resolution requires generated terrain')
    generator=deepcopy(terrain.terrain_generator)
    before={}
    for name,sub in generator.sub_terrains.items():
        if not isinstance(sub,HfPerlinNoiseTerrainCfg):
            raise ValueError('This contract only supports Perlin heightfields')
        if min(generator.size)/resolution<2:
            raise ValueError('Resolution leaves fewer than two heightfield grid points')
        before[name]=sub.resolution;sub.resolution=resolution
    if not before:raise ValueError('No Perlin terrain to configure')
    terrain.terrain_generator=generator
    return {'contract':'explicit_perlin_sampling_resolution_v1','prior_resolution_m':before,
            'resolution_m':resolution,'height_range_and_feature_parameters_unchanged':True,
            'scope':'Sampling changes the discretized terrain surface. Compare policies on identically generated terrain under this same contract; do not mix with original-grid measurements.'}


class PhysicsOverflowAudit:
    """Accumulate simulator overflow bits before episode resets can erase them.

    The OR stays on the simulator device. A bounded periodic CPU read fails the
    run promptly; it does not alter contacts, actions, RNG, or reset behavior.
    """
    def __init__(self, sim, output, check_every=1000):
        if not isinstance(check_every,int) or check_every<1:
            raise ValueError('check_every must be a positive integer')
        self.sim=sim;self.output=Path(output);self.check_every=check_every
        self.seen=sim.data.overflow.clone();self.steps=0;self.original_step=sim.step
        def audited_step():
            self.original_step()
            self.seen.bitwise_or_(sim.data.overflow)
            self.steps+=1
            if self.steps%self.check_every==0:self.check()
        sim.step=audited_step

    def report(self):
        bits=self.seen.detach().cpu().tolist()
        return {'contract':'accumulated_physics_overflow_v1','physics_steps_observed':self.steps,
                'check_every_physics_steps':self.check_every,'zero_overflow':not any(bits),
                'world_bitmasks':bits,'affected_worlds':sum(v!=0 for v in bits),
                'meaning':'MuJoCo-Warp overflow bitmasks accumulated after every physics step, including steps before autoreset; nonzero invalidates physics evidence.'}

    def check(self, save=False):
        report=self.report()
        if save or not report['zero_overflow']:
            self.output.parent.mkdir(parents=True,exist_ok=True)
            self.output.write_text(json.dumps(report,indent=2)+'\n')
        if not report['zero_overflow']:
            raise RuntimeError(f"Physics overflow in {report['affected_worlds']} worlds; see {self.output}")
        return report

    def close(self):
        self.sim.step=self.original_step
        return self.check(save=True)
