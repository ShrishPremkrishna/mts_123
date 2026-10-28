#!/usr/bin/env python3
"""Explicit 0.10 m Perlin-grid experiment with accumulated overflow rejection.

This wraps the existing locomotion CLI without changing its task defaults.
The wrapper hash, grid contract, full generated environment and overflow audit
travel with every output. Only the Bumpy MixedGaits task is accepted.
"""
import hashlib
from pathlib import Path
import sys

from mjlab.scripts import pupper_locomotion as cli


def main():
    import mjlab.envs as envs
    original_settings=cli.settings;original_provenance=cli.provenance
    original_env=envs.ManagerBasedRlEnv;context={}
    def settings(args,play):
        from mjlab.tasks.pupper.heightfield_contract import apply_perlin_resolution
        if args.task!='Mjlab-MixedGaits-Bumpy-Pupper-v3' or not args.locomotion_torso_contact:
            raise ValueError('Checked Bumpy experiment requires MixedGaits-Bumpy and explicit torso contact')
        cfg,agent=original_settings(args,play)
        args.heightfield_sampling_contract=apply_perlin_resolution(cfg,.10)
        context['output']=args.out
        return cfg,agent
    def provenance(args):
        result=original_provenance(args)
        result['checked_bumpy_wrapper']={'path':str(Path(__file__).resolve()),
            'sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'resolution_m':.10,'physics_overflow_guard':True}
        return result
    class CheckedEnvironment(original_env):
        def __init__(self,*args,**kwargs):
            from mjlab.tasks.pupper.heightfield_contract import PhysicsOverflowAudit
            super().__init__(*args,**kwargs)
            self._foundation_overflow=PhysicsOverflowAudit(self.sim,context['output']/'physics_overflow_audit.json')
        def close(self):
            try:
                audit=getattr(self,'_foundation_overflow',None)
                self._foundation_overflow=None
                if audit is not None:audit.close()
            finally:super().close()
    cli.settings=settings;cli.provenance=provenance;envs.ManagerBasedRlEnv=CheckedEnvironment
    try:return cli.main()
    finally:
        cli.settings=original_settings;cli.provenance=original_provenance;envs.ManagerBasedRlEnv=original_env


if __name__=='__main__':sys.exit(main())
