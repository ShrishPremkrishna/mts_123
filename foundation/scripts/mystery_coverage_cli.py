#!/usr/bin/env python3
"""Evaluate Mystery with real scheduled slots in every 10-second command window.

This is a different protocol from the continuous locomotion selection sweep:
reset each command, warm up for two seconds, then observe forced real slots.
It does not train, install a policy, or synthesize a successful jump capture.
"""
from pathlib import Path
import hashlib
import sys

SCHEDULE = {'p_no_jump': 0., 'min_start_s': 2.25, 'end_margin_s': 24., 'min_separation_s': 3.}


def apply_coverage_schedule(args, cfg):
    if args.command != 'evaluate' or args.task != 'Mjlab-Mystery-Flat-Pupper-v3':
        raise ValueError('Slot coverage only evaluates Mystery-Flat')
    if args.seconds != 10 or args.warmup != 2:
        raise ValueError('Reviewed slot-coverage protocol requires --seconds 10 --warmup 2')
    args.reset_each_command = True
    event = cfg.events['reset_jump_schedule']
    event.params.update(SCHEDULE)


def main():
    from mjlab.scripts import pupper_locomotion as cli
    original_settings, original_provenance = cli.settings, cli.provenance
    def settings(args, play):
        cfg, agent = original_settings(args, play)
        apply_coverage_schedule(args, cfg)
        return cfg, agent
    def provenance(args):
        result = original_provenance(args)
        result['forced_slot_coverage_probe'] = {
            'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'schedule_override': dict(SCHEDULE),
            'purpose': 'Reset each command and schedule real slots inside its scored window. Separate protocol from continuous locomotion selection; not a promotion or capture claim.',
        }
        return result
    cli.settings, cli.provenance = settings, provenance
    try:
        return cli.main()
    finally:
        cli.settings, cli.provenance = original_settings, original_provenance


if __name__ == '__main__':
    sys.exit(main())
