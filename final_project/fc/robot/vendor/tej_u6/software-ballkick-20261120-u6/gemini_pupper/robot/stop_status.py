"""Strict, fail-closed consumer of the shared joystick/gateway stop authority.

This is a software command gate, not evidence of physical stopping. A tool-server
restart must re-establish readiness; it never adopts a changed arbiter instance.
"""
from __future__ import annotations

import json
import re

PROTOCOL = 'pupper_stop_v1'
TOPIC = '/estop_controller/stop_status'
MAX_AGE_S = 0.5
_FLAGS = ('latched', 'fault', 'stop_required', 'in_flight',
          'unknown_remote_submission', 'shutting_down')
_BLOCKING = tuple(name for name in _FLAGS if name != 'in_flight')
_KEYS = frozenset(('protocol', 'instance_id', 'sequence', 'epoch', *_FLAGS))


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate stop status key')
        result[key] = value
    return result


class StopStatus:
    def __init__(self):
        self.state = None
        self.received = None
        self.instance_id = None
        self.error = 'shared stop status has not been received'
        self.permanent_error = None
        self.local_inhibit = False
        self._barrier_epoch = -1
        self._barrier_sequence = -1
        self._seen_blocked = None

    def accept(self, payload: str, now: float) -> bool:
        try:
            state = json.loads(payload, object_pairs_hook=_unique_object)
            if not isinstance(state, dict) or set(state) != _KEYS:
                raise ValueError('unexpected stop status fields')
            if state['protocol'] != PROTOCOL:
                raise ValueError('unsupported stop status protocol')
            if not isinstance(state['instance_id'], str) or not re.fullmatch('[0-9a-f]{32}', state['instance_id']):
                raise ValueError('invalid stop authority instance')
            if any(type(state[name]) is not bool for name in _FLAGS):
                raise ValueError('stop status flags must be booleans')
            if any(type(state[name]) is not int or not 0 <= state[name] < 2**64 for name in ('sequence', 'epoch')):
                raise ValueError('invalid stop status sequence or epoch')
        except (ValueError, TypeError):
            self.error = 'shared stop status is malformed or unsupported'
            return False
        if self.instance_id is not None and self.instance_id != state['instance_id']:
            self.permanent_error = 'stop authority restarted; recheck the stack and restart this tool server'
            return False
        if self.permanent_error:
            return False
        if self.state is not None:
            if state['sequence'] <= self.state['sequence']:
                return False  # never refresh freshness with duplicate/out-of-order data
            if state['epoch'] < self.state['epoch']:
                self.permanent_error = 'stop authority epoch regressed; inspect the stack and restart this tool server'
                return False
        self.instance_id = state['instance_id']
        self.state, self.received, self.error = state, now, None
        if state['fault'] or state['unknown_remote_submission'] or state['shutting_down']:
            self.permanent_error = 'stop authority reported a fault, unknown submission or shutdown; inspect/restart the stack before new motion'
        if self.local_inhibit:
            if any(state[key] for key in _BLOCKING):
                self._seen_blocked = (state['epoch'], state['sequence'])
            elif state['sequence'] > self._barrier_sequence:
                later_epoch = self._barrier_epoch >= 0 and state['epoch'] > self._barrier_epoch
                seen_release = (self._seen_blocked is not None
                                and state['epoch'] >= self._seen_blocked[0]
                                and state['sequence'] > self._seen_blocked[1])
                if later_epoch or seen_release:
                    self.local_inhibit = False
                    self._seen_blocked = None
        return True

    def inhibit(self):
        """Empty has no epoch: inhibit immediately; repeated messages do not ratchet."""
        if self.local_inhibit:
            return
        self.local_inhibit = True
        self._barrier_epoch = self.state['epoch'] if self.state else -1
        self._barrier_sequence = self.state['sequence'] if self.state else -1
        self._seen_blocked = ((self.state['epoch'], self.state['sequence'])
                              if self.state and any(self.state[key] for key in _BLOCKING) else None)

    def reason(self, now: float) -> str | None:
        if self.permanent_error:
            return self.permanent_error
        if self.error or self.state is None or self.received is None:
            return self.error or 'shared stop status is unavailable'
        if not 0 <= now - self.received <= MAX_AGE_S:
            return 'shared stop status is stale; motion is blocked'
        if self.local_inhibit:
            return 'emergency stop is inhibited until a fresh physical release is confirmed'
        flags = [name for name in _BLOCKING if self.state[name]]
        if flags:
            return 'shared stop authority blocks motion: ' + ', '.join(flags)
        return None

    def episode(self):
        return (self.instance_id, self.state['epoch'] if self.state else -1)
