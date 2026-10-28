"""Finite body-frame foot trajectories; validation performs no ROS operation."""
import json
from pathlib import Path
import numpy as np
from kinematics import LEG_FK, LEG_LIMITS, inverse_kinematics

LEGS=('front_r','front_l','back_r','back_l')
MAX_JOINT_SPEED=15.  # Engineering command bound, not a hardware approval.


class RecordedMotion:
    def __init__(self, source, max_gap=.1):
        if not np.isfinite(max_gap) or max_gap<=0:raise ValueError('Invalid maximum sample gap')
        data=json.loads(Path(source).read_text()) if isinstance(source,(str,Path)) else source
        if (data.get('schema'),data.get('frame'),data.get('units'))!=('pupper_recorded_foot_v1','base_link','metres'):
            raise ValueError('Expected body-frame recorded motion in metres')
        self.leg=data['leg']
        if self.leg not in LEGS:raise ValueError('Unknown leg')
        self.times=np.asarray(data['times_s'],float);self.positions=np.asarray(data['positions'],float)
        self.joints=np.asarray(data['joint_positions'],float)
        if self.times.ndim!=1 or len(self.times)<2 or self.positions.shape!=(len(self.times),3) or self.joints.shape!=self.positions.shape:
            raise ValueError('Motion requires matching times, foot triples and joint triples')
        if not all(np.isfinite(x).all() for x in (self.times,self.positions,self.joints)):
            raise ValueError('Nonfinite motion data')
        if self.times[0]!=0 or (np.diff(self.times)<=0).any() or np.diff(self.times).max()>max_gap or self.times[-1]>600:
            raise ValueError('Invalid duration or missing samples; gaps cannot silently become motions')
        index=LEGS.index(self.leg);fk=LEG_FK[index];limits=LEG_LIMITS[index]
        if (self.joints<limits[:,0]).any() or (self.joints>limits[:,1]).any():raise ValueError('Recorded joint outside model bounds')
        if np.max(abs(np.diff(self.joints,axis=0))/np.diff(self.times)[:,None])>MAX_JOINT_SPEED:
            raise ValueError('Recorded joint speed exceeds15rad/s; inspect or slow the source before export')
        if max(np.linalg.norm(fk(q)-p) for q,p in zip(self.joints,self.positions))>.002:
            raise ValueError('Recorded foot/joint geometry mismatch')
        # Bound interpolation between records too; endpoints alone do not prove reachability.
        q=self.joints[0].copy()
        for i in range(len(self.times)-1):
            steps=max(3,int(np.ceil((self.times[i+1]-self.times[i])*80)))
            dt=(self.times[i+1]-self.times[i])/steps
            for alpha in np.linspace(0,1,steps+1)[1:]:
                p=(1-alpha)*self.positions[i]+alpha*self.positions[i+1]
                next_q=inverse_kinematics(fk,p,q,max_iterations=150,tolerance=1e-4,strict=True,joint_limits=limits)
                if np.max(abs(next_q-q))/dt>MAX_JOINT_SPEED:raise ValueError('Interpolated joint speed exceeds15rad/s')
                q=next_q
        self.duration=float(self.times[-1])

    def at(self, seconds):
        if not np.isfinite(seconds):raise ValueError('Invalid motion clock')
        return np.array([np.interp(seconds,self.times,self.positions[:,axis]) for axis in range(3)])


class RecordedReplay:
    """Continuous IK branch with a two-second entry and an online command guard.

    Receipt clock bounds each target step. A late callback cannot authorize a
    large catch-up jump: the allowed interval is capped at one control period.
    The phase clock may pause for contact; neither clock may run backward.
    """
    def __init__(self, motion, control_period=.025):
        if not np.isfinite(control_period) or control_period<=0:raise ValueError('Invalid control period')
        self.motion=motion;self.control_period=control_period
        self.index=LEGS.index(motion.leg)
        self.started_phase=self.last_phase=self.last_wall=None
        self.start_foot=self.previous_target=None

    def target(self, phase_time, measured_joints, wall_time):
        if not np.isfinite([phase_time,wall_time]).all():raise ValueError('Invalid replay clocks')
        measured=np.asarray(measured_joints,float)
        if measured.shape!=(3,) or not np.isfinite(measured).all():raise ValueError('Invalid measured joints')
        limits=LEG_LIMITS[self.index];fk=LEG_FK[self.index]
        if (measured<limits[:,0]).any() or (measured>limits[:,1]).any():raise ValueError('Measured joint outside model bounds')
        if self.last_wall is not None and (wall_time<self.last_wall or phase_time<self.last_phase):
            raise ValueError('Replay clock ran backward')
        started=phase_time if self.started_phase is None else self.started_phase
        start=fk(measured) if self.start_foot is None else self.start_foot
        previous=measured if self.previous_target is None else self.previous_target
        elapsed=phase_time-started
        alpha=min(elapsed/2,1.);blend=alpha*alpha*(3-2*alpha)
        foot=((1-blend)*start+blend*self.motion.at(0) if elapsed<2 else self.motion.at(elapsed-2))
        target=inverse_kinematics(fk,foot,previous,max_iterations=150,tolerance=1e-4,strict=True,joint_limits=limits)
        dt=0. if self.last_wall is None else min(wall_time-self.last_wall,self.control_period)
        if np.max(abs(target-previous))>MAX_JOINT_SPEED*dt+1e-9:
            raise ValueError('Replay target step exceeds15rad/s; entry, IK branch or scheduler gap rejected')
        self.started_phase=started;self.start_foot=start
        self.previous_target=target.copy();self.last_phase=phase_time;self.last_wall=wall_time
        return target
