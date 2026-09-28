"""Acceleration-limited velocity sampling, rollout collision and stop guard."""
from math import atan2, cos, sin
import numpy as np
from .models import wrap


class LocalPlanner:
    def __init__(self, config):
        self.config = config
        self.velocity = (0.0,0.0)
        self.reason = 'initial'

    def stop(self, reason):
        self.velocity = (0.0,0.0); self.reason = reason
        return self.velocity

    def command(self, pose, goal, points, moving, dt=.128, speed_scale=1):
        cfg = self.config
        clearance = cfg['robot_radius']+cfg['safety_margin']
        relative = points-np.array([pose.x,pose.y])
        nearest = np.min(np.linalg.norm(relative,axis=1)) if len(points) else float('inf')
        if nearest < cfg['robot_radius']+.035:
            return self.stop('emergency_clearance')
        v0,w0 = self.velocity
        vmax = cfg['max_speed']*speed_scale
        dv,dw = cfg['acceleration']*dt,cfg['angular_acceleration']*dt
        vs = (np.array([0.0]) if vmax<=0 else
              np.unique(np.r_[0,np.linspace(max(-vmax*.55,v0-dv),min(vmax,v0+dv),7)]))
        ws = np.unique(np.r_[0,np.linspace(max(-cfg['max_angular_speed'],w0-dw),
                                          min(cfg['max_angular_speed'],w0+dw),9)])
        bearing = wrap(atan2(goal[1]-pose.y,goal[0]-pose.x)-pose.yaw)
        best = None
        for v in vs:
            if abs(bearing)>1.0 and v>.01:
                continue
            for w in ws:
                x,y,yaw = pose.x,pose.y,pose.yaw
                safe,min_clearance = True,2.0
                # Account for braking distance in the current scan before rollout.
                if v>0 and len(relative):
                    forward = relative @ np.array([cos(pose.yaw),sin(pose.yaw)])
                    side = abs(relative @ np.array([-sin(pose.yaw),cos(pose.yaw)]))
                    corridor = (forward>0)&(side<clearance)
                    stopping = clearance+v*v/(2*cfg['acceleration'])+v*.16
                    if np.any(forward[corridor]<stopping):
                        continue
                for t in np.arange(.1,1.61,.1):
                    x += v*cos(yaw+w*.05)*.1; y += v*sin(yaw+w*.05)*.1; yaw += w*.1
                    d = float(np.min(np.linalg.norm(points-[x,y],axis=1))) if len(points) else 2
                    min_clearance = min(min_clearance,d)
                    if d < clearance:
                        safe = False; break
                    if any(np.linalg.norm(center+velocity*t-[x,y])<clearance+.25 for center,velocity in moving):
                        safe = False; break
                if not safe:
                    continue
                end_distance = np.hypot(goal[0]-x,goal[1]-y)
                heading = abs(wrap(atan2(goal[1]-y,goal[0]-x)-yaw))
                score = -2.4*end_distance-.7*heading+.7*v+.12*min(min_clearance,1.2)-.04*abs(w-w0)
                if best is None or score>best[0]:
                    best = (score,float(v),float(w))
        if best is None:
            return self.stop('no_safe_velocity')
        self.velocity = best[1:]
        self.reason = 'tracking' if best[1]>.01 else 'turn_or_wait'
        return self.velocity

    def wheels(self, command):
        v,w = command; r,b = self.config['wheel_radius'],self.config['axle_length']
        return (v-w*b/2)/r,(v+w*b/2)/r
