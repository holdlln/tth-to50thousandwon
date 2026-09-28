"""Wheel odometry + robust point-to-line scan matching against a keyframe.

No pose graph or loop closure yet. Uncertainty is a heuristic quality indicator,
not a calibrated covariance. Rank and innovation gates reject ambiguous scans.
"""
from math import cos, sin
import numpy as np
from .models import Pose, wrap


class Localizer:
    def __init__(self, config):
        self.config=config
        self.pose=Pose(*config['initial_pose'])
        self.previous=None
        self.uncertainty=.02
        self.corrections=0
        self.last_score=0.0
        self.key_points=None
        self.key_pose=None
        self.previous_gyro=None
        self.gyro_present=False

    def predict(self,left,right,dt=None,gyro_z=None,gyro_delta=None):
        if self.previous is None:
            self.previous=(left,right)
            return self.pose
        dl,dr=np.subtract((left,right),self.previous)*self.config['wheel_radius']
        self.previous=(left,right)
        distance=(dl+dr)/2
        turn=(dr-dl)/self.config['axle_length']
        if gyro_delta is not None and np.isfinite(gyro_delta):
            turn=gyro_delta
            self.gyro_present=True
        elif gyro_z is not None and dt is not None and np.isfinite(gyro_z):
            rate=gyro_z if self.previous_gyro is None else (gyro_z+self.previous_gyro)/2
            turn=rate*dt
            self.previous_gyro=gyro_z
            self.gyro_present=True
        else:
            self.gyro_present=False
        scale=np.sinc(turn/(2*np.pi))
        self.pose.x+=distance*scale*cos(self.pose.yaw+turn/2)
        self.pose.y+=distance*scale*sin(self.pose.yaw+turn/2)
        self.pose.yaw=wrap(self.pose.yaw+turn)
        self.uncertainty=min(.5,self.uncertainty+abs(distance)*.004+abs(turn)*.0008)
        return self.pose

    def correct(self,ranges,angles,grid=None):
        if not self.config['scan_matching']:
            return
        valid=np.isfinite(ranges)&(ranges>.5)&(ranges<self.config['lidar_max_range']-.1)
        local=np.column_stack((ranges[valid]*np.cos(angles[valid]),
                               ranges[valid]*np.sin(angles[valid])))[::2]
        if len(local)<35:
            return
        world=self.pose.transform(local)
        if self.key_points is None:
            self.key_points=world; self.key_pose=Pose(self.pose.x,self.pose.y,self.pose.yaw)
            return
        reference=self.key_points
        tangent=np.roll(reference,-1,axis=0)-np.roll(reference,1,axis=0)
        lengths=np.linalg.norm(tangent,axis=1)
        normal=np.column_stack((-tangent[:,1],tangent[:,0]))/np.maximum(lengths[:,None],1e-8)
        normal_valid=(lengths>.015)&(lengths<.5)
        predicted=world.copy()
        total=np.zeros(3)
        usable=False
        for _ in range(3):
            distances=np.linalg.norm(predicted[:,None,:]-reference[None,:,:],axis=2)
            indices=distances.argmin(axis=1)
            mask=(distances[np.arange(len(indices)),indices]<.25)&normal_valid[indices]
            if mask.sum()<25:
                break
            q=predicted[mask]; nearest=reference[indices[mask]]; n=normal[indices[mask]]
            residual=((q-nearest)*n).sum(axis=1)
            robust=abs(residual)<.12
            q,n,residual=q[robust],n[robust],residual[robust]
            if len(q)<25:
                break
            offset=q-[self.pose.x,self.pose.y]
            jacobian=np.column_stack((n[:,0],n[:,1],-n[:,0]*offset[:,1]+n[:,1]*offset[:,0]))
            weights=np.minimum(1,.04/np.maximum(abs(residual),1e-8))
            hessian=jacobian.T@(weights[:,None]*jacobian)
            if np.linalg.eigvalsh(hessian)[0]<.8:
                break
            delta=-np.linalg.solve(hessian+np.diag([2,2,4]),jacobian.T@(weights*residual))
            if np.linalg.norm(delta[:2])>.08 or abs(delta[2])>.06:
                break
            usable=True; total+=delta
            c,s=np.cos(delta[2]),np.sin(delta[2])
            predicted=(predicted-[self.pose.x,self.pose.y])@np.array([[c,s],[-s,c]])+[self.pose.x,self.pose.y]+delta[:2]
            self.last_score=float(np.sqrt(np.mean(residual**2)))
            if np.linalg.norm(delta)<.001:
                break
        if usable and np.linalg.norm(total[:2])<.10 and abs(total[2])<.08:
            if np.linalg.norm(total[:2])>.002 or abs(total[2])>.002:
                # Keep the encoder translation prior strong: a scan keyframe
                # also has error. Gyro-supported heading requires less correction.
                translation_gain=self.config.get('scan_translation_gain',.2)
                yaw_gain=self.config.get('scan_heading_gain',.12) if self.gyro_present else .7
                self.pose.x+=total[0]*translation_gain; self.pose.y+=total[1]*translation_gain
                self.pose.yaw=wrap(self.pose.yaw+total[2]*yaw_gain)
                self.uncertainty=max(.02,self.uncertainty*.96)
                self.corrections+=1
        if self.pose.distance((self.key_pose.x,self.key_pose.y))>.25 or abs(wrap(self.pose.yaw-self.key_pose.yaw))>.25:
            self.key_points=self.pose.transform(local)
            self.key_pose=Pose(self.pose.x,self.pose.y,self.pose.yaw)
