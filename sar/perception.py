"""RGB segmentation + LiDAR depth association. Camera recognition is unused."""
import colorsys
from collections import deque
from math import atan, cos, sin, tan
import numpy as np
from .models import Detection, Target, wrap


def hue_image(rgb):
    data = rgb.astype(float) / 255
    maximum, minimum = data.max(axis=2), data.min(axis=2)
    delta = maximum-minimum
    hue = np.zeros_like(maximum)
    nonzero = delta > 1e-5
    for channel in range(3):
        mask = nonzero & (data[:,:,channel] == maximum)
        if channel == 0:
            hue[mask] = ((data[:,:,1]-data[:,:,2])[mask]/delta[mask]) % 6
        elif channel == 1:
            hue[mask] = (data[:,:,2]-data[:,:,0])[mask]/delta[mask]+2
        else:
            hue[mask] = (data[:,:,0]-data[:,:,1])[mask]/delta[mask]+4
    saturation = delta / np.maximum(maximum,1e-5)
    return hue/6, saturation, maximum


def components(mask, min_pixels=10):
    remaining = set(map(tuple,np.argwhere(mask)))
    while remaining:
        first = remaining.pop()
        queue, component = deque([first]), [first]
        while queue:
            y,x = queue.popleft()
            for p in ((y+1,x),(y-1,x),(y,x+1),(y,x-1)):
                if p in remaining:
                    remaining.remove(p); queue.append(p); component.append(p)
        if len(component) >= min_pixels:
            yield np.array(component)


class ColorLidarDetector:
    def __init__(self, config):
        self.config = config
        self.features = [(f['label'],colorsys.rgb_to_hsv(*(c/255 for c in f['rgb']))[0],
                          f['hue_tolerance']) for f in config['target_features']]

    def detect(self, frame, pose):
        if frame.rgb is None:
            return []
        rgb = frame.rgb[::2,::2]
        h,w = rgb.shape[:2]
        hue,saturation,brightness = hue_image(rgb)
        focal = w/(2*tan(frame.camera_fov/2))
        results = []
        for label,expected,tolerance in self.features:
            dh = abs(hue-expected); dh = np.minimum(dh,1-dh)
            mask = (dh<tolerance) & (saturation>.48) & (brightness>.23)
            mask[:int(h*.1)] = False
            mask[int(h*.9):] = False
            for points in components(mask):
                ys,xs = points.T
                if xs.max()-xs.min()<2 or ys.max()-ys.min()<3:
                    continue
                if xs.min()<=1 or xs.max()>=w-2:
                    continue  # a clipped silhouette gives biased bearing/depth
                u = float(xs.mean())
                bearing = atan((w/2-u)/focal)
                left = atan((w/2-xs.min())/focal)+.025
                right = atan((w/2-xs.max())/focal)-.025
                valid = (frame.angles >= right) & (frame.angles <= left) & np.isfinite(frame.ranges)
                valid &= (frame.ranges > .35) & (frame.ranges < self.config['lidar_max_range']-.1)
                if np.count_nonzero(valid)<2:
                    continue
                angular_width=(atan((w/2-xs.min())/focal)-atan((w/2-xs.max())/focal))
                radius=self.config.get('target_width',.36)/2
                silhouette_range=radius/max(np.sin(angular_width/2),1e-5)
                if silhouette_range>self.config['lidar_max_range']+.4:
                    continue
                # Reject a foreground person's LiDAR depth attached to a small
                # target silhouette behind them. Target width is a provided
                # visual feature, not a hidden target position.
                candidates=frame.ranges[valid]
                expected_surface=silhouette_range-radius+self.config['camera_offset']*cos(bearing)
                close=candidates[abs(candidates-expected_surface)<max(.3,silhouette_range*.25)]
                if len(close)<2:
                    continue
                depth=float(np.median(close))
                # RGB bearing origin is offset from LiDAR. Solve the horizontal
                # ray / range-circle intersection before adding target depth.
                offset = self.config['camera_offset']
                camera_range = -offset*cos(bearing)+np.sqrt(max(0,depth**2-offset**2*sin(bearing)**2))
                local = np.array([[offset+(camera_range+radius)*cos(bearing),
                                   (camera_range+radius)*sin(bearing)]])
                position = pose.transform(local)[0]
                results.append(Detection(label,position,min(.98,.6+len(points)/2000),bearing))
        return results


class TargetRegistry:
    def __init__(self):
        self.targets = []

    def update(self, detections, pose, time):
        for detection in detections:
            candidates = [t for t in self.targets if t.label==detection.label
                          and np.linalg.norm(t.position-detection.position)<.65]
            if candidates:
                t = min(candidates,key=lambda t: np.linalg.norm(t.position-detection.position))
                # Reject large innovation after confirmation (foreground depth).
                if t.confirmed and np.linalg.norm(t.position-detection.position)>.35:
                    continue
                t.position = .8*t.position+.2*detection.position
                t.last_seen = time; t.observations += 1
            else:
                t = Target(detection.label,detection.position.copy(),time,time)
                self.targets.append(t)
            t.evidence_positions.append((pose.x,pose.y,pose.yaw))
            first = t.evidence_positions[0]
            diversity = pose.distance(first[:2])>.12 or abs(wrap(pose.yaw-first[2]))>.15
            if t.observations>=3 and diversity:
                t.confirmed = True
            t.evidence_positions = t.evidence_positions[-30:]
        # Unconfirmed single glimpses are not permanent mission goals.
        self.targets = [t for t in self.targets if t.confirmed or time-t.last_seen<15]


class MotionTracker:
    """Associate compact scan clusters; only measured motion is predicted.

    The robot never receives pedestrian waypoints or velocities from evaluator.
    This is a conservative obstacle tracker, not a person classifier.
    """
    def __init__(self):
        self.previous = []
        self.last_time = None
        self.moving = []

    def update(self, points, time):
        groups = []
        if len(points):
            cuts = np.where(np.linalg.norm(np.diff(points,axis=0),axis=1)>.35)[0]+1
            for group in np.split(points,cuts):
                if 3 <= len(group) <= 120 and np.linalg.norm(group.max(axis=0)-group.min(axis=0))<.8:
                    groups.append(group.mean(axis=0))
        moving = []
        dt = time-self.last_time if self.last_time is not None else 0
        used = set()
        if .04 < dt < 1:
            for center in groups:
                choices = [(np.linalg.norm(center-prev),i,prev) for i,prev in enumerate(self.previous) if i not in used]
                if not choices:
                    continue
                d,i,prev = min(choices,key=lambda x:x[0])
                if d < .25:
                    used.add(i)
                    velocity = (center-prev)/dt
                    speed = np.linalg.norm(velocity)
                    if .08 < speed < 1.3:
                        moving.append((center,velocity))
        self.previous, self.last_time, self.moving = groups,time,moving
        return moving
