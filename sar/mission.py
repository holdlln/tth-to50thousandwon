"""Mission state machine shared by Webots and deterministic integration tests."""
from math import atan2
from dataclasses import replace
import numpy as np
from .models import wrap
from .localization import Localizer
from .mapping import OccupancyGrid
from .perception import ColorLidarDetector, TargetRegistry, MotionTracker
from .planning import shortest_paths, reconstruct, frontier_goal, approach_goal, RescueFrontierPolicy
from .control import LocalPlanner
from .sensors import sanitize_lidar


class Mission:
    def __init__(self, config, detector=None, frontier_policy=None):
        self.config = config
        self.localizer = Localizer(config)
        self.grid = OccupancyGrid(config)
        self.detector = detector or ColorLidarDetector(config)
        self.policy = frontier_policy or RescueFrontierPolicy()
        self.registry = TargetRegistry()
        self.tracker = MotionTracker()
        self.local = LocalPlanner(config)
        self.home = np.array(config['initial_pose'][:2])
        self.state = 'SCAN'
        self.scan_remaining = 2*np.pi
        self.last_yaw = config['initial_pose'][2]
        self.last_time = None
        self.last_plan = -10.0
        self.goal = None
        self.path = []
        self.active_target = None
        self.events = []
        self.excluded = []
        self.hold_since = None
        self.stall_since = None
        self.anchor = self.home.copy()
        self.return_reason = None
        self.done_reason = None
        self.last_home_cost = float('inf')
        self.no_frontier_scans = 0
        self.recovery_goal = None
        self.rejected_rays = 0

    def event(self, time, kind, **data):
        self.events.append(dict(time=float(time),kind=kind,**data))

    def start_scan(self):
        self.state='SCAN'; self.scan_remaining=2*np.pi; self.path=[]; self.goal=None

    def plan(self, time):
        cfg,grid,pose = self.config,self.grid,self.localizer.pose
        free = grid.traversable(cfg['robot_radius']+cfg['safety_margin']+.03)
        start = grid.cell((pose.x,pose.y))
        self.recovery_goal=None
        if grid.inside(start) and not free[start[1],start[0]]:
            # A corrected pose can fall inside a map inflation band. Retreat to
            # an observed free cell using the unchanged sensor collision guard.
            cells=np.argwhere(free)
            if len(cells):
                xy=grid.world(cells[:,::-1])
                lengths=np.linalg.norm(xy-[pose.x,pose.y],axis=1)
                mask=(lengths>=.3)&(lengths<=.8)
                if mask.any():
                    options=xy[mask]; costs=lengths[mask]
                    self.recovery_goal=options[int(np.argmin(costs))]
            self.goal=None; self.path=[]; self.last_plan=time
            return
        distances,parents = shortest_paths(free,start)
        home_cell = grid.cell(self.home)
        home_distances,_ = shortest_paths(free,home_cell)
        self.last_home_cost = distances.get(home_cell,float('inf'))*grid.resolution
        remaining = cfg['mission_limit_seconds']-time
        if remaining<cfg['return_reserve_seconds'] and self.state not in ('RETURN','DONE','FAILED'):
            self.state='RETURN'; self.return_reason='minimum_time_reserve'
        if np.isfinite(self.last_home_cost):
            return_time = self.last_home_cost/(cfg['max_speed']*.55)+cfg['return_reserve_seconds']
            if remaining<return_time and self.state not in ('RETURN','DONE','FAILED'):
                self.state='RETURN'; self.return_reason='time_reserve'
                self.event(time,'return_decision',reason=self.return_reason,estimated_seconds=return_time)
        if self.state=='RETURN':
            self.goal = home_cell if home_cell in distances else None
        else:
            self.active_target = None
            options=[]
            for target in self.registry.targets:
                if target.confirmed and not target.visited:
                    cell = approach_goal(grid,target.position,distances,cfg['approach_distance'])
                    if cell is not None:
                        options.append((distances[cell],target,cell))
            if options:
                _,self.active_target,self.goal = min(options,key=lambda o:o[0])
                self.state='APPROACH'
            else:
                self.excluded = [(p,t) for p,t in self.excluded if time-t<30]
                self.goal = frontier_goal(grid,free,distances,home_distances,self.policy,
                                          self.localizer.uncertainty,self.excluded)
                self.state='EXPLORE'
                if self.goal is None and distances:
                    if self.no_frontier_scans<2:
                        self.no_frontier_scans+=1; self.start_scan()
                    else:
                        self.state='RETURN'; self.return_reason='exploration_exhausted'
                        self.goal=home_cell if home_cell in distances else None
        self.path = reconstruct(parents,start,self.goal) if self.goal is not None else []
        self.last_plan=time

    def step(self, frame):
        cfg = self.config
        filtered,self.rejected_rays=sanitize_lidar(frame.ranges)
        frame=replace(frame,ranges=filtered)
        dt = .128 if self.last_time is None else max(.001,frame.time-self.last_time)
        self.last_time=frame.time
        self.localizer.predict(frame.left_encoder,frame.right_encoder,dt,frame.gyro_z,frame.gyro_delta)
        self.localizer.correct(frame.ranges,frame.angles,self.grid)
        pose = self.localizer.pose
        if frame.bumper:
            self.state='FAILED'; self.done_reason='bumper'
        if self.state in ('DONE','FAILED'):
            return self.local.stop(self.done_reason or self.state)
        valid = np.isfinite(frame.ranges)&(frame.ranges>cfg['lidar_min_range'])&(frame.ranges<cfg['lidar_max_range'])
        # A completely missing scan must not authorize motion.
        if np.count_nonzero(~np.isnan(frame.ranges)&(frame.ranges>0))<len(frame.ranges)*.6:
            return self.local.stop('invalid_lidar')
        local_points = np.column_stack((frame.ranges[valid]*np.cos(frame.angles[valid]),
                                       frame.ranges[valid]*np.sin(frame.angles[valid])))
        points = pose.transform(local_points)
        moving = self.tracker.update(points,frame.time)
        self.grid.update(pose,frame.ranges,frame.angles)
        detections = self.detector.detect(frame,pose)
        before = sum(t.confirmed for t in self.registry.targets)
        self.registry.update(detections,pose,frame.time)
        if sum(t.confirmed for t in self.registry.targets)>before:
            self.event(frame.time,'target_confirmed',targets=[dict(label=t.label,position=t.position.tolist())
                       for t in self.registry.targets if t.confirmed])
            self.last_plan=-10
        visited = sum(t.visited for t in self.registry.targets)
        expected = cfg['expected_targets']
        if expected is not None and visited>=expected and self.state!='RETURN':
            self.state='RETURN'; self.return_reason='all_targets_visited'; self.last_plan=-10
        if frame.time>=cfg['mission_limit_seconds']:
            self.state='FAILED'; self.done_reason='time_limit'
            self.event(frame.time,'mission_finished',status='FAILED',reason=self.done_reason)
            return self.local.stop(self.done_reason)
        if self.state=='RETURN' and pose.distance(self.home)<cfg['home_tolerance']:
            error = wrap(cfg['initial_pose'][2]-pose.yaw)
            if abs(error)>cfg['home_angle_tolerance']:
                return self.local.command(pose,[pose.x+np.cos(cfg['initial_pose'][2]),
                                                pose.y+np.sin(cfg['initial_pose'][2])],points,moving,dt,speed_scale=0)
            success = expected is not None and visited>=expected
            self.state='DONE'; self.done_reason='complete' if success else 'returned_incomplete'
            self.event(frame.time,'mission_finished',status=self.done_reason,visited=visited)
            return self.local.stop(self.done_reason)
        if self.state=='SCAN':
            self.scan_remaining-=abs(wrap(pose.yaw-self.last_yaw))
            self.last_yaw=pose.yaw
            if self.scan_remaining>.03:
                return self.local.command(pose,[pose.x+np.cos(pose.yaw+1.2),
                                                pose.y+np.sin(pose.yaw+1.2)],points,moving,dt,speed_scale=0)
            self.state='EXPLORE'; self.last_plan=-10
        self.last_yaw=pose.yaw
        if self.active_target is not None and not self.active_target.visited:
            target=self.active_target
            distance=pose.distance(target.position)
            heading=wrap(atan2(target.position[1]-pose.y,target.position[0]-pose.x)-pose.yaw)
            if distance<cfg['visit_distance']:
                if abs(heading)>.2:
                    self.hold_since=None
                    return self.local.command(pose,target.position,points,moving,dt,speed_scale=0)
                if frame.time-target.last_seen<.4:
                    self.hold_since=self.hold_since or frame.time
                    if frame.time-self.hold_since>=cfg['visit_hold_seconds']:
                        target.visited=True
                        self.event(frame.time,'target_visited',label=target.label,position=target.position.tolist(),
                                   robot_pose=[pose.x,pose.y,pose.yaw])
                        self.active_target=None; self.hold_since=None; self.last_plan=-10
                        self.start_scan()
                    return self.local.stop('verify_target')
                self.hold_since=None
        if frame.time-self.last_plan>=cfg['planning_period'] or not self.path:
            self.plan(frame.time)
        if self.state=='SCAN':
            self.last_yaw=pose.yaw
            return self.local.stop('start_scan')
        if not self.path:
            if self.recovery_goal is not None:
                return self.local.command(pose,self.recovery_goal,points,moving,dt,speed_scale=.5)
            return self.local.stop('no_observed_safe_path')
        goal_xy=self.grid.world(self.path[-1])
        if self.state=='RETURN':
            goal_xy=self.home
        reach_tolerance=.035 if self.state=='RETURN' else .05 if self.state=='APPROACH' else .2
        if pose.distance(goal_xy)<reach_tolerance:
            if self.state=='EXPLORE':
                self.excluded.append((goal_xy,frame.time)); self.no_frontier_scans=0; self.start_scan()
            elif self.state=='APPROACH' and self.active_target is not None:
                # A moving target estimate can shift after the global approach
                # cell was selected. Continue the guarded final approach.
                return self.local.command(pose,self.active_target.position,points,moving,dt,speed_scale=.35)
            self.last_plan=-10
            return self.local.stop('goal_reached')
        world_path = np.array([self.grid.world(c) for c in self.path])
        if self.state=='RETURN':
            world_path[-1]=self.home
        closest=int(np.argmin(np.linalg.norm(world_path-[pose.x,pose.y],axis=1)))
        lookahead=closest
        distance=0
        free=self.grid.traversable(cfg['robot_radius']+cfg['safety_margin']+.03)
        while lookahead+1<len(world_path) and distance<.65:
            from .mapping import line_cells
            candidate=self.path[lookahead+1]
            if any(not free[y,x] for x,y in line_cells(self.grid.cell((pose.x,pose.y)),candidate)):
                break
            distance+=np.linalg.norm(world_path[lookahead+1]-world_path[lookahead]); lookahead+=1
        command=self.local.command(pose,world_path[lookahead],points,moving,dt,
                                   speed_scale=.55 if self.localizer.uncertainty>.2 else 1)
        if pose.distance(self.anchor)>.15:
            self.anchor=np.array([pose.x,pose.y]); self.stall_since=None
        elif self.stall_since is None:
            self.stall_since=frame.time
        elif frame.time-self.stall_since>8:
            if self.state=='EXPLORE':
                self.excluded.append((goal_xy,frame.time))
            self.last_plan=-10; self.stall_since=None
            self.event(frame.time,'replan_after_wait',state=self.state)
        return command

    def snapshot(self, time):
        p=self.localizer.pose
        return dict(time=float(time),state=self.state,pose=[p.x,p.y,p.yaw],
                    uncertainty=self.localizer.uncertainty,scan_corrections=self.localizer.corrections,
                    home_path_m=self.last_home_cost if np.isfinite(self.last_home_cost) else None,
                    command=list(self.local.velocity),control_reason=self.local.reason,
                    goal=self.grid.world(self.goal).tolist() if self.goal is not None else None,
                    targets=[dict(label=t.label,position=t.position.tolist(),confirmed=t.confirmed,
                                  visited=t.visited,observations=t.observations) for t in self.registry.targets],
                    done_reason=self.done_reason,rejected_lidar_rays=self.rejected_rays)
