import ast
import json
from dataclasses import replace
import math
from pathlib import Path
import unittest
import numpy as np

from sar.config import load_config,ROOT
from sar.models import Pose,SensorFrame,Detection
from sar.mapping import OccupancyGrid
from sar.localization import Localizer
from sar.planning import shortest_paths,reconstruct,approach_goal
from sar.control import LocalPlanner
from sar.perception import ColorLidarDetector,TargetRegistry
from sar.mission import Mission
from sar.evaluation import pedestrian_position,mission_success
from sar.sensors import sanitize_lidar


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config()

    def frame(self,time=1,ranges=None):
        return SensorFrame(time,0,0,np.full(360,np.inf) if ranges is None else ranges,
                           np.linspace(math.pi,-math.pi,360,endpoint=False))

    def test_odometry_straight_and_turn(self):
        loc=Localizer(self.cfg); p0=replace(loc.pose)
        loc.predict(0,0); loc.predict(1,1)
        self.assertAlmostEqual(loc.pose.x,p0.x+.075)
        self.assertAlmostEqual(loc.pose.y,p0.y)
        loc.predict(0,2)
        self.assertAlmostEqual(loc.pose.yaw,.15/.32)

    def test_scan_matching_reduces_rotation_drift_in_room(self):
        cfg=dict(self.cfg,scan_matching=True)
        angles=np.linspace(math.pi,-math.pi,360,endpoint=False)
        def scan(yaw):
            a=angles+yaw
            return np.minimum(4/np.maximum(abs(np.cos(a)),1e-8),
                              3/np.maximum(abs(np.sin(a)),1e-8))
        loc=Localizer(cfg); loc.pose=Pose(0,0,0)
        loc.correct(scan(0),angles)
        loc.pose=Pose(0,0,.24)
        loc.correct(scan(.2),angles)
        self.assertLess(abs(loc.pose.yaw-.2),.04)

    def test_infinite_ray_is_free_without_fake_endpoint(self):
        grid=OccupancyGrid(self.cfg)
        grid.update(Pose(0,0,0),np.array([np.inf]),np.array([0]))
        end=grid.cell((4.9,0))
        self.assertTrue(grid.seen[end[1],end[0]])
        self.assertFalse(np.any(grid.log_odds>0))

    def test_nan_ray_does_not_create_free_space(self):
        grid=OccupancyGrid(self.cfg)
        grid.update(Pose(0,0,0),np.array([np.nan]),np.array([0]))
        cell=grid.cell((1,0))
        self.assertFalse(grid.seen[cell[1],cell[0]])

    def test_a_star_does_not_cut_blocked_corner(self):
        free=np.ones((4,4),bool); free[0,1]=False; free[1,0]=False
        distances,_=shortest_paths(free,(0,0),(1,1))
        self.assertNotIn((1,1),distances)

    def test_planner_routes_around_wall(self):
        free=np.ones((9,9),bool); free[1:8,4]=False
        distances,parents=shortest_paths(free,(1,4),(7,4))
        path=reconstruct(parents,(1,4),(7,4))
        self.assertTrue(path)
        self.assertTrue(all(free[y,x] for x,y in path))
        self.assertGreater(distances[(7,4)],6)

    def test_target_needs_multiple_viewpoints_and_merges_instance(self):
        registry=TargetRegistry()
        d=Detection('red',np.array([2.,0]),.9,0)
        for t in range(5): registry.update([d],Pose(0,0,0),t)
        self.assertFalse(registry.targets[0].confirmed)
        registry.update([d],Pose(.2,0,0),6)
        self.assertTrue(registry.targets[0].confirmed)
        self.assertEqual(len(registry.targets),1)

    def test_real_rgb_and_lidar_generate_detection(self):
        rgb=np.full((240,320,3),90,dtype=np.uint8)
        rgb[70:180,145:175]=[230,35,40]
        f=self.frame(); f.rgb=rgb
        f.ranges[abs(f.angles)<.08]=1.82
        detected=ColorLidarDetector(self.cfg).detect(f,Pose(0,0,0))
        self.assertEqual(detected[0].label,'red')
        self.assertAlmostEqual(detected[0].position[0],2,delta=.08)

    def test_foreground_lidar_does_not_become_distant_colored_target(self):
        rgb=np.full((240,320,3),90,dtype=np.uint8)
        rgb[95:135,150:172]=[230,35,40]
        f=self.frame(); f.rgb=rgb
        f.ranges[abs(f.angles)<.12]=.6
        self.assertEqual(ColorLidarDetector(self.cfg).detect(f,Pose(0,0,0)),[])

    def test_approaching_obstacle_can_trigger_reverse(self):
        planner=LocalPlanner(self.cfg)
        v,_=planner.command(Pose(0,0,0),[2,0],np.array([[.9,0]]),
                            [(np.array([1.1,0]),np.array([-.4,0]))],.128)
        self.assertLess(v,0)

    def test_local_planner_stops_instead_of_touching_obstacle(self):
        planner=LocalPlanner(self.cfg)
        cmd=planner.command(Pose(0,0,0),[2,0],np.array([[.24,0]]),[],.128)
        self.assertEqual(cmd,(0,0))

    def test_rotation_only_does_not_keep_previous_forward_velocity(self):
        planner=LocalPlanner(self.cfg); planner.velocity=(.4,0)
        v,_=planner.command(Pose(0,0,0),[0,1],np.empty((0,2)),[],.128,speed_scale=0)
        self.assertEqual(v,0)

    def test_gyro_reduces_encoder_turn_error(self):
        loc=Localizer(self.cfg); loc.predict(0,0)
        loc.predict(-1,1,dt=1,gyro_z=.3)
        self.assertLess(abs(loc.pose.yaw-.3),abs(.15/.32-.3))

    def test_high_rate_gyro_delta_is_not_smoothed_twice(self):
        loc=Localizer(self.cfg); loc.predict(0,0)
        loc.predict(-1,1,dt=1,gyro_z=.9,gyro_delta=.2)
        self.assertAlmostEqual(loc.pose.yaw,.2)

    def test_inflation_band_has_observed_recovery_goal(self):
        mission=Mission(self.cfg)
        mission.state='EXPLORE'
        mission.grid.seen[:]=True; mission.grid.log_odds[:]=-3
        p=mission.localizer.pose
        obstacle=mission.grid.cell((p.x-.3,p.y))
        mission.grid.log_odds[obstacle[1],obstacle[0]]=3
        mission.plan(1)
        self.assertIsNotNone(mission.recovery_goal)
        free=mission.grid.traversable(.38)
        x,y=mission.grid.cell(mission.recovery_goal)
        self.assertTrue(free[y,x])

    def test_bad_scan_fails_closed(self):
        mission=Mission(self.cfg)
        cmd=mission.step(self.frame(ranges=np.full(360,np.nan)))
        self.assertEqual(cmd,(0,0))
        self.assertEqual(mission.local.reason,'invalid_lidar')

    def test_isolated_lidar_spike_stays_unknown(self):
        raw=np.full(360,2.0); raw[60]=.18
        filtered,rejected=sanitize_lidar(raw)
        self.assertEqual(rejected,1)
        self.assertTrue(np.isnan(filtered[60]))
        raw[59:62]=.18
        filtered,rejected=sanitize_lidar(raw)
        self.assertEqual(rejected,0)
        self.assertTrue(np.all(filtered[59:62]==.18))

    def test_time_limit_does_not_report_success(self):
        mission=Mission(self.cfg)
        self.assertEqual(mission.step(self.frame(time=601)),(0,0))
        self.assertEqual(mission.state,'FAILED')

    def test_return_without_all_targets_is_incomplete(self):
        mission=Mission(self.cfg); mission.state='RETURN'
        mission.step(self.frame())
        self.assertEqual(mission.done_reason,'returned_incomplete')

    def test_return_cell_tolerance_cannot_stop_before_home(self):
        mission=Mission(self.cfg); mission.state='RETURN'
        mission.localizer.pose=Pose(mission.home[0],mission.home[1]+.24,0)
        command=mission.step(self.frame())
        self.assertNotEqual(mission.local.reason,'goal_reached')
        self.assertNotEqual(command,(0,0))

    def test_robot_does_not_read_ground_truth(self):
        files=list((ROOT/'sar').glob('*.py'))
        files.remove(ROOT/'sar/evaluation.py')
        files.append(ROOT/'controllers/rescue_controller/rescue_controller.py')
        for path in files:
            text=path.read_text(encoding='utf-8')
            self.assertNotIn('scenario.json',text.split('"""')[-1] if path.name=='rescue_controller.py' else text)
            tree=ast.parse(text)
            for node in ast.walk(tree):
                if isinstance(node,ast.ImportFrom):
                    self.assertNotEqual(node.module,'sar.evaluation')
                    self.assertNotIn('Supervisor',[n.name for n in node.names])

    def test_pedestrian_bounces_at_endpoints(self):
        p=dict(start=[0,0],end=[2,0],speed=.5)
        np.testing.assert_allclose(pedestrian_position(p,4),[2,0])
        np.testing.assert_allclose(pedestrian_position(p,8),[0,0])

    def test_evaluator_requires_completion_and_all_safety_conditions(self):
        event=dict(kind='mission_finished',status='complete')
        arguments=[4,4,.2,.1,0,250,600]
        self.assertTrue(mission_success(event,*arguments))
        for stopped in (None,dict(kind='manual_validation_stop'),
                        dict(kind='controller_error'),
                        dict(kind='mission_finished',status='returned_incomplete')):
            self.assertFalse(mission_success(stopped,*arguments))
        for index,value in ((0,3),(2,.4),(3,.4),(4,1),(5,601)):
            invalid=arguments.copy(); invalid[index]=value
            self.assertFalse(mission_success(event,*invalid))

    def test_medium_world_static_geometry_has_routes_to_every_target(self):
        scenario=json.loads((ROOT/'config/scenario.json').read_text())
        grid=OccupancyGrid(self.cfg)
        yy,xx=np.indices(grid.seen.shape)
        xy=grid.world(np.stack((xx,yy),axis=-1))
        grid.seen[:]=True; grid.log_odds[:]=-3
        for x,y,sx,sy in scenario['walls']+scenario['debris']:
            inside=(abs(xy[:,:,0]-x)<=sx/2+.05)&(abs(xy[:,:,1]-y)<=sy/2+.05)
            grid.log_odds[inside]=3
        for target in scenario['targets']:
            grid.log_odds[np.linalg.norm(xy-target['position'],axis=-1)<=scenario['target_radius']+.05]=3
        free=grid.traversable(.38)
        distances,_=shortest_paths(free,grid.cell(self.cfg['initial_pose'][:2]))
        for target in scenario['targets']:
            goal=approach_goal(grid,np.array(target['position']),distances)
            self.assertIsNotNone(goal,msg=target['id'])
            self.assertLess(np.linalg.norm(grid.world(goal)-target['position']),.85)


if __name__=='__main__': unittest.main()
