"""Moves obstacles and evaluates independently. No truth channel to the robot."""
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
from controller import Supervisor

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from sar.config import load_config
from sar.evaluation import pedestrian_position,scene_clearance,mission_success


def main():
    supervisor=Supervisor(); step=int(supervisor.getBasicTimeStep())
    config=load_config()
    scenario=json.loads((ROOT/'config/scenario.json').read_text(encoding='utf-8'))
    robot=supervisor.getFromDef('RESCUE_ROBOT')
    receiver=supervisor.getDevice('mission events'); receiver.enable(step)
    pedestrians=[(p,supervisor.getFromDef(p['id']).getField('translation')) for p in scenario['pedestrians']]
    directory=ROOT/'output/runs'/os.environ.get('SAR_RUN_ID',time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir(parents=True,exist_ok=True)
    visited={}; minimum=float('inf'); distance=0; previous=None; collisions=0; in_collision=False
    finished=None; last_save=-1; last_capture=-1
    truth_path=(directory/'ground_truth.jsonl').open('w',encoding='utf-8')
    while supervisor.step(step)!=-1:
        now=supervisor.getTime()
        for person,field in pedestrians:
            xy=pedestrian_position(person,now); field.setSFVec3f([*xy,0])
        xy=np.array(robot.getPosition()[:2])
        if previous is not None: distance+=float(np.linalg.norm(xy-previous))
        previous=xy
        clearance=scene_clearance(xy,now,scenario,config['robot_radius'])
        minimum=min(minimum,clearance)
        # Conservative circular footprint plus actual non-floor contacts.
        contact=any(p.point[2]>.045 for p in robot.getContactPoints(True))
        collision=clearance<-.015 or contact
        if collision and not in_collision: collisions+=1
        in_collision=collision
        while receiver.getQueueLength():
            event=json.loads(receiver.getString()); receiver.nextPacket()
            if event['kind']=='target_visited':
                estimate=np.array(event['position'])
                for target in scenario['targets']:
                    if (target['label']==event['label'] and np.linalg.norm(estimate-target['position'])<.55
                        and np.linalg.norm(xy-target['position'])<config['visit_distance']+.1):
                        visited.setdefault(target['id'],now)
            if event['kind']=='mission_finished': finished=event
            if event['kind']=='controller_error': finished=event
        if now-last_save>=1:
            matrix=robot.getOrientation()
            truth_yaw=float(np.arctan2(matrix[3],matrix[0]))
            truth_path.write(json.dumps(dict(time=now,pose=robot.getPosition(),yaw=truth_yaw,clearance=clearance,
                                             distance=distance,collisions=collisions,visited=visited))+'\n')
            truth_path.flush(); last_save=now
        supervisor.setLabel(0,f"{len(visited)}/{len(scenario['targets'])} visited | {now:.0f}s | collisions {collisions}",
                            .02,.02,.065,0xffffff,0,'Arial')
        if os.environ.get('SAR_CAPTURE')=='1' and now-last_capture>60:
            supervisor.exportImage(str(directory/'scene.png'),100); last_capture=now
        timeout=now>=config['mission_limit_seconds']+1
        if (directory/'STOP').exists():
            finished=dict(kind='manual_validation_stop')
        if finished or timeout:
            home_error=float(np.linalg.norm(xy-config['initial_pose'][:2]))
            orientation=robot.getOrientation()
            yaw=float(np.arctan2(orientation[3],orientation[0]))
            angle_error=float(abs(np.arctan2(np.sin(yaw-config['initial_pose'][2]),np.cos(yaw-config['initial_pose'][2]))))
            success=mission_success(finished,len(visited),len(scenario['targets']),home_error,
                                    angle_error,collisions,now,config['mission_limit_seconds'])
            report=dict(success=success,elapsed_seconds=now,path_length_m=distance,
                        minimum_clearance_m=minimum,collision_episodes=collisions,visited=visited,
                        total_targets=len(scenario['targets']),home_error_m=home_error,
                        home_angle_error_rad=angle_error,controller_result=finished)
            (directory/'evaluation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            supervisor.exportImage(str(directory/'scene.png'),100)
            print('EVALUATION '+json.dumps(report),flush=True)
            truth_path.close()
            if os.environ.get('SAR_BATCH')=='1': supervisor.simulationQuit(0 if success else 1)
            else: supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
            return


if __name__=='__main__': main()
