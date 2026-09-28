"""Webots adapter: reads only robot devices, never Supervisor or scenario.json."""
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
from controller import Robot

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from sar.config import load_config
from sar.models import SensorFrame
from sar.mission import Mission
from sar.artifacts import write_rgb_png


def main():
    robot=Robot()
    config=load_config()
    mission=Mission(config)
    basic=int(robot.getBasicTimeStep())
    period=basic*4
    left=robot.getDevice('left wheel motor'); right=robot.getDevice('right wheel motor')
    for motor in (left,right):
        motor.setPosition(float('inf')); motor.setVelocity(0)
    encoders=[robot.getDevice(name) for name in ('left wheel sensor','right wheel sensor')]
    lidar=robot.getDevice('lidar'); camera=robot.getDevice('camera')
    gyro=robot.getDevice('gyro')
    for device in (*encoders,lidar,camera): device.enable(period)
    gyro.enable(basic)
    emitter=robot.getDevice('mission events')
    n=lidar.getHorizontalResolution(); fov=lidar.getFov()
    angles=np.linspace(fov/2,-fov/2,n,endpoint=False)-fov/(2*n)
    directory=ROOT/'output/runs'/os.environ.get('SAR_RUN_ID',time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir(parents=True,exist_ok=True)
    event_index=0; last_log=-1; last_dump=-10
    gyro_integral=0.0; previous_gyro=None; tick=0
    (directory/'robot_config.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    with (directory/'telemetry.jsonl').open('w',encoding='utf-8') as telemetry, \
         (directory/'events.jsonl').open('w',encoding='utf-8') as event_log:
        while robot.step(basic)!=-1:
            rate=gyro.getValues()[2]
            gyro_integral+=(rate if previous_gyro is None else .5*(rate+previous_gyro))*basic/1000
            previous_gyro=rate; tick+=1
            if tick%4:
                continue
            rgb=np.frombuffer(camera.getImage(),dtype=np.uint8).reshape(camera.getHeight(),camera.getWidth(),4)[:,:,2::-1].copy()
            frame=SensorFrame(robot.getTime(),encoders[0].getValue(),encoders[1].getValue(),
                              np.array(lidar.getRangeImage()),angles,rgb,camera.getFov(),gyro_z=rate,gyro_delta=gyro_integral)
            gyro_integral=0.0
            try:
                command=mission.step(frame)
            except Exception as error:
                left.setVelocity(0); right.setVelocity(0)
                emitter.send(json.dumps(dict(kind='controller_error',error=str(error))).encode())
                raise
            speeds=mission.local.wheels(command)
            left.setVelocity(speeds[0]); right.setVelocity(speeds[1])
            for event in mission.events[event_index:]:
                payload=json.dumps(event,ensure_ascii=False,allow_nan=False)
                emitter.send(payload.encode('utf-8')); event_log.write(payload+'\n'); event_log.flush()
                print(payload,flush=True)
            event_index=len(mission.events)
            if frame.time-last_log>=1:
                snapshot=mission.snapshot(frame.time)
                snapshot['encoders']=[frame.left_encoder,frame.right_encoder]
                finite=frame.ranges[np.isfinite(frame.ranges)]
                snapshot['lidar_min']=float(finite.min()) if len(finite) else None
                snapshot['gyro']=gyro.getValues()
                telemetry.write(json.dumps(snapshot,allow_nan=False)+'\n'); telemetry.flush()
                last_log=frame.time
            if frame.time-last_dump>=10 or mission.state in ('DONE','FAILED'):
                np.savez_compressed(directory/'map.npz',log_odds=mission.grid.log_odds,
                                    seen=mission.grid.seen,origin=mission.grid.origin,
                                    resolution=mission.grid.resolution)
                write_rgb_png(directory/'final_camera.png',rgb)
                np.savez_compressed(directory/'latest_scan.npz',ranges=frame.ranges,angles=frame.angles,
                                    pose=[mission.localizer.pose.x,mission.localizer.pose.y,mission.localizer.pose.yaw])
                last_dump=frame.time
        left.setVelocity(0); right.setVelocity(0)


if __name__=='__main__': main()
