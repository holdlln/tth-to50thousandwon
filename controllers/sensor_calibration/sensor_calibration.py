"""Short sensor-axis/odometry calibration only; not used in rescue world."""
import json
import os
from pathlib import Path
import sys
import numpy as np
from controller import Robot
ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT))
from sar.config import load_config
from sar.localization import Localizer

robot=Robot(); cfg=load_config(); loc=Localizer(cfg)
period=int(robot.getBasicTimeStep())*4
left=robot.getDevice('left wheel motor'); right=robot.getDevice('right wheel motor')
for m in (left,right): m.setPosition(float('inf')); m.setVelocity(0)
encoders=[robot.getDevice(n) for n in ('left wheel sensor','right wheel sensor')]
lidar=robot.getDevice('lidar'); gyro=robot.getDevice('gyro'); camera=robot.getDevice('camera')
for d in (*encoders,lidar,gyro,camera): d.enable(period)
emitter=robot.getDevice('mission events')
directory=ROOT/'output/runs'/os.environ['SAR_RUN_ID']; directory.mkdir(parents=True,exist_ok=True)
with (directory/'calibration.jsonl').open('w') as output:
    index=0
    while robot.step(period)!=-1:
        now=robot.getTime(); values=[e.getValue() for e in encoders]
        loc.predict(*values)
        p=loc.pose
        output.write(json.dumps(dict(time=now,encoders=values,pose=[p.x,p.y,p.yaw],gyro=gyro.getValues()))+'\n'); output.flush()
        if index in (0,50):
            np.savez(directory/f'sensors_{index}.npz',ranges=lidar.getRangeImage(),
                     rgb=np.frombuffer(camera.getImage(),dtype=np.uint8).reshape(240,320,4)[:,:,2::-1])
        v,w=(.25,0) if now<7 else (0,.7) if now<14 else (0,0)
        left.setVelocity((v-w*.32/2)/.075); right.setVelocity((v+w*.32/2)/.075)
        if now>=14:
            emitter.send(json.dumps(dict(kind='mission_finished',status='calibration')).encode())
        index+=1
