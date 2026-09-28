"""Export engineering map drawings and independent validation evidence."""
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1]


def font(size):
    try: return ImageFont.truetype('C:/Windows/Fonts/arial.ttf',size)
    except OSError: return ImageFont.load_default()


def records(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line]


def map_image(scenario,title,trajectory=None,estimate=None):
    image=Image.new('RGB',(1000,930),'#edf1f5'); draw=ImageDraw.Draw(image)
    scale=70
    def point(xy): return (int(500+xy[0]*scale),int(470-xy[1]*scale))
    draw.rounded_rectangle((35,75,965,895),radius=18,fill='white')
    draw.text((45,22),title,fill='#172e43',font=font(25))
    for x in range(-6,7):
        a,b=point((x,-5)),point((x,5)); draw.line([a,b],fill='#e2e7ec')
        draw.text((a[0]-5,a[1]+12),str(x),fill='#536579',font=font(13))
    for y in range(-5,6):
        a,b=point((-6,y)),point((6,y)); draw.line([a,b],fill='#e2e7ec')
        draw.text((a[0]-25,a[1]-5),str(y),fill='#536579',font=font(13))
    for x,y,sx,sy in scenario['walls']+scenario['debris']:
        a,b=point((x-sx/2,y+sy/2)),point((x+sx/2,y-sy/2))
        draw.rectangle([a,b],fill='#576574')
    if trajectory is not None and len(trajectory)>1:
        draw.line([point(p) for p in trajectory],fill='#087d8b',width=4)
    if estimate is not None and len(estimate)>1:
        draw.line([point(p) for p in estimate],fill='#bd57ac',width=2)
    r=int(scenario['target_radius']*scale)
    for target in scenario['targets']:
        x,y=point(target['position']); draw.ellipse((x-r,y-r,x+r,y+r),fill=tuple(target['rgb']),outline='#333333',width=2)
        draw.text((x+18,y-15),target['id'],fill='#233647',font=font(18))
    for person in scenario['pedestrians']:
        a,b=point(person['start']),point(person['end'])
        draw.line([a,b],fill='#b36739',width=4)
        r=int(scenario['pedestrian_radius']*scale)
        draw.ellipse((a[0]-r,a[1]-r,a[0]+r,a[1]+r),outline='#b36739',width=3)
        draw.text((a[0],a[1]+20),person['id']+' patrol',fill='#965c32',font=font(14))
    config=json.loads((ROOT/'config/robot.json').read_text())
    x,y=point(config['initial_pose'][:2]); draw.ellipse((x-12,y-12,x+12,y+12),fill='#0e8c8c')
    draw.text((x-50,y+20),'Start / Return',fill='#087d8b',font=font(16))
    draw.text((45,865),'Coordinates: m | colored circles: targets | brown lines: moving obstacles',fill='#536579',font=font(17))
    return image


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--run-id'); args=parser.parse_args()
    scenario=json.loads((ROOT/'config/scenario.json').read_text())
    output=ROOT/'output/validation'; output.mkdir(parents=True,exist_ok=True)
    title=f"Rescue scenario: {len(scenario['targets'])} targets, {len(scenario['pedestrians'])} moving obstacles"
    map_image(scenario,title).save(output/'scenario_map.png')
    if not args.run_id: return
    run=ROOT/'output/runs'/args.run_id
    truth=records(run/'ground_truth.jsonl'); telemetry=records(run/'telemetry.jsonl')
    evaluation=json.loads((run/'evaluation.json').read_text())
    for name in ('robot_config.json','evaluation.json','events.jsonl','telemetry.jsonl','ground_truth.jsonl','map.npz','final_camera.png'):
        if (run/name).exists(): shutil.copy2(run/name,output/name)
    actual=np.array([r['pose'][:2] for r in truth]); estimated=np.array([r['pose'][:2] for r in telemetry])
    times=np.array([r['time'] for r in truth]); errors=[]
    for row in telemetry:
        i=int(np.argmin(abs(times-row['time'])))
        errors.append(float(np.linalg.norm(np.array(row['pose'][:2])-actual[i])))
    image=map_image(scenario,f"{args.run_id}: actual teal / estimated magenta",actual,estimated)
    image.save(output/'run_analysis.png')
    if (run/'map.npz').exists():
        z=np.load(run/'map.npz'); seen=z['seen']; odds=z['log_odds']; res=float(z['resolution'])
        data=np.full((*seen.shape,3),[140,153,166],dtype=np.uint8)
        data[seen&(odds<.1)]=[244,248,251]; data[seen&(odds>.5)]=[28,41,53]
        x0,y0=((np.array([-6.3,-5.3])-z['origin'])/res).astype(int)
        x1,y1=((np.array([6.3,5.3])-z['origin'])/res).astype(int)
        Image.fromarray(data[y0:y1,x0:x1][::-1]).resize((1008,848),Image.Resampling.NEAREST).save(output/'robot_map.png')
    summary=dict(run_id=args.run_id,maximum_sampled_position_error_m=max(errors),
                 mean_sampled_position_error_m=float(np.mean(errors)),evaluation=evaluation)
    (output/'analysis.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))


if __name__=='__main__': main()
