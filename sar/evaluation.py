"""Ground-truth utility for evaluator/tests only; never imported by Mission."""
import numpy as np


def mission_success(finished, visited_count, target_count, home_error, angle_error,
                    collisions, elapsed, time_limit):
    """An interrupted/error run cannot pass, even when near home by chance."""
    return bool(finished and finished.get('kind') == 'mission_finished'
                and finished.get('status') == 'complete'
                and visited_count == target_count and home_error < .35
                and angle_error < .3 and collisions == 0 and elapsed <= time_limit)


def pedestrian_position(person, time):
    start,end=np.array(person['start']),np.array(person['end'])
    distance=np.linalg.norm(end-start)
    if distance<=0: return start
    progress=(time*person['speed']/distance+person.get('phase',0))%2
    fraction=progress if progress<=1 else 2-progress
    return start+(end-start)*fraction


def box_clearance(xy,box,radius):
    x,y,sx,sy=box
    outside=np.maximum(np.abs(np.array(xy)-[x,y])-[sx/2,sy/2],0)
    return float(np.linalg.norm(outside)-radius)


def scene_clearance(xy,time,scenario,radius):
    values=[box_clearance(xy,b,radius) for b in scenario['walls']+scenario['debris']]
    values += [float(np.linalg.norm(np.array(xy)-t['position'])-radius-scenario['target_radius'])
               for t in scenario['targets']]
    values += [float(np.linalg.norm(np.array(xy)-pedestrian_position(p,time))-radius-scenario['pedestrian_radius'])
               for p in scenario['pedestrians']]
    return min(values)
