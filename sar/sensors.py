"""Reject isolated near-range spikes without marking their rays free.

Demo obstacles are wider than a few millimeters near the robot. Thin-wire
hazards need raw measurements plus additional sensor evidence.
"""
import numpy as np


def sanitize_lidar(ranges):
    filtered=np.array(ranges,dtype=float,copy=True)
    left,right=np.roll(filtered,1),np.roll(filtered,-1)
    isolated=(filtered<.4)&(filtered<left*.5)&(filtered<right*.5)
    filtered[isolated]=np.nan
    return filtered,int(isolated.sum())
