"""Small lossless RGB PNG writer; works with Unicode paths without GUI APIs."""
from pathlib import Path
import struct
import zlib
import numpy as np


def write_rgb_png(path,rgb):
    rgb=np.ascontiguousarray(rgb,dtype=np.uint8)
    height,width,channels=rgb.shape
    if channels!=3: raise ValueError('Expected RGB image')
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    raw=b''.join(b'\0'+row.tobytes() for row in rgb)
    encoded=(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,2,0,0,0))+
             chunk(b'IDAT',zlib.compress(raw,6))+chunk(b'IEND',b''))
    Path(path).write_bytes(encoded)
