"""Log-odds occupancy grid; no scenario dimensions or geometry are read here."""
from math import ceil
import numpy as np
from .models import Pose


def line_cells(start, end):
    x, y = start
    ex, ey = end
    dx, dy = abs(ex - x), abs(ey - y)
    sx, sy = (1 if x < ex else -1), (1 if y < ey else -1)
    error = dx - dy
    while True:
        yield x, y
        if x == ex and y == ey:
            break
        e = 2 * error
        if e > -dy:
            error -= dy; x += sx
        if e < dx:
            error += dx; y += sy


def dilate(mask, radius):
    result = mask.copy()
    h, w = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            y0, y1 = max(0, dy), min(h, h + dy)
            x0, x1 = max(0, dx), min(w, w + dx)
            result[y0:y1, x0:x1] |= mask[y0-dy:y1-dy, x0-dx:x1-dx]
    return result


class OccupancyGrid:
    def __init__(self, config):
        self.resolution = config['map_resolution']
        self.extent = config['map_extent']
        self.origin = np.array([-self.extent / 2] * 2)
        size = int(ceil(self.extent / self.resolution))
        self.log_odds = np.zeros((size, size), dtype=np.float32)
        self.seen = np.zeros((size, size), dtype=bool)
        self.visits = np.zeros((size, size), dtype=np.int32)
        self.max_range = config['lidar_max_range']
        self.min_range = config['lidar_min_range']

    def world_cells(self, xy):
        return np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)

    def cell(self, xy):
        return tuple(self.world_cells(xy))

    def world(self, cell):
        return self.origin + (np.asarray(cell) + 0.5) * self.resolution

    def inside(self, cells):
        cells = np.asarray(cells)
        return np.all((cells >= 0) & (cells < self.log_odds.shape[0]), axis=-1)

    def update(self, pose: Pose, ranges, angles):
        start = self.cell((pose.x, pose.y))
        if not self.inside(start):
            raise ValueError('Robot left allocated map: increase map_extent, not scenario knowledge')
        self.visits[start[1], start[0]] += 1
        # Skip NaN/zero/negative rather than falsely marking their rays free.
        for distance, angle in zip(ranges[::2], angles[::2]):
            if np.isnan(distance) or distance < self.min_range:
                continue
            hit = np.isfinite(distance) and distance < self.max_range - 0.02
            d = min(distance, self.max_range)
            end = self.cell((pose.x + d * np.cos(pose.yaw + angle),
                             pose.y + d * np.sin(pose.yaw + angle)))
            cells = list(line_cells(start, end))
            for x, y in cells[:-1] if hit else cells:
                if 0 <= x < self.log_odds.shape[1] and 0 <= y < self.log_odds.shape[0]:
                    self.log_odds[y, x] -= 0.22
                    self.seen[y, x] = True
            if hit and self.inside(end):
                x, y = end
                self.log_odds[y, x] += 0.65
                self.seen[y, x] = True
        np.clip(self.log_odds, -3, 3, out=self.log_odds)
        # Body footprint is measured free, including between sparse scan rays.
        x, y = start
        self.seen[y-1:y+2, x-1:x+2] = True
        self.log_odds[y-1:y+2, x-1:x+2] = -3

    def traversable(self, clearance):
        inflated = dilate(self.log_odds > 0.5, int(ceil(clearance / self.resolution)))
        return self.seen & (self.log_odds < 0.1) & ~inflated

    def frontiers(self, traversable):
        unknown = ~self.seen
        neighbors = np.zeros_like(unknown)
        neighbors[1:] |= unknown[:-1]; neighbors[:-1] |= unknown[1:]
        neighbors[:, 1:] |= unknown[:, :-1]; neighbors[:, :-1] |= unknown[:, 1:]
        return traversable & neighbors
