from heapq import heappop, heappush
from math import hypot, sqrt
import numpy as np

STEPS = ((1,0,1),(-1,0,1),(0,1,1),(0,-1,1),
         (1,1,sqrt(2)),(1,-1,sqrt(2)),(-1,1,sqrt(2)),(-1,-1,sqrt(2)))


def shortest_paths(free, start, goal=None):
    """Dijkstra to all cells or A* to a single cell; forbid corner cutting."""
    h, w = free.shape
    if not (0 <= start[0] < w and 0 <= start[1] < h) or not free[start[1],start[0]]:
        return {}, {}
    distances, parents = {start: 0.0}, {}
    queue = [(0.0, 0.0, start)]
    while queue:
        _, distance, cell = heappop(queue)
        if distance > distances.get(cell, float('inf')) + 1e-8:
            continue
        if cell == goal:
            break
        x, y = cell
        for dx, dy, cost in STEPS:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h and free[ny,nx]):
                continue
            if dx and dy and not (free[y,nx] and free[ny,x]):
                continue
            next_cell, candidate = (nx, ny), distance + cost
            if candidate < distances.get(next_cell, float('inf')):
                distances[next_cell] = candidate
                parents[next_cell] = cell
                heuristic = hypot(nx-goal[0],ny-goal[1]) if goal is not None else 0
                heappush(queue, (candidate+heuristic,candidate,next_cell))
    return distances, parents


def reconstruct(parents, start, goal):
    if goal != start and goal not in parents:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(parents[path[-1]])
    return path[::-1]


class RescueFrontierPolicy:
    """Editable information / travel / safe-return tradeoff."""
    def score(self, information, travel, return_cost, uncertainty, revisit):
        return 1.8 * np.log1p(information) - 0.7 * travel - 0.15 * return_cost - 3 * uncertainty - 0.25 * revisit


def frontier_goal(grid, free, distances, home_distances, policy, uncertainty, excluded):
    frontier = grid.frontiers(free)
    candidates = [(int(x),int(y)) for y,x in np.argwhere(frontier)
                  if (int(x),int(y)) in distances]
    if not candidates:
        return None
    # Sample spatial buckets to keep planning cost bounded, without choosing
    # frontier cells outside the explored connected component.
    buckets = {}
    for cell in candidates:
        key = (cell[0]//5, cell[1]//5)
        if key not in buckets or distances[cell] < distances[buckets[key]]:
            buckets[key] = cell
    scored = []
    for cell in buckets.values():
        xy = grid.world(cell)
        if any(np.linalg.norm(xy - pos) < 0.65 for pos, _ in excluded):
            continue
        x,y = cell; r = 10
        patch = ~grid.seen[max(0,y-r):y+r+1,max(0,x-r):x+r+1]
        information = int(np.count_nonzero(patch))
        travel = distances[cell]*grid.resolution
        home = home_distances.get(cell,float('inf'))*grid.resolution
        if not np.isfinite(home):
            continue
        revisit = grid.visits[max(0,y-3):y+4,max(0,x-3):x+4].sum()/20
        scored.append((policy.score(information,travel,home,uncertainty,revisit),cell))
    return max(scored)[1] if scored else None


def approach_goal(grid, target, distances, distance=0.72):
    center = grid.cell(target)
    r = int(np.ceil((distance+0.15)/grid.resolution))
    candidates = []
    for y in range(center[1]-r,center[1]+r+1):
        for x in range(center[0]-r,center[0]+r+1):
            cell = (x,y)
            if cell not in distances:
                continue
            d = np.linalg.norm(grid.world(cell)-target)
            if distance-0.12 <= d <= distance:
                # Require a clear observed line of sight to the target surface.
                from .mapping import line_cells
                ray = list(line_cells(cell,center))
                if any(not grid.inside(c) or grid.log_odds[c[1],c[0]] > .5
                       for c in ray[:-3]):
                    continue
                candidates.append((distances[cell]+abs(d-distance)*5,cell))
    return min(candidates)[1] if candidates else None
