#!/usr/bin/env python3
"""Generate section 6.5/6.10 global/local costmap illustrations.

The views share an illustrative world and the 6.4 cost palette. Identical
observations/settings make the local window agree with the global view in 6.5;
this does not model a real Nav2 local costmap as a crop of the global costmap.
Routes are simple grid searches, not Nav2 outputs or kinematic simulations.
"""

from heapq import heappop, heappush
from math import exp, hypot

from generate_costmap_figures import (
    INFLATION_RADIUS, INSCRIBED_RADIUS, OUTPUT, RESOLUTION, SCALING, color, svg_open,
)

COLS, ROWS = 40, 30
START, GOAL = (7, 7), (34, 7)
WINDOW = (0, 0, 15, 15)
WALLS = {
    (x, y) for x in range(COLS) for y in range(ROWS)
    if x in (0, COLS - 1) or y in (0, ROWS - 1)
    or (12 <= x <= 27 and 11 <= y <= 22)
}
CART = {(x, y) for x in (11, 12) for y in (6, 7)}
CLOSURE = {(x, y) for x in (12, 13) for y in range(1, 11)}


def costs_for(obstacles):
    costs = {}
    for y in range(ROWS):
        for x in range(COLS):
            distance = min(hypot(x - ox, y - oy) for ox, oy in obstacles) * RESOLUTION
            if distance == 0:
                cost = 254
            elif distance > INFLATION_RADIUS + 1e-9:
                cost = 0
            elif distance <= INSCRIBED_RADIUS + 1e-9:
                cost = 253
            else:
                cost = int(252 * exp(-SCALING * (distance - INSCRIBED_RADIUS)))
            costs[x, y] = cost
    return costs


def route(costs):
    distances, previous, queue = {START: 0.0}, {}, [(0.0, START)]
    while queue:
        distance, point = heappop(queue)
        if distance != distances[point]:
            continue
        if point == GOAL:
            points = [GOAL]
            while points[-1] != START:
                points.append(previous[points[-1]])
            return list(reversed(points))
        x, y = point
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            next_point = x + dx, y + dy
            if costs.get(next_point, 255) >= 253:
                continue
            if dx and dy and (costs.get((x + dx, y), 255) >= 253 or costs.get((x, y + dy), 255) >= 253):
                continue
            candidate = distance + hypot(dx, dy) * (1 + costs[next_point] / 60)
            if candidate < distances.get(next_point, float('inf')):
                distances[next_point] = candidate
                previous[next_point] = point
                heappush(queue, (candidate, next_point))
    raise ValueError('The illustrative goal is unreachable')


def point_xy(point, local):
    cell, left, top = (24, 76, 32) if local else (12, 16, 32)
    x, y = point
    return left + (x + 0.5) * cell, top + (y + 0.5) * cell


def inside_window(point):
    return WINDOW[0] <= point[0] < WINDOW[2] and WINDOW[1] <= point[1] < WINDOW[3]


def base_map(costs, local, title, description):
    parts = svg_open(512, 408, title, description)
    cell, left = (24, 76) if local else (12, 16)
    cols, rows = (15, 15) if local else (COLS, ROWS)
    frame = 'odom | 3.0 m x 3.0 m' if local else 'map | 8.0 m x 6.0 m'
    parts.append(f'<text x="16" y="21" fill="#33445a" font-family="sans-serif" font-size="15">{frame}</text>')
    for y in range(rows):
        for x in range(cols):
            parts.append(f'<rect x="{left + x * cell}" y="{32 + y * cell}" width="{cell}" height="{cell}" '
                         f'fill="{color(costs[x, y])}" stroke="#b8c3cf" stroke-width="0.35"/>')
    if local:
        parts.append(f'<rect x="{left}" y="32" width="360" height="360" fill="none" stroke="#4163c9" stroke-width="3" stroke-dasharray="8 5"/>')
    else:
        parts.append('<rect x="16" y="32" width="180" height="180" fill="none" stroke="#4163c9" stroke-width="3" stroke-dasharray="7 4"/>')
        for label, point in [('A', (23, 2)), ('B', (20, 27))]:
            x, y = point_xy(point, False)
            parts.append(f'<circle cx="{x}" cy="{y}" r="12" fill="#ffffff" stroke="#33445a"/>')
            parts.append(f'<text x="{x}" y="{y + 6}" text-anchor="middle" fill="#25364b" font-family="sans-serif" font-size="18" font-weight="bold">{label}</text>')
    return parts


def line(parts, points, local, stroke, width=4, dashed=False):
    coords = ' '.join(f'{x},{y}' for x, y in (point_xy(point, local) for point in points))
    dash = ' stroke-dasharray="7 5"' if dashed else ''
    parts.append(f'<polyline points="{coords}" fill="none" stroke="#ffffff" stroke-width="{width + 2}" stroke-linejoin="round"{dash}/>')
    parts.append(f'<polyline points="{coords}" fill="none" stroke="{stroke}" stroke-width="{width}" stroke-linejoin="round"{dash}/>')


def cross(parts, point, local):
    x, y = point_xy(point, local)
    parts.append(f'<path d="M{x - 7} {y - 7}L{x + 7} {y + 7}M{x - 7} {y + 7}L{x + 7} {y - 7}" '
                 'stroke="#ffffff" stroke-width="7"/>')
    parts.append(f'<path d="M{x - 7} {y - 7}L{x + 7} {y + 7}M{x - 7} {y + 7}L{x + 7} {y - 7}" '
                 'stroke="#b31d37" stroke-width="4"/>')


def robot_and_goal(parts, local):
    x, y = point_xy(START, local)
    radius = 24 if local else 12
    parts.append(f'<circle cx="{x}" cy="{y}" r="{radius}" fill="#245acf" stroke="#ffffff" stroke-width="2"/>')
    parts.append(f'<text x="{x}" y="{y + 6}" text-anchor="middle" font-family="sans-serif" font-size="18" font-weight="bold" fill="#ffffff">R</text>')
    if not local:
        x, y = point_xy(GOAL, False)
        parts.append(f'<circle cx="{x}" cy="{y}" r="11" fill="#ffffff" stroke="#087d67" stroke-width="3"/>')
        parts.append(f'<text x="{x}" y="{y + 5}" text-anchor="middle" font-family="sans-serif" font-size="16" font-weight="bold" fill="#065b4b">G</text>')


def save(parts, name):
    (OUTPUT / name).write_text('\n'.join(parts + ['</svg>']) + '\n')


def generate():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    normal = costs_for(WALLS | CART)
    closed = costs_for(WALLS | CART | CLOSURE)
    original_path, alternate_path = route(normal), route(closed)
    local_path = []
    for point in original_path:
        if not inside_window(point):
            break
        local_path.append(point)
    collision = next(point for point in local_path if closed[point] >= 253)

    parts = base_map(normal, False, '大域コストマップと全体経路', '大域のmap上で通路Aを通りRからGへ向かう。壁・設備と台車の周囲にコストがあり、青い破線枠は局所窓に対応する範囲。')
    line(parts, original_path, False, '#087d67')
    robot_and_goal(parts, False)
    save(parts, 'costmap-global-overview.svg')

    parts = base_map(normal, True, '局所コストマップと直近の動き', '青枠の範囲を拡大。局所のodom上で壁と台車のコストを評価する。緑は変換した大域経路、紫は短い動きの例。赤い破線の直進候補は台車に衝突する。目的地Gは窓の外。')
    line(parts, local_path, True, '#087d67', width=4)
    line(parts, [START, (11, 7)], True, '#b31d37', width=3, dashed=True)
    cross(parts, (11, 7), True)
    short_motion = local_path[:6]
    line(parts, short_motion, True, '#7542c8', width=6)
    end_x, end_y = point_xy(short_motion[-1], True)
    parts.append(f'<circle cx="{end_x}" cy="{end_y}" r="6" fill="#7542c8" stroke="#ffffff" stroke-width="2"/>')
    robot_and_goal(parts, True)
    save(parts, 'costmap-local-window.svg')

    parts = base_map(closed, True, '局所で通路Aの閉塞を検知', '台車などで通路Aが完全に塞がる。古い大域経路は局所の衝突領域に入り、赤い×で示す。この例では停止する。通路Bは局所窓の外。')
    line(parts, local_path, True, '#087d67', dashed=True)
    cross(parts, collision, True)
    robot_and_goal(parts, True)
    x, y = point_xy(START, True)
    parts.append(f'<rect x="{x - 34}" y="{y + 31}" width="68" height="26" rx="5" fill="#ffffff" stroke="#b31d37"/>')
    parts.append(f'<text x="{x}" y="{y + 50}" text-anchor="middle" fill="#b31d37" font-family="sans-serif" font-size="18" font-weight="bold">STOP</text>')
    save(parts, 'costmap-local-blocked.svg')

    parts = base_map(normal, False, '大域に閉塞が反映されていない場合', '大域のコストは閉塞前のままで、経路は通路Aを選び続ける。赤い×は局所で判明した衝突位置の注記で、大域に障害物セルが追加されたことを意味しない。')
    line(parts, original_path, False, '#087d67')
    cross(parts, collision, False)
    robot_and_goal(parts, False)
    save(parts, 'costmap-global-unaware.svg')

    parts = base_map(closed, False, '大域に閉塞を反映した後の再計画', '通路Aを塞ぐ障害物セルと膨張コストが大域にも入る。再計画した緑の経路は下側の通路Bを回り、目的地Gに至る。')
    line(parts, alternate_path, False, '#087d67')
    robot_and_goal(parts, False)
    save(parts, 'costmap-global-replanned.svg')
    return normal, closed, original_path, alternate_path, short_motion


if __name__ == '__main__':
    generate()
    print(f'Generated 5 global/local SVGs in {OUTPUT}')
