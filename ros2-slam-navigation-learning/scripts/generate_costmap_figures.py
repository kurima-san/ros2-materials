#!/usr/bin/env python3
"""Generate the section 6.4 SVGs with Python's standard library.

This is a teaching grid, not a Nav2 simulation. Distances are between cell
centres; the inflation formula follows Humble's InflationLayer. The example
route uses a simple Dijkstra search, not a particular Nav2 planner/controller.
Run from any directory: python3 scripts/generate_costmap_figures.py
"""

from heapq import heappop, heappush
from html import escape
from math import exp, hypot
from pathlib import Path

OUTPUT = Path(__file__).resolve().parents[1] / "assets" / "figures"
COLS, ROWS = 24, 16
CELL, MARGIN = 16, 16
RESOLUTION, INSCRIBED_RADIUS, INFLATION_RADIUS, SCALING = 0.2, 0.2, 0.8, 3.0
START, GOAL = (4, 8), (19, 8)
CART = {(x, y) for x in (12, 13) for y in (7, 8)}
WALLS = {
    (x, y) for x in range(COLS) for y in range(ROWS)
    if x in (0, COLS - 1) or y in (0, ROWS - 1)
}
OBSTACLES = WALLS | CART
UNKNOWN = {(x, y) for x in range(19, 23) for y in range(1, 4)}


def cost_at(x, y):
    if (x, y) in UNKNOWN:
        return 255
    distance = min(hypot(x - ox, y - oy) for ox, oy in OBSTACLES) * RESOLUTION
    if distance == 0:
        return 254
    if distance > INFLATION_RADIUS + 1e-9:
        return 0
    if distance <= INSCRIBED_RADIUS + 1e-9:
        return 253
    return int(252 * exp(-SCALING * (distance - INSCRIBED_RADIUS)))


COSTS = {(x, y): cost_at(x, y) for x in range(COLS) for y in range(ROWS)}


def color(cost):
    if cost == 255:
        return "url(#unknown)"
    if cost == 254:
        return "#25364b"
    if cost == 253:
        return "#d64d64"
    if cost == 0:
        return "#ffffff"
    # Pale yellow to orange, with a fixed scale shared by all figures.
    low, high = (255, 242, 188), (234, 125, 44)
    ratio = cost / 252
    return "#" + "".join(f"{round(a + (b - a) * ratio):02x}" for a, b in zip(low, high))


def svg_open(width, height, title, description):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">',
        f'<title id="title">{escape(title)}</title><desc id="desc">{escape(description)}</desc>',
        '<defs><pattern id="unknown" width="8" height="8" patternUnits="userSpaceOnUse">'
        '<rect width="8" height="8" fill="#e2e6eb"/>'
        '<path d="M-2 2L2-2M0 8L8 0M6 10L10 6" stroke="#9ba7b6" stroke-width="1.2"/>'
        '</pattern></defs>',
        f'<rect width="{width}" height="{height}" fill="#f5f7fa" rx="10"/>',
    ]


def center(point):
    x, y = point
    return MARGIN + (x + 0.5) * CELL, MARGIN + (y + 0.5) * CELL


def route():
    distances, previous, queue = {START: 0.0}, {}, [(0.0, START)]
    while queue:
        distance, point = heappop(queue)
        if distance != distances[point]:
            continue
        if point == GOAL:
            break
        x, y = point
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            next_point = (x + dx, y + dy)
            if COSTS.get(next_point, 255) >= 253:
                continue
            if dx and dy and (COSTS.get((x + dx, y), 255) >= 253 or COSTS.get((x, y + dy), 255) >= 253):
                continue
            candidate = distance + hypot(dx, dy) * (1 + COSTS[next_point] / 60)
            if candidate < distances.get(next_point, float("inf")):
                distances[next_point] = candidate
                previous[next_point] = point
                heappush(queue, (candidate, next_point))
    points = [GOAL]
    while points[-1] != START:
        points.append(previous[points[-1]])
    return list(reversed(points))


def overlays(parts, with_route=False):
    sx, sy = center(START)
    gx, gy = center(GOAL)
    if with_route:
        points = " ".join(f"{x},{y}" for x, y in map(center, route()))
        parts.append(f'<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="7" stroke-linejoin="round"/>')
        parts.append(f'<polyline points="{points}" fill="none" stroke="#087d67" stroke-width="4" stroke-linejoin="round"/>')
        # Six actual cells, expanded in the separate detail figure.
        parts.append(f'<rect x="{MARGIN + 7 * CELL}" y="{MARGIN + 8 * CELL}" width="{6 * CELL}" height="{CELL}" '
                     'fill="none" stroke="#4163c9" stroke-width="2" stroke-dasharray="4 2"/>')
    parts.extend([
        f'<circle cx="{sx}" cy="{sy}" r="{CELL}" fill="#245acf" stroke="#ffffff" stroke-width="2"/>',
        f'<text x="{sx}" y="{sy + 5}" text-anchor="middle" font-family="sans-serif" font-size="16" font-weight="bold" fill="#ffffff">R</text>',
        f'<circle cx="{gx}" cy="{gy}" r="10" fill="#ffffff" stroke="#087d67" stroke-width="3"/>',
        f'<text x="{gx}" y="{gy + 5}" text-anchor="middle" font-family="sans-serif" font-size="14" font-weight="bold" fill="#065b4b">G</text>',
        f'<text x="{MARGIN + 21 * CELL}" y="{MARGIN + 2.65 * CELL}" text-anchor="middle" font-family="sans-serif" font-size="24" font-weight="bold" fill="#3f4b5e">?</text>',
    ])


def write_map(kind, title, description):
    parts = svg_open(COLS * CELL + 2 * MARGIN, ROWS * CELL + 2 * MARGIN, title, description)
    if kind == "scene":
        parts.append(f'<rect x="{MARGIN}" y="{MARGIN}" width="{COLS * CELL}" height="{ROWS * CELL}" fill="#ffffff"/>')
        for x, y in sorted(WALLS | UNKNOWN):
            parts.append(f'<rect x="{MARGIN + x * CELL}" y="{MARGIN + y * CELL}" width="{CELL}" height="{CELL}" fill="{color(255 if (x, y) in UNKNOWN else 254)}"/>')
        parts.append(f'<rect x="{MARGIN + 12 * CELL}" y="{MARGIN + 7 * CELL}" width="{2 * CELL}" height="{2 * CELL}" rx="3" fill="#25364b"/>')
        for x in (12.2, 13.5):
            for y in (6.8, 8.8):
                parts.append(f'<rect x="{MARGIN + x * CELL}" y="{MARGIN + y * CELL}" width="5" height="6" rx="2" fill="#101c2c"/>')
    else:
        for y in range(ROWS):
            for x in range(COLS):
                value = COSTS[x, y] if kind == "inflated" else (255 if (x, y) in UNKNOWN else 254 if (x, y) in OBSTACLES else 0)
                parts.append(f'<rect x="{MARGIN + x * CELL}" y="{MARGIN + y * CELL}" width="{CELL}" height="{CELL}" '
                             f'fill="{color(value)}" stroke="#b8c3cf" stroke-width="0.5"/>')
    overlays(parts, kind == "inflated")
    parts.append('</svg>')
    (OUTPUT / f"costmap-{kind}.svg").write_text("\n".join(parts) + "\n")


def write_detail():
    parts = svg_open(600, 152, "台車へ近づく6セルの拡大", "障害物から1.0、0.8、0.6、0.4、0.2、0 mの順に、内部コストは0、41、75、138、253、254となる。")
    for index, x in enumerate(range(7, 13)):
        value = COSTS[x, 8]
        left = 12 + index * 96
        text_color = "#ffffff" if value >= 253 else "#19283b"
        parts.extend([
            f'<rect x="{left}" y="16" width="96" height="78" fill="{color(value)}" stroke="#b8c3cf"/>',
            f'<text x="{left + 48}" y="65" text-anchor="middle" font-family="sans-serif" font-size="30" font-weight="bold" fill="{text_color}">{value}</text>',
            f'<text x="{left + 48}" y="119" text-anchor="middle" font-family="sans-serif" font-size="20" fill="#33445a">{(12 - x) * RESOLUTION:.1f} m</text>',
        ])
    parts.append('<path d="M20 139H576M568 133L577 139L568 145" fill="none" stroke="#33445a" stroke-width="2"/>')
    parts.append('</svg>')
    (OUTPUT / "costmap-cell-detail.svg").write_text("\n".join(parts) + "\n")


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    write_map("scene", "同じ部屋を上から見る", "四方の壁、中央の台車、左のロボットR、右の目的地G。右上の斜線は未観測領域。")
    write_map("obstacles", "壁と台車を障害物セルにする", "実空間と同じ配置を24×16の格子へ変換。壁と台車は254、空きは0、未観測は255。")
    write_map("inflated", "周囲へコストを広げ、通る場所を選ぶ", "壁と台車の周囲に赤い衝突領域、その外に黄から橙のコスト勾配。緑の経路例が台車を回り込む。青い点線で6セルの拡大箇所を示す。")
    write_detail()
    print(f"Generated 4 SVGs in {OUTPUT}")


if __name__ == "__main__":
    main()
