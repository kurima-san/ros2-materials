#!/usr/bin/env python3
"""Teaching SVGs for layer composition, observation clearing and STVL.

These illustrate state transitions, not ROS measurements or a full sensor
simulation. STVL timing uses a linear 2 s lifetime without frustum acceleration;
marking B continues, while A is no longer marked. White/background in the voxel
memory figures is not a claim that a final Nav2 cell is FREE_SPACE.
"""

from html import escape
from math import exp, hypot

from generate_costmap_figures import OUTPUT, CART, WALLS, color, svg_open

BLUE = '#397bb6'


def text(parts, x, y, label, size=16, fill='#25364b', anchor='middle'):
    parts.append(f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-family="sans-serif" '
                 f'font-size="{size}" fill="{fill}">{escape(label)}</text>')


def line(parts, points, stroke='#087d67', width=3, dash=''):
    coords = ' '.join(f'{x},{y}' for x, y in points)
    dashed = f' stroke-dasharray="{dash}"' if dash else ''
    parts.append(f'<polyline points="{coords}" fill="none" stroke="{stroke}" stroke-width="{width}"{dashed}/>')


def rect(parts, x, y, width, height, fill, stroke='#b8c3cf', extra=''):
    parts.append(f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="{fill}" stroke="{stroke}" stroke-width="0.6" {extra}/>')


def save(name, parts):
    (OUTPUT / name).write_text('\n'.join(parts + ['</svg>']) + '\n')


def inflation(point, obstacles):
    distance = min(hypot(point[0] - x, point[1] - y) for x, y in obstacles) * 0.2
    if distance == 0:
        return 254
    if distance > 0.8 + 1e-9:
        return 0
    if distance <= 0.2 + 1e-9:
        return 253
    return int(252 * exp(-3 * (distance - 0.2)))


def layers():
    descriptions = {
        'static': '保存地図の壁だけを登録。中央の台車はまだ保存地図にない。',
        'sensor': '観測した中央の台車だけを登録。斜線はこの層で情報を持たないセル。',
        'merged': 'StaticLayerの壁とセンサー層の台車を同じ格子へ合成。',
        'inflated': '壁と台車の周囲へ衝突領域と距離に応じたコストを加えた結果。',
    }
    for kind, desc in descriptions.items():
        parts = svg_open(416, 288, 'レイヤーの合成：' + kind, desc)
        for y in range(16):
            for x in range(24):
                p = x, y
                if kind == 'static':
                    cost = 254 if p in WALLS else 0
                elif kind == 'sensor':
                    cost = 254 if p in CART else 255
                elif kind == 'merged':
                    cost = 254 if p in WALLS | CART else 0
                else:
                    cost = inflation(p, WALLS | CART)
                rect(parts, 16 + x * 16, 16 + y * 16, 16, 16, color(cost))
        save('map-layer-' + kind + '.svg', parts)


def clearing():
    for kind in ['mark', 'clear', 'unseen']:
        parts = svg_open(416, 224, '観測による障害物の登録・消去：' + kind,
                         'LはLiDAR、Aは以前の障害物位置。緑は有効な光線、下段は光線上の格子の記録。未観測は斜線。')
        text(parts, 20, 24, {'mark': 't0: hit A', 'clear': 't1: ray passes A', 'unseen': 't1: no new ray'}[kind], anchor='start')
        left, cell, row = 18, 20, 150
        sx, ax, wx = (left + (i + 0.5) * cell for i in (1, 8, 18))
        parts.append(f'<circle cx="{sx}" cy="70" r="16" fill="#245acf"/>')
        text(parts, sx, 76, 'L', 17, '#ffffff')
        rect(parts, wx - 7, 40, 14, 60, color(254))
        if kind == 'mark':
            rect(parts, ax - 10, 57, 20, 26, color(254))
        elif kind == 'clear':
            rect(parts, ax - 10, 57, 20, 26, 'none', '#4163c9', 'stroke-dasharray="4 3"')
        else:
            text(parts, ax, 78, '?', 30)
        text(parts, ax, 112, 'A')
        if kind != 'unseen':
            end = ax - 10 if kind == 'mark' else wx - 7
            line(parts, [(sx + 16, 70), (end, 70)])
            parts.append(f'<circle cx="{end}" cy="70" r="4" fill="#087d67"/>')
        for i in range(19):
            if i == 18 or (i == 8 and kind != 'clear'):
                value = 254
            elif 2 <= i < (18 if kind == 'clear' else 8):
                value = 0
            else:
                value = 255
            rect(parts, left + i * cell, row, cell, 30, color(value))
        rect(parts, left + 8 * cell, row, cell, 30, 'none', '#4163c9', 'stroke-dasharray="3 2"')
        text(parts, ax, 205, '0' if kind == 'clear' else '254', 18)
        text(parts, sx, 205, 'L', 14)
        save('map-ray-' + kind + '.svg', parts)


def rolling():
    for step, start, robot in [(0, 2, 7), (1, 10, 15)]:
        parts = svg_open(464, 254, 'odom上を移動する局所窓：' + str(step),
                         'PとQは同じodom座標に静止する障害物。青い局所窓とロボットRだけが右へ移動する。窓の外の図形は位置関係を示す注記。')
        cell, left, top = 18, 16, 30
        obstacles = {(5, 4), (17, 4)}
        for y in range(9):
            for x in range(24):
                active = start <= x < start + 11
                fill = color(inflation((x, y), obstacles)) if active else '#e6ebf1'
                rect(parts, left + x * cell, top + y * cell, cell, cell, fill)
        rect(parts, left + start * cell, top, 11 * cell, 9 * cell, 'none', '#4163c9', 'stroke-dasharray="6 4"')
        for label, x in [('P', 5), ('Q', 17)]:
            if not start <= x < start + 11:
                rect(parts, left + x * cell, top + 4 * cell, cell, cell, 'none', '#66788d', 'stroke-dasharray="3 2"')
            text(parts, left + (x + 0.5) * cell, top + 4 * cell - 8, label, 17)
        rx, ry = left + (robot + 0.5) * cell, top + 4.5 * cell
        parts.append(f'<circle cx="{rx}" cy="{ry}" r="12" fill="#245acf" stroke="#ffffff"/>')
        text(parts, rx, ry + 5, 'R', 15, '#ffffff')
        line(parts, [(22, 224), (440, 224)], '#33445a', 2)
        line(parts, [(432, 219), (440, 224), (432, 229)], '#33445a', 2)
        text(parts, 28, 216, 'odom x', 15, anchor='start')
        text(parts, 26, 21, 't' + str(step), anchor='start')
        save('map-rolling-' + str(step) + '.svg', parts)


# Sparse columns: three observed heights at A, one elevated voxel at B.
VOXELS = [(2, 2, z) for z in range(3)] + [(4, 2, 2)]


def iso(x, y, z):
    return 192 + (x - y) * 25, 112 + (x + y) * 12 - z * 27


def polygon(parts, points, fill):
    coords = ' '.join(f'{x},{y}' for x, y in points)
    parts.append(f'<polygon points="{coords}" fill="{fill}" stroke="#416786" stroke-width="1"/>')


def voxels():
    for kind in ['points', 'voxels', 'projected']:
        parts = svg_open(416, 288, '3D観測から2Dへの投影：' + kind,
                         'Aには複数の高さの観測、Bには高い位置の観測がある。床は除去済みで、採用する高さ内のボクセルをXY列ごとに集計する。')
        if kind != 'projected':
            for i in range(7):
                line(parts, [iso(i, 0, 0), iso(i, 5, 0)], '#b8c3cf', 1)
            for i in range(6):
                line(parts, [iso(0, i, 0), iso(6, i, 0)], '#b8c3cf', 1)
            line(parts, [iso(0, 0, 0), iso(0, 0, 3)], '#33445a', 2)
            text(parts, *iso(0, 0, 3.3), 'z', 18)
            text(parts, *iso(6.3, 0, 0), 'x', 18)
            text(parts, *iso(0, 5.4, 0), 'y', 18)
            for x, y, z in VOXELS:
                if kind == 'points':
                    for dx, dy, dz in [(0.2, 0.2, 0.2), (0.7, 0.2, 0.6), (0.3, 0.8, 0.8), (0.7, 0.7, 0.3)]:
                        px, py = iso(x + dx, y + dy, z + dz)
                        parts.append(f'<circle cx="{px}" cy="{py}" r="3.6" fill="{BLUE}"/>')
                else:
                    polygon(parts, [iso(x, y + 1, z), iso(x + 1, y + 1, z), iso(x + 1, y + 1, z + 1), iso(x, y + 1, z + 1)], '#7daed0')
                    polygon(parts, [iso(x + 1, y, z), iso(x + 1, y + 1, z), iso(x + 1, y + 1, z + 1), iso(x + 1, y, z + 1)], BLUE)
                    polygon(parts, [iso(x, y, z + 1), iso(x + 1, y, z + 1), iso(x + 1, y + 1, z + 1), iso(x, y + 1, z + 1)], '#b7d9ee')
            for label, x in [('A', 2), ('B', 4)]:
                px, py = iso(x + 0.5, 2.5, 0)
                text(parts, px, py + 35, label, 18)
        else:
            columns = {(x, y) for x, y, z in VOXELS}
            for y in range(5):
                for x in range(6):
                    rect(parts, 82 + x * 42, 24 + y * 42, 42, 42, color(254) if (x, y) in columns else '#e6ebf1')
            for label, x in [('A', 2), ('B', 4)]:
                text(parts, 82 + (x + 0.5) * 42, 24 + 2.5 * 42 + 7, label, 22, '#ffffff')
            text(parts, 208, 263, 'XY / mark_threshold = 1', 18)
        save('stvl-' + kind + '.svg', parts)


def decay():
    for stage, elapsed in enumerate([0.0, 1.0, 2.2]):
        parts = svg_open(416, 272, 'STVLの時間減衰：t=' + str(elapsed),
                         '青いセルはこのSTVL層に残るボクセル記録。Aは時刻0以降に再観測されず、2秒の線形寿命を超えて消える。Bは再観測され続ける。背景は最終コストマップのFreeを意味しない。')
        text(parts, 20, 26, f't = {elapsed:.1f} s', 19, anchor='start')
        for y in range(8):
            for x in range(16):
                at_a = x in (5, 6) and y in (3, 4)
                at_b = x in (10, 11) and y in (3, 4)
                active = (at_a and stage < 2) or (at_b and stage > 0)
                rect(parts, 16 + x * 24, 42 + y * 24, 24, 24, BLUE if active else '#e6ebf1')
        for label, x in [('A', 5), ('B', 10)]:
            rect(parts, 16 + x * 24, 42 + 3 * 24, 48, 48, 'none', '#416786', 'stroke-dasharray="5 3"')
            active = (label == 'A' and stage < 2) or (label == 'B' and stage > 0)
            text(parts, 16 + (x + 1) * 24, 42 + 4 * 24 + 6, label, 21, '#ffffff' if active else '#66788d')
        line(parts, [(194, 138), (244, 138)], '#66788d', 2)
        line(parts, [(237, 133), (244, 138), (237, 143)], '#66788d', 2)
        label = ['A: age 0.0 s', 'A: age 1.0 s / B: refreshed', 'A: expired / B: refreshed'][stage]
        text(parts, 208, 258, label, 17)
        save('stvl-decay-' + str(stage) + '.svg', parts)


def lifetime():
    parts = svg_open(720, 240, '記録が残る時間の比較', '新しいClearingがない標準ObstacleLayerは記録が残る。線形寿命2秒のSTVLは再登録がなければ2秒後に期限切れとなり、1.5秒で再登録されれば約3.5秒まで延長される。')
    left, scale = 220, 112
    for i in range(5):
        x = left + i * scale
        line(parts, [(x, 25), (x, 198)], '#b8c3cf', 1)
        text(parts, x, 225, f'{i} s', 16)
    for label, y, end in [('ObstacleLayer', 56, 4), ('STVL', 112, 2), ('STVL + refresh', 168, 3.5)]:
        text(parts, 15, y + 6, label, 18, anchor='start')
        rect(parts, left, y - 12, end * scale, 24, BLUE, BLUE)
    line(parts, [(left + 4 * scale - 8, 50), (left + 4 * scale, 56), (left + 4 * scale - 8, 62)], '#ffffff', 2)
    x = left + 1.5 * scale
    line(parts, [(x, 144), (x, 191)], '#7542c8', 3)
    text(parts, x, 140, '1.5 s', 16, '#7542c8')
    save('stvl-lifetime.svg', parts)


def frustum():
    parts = svg_open(416, 288, '視野モデルと減衰の加速', '実際の3D視野を上から見た模式図。Aは視野モデル内、Bは外。視野内の古い記録は設定に応じて減衰を加速する。視野モデルだけで遮蔽や空き領域を保証できない。')
    polygon(parts, [(58, 155), (370, 35), (370, 265)], '#dfedf8')
    parts.append('<circle cx="58" cy="155" r="18" fill="#245acf"/>')
    text(parts, 58, 161, 'L', 18, '#ffffff')
    for label, x, y in [('A', 258, 128), ('B', 118, 48)]:
        rect(parts, x - 12, y - 12, 24, 24, BLUE)
        text(parts, x, y + 6, label, 17, '#ffffff')
    text(parts, 265, 190, 'FOV / frustum', 20)
    save('stvl-frustum.svg', parts)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    layers()
    clearing()
    rolling()
    voxels()
    decay()
    lifetime()
    frustum()
    print('Generated 17 layer, clearing, rolling-window and STVL SVGs.')


if __name__ == '__main__':
    main()
