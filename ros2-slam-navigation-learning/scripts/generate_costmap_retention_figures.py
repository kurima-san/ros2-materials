#!/usr/bin/env python3
"""Compare retention at one fixed location, not complete final costmaps.

A is marked at t=0 and never marked again. Linear STVL lifetime is 2 s,
without frustum acceleration or reuse of old input. Costmap updates and the
ROS clock continue; resets, footprint clearing and rolling-window removal
are excluded. No observation is not a measurement of free space.
"""
from generate_costmap_figures import OUTPUT, svg_open
from generate_map_lifecycle_figures import text, rect, save

COLORS = {'kept': '#397bb6', 'cleared': '#d7eee7', 'expired': '#e6ebf1'}
LABELS = {'kept': 'Aを保持', 'cleared': '空きで消去', 'expired': '期限で消去'}
TIMES = ['0.0秒', '0.5秒', '1.0秒', '2.2秒']


def make(name, ray_at_one):
    title = '1秒でAを通る有効な光線を観測' if ray_at_one else 'Aを再観測できないまま時間が進む'
    parts = svg_open(840, 350, title,
                     '3行は観測消去ありのObstacleLayer、登録のみのObstacleLayer、線形寿命2秒のSTVL。青は保持、緑は空き観測による消去、灰色は期限切れによる消去。')
    text(parts, 420, 29, title, 22)
    left, width, gap = 250, 126, 13
    rows = [
        ('A：観測で消す', 'ObstacleLayer / clearing=true',
         ['kept', 'kept', 'cleared' if ray_at_one else 'kept', 'cleared' if ray_at_one else 'kept']),
        ('B：登録のみ', 'ObstacleLayer / clearing=false', ['kept'] * 4),
        ('C：2秒の寿命', 'STVL / 線形・加速なし', ['kept', 'kept', 'kept', 'expired']),
    ]
    for index, label in enumerate(TIMES):
        x = left + index * (width + gap) + width/2
        text(parts, x, 63, label, 18)
        event = 'Aを登録' if index == 0 else ('Aを光線が通過' if ray_at_one and index == 2 else 'Aの再登録なし')
        text(parts, x, 84, event, 13, '#66788d')
    for row, (label, sub, states) in enumerate(rows):
        y = 106 + row * 67
        text(parts, 18, y+19, label, 20, anchor='start')
        text(parts, 18, y+41, sub, 13, '#66788d', anchor='start')
        for index, state in enumerate(states):
            x = left + index * (width + gap)
            rect(parts, x, y, width, 46, COLORS[state], '#b8c3cf', 'rx="6"')
            text(parts, x+width/2, y+29, LABELS[state], 19, '#ffffff' if state=='kept' else '#25364b')
    for x,state in [(33,'kept'),(250,'cleared'),(490,'expired')]:
        rect(parts,x,319,17,17,COLORS[state])
        text(parts,x+27,333,{'kept':'記録が残る','cleared':'空き観測で更新','expired':'古い記録を破棄'}[state],15,anchor='start')
    save(name,parts)


def main():
    OUTPUT.mkdir(parents=True,exist_ok=True)
    make('costmap-retention-unobserved.svg',False)
    make('costmap-retention-clearing.svg',True)
    print('Generated 2 observation/retention/expiry comparison SVGs.')


if __name__ == '__main__':
    main()
