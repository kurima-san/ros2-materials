#!/usr/bin/env python3
"""Illustrate Inflation tuning and Humble RPP cost-based speed regulation.

Distances are to ideal wall lines; grids are schematic, not a Nav2 simulation.
The speed plot omits curvature, acceleration, goal approach and external limits.
"""
from math import exp, hypot
from generate_costmap_figures import OUTPUT, color, svg_open
from generate_map_lifecycle_figures import text, line, rect, save

ROBOT_RADIUS = 0.30


def cost(distance, radius=1.20, scaling=3.0):
    if distance <= 0:
        return 254
    if distance > radius + 1e-9:
        return 0
    if distance <= ROBOT_RADIUS + 1e-9:
        return 253
    return int(252 * exp(-scaling * (distance - ROBOT_RADIUS)))


def corridor(name, width, radius):
    parts = svg_open(416, 340, '通路幅とInflation半径',
                     f'円形機体の直径は0.60 m、通路幅は{width:.2f} m。Inflation半径{radius:.2f} mのコストを示す模式図。')
    text(parts, 208, 27, f'inflation_radius = {radius:.2f} m', 18)
    scale, cy = 200, 168
    for row in range(24):
        for col in range(36):
            distance = width / 2 - abs((48 + (row + 0.5) * 10 - cy) / scale)
            rect(parts, 28 + col * 10, 48 + row * 10, 10, 10, color(cost(distance, radius)), '#b8c3cf')
    for y in [cy - width * scale / 2, cy + width * scale / 2]:
        line(parts, [(28, y), (388, y)], '#25364b', 3)
    line(parts, [(36, cy), (378, cy)], '#087d67', 3, '5 4')
    parts.append(f'<circle cx="208" cy="{cy}" r="60" fill="#245acf" fill-opacity="0.12" stroke="#245acf" stroke-width="3"/>')
    text(parts, 208, cy + 6, 'R', 20, '#245acf')
    if width <= 2 * ROBOT_RADIUS:
        line(parts, [(196, cy-12), (220, cy+12)], '#b31d37', 4)
        line(parts, [(196, cy+12), (220, cy-12)], '#b31d37', 4)
    text(parts, 208, 312, f'width = {width:.2f} m / center cost = {cost(width / 2, radius)}', 17)
    save(name, parts)


def curves():
    parts = svg_open(720, 330, 'cost_scaling_factorと距離コスト',
                     '内接半径0.30 m、Inflation半径1.20 mを固定し、減衰係数3と10を比較。大きい係数ほど距離に対し急にコストが下がる。')
    left, top, width, height = 58, 55, 620, 220
    x = lambda d: left + d / 1.5 * width
    y = lambda c: top + height * (1 - c / 260)
    rect(parts, x(0), top, x(.3)-left, height, '#f8dce2', 'none')
    for c in [0, 100, 200, 253]:
        line(parts, [(left, y(c)), (left+width, y(c))], '#cbd3dd', 1)
        text(parts, left-9, y(c)+5, str(c), 14, anchor='end')
    for d in [0, .3, .6, .9, 1.2, 1.5]:
        text(parts, x(d), 299, f'{d:.1f}', 15)
    for k,stroke in [(3.0,'#2478b8'),(10.0,'#7542c8')]:
        points=[(x(i/1000),y(cost(i/1000,scaling=k))) for i in range(1,1501)]
        line(parts,points,stroke,3)
    line(parts,[(x(1.2),top),(x(1.2),top+height)],'#66788d',1,'5 4')
    text(parts, 155, 26, 'k = 3.0', 18, '#2478b8')
    text(parts, 310, 26, 'k = 10.0', 18, '#7542c8')
    text(parts, 552, 26, 'radius = 1.20 m', 17)
    text(parts, 380, 323, 'distance from obstacle (m)', 15)
    save('costmap-tuning-curves.svg', parts)


# A 2.50 m wide area connects to a 1.00 m corridor and another wide area.
SEGMENTS = [((20,50),(240,50)),((240,50),(240,110)),((240,110),(480,110)),
            ((480,110),(480,50)),((480,50),(700,50)),
            ((20,250),(240,250)),((240,250),(240,190)),((240,190),(480,190)),
            ((480,190),(480,250)),((480,250),(700,250))]


def wall_distance(x, y):
    distances=[]
    for (ax,ay),(bx,by) in SEGMENTS:
        fraction=max(0,min(1,((x-ax)*(bx-ax)+(y-ay)*(by-ay))/((bx-ax)**2+(by-ay)**2)))
        distances.append(hypot(x-(ax+fraction*(bx-ax)),y-(ay+fraction*(by-ay))))
    return min(distances)/80


def regulated_speed(distance):
    return max(.1, .8 * distance / .8) if distance < .8 else .8


def speed():
    parts=svg_open(720, 430, '同じInflationで広場と狭路の速度を変える',
                   'Inflation半径1.20 m、減衰係数3を固定。開けた場所は0.80 m/s、幅1 mの通路中央は壁から0.50 mなので約0.50 m/s。下段はRPPの障害物近接による速度候補。')
    for label,x in [('A: 0.80 m/s',100),('B: about 0.50 m/s',360),('C: 0.80 m/s',620)]:
        text(parts,x,26,label,17)
    for row in range(25):
        for col in range(85):
            px,py=20+(col+.5)*8,50+(row+.5)*8
            blocked=240<px<480 and not 110<py<190
            rect(parts,px-4,py-4,8,8,color(254 if blocked else cost(wall_distance(px,py))), 'none')
    for a,b in SEGMENTS:
        line(parts,[a,b],'#25364b',4)
    line(parts,[(30,150),(690,150)],'#087d67',3,'5 5')
    for label,x in [('A',100),('B',360),('C',620)]:
        parts.append(f'<circle cx="{x}" cy="150" r="24" fill="#245acf" fill-opacity="0.16" stroke="#245acf" stroke-width="2"/>')
        text(parts,x,157,label,20,'#245acf')
    text(parts,360,278,'inflation_radius = 1.20 m (fixed)',17)
    for v in [0,.5,.8]:
        y=386-v*100
        line(parts,[(58,y),(700,y)],'#cbd3dd',1)
        text(parts,48,y+5,f'{v:.1f}',14,anchor='end')
    points=[(x,386-regulated_speed(wall_distance(x,150))*100) for x in range(58,701)]
    line(parts,points,'#7542c8',3)
    text(parts,360,417,'path position: A → B → C / speed (m/s)',16)
    save('costmap-tuning-speed.svg',parts)


def main():
    OUTPUT.mkdir(parents=True,exist_ok=True)
    corridor('costmap-tuning-wide-radius.svg',1.0,1.2)
    corridor('costmap-tuning-small-radius.svg',1.0,.4)
    corridor('costmap-tuning-too-narrow.svg',.5,1.2)
    curves()
    speed()
    print('Generated 5 Inflation and speed regulation figures.')


if __name__ == '__main__':
    main()
