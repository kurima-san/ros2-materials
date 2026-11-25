#!/usr/bin/env python3
"""Generate explanatory 3D-height/2D-projection diagrams, not measured Nav2 output."""
from html import escape
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / 'assets/figures'
INK, RED, BLUE, GREEN = '#25364b', '#b83449', '#2764b0', '#167451'

def start(title, desc, height=440):
    return [f'<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="{height}" viewBox="0 0 1000 {height}" role="img" aria-labelledby="title desc">', f'<title id="title">{escape(title)}</title><desc id="desc">{escape(desc)}</desc>', '<rect width="100%" height="100%" rx="12" fill="#f8fafc"/>', '<g font-family="Noto Sans CJK JP, sans-serif" fill="#25364b">']

def text(p, x, y, value, size=18, color=INK):
    p.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}">{escape(value)}</text>')

def rect(p, x, y, w, h, fill, stroke=INK):
    p.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" rx="3"/>')

def line(p, x1, y1, x2, y2, color=INK, dashed=False):
    p.append(f'<path d="M{x1},{y1} L{x2},{y2}" fill="none" stroke="{color}" stroke-width="3"'+ (' stroke-dasharray="7 5"' if dashed else '') + '/>')

def save(name, p):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text('\n'.join(p + ['</g></svg>']))

p = start('高さを選んでから2Dへ投影する', '側面図で床、低い箱、梁、天井を区別し、機体が衝突する高さの箱と梁だけをXYセルへ投影する。寸法は模式例。')
text(p, 24, 32, '① 側面：高さを選ぶ（模式図）', 21)
rect(p, 45, 150, 565, 100, '#dceee7', '#dceee7')
line(p, 45, 270, 610, 270)
line(p, 45, 65, 610, 65)
text(p, 470, 55, '高い天井：除外', 16)
text(p, 45, 294, '床：除去', 16)
rect(p, 75, 175, 95, 45, '#d7e5f7', BLUE)
line(p, 90, 220, 80, 270, BLUE); line(p, 155, 220, 165, 270, BLUE)
text(p, 65, 160, '機体＋搭載物', 16)
rect(p, 260, 228, 55, 42, '#f3c1c9', RED)
text(p, 245, 212, '低い箱', 17, RED)
rect(p, 430, 155, 65, 28, '#f3c1c9', RED)
line(p, 460, 65, 460, 155)
text(p, 420, 140, '低い梁', 17, RED)
for x,y in [(260,230),(285,228),(315,242),(430,157),(455,183),(495,169),(215,270),(360,270),(340,65)]:
    p.append(f'<circle cx="{x}" cy="{y}" r="4" fill="{BLUE}"/>')
text(p, 640, 32, '② XYへ投影した結果', 21)
for i in range(6):
    rect(p, 650+i*48, 175, 48, 48, '#ffffff', '#8794a5')
rect(p, 650+2*48, 175, 48, 48, INK)
rect(p, 650+4*48, 175, 48, 48, INK)
text(p, 650, 252, '箱も梁も「そのXYは障害物」', 17)
text(p, 650, 282, '高さの違いは2D格子に残らない', 17)
text(p, 24, 345, '緑の帯：床からの相対高さで選ぶ範囲。実際は床面・機体形状・余裕から決める。', 18)
text(p, 24, 382, '高い天井まで残すと通路を塞ぎ、低い箱や梁を消すと衝突対象を見落とす。', 18)
text(p, 24, 416, '穴や段差の危険は「点がない＝通れる」では判定できない。別の地形判定が必要。', 17)
save('costmap-height-projection.svg', p)

p = start('相対高さが同じでも絶対Zは変わる', '同じ高さ15cmの障害物が、odom床0mではZ0.15m、map床マイナス0.30mではZマイナス0.15mになる。map下限0mなら除外。上りではmap上限2mを超える例。', 420)
for x,title in [(20,'Local：odom基準'),(350,'Global：map基準'),(680,'上り先：map基準')]:
    rect(p, x, 15, 300, 325, '#ffffff', '#cbd5e1'); text(p, x+15, 47, title, 21)
# separate conceptual axes, not a shared scale
for x, floor, point, threshold, limit, label in [(20,240,200,280,'下限 −2 m','床 0 → 点 +0.15 m'),(350,260,220,180,'仮の下限 0 m','床 −0.30 → 点 −0.15 m'),(680,220,160,180,'仮の上限 2 m','床 1.90 → 点 2.05 m')]:
    line(p,x+25,floor,x+270,floor);rect(p,x+110,point,50,floor-point,'#f3c1c9',RED)
    line(p,x+25,threshold,x+270,threshold,BLUE,True)
    text(p,x+25,threshold-12,limit,16,BLUE)
    text(p,x+15,95,label,16)
text(p,45,320,'入力高さ条件を通過',18,GREEN)
text(p,375,320,'同じ箱なのに下限で除外',18,RED)
text(p,705,320,'上面の観測点が上限で除外',17,RED)
p.append(f'<circle cx="815" cy="160" r="5" fill="{BLUE}"/>')
text(p,24,375,'説明用の条件。実効値はsource・layerで確認する。各図の縦軸は共通縮尺ではない。',17)
text(p,24,403,'1%勾配でも100 mで床の高度は1 m変化する。床から15 cmと、mapのZは別の量。',17)
save('costmap-height-frames.svg', p)

p = start('障害物が片側だけに入った場合の違い', 'Global欠落Local検出では旧経路への追従が続くが局所で停止可能。両方欠落では衝突する指令の恐れ。両方検出と再計画では迂回経路が候補になる。', 420)
for x,title in [(20,'① Global欠落・Local検出'),(350,'② GlobalもLocalも欠落'),(680,'③ 両方検出＋再計画')]:
    rect(p,x,15,300,325,'#ffffff','#cbd5e1');text(p,x+12,48,title,18)
    rect(p,x+30,100,240,160,'#eef2f7','#8794a5')
    p.append(f'<circle cx="{x+62}" cy="180" r="16" fill="{BLUE}"/>')
    text(p,x+52,186,'R',16,'#ffffff');text(p,x+245,185,'G',18)
    rect(p,x+153,135,25,90,'#f3c1c9',RED)
    text(p,x+140,125,'実物',15,RED)
    if x==680:
        p.append(f'<path d="M{x+80},180 L{x+110},115 L{x+210},115 L{x+240},180" fill="none" stroke="{GREEN}" stroke-width="4"/>')
        text(p,x+15,295,'閉塞を避ける経路へ更新',17,GREEN)
        text(p,x+15,322,'更新済みPathを局所へ渡す',16)
    else:
        line(p,x+80,180,x+240,180,GREEN,True)
        if x==20:
            line(p,x+130,161,x+130,199,RED)
            text(p,x+15,295,'閉塞側の局所目標が続く場合',17)
            text(p,x+15,322,'Localで停止・回避は可能',16,RED)
        else:
            text(p,x+15,295,'計画も制御も障害物を見落とす',17,RED)
            text(p,x+15,322,'衝突方向の指令が出る恐れ',16,RED)
text(p,24,375,'緑：大域経路の模式例。赤：実物の閉塞（Costmapで見えているとは限らない）。',17)
text(p,24,403,'両方への反映だけで経路は変わらない。再計画の実行とControllerへの新Pathの受け渡しが必要。',16)
save('costmap-dual-obstacle-flow.svg', p)
