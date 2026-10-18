#!/usr/bin/env python3
"""Reproduce the 0.45 m robot / 0.80 m Inflation teaching figures.

Requires NumPy, Matplotlib and fonts-noto-cjk (Debian/Ubuntu font path).
Exports self-contained SVGs with outlined Japanese text and accessible titles.
The diagrams use ideal walls and no footprint padding; they are not ROS output.
"""
from pathlib import Path
import re
from html import escape
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle,Rectangle

OUT=Path(__file__).resolve().parents[1] / "assets" / "figures"
OUT.mkdir(parents=True,exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':12,'axes.titleweight':'bold','axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'path','svg.hashsalt':'costmap-radius-example','axes.unicode_minus':False})
BLUE='#245acf';RED='#cf526a';INK='#25364b';GREEN='#087d67';GREY='#65778b'

def cost(d,k=3):
 d=np.asarray(d)
 return np.where(d<=0,254,np.where(d<=.45,253,np.where(d<=.8,np.floor(252*np.exp(-k*(d-.45))),0)))

def rgba(c):
 c=np.asarray(c);a=np.ones(c.shape+(4,))
 low=np.array([255,242,188])/255;high=np.array([234,125,44])/255
 a[...,:3]=low+(high-low)*(np.minimum(c,252)/252)[...,None]
 a[c==0,:3]=1
 a[c==253,:3]=np.array([207,82,106])/255
 a[c==254,:3]=np.array([37,54,75])/255
 return a

def finish(fig,name):
 descriptions={
  'three-parameters': ('3つのコストマップ設定', '半径0.45 mの円形機体、障害物から0.80 mの膨張範囲、係数3.0による距離コストの減衰を比較する。'),
  'corridor-widths': ('直径0.90 mの機体と通路幅', '幅0.80 mでは機体が壁に重なり、幅0.90 mでは接触する。幅1.20 mなら片側0.15 mの隙間があり、中央コストは約160。'),
 }
 title,description=descriptions[name]
 path=OUT/('costmap-radius-example-'+name+'.svg')
 fig.savefig(path,facecolor='white',metadata={'Date':None})
 svg=path.read_text()
 svg,n=re.subn(r'<svg\b([^>]*?)>',lambda m:'<svg'+m[1]+' role="img" aria-labelledby="figure-title figure-desc">\n'
              +'<title id="figure-title">'+escape(title)+'</title>\n'
              +'<desc id="figure-desc">'+escape(description)+'</desc>',svg,count=1)
 assert n==1
 path.write_text('\n'.join(line.rstrip() for line in svg.splitlines())+'\n')
 plt.close(fig)

fig,axes=plt.subplots(1,3,figsize=(14,5.1),gridspec_kw={'width_ratios':[1,1.3,1.3]})
fig.subplots_adjust(left=.035,right=.985,top=.74,bottom=.23,wspace=.30)
fig.suptitle('robot_radius = 0.45 m  /  inflation_radius = 0.80 m  /  cost_scaling_factor = 3.0',fontsize=16,y=.97)
ax=axes[0];ax.set_title('① ロボットの大きさ',fontsize=15,pad=24)
ax.set_xlim(-.65,.65);ax.set_ylim(-.72,.66);ax.set_aspect('equal');ax.axis('off')
ax.add_patch(Circle((0,0),.45,facecolor='#e3ecff',edgecolor=BLUE,lw=2.5))
ax.scatter([0],[0],s=30,color=BLUE,zorder=3)
ax.annotate('',(.45,0),(0,0),arrowprops=dict(arrowstyle='<->',color=BLUE,lw=2))
ax.text(.22,.07,'0.45 m',ha='center',color=BLUE,fontsize=13)
ax.annotate('',(.45,-.57),(-.45,-.57),arrowprops=dict(arrowstyle='<->',color=INK,lw=1.7))
ax.text(0,-.65,'直径 0.90 m',ha='center',va='top',fontsize=13)
ax=axes[1];ax.set_title('② 障害物から広げる範囲',fontsize=15,pad=24)
ax.set_xlim(-.05,1.08);ax.set_ylim(0,1);ax.axis('off')
xs=np.linspace(0,1.05,1051)
ax.imshow(rgba(cost(xs))[None,:,:],extent=(0,1.05,.34,.72),aspect='auto',origin='lower')
ax.add_patch(Rectangle((-.035,.30),.035,.46,color=INK))
ax.text(-.015,.79,'壁',ha='center',fontsize=12)
ax.annotate('',(.8,.85),(0,.85),arrowprops=dict(arrowstyle='<->',color=INK,lw=1.8))
ax.text(.4,.90,'障害物から 0.80 m',ha='center',fontsize=13)
ax.text(.225,.53,'中心を置くと\n接触・衝突',ha='center',va='center',color='white',fontsize=12)
ax.text(.625,.53,'避けたい\nコスト',ha='center',va='center',fontsize=12)
ax.text(.94,.53,'膨張\nなし',ha='center',va='center',fontsize=11)
for x,t in [(0,'0'),(.45,'0.45'),(.8,'0.80')]:
 ax.plot([x,x],[.30,.74],color=GREY,lw=.9,ls=':');ax.text(x,.23,t,ha='center',fontsize=12)
ax.text(.5,.08,'距離 d（m）／ロボット中心位置の評価',ha='center',fontsize=11)
ax=axes[2];ax.set_title('③ 範囲内でのコストの下がり方',fontsize=15,pad=24)
ds=np.linspace(.0001,1.05,2500)
ax.axvspan(0,.45,color=RED,alpha=.12)
ax.plot(ds,cost(ds),color=BLUE,lw=2.5,label='設定値 3.0')
ax.plot(ds,cost(ds,6),color=GREY,lw=1.6,ls='--',label='参考：6.0なら急に下がる')
ax.axvline(.8,color=GREY,lw=.8,ls=':')
ax.scatter([.8],[88],color=BLUE,s=28,zorder=5)
ax.annotate('約88',(.8,88),(.87,130),arrowprops=dict(arrowstyle='-',color=BLUE),color=BLUE,fontsize=12)
ax.set(xlim=(0,1.05),ylim=(-6,280),xticks=[0,.45,.8,1.0],yticks=[0,100,200,253],xlabel='障害物からの距離 d（m）')
ax.grid(axis='y',alpha=.18);ax.legend(loc='lower left',bbox_to_anchor=(-.03,-.42),fontsize=11,frameon=False)
fig.text(.035,.065,'円形モデル・追加Paddingなしの模式図。コストは「機体中心をその位置に置く候補」の評価。格子の丸めは簡略化。',fontsize=12,color=GREY)
finish(fig,'three-parameters')

fig,axes=plt.subplots(1,3,figsize=(12.4,4.9))
fig.subplots_adjust(left=.035,right=.985,top=.76,bottom=.25,wspace=.20)
fig.suptitle('直径0.90 mのロボットは、どの幅の通路を通れるか',fontsize=18,y=.97)
fig.text(.5,.875,'青い円が実物の機体。背景色はロボット中心位置のコスト。',ha='center',color=GREY,fontsize=12)
ys=np.linspace(-.8,.8,401)
for ax,w,note in zip(axes,[.8,.9,1.2],['0.10 m足りず、壁に重なる','壁に接触する。余裕はゼロ','片側0.15 mの隙間がある']):
 ax.set_title(f'通路幅 {w:.2f} m',fontsize=15,pad=12)
 cs=cost(w/2-np.abs(ys))
 arr=np.repeat(rgba(cs)[:,None,:],4,axis=1)
 ax.imshow(arr,extent=(-.75,.75,-.8,.8),origin='lower',aspect='equal')
 for wy in [-w/2,w/2]:ax.axhline(wy,color=INK,lw=2)
 ax.add_patch(Circle((0,0),.45,facecolor=(.14,.35,.81,.10),edgecolor=BLUE,lw=2.5,zorder=2.5))
 ax.plot([-.65,.65],[0,0],ls='--',lw=1.4,color=GREEN)
 if w<=.9:ax.text(0,0,'×',ha='center',va='center',fontsize=30,color='#102347')
 else:ax.text(0,0,'R',ha='center',va='center',fontsize=20,color=BLUE)
 ax.set(xlim=(-.75,.75),ylim=(-.8,.8));ax.axis('off')
 ax.text(.5,-.10,note,ha='center',transform=ax.transAxes,fontsize=12)
 ax.text(.5,-.23,'通行不可' if w<=.9 else '通行候補になる（中央コスト 約160）',ha='center',transform=ax.transAxes,fontsize=12,color='#aa2742' if w<=.9 else GREEN)
fig.text(.035,.045,'実際の必要幅には、Footprintの形状、Padding、地図の解像度、位置誤差なども影響する。',fontsize=12,color=GREY)
finish(fig,'corridor-widths')
print('Created 2 radius example SVG diagrams in',OUT)
print('Costs:',{float(d):int(cost(d)) for d in [.45,.50,.60,.80,.81]})
