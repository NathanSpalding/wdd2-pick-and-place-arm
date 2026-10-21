"""Test figures for report Chapter 4: encoder error (4.2), step response (4.4),
grasp current (4.5), positioning accuracy (4.6) and place error (4.7).

Run from anywhere:  python fig_tests.py
Data folders default to ../../test-data/2026-10-12/csv and ../../test-data/2026-10-13/csv;
override with the environment variables WDD2_DAY1 and WDD2_DAY2. PNGs are written to the
current folder (300 dpi).
"""
import os
HERE=os.path.dirname(os.path.abspath(__file__))
D1=os.environ.get('WDD2_DAY1',os.path.join(HERE,'..','..','test-data','2026-10-12','csv'))+os.sep
D2=os.environ.get('WDD2_DAY2',os.path.join(HERE,'..','..','test-data','2026-10-13','csv'))+os.sep
O=''
import csv, collections, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=D1
plt.rcParams.update({'font.family':'STIXGeneral','mathtext.fontset':'stix','font.size':10,
 'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#52514e','axes.linewidth':0.8,
 'xtick.color':'#52514e','ytick.color':'#52514e','axes.grid':True,'grid.color':'#e4e3df','grid.linewidth':0.6,
 'lines.linewidth':1.6,'legend.frameon':False,'savefig.dpi':300})
BLUE,ORANGE,GREY='#2a78d6','#eb6834','#8a8984'

# 8.2 encoder error
rows=list(csv.DictReader(open(D+'8.2_encoder_vs_reference_sweep1.csv')))
fig,axs=plt.subplots(2,2,figsize=(6.3,4.4),sharey=True)
for ax,j in zip(axs.flat,['base','shoulder','elbow','wrist']):
    r=[x for x in rows if x['joint']==j]
    ax.axhspan(-0.5,0.5,color='#eef3fb',zorder=0,lw=0)
    for y in (-1,1): ax.axhline(y,color=GREY,ls='--',lw=0.9)
    norm=[x for x in r if 'touched' not in x['note']]; t=[x for x in r if 'touched' in x['note']]
    ax.plot([float(x['reference_deg']) for x in norm],[float(x['error_deg']) for x in norm],'o',ms=5,color=BLUE,zorder=3)
    if t: ax.plot([float(x['reference_deg']) for x in t],[float(x['error_deg']) for x in t],'o',ms=5,mfc='white',mec=BLUE,mew=1.3,zorder=3)
    ax.set_title(j.capitalize(),fontsize=10,loc='left'); ax.set_ylim(-1.3,1.3)
for ax in axs[1]: ax.set_xlabel('Reference angle (°)')
for ax in axs[:,0]: ax.set_ylabel('Encoder error (°)')
fig.tight_layout(); fig.savefig(O+'test-encoder-error.png'); plt.close()

# 8.4 step plots
def load(f):
    t,c={},collections.defaultdict(list)
    rr=list(csv.DictReader(open(D+f)))
    return rr
def window(rr,t0,col,dur):
    xs,ys=[],[]
    for r in rr:
        t=float(r['t_s'])
        if t0-0.5<=t<=t0+dur and r[col]: xs.append(t-t0); ys.append(float(r[col]))
    return xs,ys
fig,(a,b)=plt.subplots(1,2,figsize=(6.3,2.7))
rr=load('8.4_timeseries_shoulder_500g.csv')
x,y=window(rr,21.04,'shoulder_deg',5); a.plot(x,y,color=BLUE,label='Trim on')
x,y=window(rr,54.52,'shoulder_deg',5); a.plot(x,y,color=ORANGE,label='Trim off')
a.plot([-0.5,0,0,5],[20,20,30,30],color=GREY,ls='--',lw=1,label='Target')
a.set_xlabel('Time after command (s)'); a.set_ylabel('Shoulder angle (°)'); a.set_title('(a) Shoulder, 500 g',fontsize=10,loc='left')
a.legend(loc='lower right',fontsize=9)
rr=load('8.4_timeseries_base_0g.csv')
x,y=window(rr,5.12,'base_deg',5.5); b.plot(x,y,color=BLUE,label='Trim on')
x,y=window(rr,41.9,'base_deg',4.5); b.plot(x,y,color=ORANGE,label='Trim off')
b.plot([-0.5,0,0,5.5],[35,35,45,45],color=GREY,ls='--',lw=1)
b.set_xlabel('Time after command (s)'); b.set_ylabel('Base angle (°)'); b.set_title('(b) Base, no load',fontsize=10,loc='left')
b.set_ylim(43,46.5); b.set_xlim(-0.2,5.5)
fig.tight_layout(); fig.savefig(O+'test-step-response.png'); plt.close()

# 8.5 grasp currents
rows=list(csv.DictReader(open(D+'8.5_grasp_trials.csv')))
order=['empty','50 g','100 g','200 g','300 g','400 g','500 g']
fig,ax=plt.subplots(figsize=(6.3,2.9))
for i,p in enumerate(order):
    r=[x for x in rows if x['payload']==p]
    for k,x in enumerate(r):
        dx=(k-(len(r)-1)/2)*0.045
        held = x['held_in_lift']!='N'
        ax.plot(i+dx,float(x['median_current_A']),'o',ms=5,color=BLUE if held else 'white',mec=BLUE,mew=1.3,zorder=3)
ax.axhline(0.04,color=ORANGE,ls='--',lw=1.2)
ax.text(6.45,0.046,'Threshold 0.04 A',ha='right',va='bottom',fontsize=9,color='#0b0b0b')
ax.set_xticks(range(7)); ax.set_xticklabels(['Empty','50','100','200','300','400','500'])
ax.set_xlabel('Nominal payload (g)'); ax.set_ylabel('Median current (A)'); ax.set_ylim(0,0.23)
fig.tight_layout(); fig.savefig(O+'test-grasp-current.png'); plt.close()

# ---- day 2: accuracy and place error
import numpy as np
D=D2
rng=np.random.default_rng(1)
def circ(ax,r):
    t=np.linspace(0,2*np.pi,200); ax.plot(r*np.cos(t),r*np.sin(t),color=GREY,ls='--',lw=1)
allr=list(csv.DictReader(open(D+'8.6_approaches.csv')))
rows=[r for r in allr if r['lateral_mm']]
s={x['target']:x for x in csv.DictReader(open(D+'8.6_summary.csv'))}
fig,axs=plt.subplots(1,3,figsize=(6.6,2.6))
def panel(ax,rs,title,cent):
    lat=np.array([float(r['lateral_mm']) for r in rs]); rad=np.array([float(r['radial_mm']) for r in rs])
    ax.plot(lat+rng.uniform(-.25,.25,len(lat)),rad+rng.uniform(-.25,.25,len(rad)),'o',ms=3.5 if len(rs)>20 else 5,color=BLUE,alpha=0.55 if len(rs)>20 else 0.8,mew=0,label='Approach')
    ax.plot([float(s[t]['mean_lateral_mm']) for t in cent],[float(s[t]['mean_radial_mm']) for t in cent],'D',ms=5.5,color=ORANGE,mec='white',mew=0.8,label='Centroid')
    circ(ax,10); ax.plot(0,0,'+',color='#0b0b0b',ms=9,mew=1.2)
    ax.set_aspect('equal'); ax.set_xlim(-12,12); ax.set_ylim(-12,12); ax.set_xticks([-10,0,10]); ax.set_yticks([-10,0,10])
    ax.set_xlabel('Lateral error (mm)'); ax.set_title(title,fontsize=10,loc='left')
panel(axs[0],rows,'(a) All targets',list(s))
axs[0].set_ylabel('Radial error (mm)'); axs[0].legend(loc='upper left',fontsize=7.5,handletextpad=0.2,borderaxespad=0.1)
panel(axs[1],[r for r in rows if r['target']=='T2'],'(b) Best: T2',['T2'])
panel(axs[2],[r for r in rows if r['target']=='T8'],'(c) Worst: T8',['T8'])
fig.tight_layout(); fig.savefig(O+'test-accuracy.png'); plt.close()

rows=list(csv.DictReader(open(D+'8.7_cycles.csv')))
fig,ax=plt.subplots(figsize=(3.6,3.2))
for p,c in [('50 g',BLUE),('500 g',ORANGE)]:
    r=[x for x in rows if x['payload']==p]
    la=np.array([float(x['lateral_mm']) for x in r]); ra=np.array([float(x['radial_mm']) for x in r])
    ax.plot(la+rng.uniform(-.25,.25,len(la)),ra+rng.uniform(-.25,.25,len(ra)),'o',ms=5,color=c,mec='white',mew=0.6,label=p)
circ(ax,10); ax.plot(0,0,'+',color='#0b0b0b',ms=9,mew=1.2)
ax.set_aspect('equal'); ax.set_xlim(-11,11); ax.set_ylim(-11,11)
ax.set_xlabel('Lateral error (mm)'); ax.set_ylabel('Radial error (mm)')
ax.legend(loc='upper left',fontsize=9,handletextpad=0.3)
fig.tight_layout(); fig.savefig(O+'test-place-error.png'); plt.close()
