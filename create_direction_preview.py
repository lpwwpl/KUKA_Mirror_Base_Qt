import math
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle, FancyArrowPatch

ROOT = Path('/mnt/data/KUKA_Mirror_Base_Qt5_12_Method1_Enhanced_v2')
VIS = ROOT / 'visuals'
VIS.mkdir(exist_ok=True)
plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

R = 349.0
K = -0.60117
beta = math.radians(-20.8)

def sag(r):
    return r*r/(R*(1+math.sqrt(max(0.0,1-(1+K)*r*r/(R*R)))))

def rot(x,z):
    # WORLD side-view mapping for B=-20.8 deg. +Z_M points left/up,
    # where the robot is drawn.
    return math.cos(beta)*x + math.sin(beta)*z, -math.sin(beta)*x + math.cos(beta)*z

df = pd.read_csv(ROOT/'sample/circle_in_trajectory.csv')
indices = np.unique(np.linspace(0,len(df)-1,96).astype(int))

fig,(ax_side,ax_top)=plt.subplots(1,2,figsize=(12.5,5.7))
fig.suptitle('修正版：凹面开口朝机器人；外到内同心圆始终同向',fontsize=16,fontweight='bold')

# Side view
x=np.linspace(-325,325,500)
z=np.array([sag(abs(v)) for v in x])
xw,zw=np.vectorize(rot)(x,z)
xt,zt=np.vectorize(rot)(x,np.full_like(x,-35.0))
ax_side.plot(xt,zt,lw=8,color='#888888')
ax_side.plot(xw,zw,lw=5,color='#2878B5')
ax_side.set_aspect('equal')
ax_side.set_xlim(-430,430); ax_side.set_ylim(-100,330)
ax_side.grid(alpha=.18); ax_side.set_title('侧视图：机器人位于凹面开口侧')
ax_side.set_xlabel('WORLD示意 X'); ax_side.set_ylabel('WORLD示意 Z')

# Robot at left/opening side
ax_side.add_patch(Rectangle((-390,-70),85,55,facecolor='#E99A25',edgecolor='none'))
ax_side.plot([-348,-285,-190],[-15,75,135],lw=13,color='#E99A25',solid_capstyle='round')
# centre +Z_M opening direction
v0=np.array(rot(0,0)); vz=np.array(rot(0,120))-v0
vz=vz/np.linalg.norm(vz)
wrist=v0+vz*155
ax_side.plot([-190,wrist[0]],[135,wrist[1]],lw=13,color='#E99A25',solid_capstyle='round')
ax_side.plot([wrist[0],v0[0]],[wrist[1],v0[1]],lw=7,color='#444444',solid_capstyle='round')
ax_side.scatter([v0[0]],[v0[1]],s=38,color='#D9534F',zorder=5)
ax_side.add_patch(FancyArrowPatch(v0,v0+vz*95,arrowstyle='-|>',mutation_scale=16,lw=2.5,color='#2E8B57'))
ax_side.text(v0[0]+vz[0]*105-65,v0[1]+vz[1]*105+10,'+Z_M / 凹面开口',color='#2E8B57')
ax_side.text(-398,-93,'KUKA（机器人侧）')
ax_side.text(95,210,'凹面背离台面、朝向机器人',bbox=dict(boxstyle='round',facecolor='#eef6ff'))

side_point,=ax_side.plot([],[],'o',color='#D9534F',ms=7)
side_normal=ax_side.quiver([],[],[],[],angles='xy',scale_units='xy',scale=1,color='#D9534F',width=.008)

# Top view of actual local path
ax_top.add_patch(Circle((0,0),315,fill=False,lw=2,color='#777777'))
ax_top.plot(df.x_m,df.y_m,lw=.75,color='#9CBFE0')
ax_top.set_xlim(-335,335); ax_top.set_ylim(-335,335); ax_top.set_aspect('equal')
ax_top.grid(alpha=.18); ax_top.set_title('从 +Z_M 开口侧看：所有圆逆时针')
ax_top.set_xlabel('X_M / mm'); ax_top.set_ylabel('Y_M / mm')
# fixed direction symbol
ax_top.add_patch(FancyArrowPatch((270,0),(250,105),connectionstyle='arc3,rad=0.25',arrowstyle='-|>',mutation_scale=18,lw=2.5,color='#6F4AA8'))
ax_top.text(100,290,'每一圈同方向',color='#6F4AA8',fontweight='bold')
trace,=ax_top.plot([],[],lw=2,color='#2878B5')
top_point,=ax_top.plot([],[],'o',color='#D9534F',ms=7)
info=fig.text(.5,.025,'',ha='center',fontsize=11)

def update(n):
    idx=int(indices[n])
    d=df.iloc[idx]
    trace.set_data(df.x_m.iloc[:idx+1],df.y_m.iloc[:idx+1])
    top_point.set_data([d.x_m],[d.y_m])

    radial=math.hypot(d.x_m,d.y_m)
    # Rotate current radial section to local +X for a readable side projection.
    local_x=radial
    px,pz=rot(local_x,d.z_m)
    nx_rad=(d.nx*d.x_m+d.ny*d.y_m)/(radial if radial>1e-9 else 1.0)
    tipx,tipz=rot(local_x+55*nx_rad,d.z_m+55*d.nz)
    side_point.set_data([px],[pz])
    side_normal.set_offsets(np.array([[px,pz]]))
    side_normal.set_UVC(np.array([tipx-px]),np.array([tipz-pz]))
    info.set_text(f'轨迹点 {idx+1}/{len(df)}    半径 r={radial:.1f} mm    每个圆均沿相同逆时针方向')
    return trace,top_point,side_point,side_normal,info

ani=FuncAnimation(fig,update,frames=len(indices),interval=65,blit=False)
fig.tight_layout(rect=(0,0.06,1,.93))
ani.save(VIS/'04_凹面朝机器人_同心圆同向修正版.gif',writer=PillowWriter(fps=12),dpi=90)
update(0)
fig.savefig(VIS/'04_凹面朝机器人_同心圆同向修正版.png',dpi=170,bbox_inches='tight')
plt.close(fig)
print('created preview')
