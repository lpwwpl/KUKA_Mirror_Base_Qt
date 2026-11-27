import math
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Arc, Circle, Polygon, FancyArrowPatch

ROOT = Path('/mnt/data/KUKA_Mirror_Base_Qt5_12_Method1_Enhanced')
VIS = ROOT / 'visuals'
VIS.mkdir(exist_ok=True)
plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

R = 349.0
K = -0.60117
beta = math.radians(-20.8)

def sag(r):
    return r*r / (R*(1+math.sqrt(max(0.0, 1-(1+K)*r*r/(R*R)))))

def rot_y(x,z):
    return math.cos(beta)*x + math.sin(beta)*z, -math.sin(beta)*x + math.cos(beta)*z

# Figure 1: complete method overview
fig, axs = plt.subplots(2,3, figsize=(16,9))
fig.suptitle('方法一：建立真实镜面 BASE，再用局部坐标生成凹面加工轨迹', fontsize=20, fontweight='bold')

# 1 installation
ax=axs[0,0]; ax.set_title('① 镜子放到最终位置', fontweight='bold')
xx=np.linspace(-330,330,300); zz=np.array([sag(abs(x)) for x in xx]); xr,zr=np.vectorize(rot_y)(xx,zz)
xt,zt=np.vectorize(rot_y)(xx,np.full_like(xx,-35))
ax.plot(xt,zt,lw=8,color='#888888',label='-20.8° 台面')
ax.plot(xr,zr,lw=6,color='#2878B5',label='凹面镜')
ax.scatter([0], [0], s=70, color='#D9534F'); ax.text(8,3,'P0：光学顶点')
ax.annotate('Y方向整体移动\n只改变BASE原点位置',xy=(0,0),xytext=(-200,180),arrowprops=dict(arrowstyle='->'))
ax.axis('equal'); ax.grid(alpha=.2); ax.legend(loc='lower right'); ax.set_xlabel('示意 X'); ax.set_ylabel('示意 Z')

# 2 five points top view
ax=axs[0,1]; ax.set_title('② 在 WORLD 下测量五点', fontweight='bold')
ax.add_patch(Circle((0,0),1,fill=False,lw=4,color='#2878B5'))
pts={'P0':(0,0),'PX+':(.72,0),'PX-':(-.72,0),'PY+':(0,.72),'PY-':(0,-.72)}
for name,(x,y) in pts.items():
    ax.scatter([x],[y],s=80,color='#D9534F'); ax.text(x+.05,y+.05,name)
ax.annotate('PX±等半径对称',xy=(.72,0),xytext=(-.1,.92),arrowprops=dict(arrowstyle='->'))
ax.annotate('PY±等半径对称',xy=(0,-.72),xytext=(.2,-1.05),arrowprops=dict(arrowstyle='->'))
ax.set_xlim(-1.25,1.25); ax.set_ylim(-1.25,1.25); ax.set_aspect('equal'); ax.grid(alpha=.2); ax.set_xlabel('镜面局部 X 方向'); ax.set_ylabel('镜面局部 Y 方向')

# 3 vectors/base
ax=axs[0,2]; ax.set_title('③ 计算 X_M、Y_M、Z_M 和 BASE', fontweight='bold')
ax.axis('off')
ax.text(.03,.85,'eX = normalize(PX+ − PX−)',fontsize=13)
ax.text(.03,.67,'vY = PY+ − PY−',fontsize=13)
ax.text(.03,.49,'eY = normalize(vY − (vY·eX)eX)',fontsize=13)
ax.text(.03,.31,'eZ = normalize(eX × eY)',fontsize=13)
ax.text(.03,.10,'R_WM = [ eX  eY  eZ ]     T_WM = P0',fontsize=13,bbox=dict(boxstyle='round',facecolor='#eef6ff',edgecolor='#2878B5'))
ax.annotate('',xy=(.85,.55),xytext=(.60,.55),arrowprops=dict(arrowstyle='->',lw=3,color='#D9534F')); ax.text(.86,.55,'X_M',va='center')
ax.annotate('',xy=(.65,.85),xytext=(.65,.55),arrowprops=dict(arrowstyle='->',lw=3,color='#2878B5')); ax.text(.65,.88,'Z_M',ha='center')
ax.text(.58,.48,'P0',fontweight='bold')

# 4 normals
ax=axs[1,0]; ax.set_title('④ 局部曲面点与逐点法向', fontweight='bold')
ax.plot(xx,zz,lw=5,color='#2878B5')
for x in np.linspace(-280,280,7):
    z=sag(abs(x)); n=np.array([-x, R-(1+K)*z]); n=n/np.linalg.norm(n)
    ax.arrow(x,z,n[0]*45,n[1]*45,head_width=8,head_length=10,length_includes_head=True,color='#2E8B57')
ax.text(-310,190,'中心和边缘法向不同\n所以每个点ABC必须变化',bbox=dict(boxstyle='round',facecolor='#fff4dc'))
ax.axis('equal'); ax.grid(alpha=.2); ax.set_xlabel('x_M / mm'); ax.set_ylabel('z_M / mm')

# 5 circles outside in
ax=axs[1,1]; ax.set_title('⑤ 同心圆从外向内', fontweight='bold')
for r in np.arange(315,0,-30): ax.add_patch(Circle((0,0),r,fill=False,lw=1.8,color='#6F4AA8'))
ax.plot([315,0],[0,0],ls='--',color='#6F4AA8',lw=2)
ax.scatter([315],[0],s=90,color='#2E8B57',label='起点：外圈'); ax.scatter([0],[0],s=90,color='#D9534F',label='终点：中心')
ax.annotate('完整走完一圈\n再连接下一内圈',xy=(225,0),xytext=(-260,220),arrowprops=dict(arrowstyle='->'))
ax.set_xlim(-340,340); ax.set_ylim(-340,340); ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='lower right'); ax.set_xlabel('x_M / mm'); ax.set_ylabel('y_M / mm')

# 6 controller
ax=axs[1,2]; ax.set_title('⑥ KUKA 用 BASE 自动换算', fontweight='bold'); ax.axis('off')
code='''$TOOL = TOOL_DATA[1]\n$BASE = BASE_DATA[5]\n\nLIN {X xM,Y yM,Z zM,\n     A aM,B bM,C cM} C_DIS'''
ax.text(.04,.62,code,family='monospace',fontsize=13,bbox=dict(boxstyle='round',facecolor='#f4f4f4',edgecolor='#999999'))
ax.text(.04,.40,'控制器内部执行：',fontsize=13,fontweight='bold')
ax.text(.04,.28,'P_W = R_WM · P_M + T_WM',fontsize=13)
ax.text(.04,.16,'R_WT = R_WM · R_MT',fontsize=13)
ax.text(.04,.02,'注意：轨迹点B不能再统一减20.8°',fontsize=13,color='#B22222',fontweight='bold')

fig.tight_layout(rect=(0,0,1,.95))
fig.savefig(VIS/'01_方法一完整步骤图.png',dpi=180,bbox_inches='tight')
plt.close(fig)

# Figure 2: five-point/base detailed
fig,axs=plt.subplots(1,2,figsize=(14,6))
fig.suptitle('五点如何确定真实凹镜 BASE',fontsize=19,fontweight='bold')
ax=axs[0]
ax.add_patch(Circle((0,0),1,fill=False,lw=4,color='#2878B5'))
for name,(x,y) in pts.items():
    ax.scatter([x],[y],s=100,color='#D9534F'); ax.text(x+.06,y+.06,name,fontsize=12)
ax.arrow(-.72,0,1.35,0,head_width=.06,head_length=.08,color='#D9534F',length_includes_head=True)
ax.arrow(0,-.72,0,1.35,head_width=.06,head_length=.08,color='#2E8B57',length_includes_head=True)
ax.text(.2,.08,'eX',color='#D9534F',fontsize=14); ax.text(.07,.28,'eY',color='#2E8B57',fontsize=14)
ax.set_title('顶视图：用对称点差向量求 X、Y'); ax.set_aspect('equal'); ax.set_xlim(-1.2,1.2); ax.set_ylim(-1.2,1.2); ax.grid(alpha=.2)

ax=axs[1]
ax.plot(xr,zr,lw=6,color='#2878B5')
p0=np.array(rot_y(0,0)); ex=np.array(rot_y(110,0))-p0; ez=np.array(rot_y(0,110))-p0
ax.arrow(p0[0],p0[1],ex[0],ex[1],head_width=10,head_length=12,color='#D9534F',length_includes_head=True)
ax.arrow(p0[0],p0[1],ez[0],ez[1],head_width=10,head_length=12,color='#2878B5',length_includes_head=True)
ax.scatter([p0[0]],[p0[1]],s=80,color='#D9534F')
ax.text(p0[0]+8,p0[1]-12,'P0 = BASE.XYZ'); ax.text(p0[0]+ex[0]+5,p0[1]+ex[1],'X_M'); ax.text(p0[0]+ez[0]+5,p0[1]+ez[1],'Z_M')
ax.text(-300,210,'BASE.ABC 由 R_WM=[eX eY eZ] 换算\n测试结果：A=0°, B=-20.8°, C=0°',bbox=dict(boxstyle='round',facecolor='#eef6ff'))
ax.set_title('侧视图：BASE 原点和倾斜姿态'); ax.axis('equal'); ax.grid(alpha=.2); ax.set_xlabel('WORLD示意 X'); ax.set_ylabel('WORLD示意 Z')
fig.tight_layout(rect=(0,0,1,.92)); fig.savefig(VIS/'02_五点与BASE方向图.png',dpi=180,bbox_inches='tight'); plt.close(fig)

# Animation from generated trajectory
df=pd.read_csv(ROOT/'sample/circle_in_trajectory.csv')
# sample frames equally over trajectory
frame_indices=np.unique(np.linspace(0,len(df)-1,80).astype(int))
fig,(ax_top,ax_side)=plt.subplots(1,2,figsize=(12,5.5))
fig.suptitle('测试轨迹动画：同心圆从外向内（局部镜面 BASE）',fontsize=16,fontweight='bold')

ax_top.set_xlim(-335,335); ax_top.set_ylim(-335,335); ax_top.set_aspect('equal'); ax_top.grid(alpha=.2); ax_top.set_xlabel('x_M / mm'); ax_top.set_ylabel('y_M / mm'); ax_top.set_title('顶视图')
ax_top.add_patch(Circle((0,0),315,fill=False,color='#777777',lw=2))
ax_top.plot(df.x_m,df.y_m,color='#8FB8DF',lw=.8)
trace_top,=ax_top.plot([],[],color='#2878B5',lw=2)
point_top,=ax_top.plot([],[],'o',color='#D9534F',ms=7)
normal_top=ax_top.quiver([],[],[],[],angles='xy',scale_units='xy',scale=1,color='#2E8B57',width=.006)

x=np.linspace(-330,330,400); z=np.array([sag(abs(v)) for v in x])
ax_side.plot(x,z,color='#2878B5',lw=4); ax_side.set_xlim(-335,335); ax_side.set_ylim(-20,230); ax_side.grid(alpha=.2); ax_side.set_xlabel('径向投影 / mm'); ax_side.set_ylabel('z_M / mm'); ax_side.set_title('剖面与法向（将当前点旋转到X-Z剖面）')
point_side,=ax_side.plot([],[],'o',color='#D9534F',ms=7)
normal_side=ax_side.quiver([],[],[],[],angles='xy',scale_units='xy',scale=1,color='#2E8B57',width=.008)
info=fig.text(.5,.02,'',ha='center',fontsize=11)

def update(frame_no):
    idx=frame_indices[frame_no]
    d=df.iloc[idx]
    trace_top.set_data(df.x_m.iloc[:idx+1],df.y_m.iloc[:idx+1])
    point_top.set_data([d.x_m],[d.y_m])
    normal_top.set_offsets(np.array([[d.x_m,d.y_m]]))
    normal_top.set_UVC(np.array([d.nx*50]),np.array([d.ny*50]))
    radial=math.hypot(d.x_m,d.y_m)
    sign=1 if d.x_m>=0 else -1
    rp=sign*radial
    point_side.set_data([rp],[d.z_m])
    nr=(d.nx*d.x_m+d.ny*d.y_m)/(radial if radial>1e-9 else 1.0)
    normal_side.set_offsets(np.array([[rp,d.z_m]]))
    normal_side.set_UVC(np.array([nr*50]),np.array([d.nz*50]))
    info.set_text(f'点 {idx+1}/{len(df)}    r={radial:.1f} mm    z={d.z_m:.1f} mm    法向=({d.nx:.3f},{d.ny:.3f},{d.nz:.3f})')
    return trace_top,point_top,normal_top,point_side,normal_side,info

ani=FuncAnimation(fig,update,frames=len(frame_indices),interval=50,blit=False)
ani.save(VIS/'03_同心圆外到内轨迹仿真.gif',writer=PillowWriter(fps=12),dpi=85)
plt.close(fig)
print('created', list(VIS.iterdir()))
