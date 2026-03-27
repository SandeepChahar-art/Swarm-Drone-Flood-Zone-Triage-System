#!/usr/bin/env python3
"""
swarm_visualizer.py — Photorealistic Flood Rescue Command Center
Matches screenshot: realistic buildings, water reflections, ripples, dramatic lighting.
Pure pygame — no OpenGL needed.
"""
import rclpy
from rclpy.node import Node
import json, math, threading, sys, random, time
from collections import deque
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

try:
    import pygame
except ImportError:
    print("pip3 install pygame"); sys.exit(1)

# ── Layout ────────────────────────────────────────────────────────────────────
VIEW_W, VIEW_H = 820, 820
CHART_W        = 380
PANEL_W        = 400
TOTAL_W        = VIEW_W + CHART_W + PANEL_W
TOTAL_H        = VIEW_H
FPS            = 30
NUM_DRONES     = 5

WX_MIN, WX_MAX = -1.0, 11.0
WY_MIN, WY_MAX = -1.0, 14.0
WCX = (WX_MIN+WX_MAX)/2
WCY = (WY_MIN+WY_MAX)/2

# ── Palette ───────────────────────────────────────────────────────────────────
BG_DARK  = (  7,  13,  22)
BG_MID   = ( 10,  18,  32)
BG_PANEL = (  8,  16,  28)
ACCENT   = (  0, 200, 175)
ACCENT2  = (  0, 155, 215)
TEXT_DIM = ( 70, 100, 120)
TEXT_MAIN= (195, 215, 228)

SKY_TOP  = (  7,  14,  26)
SKY_BOT  = ( 16,  30,  48)
GROUND_C = ( 25,  20,  12)   # dark mud

# Water - murky brownish-teal flood
WATER_DEEP  = ( 12,  42,  62)
WATER_MID   = ( 18,  58,  80)
WATER_LIGHT = ( 28,  75, 100)
WATER_FOAM  = (145, 175, 195)
WATER_GLINT = (180, 210, 230)

# Building materials
CONC_DARK  = ( 45,  42,  38)   # dark concrete
CONC_MID   = ( 62,  58,  52)   # mid concrete
CONC_LIGHT = ( 82,  76,  68)   # light face
CONC_ROOF  = ( 35,  32,  28)   # roof
BRICK_DARK = ( 55,  30,  20)
BRICK_MID  = ( 72,  40,  28)
WIN_LIT    = (220, 195, 130)   # lit window
WIN_DARK   = ( 30,  35,  45)   # dark window
RAIN_C     = (100, 138, 168)

DRONE_COL=[(0,255,170),(0,200,255),(255,110,50),(255,215,0),(200,80,255)]
P_CRIT=(220,45,45); P_HIGH=(255,148,0); P_MED=(200,200,0)
P_LOW=(45,210,95);  P_DONE=(35,220,115); P_ENRTE=(0,175,255)

GRID_COLS, GRID_ROWS = 12, 15
WIND_X = 0.25   # must match drone_controller.py
WIND_Y = 0.10
coverage_grid=[[0.0]*GRID_COLS for _ in range(GRID_ROWS)]

# ── Flood zones — define water depth only, no boxes ──────────────────────────
FLOOD_ZONES=[
    {"name":"Market Area",    "x":(1.0,4.0),"y":(1.0,5.0), "wh":1.3,"type":"flood"},
    {"name":"Residential",    "x":(5.0,9.0),"y":(1.0,6.0), "wh":0.8,"type":"flood"},
    {"name":"Low-lying Area", "x":(1.5,4.5),"y":(7.0,13.0),"wh":1.5,"type":"swamp"},
    {"name":"Main Road",      "x":(4.0,8.0),"y":(6.0,10.0),"wh":0.6,"type":"flood"},
]

# ── Buildings: (cx, cy, w, d, h, damage, style) ───────────────────────────────
# style: 0=concrete tower, 1=brick low-rise, 2=warehouse
BUILDINGS=[
    (0.3,1.5,  0.8,0.65,3.2,0.85,0),(2.2,0.5,  1.0,0.80,3.8,0.45,1),
    (3.5,2.2,  0.7,0.65,2.4,0.90,0),(5.5,0.4,  1.2,1.00,4.0,0.28,0),
    (7.5,1.2,  0.9,0.80,3.0,0.55,1),(8.8,2.5,  0.8,0.68,2.2,0.65,2),
    (9.5,5.0,  1.0,0.90,3.2,0.38,0),(0.5,6.0,  0.8,0.68,2.6,0.82,1),
    (2.8,8.5,  1.1,0.90,3.4,0.55,0),(4.5,11.5, 1.0,0.90,3.0,0.48,1),
    (1.0,12.5, 0.9,0.80,2.4,0.88,0),(7.0,8.0,  1.0,0.80,3.2,0.38,0),
    (9.0,9.5,  0.8,0.68,2.5,0.65,2),(6.5,12.0, 0.9,0.80,2.8,0.52,1),
    (3.0,13.5, 0.8,0.68,2.0,0.78,0),(8.5,12.5, 0.9,0.80,2.8,0.42,0),
]

random.seed(42)
DEBRIS=[(random.uniform(WX_MIN+.4,WX_MAX-.4),
         random.uniform(WY_MIN+.4,WY_MAX-.4),
         random.uniform(.04,.18)) for _ in range(80)]
RAIN_DROPS=[[random.randint(0,VIEW_W),random.randint(0,VIEW_H),
             random.uniform(9,18),random.uniform(2.0,4.2)] for _ in range(380)]
RIPPLE_PTS=[(random.uniform(WX_MIN,WX_MAX),
             random.uniform(WY_MIN,WY_MAX),
             random.uniform(0,2*math.pi),
             random.uniform(.5,2.0)) for _ in range(200)]

def _dot3(ax,ay,az,bx,by,bz): return ax*bx+ay*by+az*bz
def _cross3(ax,ay,az,bx,by,bz): return (ay*bz-az*by,az*bx-ax*bz,ax*by-ay*bx)
def _norm3(x,y,z):
    d=math.sqrt(x*x+y*y+z*z); return (x/d,y/d,z/d) if d>1e-9 else (0.,0.,1.)
def blend(col,a,bg=BG_DARK):
    return tuple(max(0,min(255,int(col[i]*a+bg[i]*(1-a)))) for i in range(3))
def prio_col(p): return{"CRITICAL":P_CRIT,"HIGH":P_HIGH,"MEDIUM":P_MED,"LOW":P_LOW}.get(p,P_LOW)
def batt_col(b): return (50,220,100) if b>50 else (255,200,0) if b>25 else (220,60,60)
def surv_col(s): return P_LOW if s>=.6 else (P_MED if s>=.35 else P_CRIT)

# ── Camera ────────────────────────────────────────────────────────────────────
class Camera3D:
    def __init__(self): self.reset()
    def reset(self):
        self.theta=215.; self.phi=28.; self.dist=26.
        self.tx=WCX; self.ty=WCY; self.tz=0.; self.fov=50.
        self._drag=False; self._last=(0,0)
    def on_down(self,p): self._drag=True; self._last=p
    def on_up(self): self._drag=False
    def on_move(self,p):
        if not self._drag: return
        dx=p[0]-self._last[0]; dy=p[1]-self._last[1]
        self.theta+=dx*.5; self.phi=max(8.,min(75.,self.phi-dy*.4)); self._last=p
    def on_scroll(self,d): self.dist=max(3.,min(60.,self.dist-d*2.0))
    def _axes(self):
        cx=self.tx+self.dist*math.cos(math.radians(self.phi))*math.cos(math.radians(self.theta))
        cy=self.ty+self.dist*math.cos(math.radians(self.phi))*math.sin(math.radians(self.theta))
        cz=self.tz+self.dist*math.sin(math.radians(self.phi))
        fwd=_norm3(self.tx-cx,self.ty-cy,self.tz-cz)
        right=_norm3(*_cross3(*fwd,0.,0.,1.))
        up2=_norm3(*_cross3(*right,*fwd))
        return cx,cy,cz,fwd,right,up2
    def project(self,wx,wy,wz):
        cx,cy,cz,fwd,right,up2=self._axes()
        dx,dy,dz=wx-cx,wy-cy,wz-cz
        cam_x=_dot3(dx,dy,dz,*right); cam_y=_dot3(dx,dy,dz,*up2); cam_z=_dot3(dx,dy,dz,*fwd)
        if cam_z<.1: return None
        f=(VIEW_H/2)/math.tan(math.radians(self.fov/2))
        return (int(VIEW_W/2+cam_x/cam_z*f), int(VIEW_H/2-cam_y/cam_z*f))
    def depth(self,wx,wy,wz):
        cx,cy,cz,*_=self._axes(); return math.sqrt((wx-cx)**2+(wy-cy)**2+(wz-cz)**2)
    def ray_ground(self,mx,my):
        cx,cy,cz,fwd,right,up2=self._axes()
        f=(VIEW_H/2)/math.tan(math.radians(self.fov/2))
        rx=(mx-VIEW_W/2)/f; ry=-(my-VIEW_H/2)/f
        dx=fwd[0]+right[0]*rx+up2[0]*ry; dy=fwd[1]+right[1]*rx+up2[1]*ry; dz=fwd[2]+right[2]*rx+up2[2]*ry
        if abs(dz)<1e-6: return None
        t=-cz/dz; return (cx+dx*t,cy+dy*t) if t>0 else None

# ── ROS Node ──────────────────────────────────────────────────────────────────
class VisNode(Node):
    def __init__(self,n):
        super().__init__('swarm_visualizer')
        self.n=n; self.lock=threading.Lock()
        self.poses,self.statuses,self.vels={},{},{}
        self.centre=(WCX,WCY,2.0); self.tasks={}; self._cc=0
        for i in range(n):
            self.create_subscription(PoseStamped,f'/drone_{i}/pose',   lambda m,d=i:self._pose(m,d),10)
            self.create_subscription(String,      f'/drone_{i}/status', lambda m,d=i:self._stat(m,d),10)
            self.create_subscription(Twist,       f'/drone_{i}/cmd_vel',lambda m,d=i:self._vel(m,d),10)
        self.create_subscription(PoseStamped,'/swarm/centre',       self._centre,10)
        self.create_subscription(String,     '/swarm/triage_data',  self._triage,10)
        self.create_subscription(PoseStamped,'/swarm/detected_task',self._basic,10)
        self.task_pub=self.create_publisher(PoseStamped,'/swarm/detected_task',10)
    def _pose(self,m,d):
        with self.lock: self.poses[d]=(m.pose.position.x,m.pose.position.y,m.pose.position.z)
    def _stat(self,m,d):
        try:
            data=json.loads(m.data)
            with self.lock:
                self.statuses[d]=data
                if data.get('state')=='task_complete':
                    for k in list(self.tasks):
                        if self.tasks[k].get('task_id')==data.get('task_id'): self.tasks[k]['state']='complete'
                elif data.get('state')=='tasked':
                    for k in list(self.tasks):
                        if self.tasks[k].get('task_id')==data.get('task_id'): self.tasks[k]['state']='assigned'
        except: pass
    def _vel(self,m,d):
        with self.lock: self.vels[d]=(m.linear.x,m.linear.y,getattr(m.linear,'z',0.))
    def _centre(self,m):
        with self.lock: self.centre=(m.pose.position.x,m.pose.position.y,m.pose.position.z)
    def _triage(self,m):
        try:
            data=json.loads(m.data); key=f"task_{data['task_id']}"
            with self.lock:
                if key not in self.tasks: self.tasks[key]=data
        except: pass
    def _basic(self,m):
        try:
            parts=m.header.frame_id.split(':'); tid=int(parts[0]); urg=float(parts[1]) if len(parts)>1 else .5
        except: tid=1000+self._cc; urg=.5
        key=f"task_{tid}"
        with self.lock:
            if key not in self.tasks:
                self.tasks[key]={"task_id":tid,"label":f"manual_{tid}","person_name":"Survivor",
                    "age_group":"?","description":"Manually reported location",
                    "x":m.pose.position.x,"y":m.pose.position.y,"z":0.,"zone":"Manual",
                    "water_level":.5,"water_category":"Knee Deep","water_height_m":.6,
                    "condition":.5,"survival_now":.55,"survival_if_rescued":.90,
                    "urgency":urg,"priority":"HIGH","state":"pending","spawn_time":0}
    def spawn(self,wx,wy):
        tid=1000+self._cc; self._cc+=1
        msg=PoseStamped(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id=f"{tid}:0.6"
        msg.pose.position.x=float(wx); msg.pose.position.y=float(wy); msg.pose.position.z=0.
        self.task_pub.publish(msg)
    def snap(self):
        with self.lock:
            return (dict(self.poses),dict(self.statuses),dict(self.vels),
                    self.centre,{k:dict(v) for k,v in self.tasks.items()})

# ── Chart helpers ─────────────────────────────────────────────────────────────
def section_hdr(surf,x,y,w,text,font):
    pygame.draw.rect(surf,(0,55,85),(x,y,w,26))
    pygame.draw.rect(surf,(0,200,175),(x,y,5,26))
    surf.blit(font.render(text,True,ACCENT),(x+10,y+4))
    return y+30

def draw_line_chart(surf,rect,series,colors,labels,title,fs,ft,y_max=100):
    x,y,w,h=rect
    pygame.draw.rect(surf,(8,18,32),(x,y,w,h))
    pygame.draw.rect(surf,(0,50,80),(x,y,w,h),1)
    surf.blit(ft.render(title,True,ACCENT),(x+5,y+4))
    oy=y+22; oh=h-34
    for gi in range(5):
        gy=oy+oh-int(oh*gi/4)
        pygame.draw.line(surf,(16,36,54),(x+22,gy),(x+w-4,gy),1)
        surf.blit(fs.render(str(int(y_max*gi/4)),True,TEXT_DIM),(x+2,gy-5))
    for pts,col in zip(series,colors):
        if len(pts)<2: continue
        n=len(pts)
        spts=[(x+22+int(i/max(n-1,1)*(w-26)),oy+oh-int(min(max(v,0),y_max)/y_max*oh)) for i,v in enumerate(pts)]
        # Fill under line
        fill_pts=spts+[(spts[-1][0],oy+oh),(spts[0][0],oy+oh)]
        s2=pygame.Surface((VIEW_W+CHART_W,VIEW_H),pygame.SRCALPHA)
        pygame.draw.polygon(s2,(*col,25),fill_pts)
        surf.blit(s2,(0,0))
        pygame.draw.lines(surf,col,False,spts,2)
        surf.blit(fs.render(f'{pts[-1]:.0f}',True,col),(spts[-1][0]+2,spts[-1][1]-8))
    surf.blit(fs.render('0',True,TEXT_DIM),(x+22,oy+oh+2))
    surf.blit(fs.render('2 sec',True,TEXT_DIM),(x+w//2-25,oy+oh+2))
    lx=x+5
    for col,lbl in zip(colors,labels):
        pygame.draw.line(surf,col,(lx,y+h-8),(lx+14,y+h-8),2)
        surf.blit(fs.render(f'— {lbl}',True,col),(lx,y+h-12)); lx+=len(lbl)*5+38

def draw_bar_line_chart(surf,rect,bars,line,bcol,lcol,title,fs,ft):
    x,y,w,h=rect
    pygame.draw.rect(surf,(8,18,32),(x,y,w,h))
    pygame.draw.rect(surf,(0,50,80),(x,y,w,h),1)
    surf.blit(ft.render(title,True,ACCENT),(x+5,y+4))
    oy=y+22; oh=h-38
    if not bars: return
    bmax=max(max(bars),1); lmax=max(max(line),1) if line else 1
    n=len(bars); bw=max(5,(w-28)//(n+1))
    for gi in range(4):
        gy=oy+oh-int(oh*gi/3); pygame.draw.line(surf,(16,36,54),(x+4,gy),(x+w-4,gy),1)
        surf.blit(fs.render(str(int(bmax*gi/3)),True,TEXT_DIM),(x+2,gy-5))
    for i,v in enumerate(bars):
        bx=x+16+i*(bw+3); bh=max(1,int(v/bmax*oh))
        # Gradient bar
        for bi in range(bh):
            t=bi/bh; c=tuple(int(bcol[j]*(0.5+t*0.5)) for j in range(3))
            pygame.draw.line(surf,c,(bx,oy+oh-bi),(bx+bw,oy+oh-bi))
        pygame.draw.rect(surf,(*bcol,100),(bx,oy+oh-bh,bw,bh),1)
    if len(line)>=2:
        lpts=[(x+16+i*(bw+3)+bw//2, oy+oh-int(v/lmax*oh)) for i,v in enumerate(line)]
        pygame.draw.lines(surf,lcol,False,lpts,2)
        pygame.draw.circle(surf,lcol,lpts[-1],4)
        for lp in lpts: pygame.draw.circle(surf,lcol,lp,2)
    surf.blit(fs.render('Time',True,TEXT_DIM),(x+4,oy+oh+3))
    surf.blit(fs.render('16 sec',True,TEXT_DIM),(x+w//2-20,oy+oh+3))
    surf.blit(fs.render('Time',True,TEXT_DIM),(x+w-26,oy+oh+3))

def draw_donut(surf,cx,cy,r,data,colors,labels,fs):
    total=sum(data) if sum(data)>0 else 1
    angle=-math.pi/2
    for v,col in zip(data,colors):
        sweep=2*math.pi*v/total
        pts=[(cx,cy)]
        steps=max(2,int(sweep*28))
        for s in range(steps+1):
            a=angle+sweep*s/steps
            pts.append((int(cx+r*math.cos(a)),int(cy+r*math.sin(a))))
        if len(pts)>2:
            pygame.draw.polygon(surf,col,pts)
            # Edge highlight
            for s in range(steps):
                a1=angle+sweep*s/steps; a2=angle+sweep*(s+1)/steps
                p1=(int(cx+r*math.cos(a1)),int(cy+r*math.sin(a1)))
                p2=(int(cx+r*math.cos(a2)),int(cy+r*math.sin(a2)))
                pygame.draw.line(surf,blend(col,1.3,(255,255,255)),p1,p2,1)
        angle+=sweep
    pygame.draw.circle(surf,(8,18,32),(cx,cy),int(r*.56))
    pygame.draw.circle(surf,(0,65,90),(cx,cy),int(r*.56),1)
    ly=cy-r//2
    for col,lbl,v in zip(colors,labels,data):
        pct=int(v/total*100)
        pygame.draw.rect(surf,col,(cx+r+6,ly,10,10))
        surf.blit(fs.render(lbl,True,TEXT_MAIN),(cx+r+20,ly))
        surf.blit(fs.render(f'{pct}%',True,col),(cx+r+20+len(lbl)*5+2,ly))
        ly+=16

def draw_heatmap(surf,rect,grid,fs,ft,title):
    x,y,w,h=rect
    pygame.draw.rect(surf,(8,18,32),(x,y,w,h))
    pygame.draw.rect(surf,(0,50,80),(x,y,w,h),1)
    surf.blit(ft.render(title,True,ACCENT),(x+5,y+4))
    rows=len(grid); cols=len(grid[0]) if rows else 1
    cw=(w-8)//cols; ch=(h-28)//rows
    for r in range(rows):
        for c in range(cols):
            v=min(grid[r][c],1.)
            if v>.01: col=(int(v*5),int(v*165+15),int(v*160+20))
            else: col=(8,18,30)
            pygame.draw.rect(surf,col,(x+4+c*cw,y+24+r*ch,cw-1,ch-1))
    for i in range(w-8):
        t=i/(w-8); c=(int(t*5),int(t*165+15),int(t*160+20))
        pygame.draw.line(surf,c,(x+4+i,y+h-9),(x+4+i,y+h-2))
    for lbl,pos in [('10%',0),('25%',(w-8)//4),('50%',(w-8)//2-6),('75%',3*(w-8)//4),('100%',w-30)]:
        surf.blit(fs.render(lbl,True,TEXT_DIM),(x+4+pos,y+h-12))

def draw_sparkline(surf,rect,pts,col):
    if len(pts)<2: return
    x,y,w,h=rect; mn=min(pts); mx=max(pts)+.001; rng=mx-mn
    sp=[(x+int(i/(len(pts)-1)*w), y+h-int((p-mn)/rng*h)) for i,p in enumerate(pts)]
    pygame.draw.lines(surf,col,False,sp,1)

# ══════════════════════════════════════════════════════════════════════════════
#  Main Renderer
# ══════════════════════════════════════════════════════════════════════════════
class Renderer:
    def __init__(self,n):
        pygame.init()
        self.n=n
        pygame.display.set_caption('Flood Rescue Command — UAV Swarm')
        # Get actual screen resolution and go maximized
        info = pygame.display.Info()
        sw, sh = info.current_w, info.current_h
        self.screen = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)
        # Recalculate layout to fill screen
        global VIEW_W, VIEW_H, CHART_W, PANEL_W, TOTAL_W, TOTAL_H
        PANEL_W = 330
        CHART_W = int((sw - PANEL_W) * 0.40)
        VIEW_W  = sw - CHART_W - PANEL_W
        VIEW_H  = sh
        TOTAL_W = sw
        TOTAL_H = sh
        self.clock=pygame.time.Clock()
        # Pick best available monospace font
        _avail = pygame.font.get_fonts()
        _fn = next((f for f in ['dejavusansmono','couriernew','liberationmono','freemono'] if f in _avail), None)
        def _f(sz, bold=False):
            if _fn: return pygame.font.SysFont(_fn, sz, bold=bold)
            return pygame.font.SysFont('monospace', sz, bold=bold)
        self.f7  = _f(11)
        self.f8  = _f(13)
        self.f9  = _f(14)
        self.f10 = _f(15, bold=True)
        self.f11 = _f(17, bold=True)
        self.f14 = _f(20, bold=True)
        self.cam  =Camera3D()
        self.trail={i:[] for i in range(n)}
        self.tick=0; self.rotor=0.
        MAX=60
        self.surv_i=deque([50.]*5,maxlen=MAX)
        self.surv_e=deque([35.]*5,maxlen=MAX)
        self.surv_a=deque([70.]*5,maxlen=MAX)
        self.rescue_h=deque([0]*8,maxlen=8)
        self.deploy_h=deque([0]*8,maxlen=8)
        self._lct=0
        self.minimap=pygame.Surface((160,160))
        # Pre-build random seeds per building for windows/cracks
        self._bseeds={i:random.Random(i*137+42) for i in range(len(BUILDINGS))}

    def _p(self,wx,wy,wz=0.): return self.cam.project(wx,wy,wz)

    # ── Main draw ──────────────────────────────────────────────────────────────
    def draw(self,poses,statuses,vels,centre,tasks):
        self.tick+=1; self.rotor=(self.rotor+7)%360
        for d,p in poses.items():
            self.trail[d].append(p)
            if len(self.trail[d])>80: self.trail[d].pop(0)
            gx=int((p[0]-WX_MIN)/(WX_MAX-WX_MIN)*GRID_COLS)
            gy=int((p[1]-WY_MIN)/(WY_MAX-WY_MIN)*GRID_ROWS)
            if 0<=gx<GRID_COLS and 0<=gy<GRID_ROWS:
                coverage_grid[gy][gx]=min(1.,coverage_grid[gy][gx]+.015)

        now=time.time()
        if now-self._lct>2:
            self._lct=now
            pend=[t for t in tasks.values() if t.get('state')!='complete']
            done=[t for t in tasks.values() if t.get('state')=='complete']
            inf_s=[t.get('survival_now',.5)*100 for t in pend if 'infant' in t.get('label','')]
            eld_s=[t.get('survival_now',.5)*100 for t in pend if any(k in t.get('label','') for k in ['elderly','cardiac'])]
            adu_s=[t.get('survival_now',.5)*100 for t in pend]
            self.surv_i.append(sum(inf_s)/len(inf_s) if inf_s else max(5.,self.surv_i[-1]-.5))
            self.surv_e.append(sum(eld_s)/len(eld_s) if eld_s else max(5.,self.surv_e[-1]-.5))
            self.surv_a.append(sum(adu_s)/len(adu_s) if adu_s else max(5.,self.surv_a[-1]-.3))
            self.rescue_h.append(len(done))
            self.deploy_h.append(sum(1 for s in statuses.values() if s.get('state')=='tasked'))

        view=pygame.Surface((VIEW_W,VIEW_H))
        # Sky gradient — bake once then reuse
        if not hasattr(self,'_sky') or self._sky.get_size()!=(VIEW_W,VIEW_H):
            self._sky=pygame.Surface((VIEW_W,VIEW_H))
            for _r in range(VIEW_H):
                _t=_r/VIEW_H; _c=tuple(int(SKY_TOP[i]*(1-_t)+SKY_BOT[i]*_t) for i in range(3))
                pygame.draw.line(self._sky,_c,(0,_r),(VIEW_W,_r))
        view.blit(self._sky,(0,0))

        self._ground(view)
        self._water_base(view)
        self._debris(view)
        self._buildings(view)           # buildings WITH reflections inside
        self._water_surface(view)       # water on top after buildings drawn below
        self._ripples(view)
        self._grid(view)
        self._trails(view)
        self._formation(view,poses)
        self._tasks(view,tasks)
        self._drones(view,poses,statuses,vels)
        self._rain(view)
        self._wind_arrow(view)
        self._minimap_build(poses,tasks)
        self._minimap_blit(view)
        view.blit(self.f7.render('LDrag:rotate  Scroll:zoom  R:reset  Click:spawn  ESC:quit',
                                  True,(35,62,85)),(6,VIEW_H-11))
        self.screen.blit(view,(0,0))
        self._chart_panel(tasks,statuses)
        self._command_panel(statuses,tasks)
        pygame.display.flip(); self.clock.tick(FPS)

    # ── Ground ─────────────────────────────────────────────────────────────────
    def _ground(self,surf):
        pts=[self._p(WX_MIN,WY_MIN,-.01),self._p(WX_MAX,WY_MIN,-.01),
             self._p(WX_MAX,WY_MAX,-.01),self._p(WX_MIN,WY_MAX,-.01)]
        if all(pts): pygame.draw.polygon(surf,GROUND_C,pts)

    def _grid(self,surf):
        gc=(22,40,58)
        for wx in range(int(WX_MIN),int(WX_MAX)+1):
            a=self._p(wx,WY_MIN,0); b=self._p(wx,WY_MAX,0)
            if a and b: pygame.draw.line(surf,gc,a,b,1)
        for wy in range(int(WY_MIN),int(WY_MAX)+1):
            a=self._p(WX_MIN,wy,0); b=self._p(WX_MAX,wy,0)
            if a and b: pygame.draw.line(surf,gc,a,b,1)

    # ── Water base (muddy ground under water) ──────────────────────────────────
    def _water_base(self,surf):
        for z in FLOOD_ZONES:
            x0,x1=z['x']; y0,y1=z['y']
            gp=[self._p(x0,y0,0),self._p(x1,y0,0),self._p(x1,y1,0),self._p(x0,y1,0)]
            if all(gp):
                s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
                mc=(35,28,15) if z['type']=='flood' else (22,35,18)
                pygame.draw.polygon(s,(*mc,130),gp); surf.blit(s,(0,0))

    # ── Buildings with photorealistic rendering and water reflections ──────────
    def _buildings(self,surf):
        sb=sorted(enumerate(BUILDINGS),key=lambda b:self.cam.depth(b[1][0],b[1][1],b[1][4]/2),reverse=True)

        for idx,(cx,cy,w,d,h,dmg,style) in sb:
            x0,x1=cx-w/2,cx+w/2; y0,y1=cy-d/2,cy+d/2
            ah=h*(1-.45*dmg)

            # Material colours based on style
            if style==0:   # concrete tower
                fc_front=CONC_MID; fc_side=CONC_DARK; fc_roof=CONC_ROOF
                fc_win_lit=WIN_LIT; fc_win_dk=WIN_DARK
            elif style==1: # brick
                fc_front=BRICK_MID; fc_side=BRICK_DARK; fc_roof=(40,30,22)
                fc_win_lit=(230,200,140); fc_win_dk=(25,30,40)
            else:          # warehouse
                fc_front=(50,52,48); fc_side=(35,37,33); fc_roof=(28,30,26)
                fc_win_lit=WIN_LIT; fc_win_dk=(20,25,35)

            # Damage-darken
            dk=dmg*.35
            fc_front=tuple(max(0,int(c*(1-dk))) for c in fc_front)
            fc_side =tuple(max(0,int(c*(1-dk*.7))) for c in fc_side)

            # Find if in flood zone
            in_flood=False; fwh=0.
            for z in FLOOD_ZONES:
                if z['x'][0]<=cx<=z['x'][1] and z['y'][0]<=cy<=z['y'][1]:
                    in_flood=True; fwh=z['wh']; break

            # ── WATER REFLECTION (draw below water surface, mirrored) ──────────
            if in_flood and fwh>0:
                refl_faces=[
                    [(x0,y0,2*fwh),(x1,y0,2*fwh),(x1,y0,2*fwh-ah),(x0,y0,2*fwh-ah)],
                    [(x1,y0,2*fwh),(x1,y1,2*fwh),(x1,y1,2*fwh-ah),(x1,y0,2*fwh-ah)],
                ]
                shimmer=int(38+18*abs(math.sin(self.tick*.04+cx)))
                for rf,orig_c in zip(refl_faces,[fc_front,fc_side]):
                    rpts=[self._p(*v) for v in rf]
                    if all(rpts):
                        # Desaturate, darken, blue-tint reflection
                        avg=sum(orig_c)//3
                        rc=(max(0,avg//4+8), max(0,avg//3+18), max(0,avg//2+35))
                        s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
                        pygame.draw.polygon(s,(*rc,shimmer),rpts)
                        surf.blit(s,(0,0))

                # Vertical reflection streak on water
                bcp=self._p(cx,y0,fwh)
                bcp2=self._p(cx,y0,fwh-.6)
                if bcp and bcp2:
                    for si in range(3):
                        ox=random.Random(idx*10+si).randint(-4,4)
                        streak_alpha=int(20+15*abs(math.sin(self.tick*.06+si)))
                        s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
                        pygame.draw.line(s,(*WATER_LIGHT,streak_alpha),
                                        (bcp[0]+ox,bcp[1]),(bcp2[0]+ox,bcp2[1]),2)
                        surf.blit(s,(0,0))

            # ── ACTUAL BUILDING FACES ──────────────────────────────────────────
            # Back faces first
            faces_back=[
                ([(x0,y1,0),(x0,y0,0),(x0,y0,ah),(x0,y1,ah)], fc_side),
                ([(x1,y1,0),(x0,y1,0),(x0,y1,ah),(x1,y1,ah)], fc_side),
            ]
            faces_front=[
                ([(x0,y0,0),(x1,y0,0),(x1,y0,ah),(x0,y0,ah)], fc_front),
                ([(x1,y0,0),(x1,y1,0),(x1,y1,ah),(x1,y0,ah)], fc_side),
            ]

            for face_list in [faces_back, faces_front]:
                for face,fc in face_list:
                    pts=[self._p(*v) for v in face]; 
                    if not all(pts): continue
                    pygame.draw.polygon(surf,fc,pts)
                    # Edge lines
                    pygame.draw.polygon(surf,tuple(max(0,c-18) for c in fc),pts,1)

            # ── WINDOWS ──────────────────────────────────────────────────────
            rs=self._bseeds[idx % len(BUILDINGS)]
            n_win_x=max(1,int(w*4)); n_win_z=max(1,int(ah*2.5))
            for wi in range(n_win_x):
                for wj in range(n_win_z):
                    seed=idx*1000+wi*100+wj
                    rw=random.Random(seed)
                    lit=rw.random()>(dmg*.6+.2)
                    wc=fc_win_lit if lit else fc_win_dk
                    # Flicker slightly
                    if lit and rw.random()<.05:
                        flicker=abs(math.sin(self.tick*.3+seed))
                        wc=tuple(int(wc[j]*(.7+flicker*.3)) for j in range(3))
                    wxp=x0+w*(wi+.5)/n_win_x; wzp=ah*.15+ah*.75*(wj+.5)/n_win_z
                    wp=self._p(wxp,y0,wzp)
                    if wp:
                        wr=max(1,int(3/max(self.cam.depth(wxp,y0,wzp)/5,1)))
                        pygame.draw.rect(surf,wc,(wp[0]-wr,wp[1]-int(wr*1.4),wr*2,int(wr*2.8)))
                        if lit:
                            s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
                            pygame.draw.rect(s,(*wc,35),(wp[0]-wr*2,wp[1]-wr*3,wr*4,wr*6))
                            surf.blit(s,(0,0))

            # Roof
            roof=[self._p(x0,y0,ah),self._p(x1,y0,ah),self._p(x1,y1,ah),self._p(x0,y1,ah)]
            if all(roof):
                pygame.draw.polygon(surf,fc_roof,roof)
                pygame.draw.polygon(surf,tuple(max(0,c-15) for c in fc_roof),roof,1)

            # ── CRACKS ───────────────────────────────────────────────────────
            if dmg>.4:
                for ci in range(int(dmg*4)):
                    rc2=random.Random(idx*50+ci)
                    tx0=x0+rc2.uniform(.1,.9)*(x1-x0); tz0=rc2.uniform(.1,.85)*ah
                    cp0=self._p(tx0,y0,tz0)
                    cp1=self._p(tx0+rc2.uniform(-.12,.12),y0,min(ah,tz0+rc2.uniform(-.2,.3)*ah))
                    if cp0 and cp1: pygame.draw.line(surf,(15,12,8),cp0,cp1,1)

            # ── RUBBLE ───────────────────────────────────────────────────────
            if dmg>.65:
                rh=h*dmg*.28
                for ri in range(5):
                    rb=random.Random(idx*200+ri)
                    rp=self._p(x0+rb.uniform(0,w),y0+rb.uniform(-.08,d+.08),rh*rb.uniform(.2,.9))
                    if rp:
                        rsz=max(2,int(5/max(self.cam.depth(cx,cy,rh/2)/5,1)))
                        pygame.draw.circle(surf,(58,48,32),rp,rsz)

            if dmg>.72:
                rp=self._p(cx,cy,ah+.1)
                if rp:
                    s=surf.subsurface(pygame.Rect(max(0,rp[0]-25),max(0,rp[1]-6),80,14)) if \
                      0<=rp[0]-25 and rp[0]+55<VIEW_W and 0<=rp[1]-6 and rp[1]+8<VIEW_H else None
                    surf.blit(self.f7.render('COLLAPSED',True,(138,68,32)),(rp[0]-24,rp[1]))

    # ── Water surface (translucent, drawn AFTER buildings) ─────────────────────
    def _water_surface(self,surf):
        wave=math.sin(self.tick*.038)*.048
        s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
        for z in FLOOD_ZONES:
            x0,x1=z['x']; y0,y1=z['y']; wh=z['wh']
            is_swamp=z['type']=='swamp'
            wbase=WATER_DEEP if not is_swamp else (15,45,28)
            ripple=.32+.08*math.sin(self.tick*.038+x0*1.3)

            # Main water polygon
            wp=[self._p(x0,y0,wh+wave),self._p(x1,y0,wh+wave),
                self._p(x1,y1,wh+wave),self._p(x0,y1,wh+wave)]
            if all(wp):
                pygame.draw.polygon(s,(*wbase,int(ripple*185)),wp)

            # Sub-divide into strips for depth gradient effect
            steps=6
            for si in range(steps):
                t0=si/steps; t1=(si+1)/steps
                ym0=y0+t0*(y1-y0); ym1=y0+t1*(y1-y0)
                sp=[self._p(x0,ym0,wh+wave*.5),self._p(x1,ym0,wh+wave*.5),
                    self._p(x1,ym1,wh+wave),   self._p(x0,ym1,wh+wave)]
                if all(sp):
                    depth=t0  # darker near camera (front = higher t)
                    wc=tuple(int(wbase[j]*(1+depth*.4)) for j in range(3))
                    pygame.draw.polygon(s,(*wc,int(ripple*40)),sp)

            # Shoreline highlight
            if all(wp):
                shore=tuple(min(255,c+30) for c in wbase)
                pygame.draw.polygon(surf,shore,wp,1)

            # Light glint streaks
            glint_alpha=int(30+25*abs(math.sin(self.tick*.06+x0)))
            for gi in range(4):
                gx=x0+(gi+.5)*(x1-x0)/4
                gy=y0+(y1-y0)*.5
                gp0=self._p(gx-.3,gy,wh+wave+.02); gp1=self._p(gx+.3,gy,wh+wave+.02)
                if gp0 and gp1:
                    pygame.draw.line(surf,(*WATER_GLINT,glint_alpha),gp0,gp1,1)

            # Zone label
            lp=self._p((x0+x1)/2,(y0+y1)/2,wh+.28)
            if lp:
                c=(50,128,58) if is_swamp else (48,105,148)
                surf.blit(self.f7.render(z['name'],True,c),(lp[0]-18,lp[1]))

        surf.blit(s,(0,0))

    # ── Ripples ────────────────────────────────────────────────────────────────
    def _ripples(self,surf):
        t=self.tick*.028
        for wx,wy,phase,speed in RIPPLE_PTS:
            in_z=any(z['x'][0]<=wx<=z['x'][1] and z['y'][0]<=wy<=z['y'][1] for z in FLOOD_ZONES)
            if not in_z: continue
            wh=next((z['wh'] for z in FLOOD_ZONES if z['x'][0]<=wx<=z['x'][1] and z['y'][0]<=wy<=z['y'][1]),0)
            age=(t*speed+phase)%(2*math.pi)
            r_size=max(1,int(3+4*age/(2*math.pi)))
            alpha=int(120*(1-age/(2*math.pi)))
            fp=self._p(wx,wy,wh+.04)
            if fp and alpha>10:
                s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
                pygame.draw.ellipse(s,(*WATER_FOAM,alpha),
                    (fp[0]-r_size*2,fp[1]-r_size,r_size*4,r_size*2),1)
                surf.blit(s,(0,0))

    # ── Debris ─────────────────────────────────────────────────────────────────
    def _debris(self,surf):
        for dx,dy,dz in DEBRIS:
            dp=self._p(dx,dy,dz)
            if dp:
                r=max(2,int(4/max(self.cam.depth(dx,dy,dz)/7,1)))
                pygame.draw.circle(surf,(52,42,24),dp,r)
                if r>2:
                    a=math.radians((dx*17+dy*13)%180)
                    pygame.draw.line(surf,(62,50,28),dp,(int(dp[0]+r*1.3*math.cos(a)),int(dp[1]+r*1.3*math.sin(a))),1)

    # ── Trails ─────────────────────────────────────────────────────────────────
    def _trails(self,surf):
        for d,hist in self.trail.items():
            if len(hist)<2: continue
            col=DRONE_COL[d%len(DRONE_COL)]
            for i in range(1,len(hist)):
                a=self._p(*hist[i-1]); b=self._p(*hist[i])
                if a and b: pygame.draw.line(surf,blend(col,i/len(hist)*.45,SKY_BOT),a,b,1)

    def _formation(self,surf,poses):
        ids=sorted(poses.keys())
        for i in range(len(ids)):
            for j in range(i+1,len(ids)):
                a=self._p(*poses[ids[i]]); b=self._p(*poses[ids[j]])
                if a and b: pygame.draw.line(surf,(15,38,62),a,b,1)

    # ── Tasks ──────────────────────────────────────────────────────────────────
    def _tasks(self,surf,tasks):
        st=sorted(tasks.items(),key=lambda kv:self.cam.depth(kv[1]['x'],kv[1]['y'],0),reverse=True)
        for tid,t in st:
            tx,ty=t['x'],t['y']; state=t.get('state','pending'); prio=t.get('priority','MEDIUM')
            pname=t.get('person_name','?'); sn=t.get('survival_now',.5); sr=t.get('survival_if_rescued',.9)
            wh=t.get('water_height_m',.8); urg=t.get('urgency',.5)
            col=P_DONE if state=='complete' else (P_ENRTE if state=='assigned' else prio_col(prio))
            bz=min(wh,.55) if wh>0 else .04
            bp=self._p(tx,ty,bz)
            if not bp: continue
            if prio=='CRITICAL' and state=='pending':
                pulse=int(5+5*abs(math.sin(self.tick*.1)))
                s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
                pygame.draw.circle(s,(*P_CRIT,60),bp,pulse+12,2)
                pygame.draw.circle(s,(*P_CRIT,30),bp,pulse+22,1)
                surf.blit(s,(0,0))
            r=max(4,int(9/max(self.cam.depth(tx,ty,bz)/7,1)))
            # Glow
            gs=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
            pygame.draw.circle(gs,(*col,45),bp,r+6)
            surf.blit(gs,(0,0))
            pygame.draw.circle(surf,col,bp,r)
            pygame.draw.circle(surf,(215,230,240),bp,r,1)
            sym='v' if state=='complete' else ('>' if state=='assigned' else '!')
            ls=self.f7.render(sym,True,(255,255,255))
            surf.blit(ls,(bp[0]-ls.get_width()//2,bp[1]-ls.get_height()//2))
            tz=bz+.5+urg*.7; tp=self._p(tx,ty,tz)
            if tp:
                pygame.draw.line(surf,blend(col,.58,SKY_BOT),bp,tp,2)
                pygame.draw.circle(surf,col,tp,max(3,r-3))
                lx,ly=tp[0]+5,tp[1]-22
                surf.blit(self.f8.render(pname[:14],True,col),(lx,ly))
                if state!='complete':
                    surf.blit(self.f7.render(f'Surv:{sn*100:.0f}%-{sr*100:.0f}%',True,surv_col(sn)),(lx,ly+11))
                else:
                    surf.blit(self.f7.render('RESCUED',True,P_DONE),(lx,ly+11))

    # ── Drones ─────────────────────────────────────────────────────────────────
    def _drones(self,surf,poses,statuses,vels):
        sd=sorted(poses.items(),key=lambda kv:self.cam.depth(*kv[1]),reverse=True)
        for d,pos in sd:
            wx,wy,wz=pos; col=DRONE_COL[d%len(DRONE_COL)]
            st=statuses.get(d,{}); state=st.get('state','idle'); batt=st.get('battery',100)
            sp=self._p(wx,wy,wz)
            if not sp: continue
            bc=P_HIGH if state=='tasked' else col
            shp=self._p(wx,wy,.02)
            if shp:
                pygame.draw.circle(surf,(0,0,0),shp,max(3,int(7-wz*.4)))
                pygame.draw.line(surf,blend(col,.16,SKY_BOT),shp,sp,1)
                if state=='tasked':
                    s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
                    pygame.draw.line(s,(*col,28),sp,shp,5); surf.blit(s,(0,0))
            # Body glow
            gs=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
            pygame.draw.circle(gs,(*bc,35),sp,16)
            surf.blit(gs,(0,0))
            R=9
            hp=[(sp[0]+R*math.cos(math.radians(60*i-30)),sp[1]+R*math.sin(math.radians(60*i-30))) for i in range(6)]
            pygame.draw.polygon(surf,bc,hp); pygame.draw.polygon(surf,(230,240,248),hp,1)
            for arm_a in [45,135,225,315]:
                ra=math.radians(arm_a); ax=int(sp[0]+13*math.cos(ra)); ay=int(sp[1]+13*math.sin(ra))
                pygame.draw.line(surf,blend(col,.45,SKY_BOT),sp,(ax,ay),2)
                spin=math.radians(self.rotor+arm_a*1.5)
                for blade in range(2):
                    ba=spin+blade*math.pi
                    pygame.draw.line(surf,col,(ax,ay),(int(ax+6*math.cos(ba)),int(ay+6*math.sin(ba))),2)
                pygame.draw.circle(surf,col,(ax,ay),4,1)
            if self.tick%18<9: pygame.draw.circle(surf,(255,50,50),sp,3)
            lb=self.f10.render(str(d),True,(255,255,255))
            surf.blit(lb,(sp[0]-lb.get_width()//2,sp[1]-lb.get_height()//2))
            bw=26; bx=sp[0]-bw//2; by=sp[1]-R-8
            pygame.draw.rect(surf,(16,16,16),(bx,by,bw,4))
            pygame.draw.rect(surf,batt_col(batt),(bx,by,int(bw*batt/100),4))
            surf.blit(self.f7.render(f'z={wz:.1f}m',True,blend(col,.70,SKY_BOT)),(sp[0]+R+2,sp[1]-4))
            # Payload crate icon — yellow box above drone when carrying rescue kit
            if st.get('payload', False):
                pygame.draw.rect(surf,(200,160,0),(sp[0]-5,sp[1]-R-18,10,8))
                pygame.draw.rect(surf,(255,210,40),(sp[0]-5,sp[1]-R-18,10,8),1)
                surf.blit(self.f7.render('KIT',True,(255,210,40)),(sp[0]-7,sp[1]-R-28))
            # Wind drift arrow — shows wind pushing this drone
            wx_d = st.get('wind_x', WIND_X); wy_d = st.get('wind_y', WIND_Y)
            if math.hypot(wx_d, wy_d) > 0.05:
                drift_p = self._p(wx+wx_d*0.5, wy+wy_d*0.5, wz)
                if drift_p:
                    pygame.draw.line(surf,(50,110,200),sp,drift_p,1)
                    pygame.draw.circle(surf,(80,150,255),drift_p,2)

    # ── Rain ───────────────────────────────────────────────────────────────────
    def _wind_arrow(self,surf):
        """Draw wind direction arrow + speed indicator in top-left corner."""
        # Arrow base position (screen top-left area)
        ax, ay = 30, 30
        spd = math.hypot(WIND_X, WIND_Y)
        # Project wind vector onto screen — wind_x = east = screen right, wind_y = north = screen up
        wx_s =  WIND_X * 40
        wy_s = -WIND_Y * 40   # invert Y for screen
        ex, ey = int(ax + wx_s), int(ay + wy_s)
        # Glow
        pygame.draw.line(surf, (40, 100, 160), (ax, ay), (ex, ey), 4)
        pygame.draw.line(surf, (100, 180, 255), (ax, ay), (ex, ey), 2)
        # Arrowhead
        pygame.draw.circle(surf, (100, 180, 255), (ex, ey), 4)
        # Base dot
        pygame.draw.circle(surf, (60, 140, 200), (ax, ay), 3)
        # Label
        f7 = self.f8
        surf.blit(f7.render(f'WIND {spd:.1f}m/s', True, (80, 160, 220)), (ax - 10, ay + 10))
        # Gust indicator — small extra arrow
        surf.blit(self.f7.render(f'E:{WIND_X:+.2f} N:{WIND_Y:+.2f}', True, (60, 120, 170)), (ax - 10, ay + 24))

    def _rain(self,surf):
        s=pygame.Surface((VIEW_W,VIEW_H),pygame.SRCALPHA)
        for drop in RAIN_DROPS:
            x,y,length,speed=drop; drop[1]=(y+speed*2.2)%VIEW_H
            if drop[1]<speed*2: drop[0]=random.randint(0,VIEW_W)
            pygame.draw.line(s,(*RAIN_C,78),(int(x),int(y)),(int(x-length*.28),int(y+length)),1)
        surf.blit(s,(0,0))

    def _minimap_build(self,poses,tasks):
        mm=self.minimap; mm.fill((7,15,26))
        for z in FLOOD_ZONES:
            x0,x1=z['x']; y0,y1=z['y']
            mx0=int((x0-WX_MIN)/(WX_MAX-WX_MIN)*160); mx1=int((x1-WX_MIN)/(WX_MAX-WX_MIN)*160)
            my0=int((y0-WY_MIN)/(WY_MAX-WY_MIN)*160); my1=int((y1-WY_MIN)/(WY_MAX-WY_MIN)*160)
            pygame.draw.rect(mm,(0,45,75),(mx0,my0,mx1-mx0,my1-my0))
        for t in tasks.values():
            mx=int((t['x']-WX_MIN)/(WX_MAX-WX_MIN)*160); my=int((t['y']-WY_MIN)/(WY_MAX-WY_MIN)*160)
            col=P_DONE if t.get('state')=='complete' else (P_HIGH if t.get('state')=='assigned' else P_CRIT)
            pygame.draw.circle(mm,col,(mx,my),3)
        for d,p in poses.items():
            mx=int((p[0]-WX_MIN)/(WX_MAX-WX_MIN)*160); my=int((p[1]-WY_MIN)/(WY_MAX-WY_MIN)*160)
            pygame.draw.circle(mm,DRONE_COL[d%len(DRONE_COL)],(mx,my),4)
        pygame.draw.rect(mm,(0,175,158),mm.get_rect(),1)

    def _minimap_blit(self,surf):
        pygame.draw.rect(surf,(6,13,22),(VIEW_W-168,VIEW_H-172,168,172))
        surf.blit(self.minimap,(VIEW_W-164,VIEW_H-164))
        surf.blit(self.f7.render('live feed',True,ACCENT),(VIEW_W-130,VIEW_H-175))

    # ── Analytics panel ────────────────────────────────────────────────────────
    def _chart_panel(self,tasks,statuses):
        px=VIEW_W; pw=CHART_W
        p=pygame.Surface((pw,TOTAL_H)); p.fill(BG_MID)
        pygame.draw.rect(p,(0,50,85),(0,0,pw,TOTAL_H),1)

        draw_line_chart(p,(4,4,pw-8,190),
            [list(self.surv_i),list(self.surv_e),list(self.surv_a)],
            [(80,210,255),(255,175,70),(70,215,115)],['Infant','Elderly','Adult'],
            'SURVIVAL PROBABILITY OVER TIME',self.f7,self.f9,y_max=100)
        draw_heatmap(p,(4,200,pw-8,190),coverage_grid,self.f7,self.f9,'SEARCH GRID COVERAGE HEAT MAP')
        draw_bar_line_chart(p,(4,396,pw-8,190),
            list(self.rescue_h),list(self.deploy_h),(35,155,110),(255,175,55),
            'UAV DEPLOYMENT VS. RESCUES',self.f7,self.f9)

        done   =sum(1 for t in tasks.values() if t.get('state')=='complete')
        enroute=sum(1 for t in tasks.values() if t.get('state')=='assigned')
        pending=sum(1 for t in tasks.values() if t.get('state')=='pending')
        crit   =sum(1 for t in tasks.values() if t.get('state')=='pending' and t.get('priority')=='CRITICAL')
        pygame.draw.rect(p,(8,18,32),(4,592,pw-8,224))
        pygame.draw.rect(p,(0,50,80),(4,592,pw-8,224),1)
        p.blit(self.f9.render('SITUATION',True,ACCENT),(10,598))
        p.blit(self.f7.render('Case Status Distribution',True,TEXT_DIM),(10,612))
        draw_donut(p,pw//2-22,706,84,[max(done,1),max(enroute,1),max(pending,1),max(crit,1)],
            [P_DONE,P_ENRTE,P_MED,P_CRIT],['Rescued','En Route','Pending','Critical'],self.f7)
        self.screen.blit(p,(px,0))

    # ── Command panel ──────────────────────────────────────────────────────────
    def _command_panel(self,statuses,tasks):
        px=VIEW_W+CHART_W; pw=PANEL_W
        p=pygame.Surface((pw,TOTAL_H)); p.fill(BG_PANEL)
        pygame.draw.rect(p,(0,70,110),(0,0,pw,TOTAL_H),1)
        y=0
        pygame.draw.rect(p,(0,50,85),(0,0,pw,30))
        pygame.draw.rect(p,(0,165,152),(0,0,3,30)); pygame.draw.rect(p,(0,165,152),(pw-3,0,3,30))
        t=self.f14.render('FLOOD RESCUE OPS',True,ACCENT)
        p.blit(t,(pw//2-t.get_width()//2,6)); y=34

        y=section_hdr(p,4,y,pw-8,'FLEET',self.f11)
        for d in range(self.n):
            col=DRONE_COL[d%len(DRONE_COL)]; st=statuses.get(d,{})
            batt=st.get('battery',100); state=st.get('state','idle')
            tid=st.get('task_id',-1); alt=st.get('z',0.)
            ss='PATRO' if state=='patrol' else (f'T{tid}' if state=='tasked' else state.upper()[:5])
            sc=P_HIGH if state=='tasked' else (58,172,58)
            # ── UAV row ──────────────────────────────────────────────────────
            bw = pw - 16   # bar width = full panel width minus margins
            pygame.draw.rect(p,(11,22,38),(4,y,pw-8,24))
            pygame.draw.rect(p,col,(4,y,4,24))
            # Name + state + altitude — left side
            p.blit(self.f10.render(f'UAV-{d}',True,col),(10,y+4))
            p.blit(self.f9.render(ss,True,sc),(82,y+5))
            p.blit(self.f8.render(f'z={alt:.1f}m',True,TEXT_DIM),(145,y+5))
            # Sparkline — middle section
            spark_x = 200; spark_w = pw - 245
            draw_sparkline(p,(spark_x,y+6,spark_w,12),[batt+random.uniform(-1,1) for _ in range(10)],col)
            # Battery % — right aligned
            bpct_txt = self.f8.render(f'{batt:.0f}%',True,batt_col(batt))
            p.blit(bpct_txt,(pw-bpct_txt.get_width()-6,y+5))
            y += 26
            # Battery bar — full width
            pygame.draw.rect(p,(14,28,44),(8,y,bw,5))
            pygame.draw.rect(p,batt_col(batt),(8,y,int(bw*batt/100),5)); y+=7
            # Kit info — two compact lines
            km   = st.get('kit_mass_left', 0.80)
            kitl = st.get('kit_items_left', ['life_jacket','medicine_box','rope_25m','food_water'])
            rdone= st.get('rescues_done', 0)
            kit_pct = min(km/0.80, 1.0)
            kit_c   = (50,220,100) if kit_pct>0.5 else (255,200,0) if kit_pct>0.2 else (220,60,60)
            # Kit bar — full width
            pygame.draw.rect(p,(14,28,44),(8,y,bw,4))
            pygame.draw.rect(p,kit_c,(8,y,int(bw*kit_pct),4)); y+=6
            # Kit text — line 1: mass + rescues
            p.blit(self.f7.render(f'Kit: {km:.2f}kg',True,kit_c),(8,y))
            p.blit(self.f7.render(f'Rescues: {rdone}',True,TEXT_DIM),(pw//2-20,y)); y+=12
            # Kit text — line 2: remaining items (abbreviated)
            short_items = '  '.join([k[:4].upper() for k in kitl]) if kitl else 'KIT EMPTY'
            p.blit(self.f7.render(short_items,True,kit_c),(8,y)); y+=13

        y+=3; y=section_hdr(p,4,y,pw-8,'SITUATION',self.f11)
        done   =sum(1 for t in tasks.values() if t.get('state')=='complete')
        enroute=sum(1 for t in tasks.values() if t.get('state')=='assigned')
        pending=sum(1 for t in tasks.values() if t.get('state')=='pending')
        crit   =sum(1 for t in tasks.values() if t.get('state')=='pending' and t.get('priority')=='CRITICAL')
        draw_donut(p,58,y+58,46,[max(done,1),max(enroute,1),max(pending,1),max(crit,1)],
            [P_DONE,P_ENRTE,P_MED,P_CRIT],['Rescued','En Route','Pending','Critical'],self.f7)
        sx=130; p.blit(self.f9.render('Case Status Distribution',True,TEXT_DIM),(sx,y)); y+=14
        for lbl,val,col in [('Total    :',len(tasks),TEXT_MAIN),('Pending  :',pending,P_CRIT),
            ('En route :',enroute,P_ENRTE),('Rescued  :',done,P_DONE),('Critical :',crit,(200,50,50))]:
            p.blit(self.f9.render(lbl,True,TEXT_DIM),(sx,y))
            p.blit(self.f9.render(str(val),True,col),(sx+90,y)); y+=13
        y+=44; y=section_hdr(p,4,y,pw-8,'PRIORITY QUEUE',self.f11)
        pl=[(k,t) for k,t in tasks.items() if t.get('state')=='pending']
        pl.sort(key=lambda x:x[1].get('urgency',0),reverse=True)
        if not pl: p.blit(self.f9.render('All units assigned',True,(38,95,55)),(8,y)); y+=14
        else:
            for k,t in pl[:4]:
                prio=t.get('priority','MEDIUM'); col=prio_col(prio)
                pname=t.get('person_name','?'); surv=t.get('survival_now',.5)
                wcat=t.get('water_category','?'); urg=t.get('urgency',0)
                bh=36; pygame.draw.rect(p,(11,22,38),(4,y,pw-8,bh))
                pygame.draw.rect(p,(*col,50),(4,y,pw-8,bh),1)
                pygame.draw.rect(p,col,(4,y,3,bh))
                p.blit(self.f10.render(prio[:4],True,col),(8,y+3))
                p.blit(self.f9.render(pname[:14],True,TEXT_MAIN),(56,y+3))
                p.blit(self.f8.render(f'High {surv*100:.0f}%  Urg:{urg:.2f}  {wcat[:10]}',True,TEXT_DIM),(8,y+20))
                draw_sparkline(p,(pw-78,y+5,66,24),[surv*100+random.uniform(-2,1) for _ in range(8)],col)
                p.blit(self.f7.render(f'{urg:.2f}',True,col),(pw-12,y+3))
                p.blit(self.f7.render('trend',True,TEXT_DIM),(pw-48,y+26)); y+=bh+2

        y+=2; y=section_hdr(p,4,y,pw-8,'RESCUED LOG',self.f11)
        done_d={k:t for k,t in tasks.items() if t.get('state')=='complete'}
        if not done_d: p.blit(self.f9.render('No rescues yet',True,(32,62,42)),(8,y))
        else:
            for k,t in list(done_d.items())[-5:]:
                pname=t.get('person_name','?'); age=t.get('age_group','?'); desc=t.get('description','')
                p.blit(self.f10.render(pname,True,P_DONE),(8,y)); y+=12
                p.blit(self.f7.render(f'  Age {age}  ({t["x"]:.1f},{t["y"]:.1f})',True,(42,115,58)),(8,y)); y+=10
                p.blit(self.f7.render(f'  {desc[:34]}',True,(32,90,46)),(8,y)); y+=10
                if y>TOTAL_H-18: break
        self.screen.blit(p,(px,0))

# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node=VisNode(NUM_DRONES); renderer=Renderer(NUM_DRONES)
    threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start()
    print('[swarm_visualizer] Flood Rescue Command — ESC to quit')
    running=True
    while running:
        for e in pygame.event.get():
            if e.type==pygame.QUIT: running=False
            elif e.type==pygame.KEYDOWN:
                if e.key==pygame.K_ESCAPE: running=False
                elif e.key==pygame.K_r: renderer.cam.reset()
                elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):  renderer.cam.on_scroll(3)
                elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS): renderer.cam.on_scroll(-3)
            elif e.type==pygame.MOUSEBUTTONDOWN:
                mx,my=e.pos
                if mx<VIEW_W:
                    if e.button==1: renderer.cam.on_down(e.pos)
                    elif e.button==4: renderer.cam.on_scroll(2)
                    elif e.button==5: renderer.cam.on_scroll(-2)
            elif e.type==pygame.MOUSEBUTTONUP:
                if e.button==1:
                    lx,ly=renderer.cam._last; cx,cy=e.pos
                    if abs(cx-lx)<5 and abs(cy-ly)<5 and cx<VIEW_W:
                        wxy=renderer.cam.ray_ground(cx,cy)
                        if wxy:
                            wx,wy=wxy
                            if WX_MIN<=wx<=WX_MAX and WY_MIN<=wy<=WY_MAX: node.spawn(wx,wy)
                    renderer.cam.on_up()
            elif e.type==pygame.MOUSEMOTION:
                if e.pos[0]<VIEW_W: renderer.cam.on_move(e.pos)
            elif e.type==pygame.MOUSEWHEEL: renderer.cam.on_scroll(e.y*3)
        renderer.draw(*node.snap())
    pygame.quit(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
