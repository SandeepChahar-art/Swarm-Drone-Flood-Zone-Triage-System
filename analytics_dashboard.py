#!/usr/bin/env python3
"""
analytics_dashboard.py — Parallel Analytics for Drone Swarm
Runs alongside swarm_visualizer.py, subscribes to same ROS topics.
Shows:
  - Swarm centre XY trajectory plot (top-down path trace)
  - Efficiency metrics (rescue rate, coverage, response time, battery)
  - Per-drone performance bars
  - Real-time mission clock and KPIs
  - Live graphs: tasks over time, battery drain, coverage growth

Run:
  python analytics_dashboard.py
"""
import rclpy
from rclpy.node import Node
import json, math, threading, sys, time
from collections import deque
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    import pygame
except ImportError:
    print("pip3 install pygame"); sys.exit(1)

# ── Layout ────────────────────────────────────────────────────────────────────
WIN_W, WIN_H = 1200, 800
FPS          = 30
NUM_DRONES   = 5

# World bounds (same as main sim)
WX_MIN, WX_MAX = -1.0, 11.0
WY_MIN, WY_MAX = -1.0, 14.0

# ── Palette ───────────────────────────────────────────────────────────────────
BG       = (  7,  13,  22)
BG2      = ( 10,  18,  32)
BG3      = ( 13,  24,  40)
ACCENT   = (  0, 200, 175)
ACCENT2  = (  0, 155, 215)
ACCENT3  = (255, 175,  55)
TEXT     = (195, 215, 228)
TEXT_DIM = ( 70, 100, 120)
GREEN    = ( 40, 220, 115)
ORANGE   = (255, 148,   0)
RED      = (220,  45,  45)
CYAN     = (  0, 210, 220)
PURPLE   = (175,  80, 255)
GRID_C   = ( 18,  35,  52)

DRONE_COL=[(0,255,170),(0,200,255),(255,110,50),(255,215,0),(200,80,255)]

def blend(col,a,bg=BG):
    return tuple(max(0,min(255,int(col[i]*a+bg[i]*(1-a)))) for i in range(3))

def batt_col(b): return (50,220,100) if b>50 else (255,200,0) if b>25 else (220,60,60)

def eff_col(v):
    """Colour based on 0-1 efficiency value."""
    if v>=.75: return GREEN
    if v>=.45: return ORANGE
    return RED

# ── ROS Analytics Node ────────────────────────────────────────────────────────
class AnalyticsNode(Node):
    def __init__(self):
        super().__init__('analytics_dashboard')
        self.lock = threading.Lock()

        # Swarm centre history for path plot
        self.centre_path = deque(maxlen=800)   # (x,y,t)
        self.centre_now  = (WX_MIN+WX_MAX)/2, (WY_MIN+WY_MAX)/2

        # Per-drone data
        self.drone_data = {i:{
            'battery':100., 'state':'idle', 'task_id':-1,
            'z':2., 'tasks_done':0,
            'battery_hist':deque([100.]*60, maxlen=120),
            'pos':(0.,0.,2.),
        } for i in range(NUM_DRONES)}

        # Task tracking
        self.tasks        = {}
        self.rescue_times = []   # seconds to rescue each task
        self._task_spawn  = {}   # task_id -> spawn timestamp

        # Time series (sampled every 2 s)
        MAX=120
        self.ts_rescued   = deque([0]*MAX, maxlen=MAX)
        self.ts_pending   = deque([0]*MAX, maxlen=MAX)
        self.ts_assigned  = deque([0]*MAX, maxlen=MAX)
        self.ts_coverage  = deque([0.]*MAX, maxlen=MAX)
        self.ts_avg_surv  = deque([60.]*MAX, maxlen=MAX)
        self.ts_fleet_bat = deque([100.]*MAX, maxlen=MAX)

        # Coverage grid
        self.cov_grid = [[0.]*12 for _ in range(15)]

        self.start_time = time.time()
        self._last_sample = 0.

        # ROS subscriptions
        self.create_subscription(PoseStamped, '/swarm/centre', self._cb_centre, 10)
        self.create_subscription(String,      '/swarm/triage_data', self._cb_triage, 10)
        for i in range(NUM_DRONES):
            self.create_subscription(PoseStamped, f'/drone_{i}/pose',
                lambda m,d=i: self._cb_pose(m,d), 10)
            self.create_subscription(String, f'/drone_{i}/status',
                lambda m,d=i: self._cb_status(m,d), 10)

        self.create_timer(2.0, self._sample)

    def _cb_centre(self, m):
        with self.lock:
            x,y = m.pose.position.x, m.pose.position.y
            self.centre_now = (x,y)
            self.centre_path.append((x, y, time.time()-self.start_time))

    def _cb_pose(self, m, d):
        with self.lock:
            pos=(m.pose.position.x, m.pose.position.y, m.pose.position.z)
            self.drone_data[d]['pos']=pos
            # Coverage
            gx=int((pos[0]-WX_MIN)/(WX_MAX-WX_MIN)*12)
            gy=int((pos[1]-WY_MIN)/(WY_MAX-WY_MIN)*15)
            if 0<=gx<12 and 0<=gy<15:
                self.cov_grid[gy][gx]=min(1., self.cov_grid[gy][gx]+.012)

    def _cb_status(self, m, d):
        try:
            data=json.loads(m.data)
            with self.lock:
                dd=self.drone_data[d]
                old_state=dd['state']
                dd['battery']=data.get('battery',dd['battery'])
                dd['state']  =data.get('state','idle')
                dd['task_id']=data.get('task_id',-1)
                dd['z']      =data.get('z',dd['z'])
                dd['battery_hist'].append(dd['battery'])
                # Count completions
                if old_state=='tasked' and dd['state']=='patrol':
                    dd['tasks_done']+=1
                # Update task state
                if data.get('state')=='task_complete':
                    tid=data.get('task_id',-1)
                    key=f"task_{tid}"
                    if key in self.tasks:
                        self.tasks[key]['state']='complete'
                        if tid in self._task_spawn:
                            rt=time.time()-self._task_spawn[tid]
                            self.rescue_times.append(rt)
                elif data.get('state')=='tasked':
                    for k in self.tasks:
                        if self.tasks[k].get('task_id')==data.get('task_id'):
                            self.tasks[k]['state']='assigned'
        except: pass

    def _cb_triage(self, m):
        try:
            data=json.loads(m.data)
            key=f"task_{data['task_id']}"
            with self.lock:
                if key not in self.tasks:
                    self.tasks[key]=data
                    self._task_spawn[data['task_id']]=time.time()
        except: pass

    def _sample(self):
        with self.lock:
            done    = sum(1 for t in self.tasks.values() if t.get('state')=='complete')
            pending = sum(1 for t in self.tasks.values() if t.get('state')=='pending')
            assigned= sum(1 for t in self.tasks.values() if t.get('state')=='assigned')
            surv_vals=[t.get('survival_now',0.5)*100 for t in self.tasks.values() if t.get('state')!='complete']
            avg_surv=sum(surv_vals)/len(surv_vals) if surv_vals else 60.
            fleet_bat=sum(d['battery'] for d in self.drone_data.values())/NUM_DRONES
            cov_pct=sum(self.cov_grid[r][c] for r in range(15) for c in range(12))/(15*12)*100
            self.ts_rescued.append(done)
            self.ts_pending.append(pending)
            self.ts_assigned.append(assigned)
            self.ts_coverage.append(cov_pct)
            self.ts_avg_surv.append(avg_surv)
            self.ts_fleet_bat.append(fleet_bat)

    def snap(self):
        with self.lock:
            return {
                'centre_path' : list(self.centre_path),
                'centre_now'  : self.centre_now,
                'drone_data'  : {d: dict(v) for d,v in self.drone_data.items()},
                'tasks'       : {k: dict(v) for k,v in self.tasks.items()},
                'rescue_times': list(self.rescue_times),
                'ts_rescued'  : list(self.ts_rescued),
                'ts_pending'  : list(self.ts_pending),
                'ts_assigned' : list(self.ts_assigned),
                'ts_coverage' : list(self.ts_coverage),
                'ts_avg_surv' : list(self.ts_avg_surv),
                'ts_fleet_bat': list(self.ts_fleet_bat),
                'cov_grid'    : [row[:] for row in self.cov_grid],
                'elapsed'     : time.time()-self.start_time,
            }

# ── Drawing helpers ───────────────────────────────────────────────────────────
def draw_panel(surf, rect, title, font_t, font_s):
    x,y,w,h=rect
    pygame.draw.rect(surf, BG2, (x,y,w,h))
    pygame.draw.rect(surf, (0,60,95), (x,y,w,h), 1)
    pygame.draw.rect(surf, (0,55,82), (x,y,w,20))
    pygame.draw.rect(surf, ACCENT, (x,y,3,20))
    surf.blit(font_t.render(title, True, ACCENT), (x+8, y+3))
    return y+24

def draw_sparkline(surf, rect, pts, col, fill=False):
    if len(pts)<2: return
    x,y,w,h=rect
    mn=min(pts); mx=max(pts)+.001; rng=mx-mn
    sp=[(x+int(i/(len(pts)-1)*w), y+h-int((p-mn)/rng*h)) for i,p in enumerate(pts)]
    if fill:
        fp=sp+[(x+w,y+h),(x,y+h)]
        s=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
        pygame.draw.polygon(s,(*col,35),fp); surf.blit(s,(0,0))
    pygame.draw.lines(surf, col, False, sp, 2)
    pygame.draw.circle(surf, col, sp[-1], 3)

def draw_hbar(surf, x, y, w, h, val, col, bg=(20,38,55)):
    pygame.draw.rect(surf, bg, (x,y,w,h))
    fw=int(w*max(0,min(1,val)))
    if fw>0:
        for px in range(fw):
            t=px/max(fw,1)
            c=blend(col,0.5+t*0.5,(0,0,0))
            pygame.draw.line(surf,c,(x+px,y),(x+px,y+h))
    pygame.draw.rect(surf,(0,50,80),(x,y,w,h),1)

def draw_multiline_chart(surf, rect, series, colors, y_max, grid_lines, font_s):
    x,y,w,h=rect
    # Grid
    for gi in range(grid_lines+1):
        gy=y+h-int(h*gi/grid_lines)
        pygame.draw.line(surf,(18,35,52),(x,gy),(x+w,gy),1)
        v=int(y_max*gi/grid_lines)
        surf.blit(font_s.render(str(v),True,TEXT_DIM),(x-22,gy-5))
    pygame.draw.line(surf,(30,55,80),(x,y),(x,y+h),1)
    pygame.draw.line(surf,(30,55,80),(x,y+h),(x+w,y+h),1)
    for pts,col in zip(series,colors):
        if len(pts)<2: continue
        n=len(pts)
        sp=[(x+int(i/(n-1)*w), y+h-int(min(max(pts[i],0),y_max)/y_max*h)) for i in range(n)]
        # Fill
        fp=sp+[(x+w,y+h),(x,y+h)]
        s=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
        pygame.draw.polygon(s,(*col,20),fp); surf.blit(s,(0,0))
        pygame.draw.lines(surf,col,False,sp,2)

def kpi_card(surf, x, y, w, h, label, value, unit, col, font_l, font_s):
    pygame.draw.rect(surf, BG3, (x,y,w,h))
    pygame.draw.rect(surf, (*col,80), (x,y,w,h), 1)
    pygame.draw.rect(surf, col, (x,y,3,h))
    vt=font_l.render(str(value), True, col)
    ut=font_s.render(unit, True, TEXT_DIM)
    lt=font_s.render(label, True, TEXT_DIM)
    surf.blit(lt, (x+8, y+4))
    surf.blit(vt, (x+8, y+18))
    surf.blit(ut, (x+8+vt.get_width()+4, y+26))

# ── Main Renderer ─────────────────────────────────────────────────────────────
class DashRenderer:
    def __init__(self):
        pygame.init()
        self.screen=pygame.display.set_mode((WIN_W,WIN_H))
        pygame.display.set_caption('UAV Swarm Analytics Dashboard')
        self.clock=pygame.time.Clock()
        self.f7 =pygame.font.SysFont('monospace', 7)
        self.f8 =pygame.font.SysFont('monospace', 8)
        self.f9 =pygame.font.SysFont('monospace', 9)
        self.f10=pygame.font.SysFont('monospace',10,bold=True)
        self.f11=pygame.font.SysFont('monospace',11,bold=True)
        self.f13=pygame.font.SysFont('monospace',13,bold=True)
        self.f16=pygame.font.SysFont('monospace',16,bold=True)
        self.f22=pygame.font.SysFont('monospace',22,bold=True)
        self.tick=0

    def draw(self, data):
        self.tick+=1
        self.screen.fill(BG)

        elapsed    = data['elapsed']
        tasks      = data['tasks']
        drone_data = data['drone_data']
        cpath      = data['centre_path']
        cnow       = data['centre_now']
        rt         = data['rescue_times']

        total   = len(tasks)
        done    = sum(1 for t in tasks.values() if t.get('state')=='complete')
        pending = sum(1 for t in tasks.values() if t.get('state')=='pending')
        en_route= sum(1 for t in tasks.values() if t.get('state')=='assigned')
        fleet_bat=sum(d['battery'] for d in drone_data.values())/NUM_DRONES
        avg_rt  = sum(rt)/len(rt) if rt else 0.
        cov_pct =sum(data['cov_grid'][r][c] for r in range(15) for c in range(12))/(15*12)*100
        rescue_rate=done/max(elapsed/60,1)  # rescues per minute

        mm,ss=divmod(int(elapsed),60)
        hh,mm2=divmod(mm,60)
        time_str=f'{hh:02d}:{mm2:02d}:{ss:02d}'

        # ── Top header ────────────────────────────────────────────────────────
        pygame.draw.rect(self.screen,(0,45,72),(0,0,WIN_W,42))
        pygame.draw.line(self.screen,ACCENT,(0,42),(WIN_W,42),2)
        self.screen.blit(self.f16.render('UAV SWARM — EFFICIENCY ANALYTICS DASHBOARD',True,ACCENT),(16,10))
        self.screen.blit(self.f11.render(f'MISSION TIME  {time_str}',True,ACCENT2),(WIN_W-230,14))

        # ── KPI row ───────────────────────────────────────────────────────────
        kpis=[
            ('RESCUES',       str(done),         'ppl',    GREEN),
            ('RESCUE RATE',   f'{rescue_rate:.2f}','rpm',   CYAN),
            ('AVG RESP TIME', f'{avg_rt:.0f}',   'sec',    ACCENT3),
            ('COVERAGE',      f'{cov_pct:.1f}',  '%',      ACCENT2),
            ('FLEET BATTERY', f'{fleet_bat:.0f}','%',      (100,200,255)),
            ('PENDING',       str(pending),       'tasks',  ORANGE),
            ('EN ROUTE',      str(en_route),      'drones', PURPLE),
            ('TOTAL TASKS',   str(total),         '',       TEXT),
        ]
        kw=(WIN_W-16)//len(kpis); ky=48
        for i,(lbl,val,unit,col) in enumerate(kpis):
            kpi_card(self.screen,8+i*kw,ky,kw-4,56,lbl,val,unit,col,self.f16,self.f8)

        # ── Section layout ────────────────────────────────────────────────────
        #   Left col: Centre path plot + drone performance
        #   Right col: Time series charts

        LX, RX = 8, 618
        TY      = 116
        LW, RW  = 600, WIN_W-RX-8
        SH      = WIN_H-TY-8

        # ╔══════════════════════════════╗
        # ║  SWARM CENTRE PATH PLOT      ║  LX, TY, LW, 300
        # ╠══════════════════════════════╣
        # ║  DRONE PERFORMANCE TABLE     ║  LX, TY+310, LW, SH-310
        # ╚══════════════════════════════╝

        # ── Centre path plot ──────────────────────────────────────────────────
        PH=310
        py=draw_panel(self.screen,(LX,TY,LW,PH),'SWARM CENTRE MOVEMENT TRAJECTORY',self.f11,self.f8)

        # Plot area
        PAD=32; px_=LX+PAD; pw_=LW-PAD*2; ph_=PH-py+TY-12

        # World → plot coords
        def w2p(wx,wy):
            sx=px_+int((wx-WX_MIN)/(WX_MAX-WX_MIN)*pw_)
            sy=py+ph_-int((wy-WY_MIN)/(WY_MAX-WY_MIN)*ph_)
            return sx,sy

        # Grid
        for gx in range(0,13,2):
            a=w2p(WX_MIN+gx*(WX_MAX-WX_MIN)/12,WY_MIN)
            b=w2p(WX_MIN+gx*(WX_MAX-WX_MIN)/12,WY_MAX)
            pygame.draw.line(self.screen,GRID_C,a,b,1)
        for gy in range(0,16,3):
            a=w2p(WX_MIN,WY_MIN+gy*(WY_MAX-WY_MIN)/15)
            b=w2p(WX_MAX,WY_MIN+gy*(WY_MAX-WY_MIN)/15)
            pygame.draw.line(self.screen,GRID_C,a,b,1)

        # Waypoints reference (same as swarm_manager)
        waypoints=[(0,0),(8,0),(8,4),(0,4),(0,8),(8,8),(8,12),(0,12)]
        for i in range(len(waypoints)):
            wp=waypoints[i]; nwp=waypoints[(i+1)%len(waypoints)]
            a=w2p(*wp); b=w2p(*nwp)
            pygame.draw.line(self.screen,(0,55,85),a,b,1)
            pygame.draw.circle(self.screen,(0,80,110),a,4)

        # Centre path — colour by age (old=dim, new=bright)
        if len(cpath)>=2:
            n=len(cpath)
            for i in range(1,n):
                t=i/n
                col=blend(ACCENT,t*0.8,(0,30,50))
                pa=w2p(cpath[i-1][0],cpath[i-1][1])
                pb=w2p(cpath[i][0],  cpath[i][1])
                pygame.draw.line(self.screen,col,pa,pb,2)

        # Current position
        cp=w2p(*cnow)
        s=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
        pygame.draw.circle(s,(*ACCENT,60),cp,14); self.screen.blit(s,(0,0))
        pygame.draw.circle(self.screen,ACCENT,cp,7)
        pygame.draw.circle(self.screen,(255,255,255),cp,7,1)
        self.screen.blit(self.f7.render(f'({cnow[0]:.1f},{cnow[1]:.1f})',True,ACCENT),(cp[0]+9,cp[1]-5))

        # Axes labels
        self.screen.blit(self.f7.render('X →',True,TEXT_DIM),(LX+LW-28,py+ph_+2))
        self.screen.blit(self.f7.render('Y',True,TEXT_DIM),(LX+2,py+2))
        self.screen.blit(self.f7.render(f'Path length: {len(cpath)} pts',True,TEXT_DIM),(px_,py+ph_+2))

        # Legend
        self.screen.blit(self.f7.render('─── Centre path',True,ACCENT),(LX+LW-140,py))
        self.screen.blit(self.f7.render('─── Waypoint route',True,(0,80,110)),(LX+LW-140,py+10))

        # ── Drone performance table ───────────────────────────────────────────
        DY=TY+PH+6
        DH=SH-PH-6
        draw_panel(self.screen,(LX,DY,LW,DH),'PER-DRONE PERFORMANCE',self.f11,self.f8)
        dy=DY+26
        col_x=[LX+8,LX+78,LX+148,LX+210,LX+280,LX+360,LX+430,LX+510]
        hdrs=['DRONE','STATE','ALT','BATTERY','TASKS DONE','EFFICIENCY','BATTERY TREND','ACTIVITY']
        for hx,hdr in zip(col_x,hdrs):
            self.screen.blit(self.f7.render(hdr,True,TEXT_DIM),(hx,dy))
        dy+=13
        pygame.draw.line(self.screen,(0,60,95),(LX+4,dy),(LX+LW-4,dy),1); dy+=4

        for d in range(NUM_DRONES):
            dd=drone_data[d]; col=DRONE_COL[d]
            state=dd['state']; batt=dd['battery']
            tasks_done=dd['tasks_done']; alt=dd['z']
            bat_hist=list(dd['battery_hist'])

            # Efficiency: weighted score
            batt_eff=batt/100.
            task_eff=min(tasks_done/max(done/NUM_DRONES,1),1.) if done>0 else 0.
            state_eff=1. if state=='tasked' else (.6 if state=='patrol' else .2)
            eff=round((batt_eff*.3+task_eff*.5+state_eff*.2),2)

            sc=GREEN if state=='tasked' else (CYAN if state=='patrol' else ORANGE)
            row_bg=(12,24,38) if d%2==0 else (10,20,34)
            pygame.draw.rect(self.screen,row_bg,(LX+4,dy,LW-8,26))
            pygame.draw.rect(self.screen,(*col,40),(LX+4,dy,3,26))

            self.screen.blit(self.f10.render(f'UAV-{d}',True,col),(col_x[0],dy+6))
            self.screen.blit(self.f9.render(state[:6].upper(),True,sc),(col_x[1],dy+6))
            self.screen.blit(self.f9.render(f'{alt:.1f}m',True,TEXT),(col_x[2],dy+6))

            draw_hbar(self.screen,col_x[3],dy+8,60,10,batt/100,batt_col(batt))
            self.screen.blit(self.f7.render(f'{batt:.0f}%',True,batt_col(batt)),(col_x[3]+64,dy+6))

            self.screen.blit(self.f10.render(str(tasks_done),True,GREEN),(col_x[4]+15,dy+6))
            draw_hbar(self.screen,col_x[5],dy+8,60,10,eff,eff_col(eff))
            self.screen.blit(self.f7.render(f'{eff*100:.0f}%',True,eff_col(eff)),(col_x[5]+64,dy+6))

            # Battery sparkline
            draw_sparkline(self.screen,(col_x[6],dy+4,70,18),bat_hist,batt_col(batt))

            # Activity indicator
            if state=='tasked':
                act_col=ORANGE; act='RESCUE'
            elif state=='patrol':
                act_col=CYAN; act='PATROL'
            else:
                act_col=TEXT_DIM; act='IDLE'
            if self.tick%20<10 and state=='tasked':
                pygame.draw.circle(self.screen,act_col,(col_x[7]+5,dy+13),4)
            self.screen.blit(self.f8.render(act,True,act_col),(col_x[7]+12,dy+6))

            dy+=28
            if dy>DY+DH-10: break

        # ── Right column: time series charts ──────────────────────────────────

        # Chart 1: Tasks over time
        CH=(SH-20)//3-8
        cy=TY
        draw_panel(self.screen,(RX,cy,RW,CH+20),'TASK STATUS OVER TIME',self.f11,self.f8)
        draw_multiline_chart(self.screen,(RX+28,cy+26,RW-34,CH-12),
            [data['ts_rescued'],data['ts_assigned'],data['ts_pending']],
            [GREEN,ACCENT2,ORANGE],
            max(max(data['ts_rescued']+[1]),max(data['ts_pending']+[1]),1)*1.1,
            4,self.f7)
        # Legend
        for i,(lbl,col) in enumerate(zip(['Rescued','En Route','Pending'],[GREEN,ACCENT2,ORANGE])):
            pygame.draw.line(self.screen,col,(RX+28+i*80,cy+CH+12),(RX+52+i*80,cy+CH+12),2)
            self.screen.blit(self.f7.render(lbl,True,col),(RX+55+i*80,cy+CH+8))
        cy+=CH+26

        # Chart 2: Coverage + avg survival
        draw_panel(self.screen,(RX,cy,RW,CH+20),'COVERAGE % & AVG SURVIVAL PROBABILITY',self.f11,self.f8)
        draw_multiline_chart(self.screen,(RX+28,cy+26,RW-34,CH-12),
            [data['ts_coverage'],data['ts_avg_surv']],
            [CYAN,ACCENT3],100,4,self.f7)
        for i,(lbl,col) in enumerate(zip(['Coverage','Avg Survival'],[CYAN,ACCENT3])):
            pygame.draw.line(self.screen,col,(RX+28+i*110,cy+CH+12),(RX+52+i*110,cy+CH+12),2)
            self.screen.blit(self.f7.render(lbl,True,col),(RX+55+i*110,cy+CH+8))
        cy+=CH+26

        # Chart 3: Fleet battery
        draw_panel(self.screen,(RX,cy,RW,CH+20),'FLEET BATTERY LEVEL OVER TIME',self.f11,self.f8)
        draw_multiline_chart(self.screen,(RX+28,cy+26,RW-34,CH-12),
            [data['ts_fleet_bat']],[(100,200,255)],100,4,self.f7)
        # Battery zones
        s2=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
        ca=RX+28; cw2=RW-34
        lo=cy+26+(CH-12)-int((CH-12)*.2)
        pygame.draw.rect(s2,(*RED,15),(ca,lo,cw2,(CH-12)-int((CH-12)*.2)+12))
        self.screen.blit(s2,(0,0))
        self.screen.blit(self.f7.render('LOW BATTERY ZONE',True,(*RED,150)),(ca+4,lo+2))
        pygame.draw.line(self.screen,col,(RX+28,cy+CH+12),(RX+52,cy+CH+12),2)
        self.screen.blit(self.f7.render('Fleet avg battery',True,(100,200,255)),(RX+55,cy+CH+8))

        # ── Response time histogram (bottom right) ────────────────────────────
        rty=cy+CH+28
        rth=WIN_H-rty-8
        if rth>50:
            draw_panel(self.screen,(RX,rty,RW,rth),'RESCUE RESPONSE TIME DISTRIBUTION',self.f11,self.f8)
            if rt:
                mn,mx=min(rt),max(rt)+.001
                bins=8; bw2=(RW-24)//bins
                counts=[0]*bins
                for v in rt:
                    bi=min(int((v-mn)/(mx-mn)*bins),bins-1); counts[bi]+=1
                mc=max(counts) if counts else 1
                barea_y=rty+26; barea_h=rth-38
                for bi,cnt in enumerate(counts):
                    bx=RX+8+bi*bw2; bh2=int(cnt/mc*(barea_h-12))
                    t=bi/bins; c=blend(GREEN,t,ORANGE)
                    pygame.draw.rect(self.screen,c,(bx,barea_y+barea_h-12-bh2,bw2-3,bh2))
                    self.screen.blit(self.f7.render(f'{mn+bi*(mx-mn)/bins:.0f}s',True,TEXT_DIM),
                                    (bx,barea_y+barea_h-10))
                self.screen.blit(self.f7.render(f'Avg: {avg_rt:.0f}s  Min: {min(rt):.0f}s  Max: {max(rt):.0f}s  n={len(rt)}',
                    True,TEXT),(RX+8,rty+rth-14))
            else:
                self.screen.blit(self.f9.render('Waiting for rescue data...',True,TEXT_DIM),(RX+20,rty+rth//2))

        pygame.display.flip()
        self.clock.tick(FPS)

# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node    = AnalyticsNode()
    renderer= DashRenderer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('[analytics_dashboard] Running — ESC to quit')

    running=True
    while running:
        for e in pygame.event.get():
            if e.type==pygame.QUIT: running=False
            elif e.type==pygame.KEYDOWN and e.key==pygame.K_ESCAPE: running=False

        renderer.draw(node.snap())

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
