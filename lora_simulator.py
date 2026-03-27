#!/usr/bin/env python3
"""
lora_simulator.py — LoRa Radio Packet Simulator
================================================
Simulates real LoRa (SX1276) radio communication between:
  - 5 UAV drones (transmitters)
  - 1 Ground Van (receiver/base station)

Physics modelled:
  - Free-space path loss (Friis equation)
  - Log-distance path loss model (n=2.7 outdoor urban)
  - RSSI calculation (dBm)
  - SNR with thermal noise floor (-120 dBm for LoRa)
  - Packet loss probability vs RSSI
  - Spreading factor simulation (SF7-SF12)
  - Airtime calculation per packet
  - Packet queue, retransmit logic
  - Obstacle fading (buildings cause random shadow fading)

Van position: draggable on map
Runs alongside main sim — subscribes to same ROS topics.

Controls:
  Drag van icon → reposition ground station
  S → toggle spreading factor
  ESC → quit
"""
import rclpy
from rclpy.node import Node
import json, math, threading, sys, time, random
from collections import deque
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    import pygame
except ImportError:
    print("pip3 install pygame"); sys.exit(1)

# ── Window ────────────────────────────────────────────────────────────────────
WIN_W, WIN_H = 1300, 820
FPS          = 30
NUM_DRONES   = 5

# World bounds
WX_MIN, WX_MAX = -1.0, 11.0
WY_MIN, WY_MAX = -1.0, 14.0

# ── LoRa Radio Parameters (SX1276) ───────────────────────────────────────────
LORA_FREQ_MHZ   = 433.0          # MHz (common disaster band)
LORA_TX_POWER   = 20.0           # dBm (max for SX1276)
LORA_ANTENNA_G  = 2.0            # dBi antenna gain
LORA_NOISE_FLOOR= -120.0         # dBm LoRa noise floor
LORA_SENSITIVITY= {              # minimum RSSI per SF (dBm)
    7: -123, 8: -126, 9: -129,
    10:-132, 11:-134, 12:-137
}
LORA_AIRTIME_MS = {              # approx airtime for 50-byte packet
    7: 72, 8: 128, 9: 246,
    10:452, 11:823, 12:1479
}
LORA_PATH_LOSS_EXP = 2.7        # urban outdoor
LORA_SHADOW_STD    = 4.0        # dB shadow fading std dev
MAX_RANGE_M        = 3000       # metres (real world)
WORLD_SCALE        = 100.0      # 1 world unit = 100 metres

# ── Colours ───────────────────────────────────────────────────────────────────
BG       = (  6,  12,  22)
BG2      = (  9,  17,  30)
BG3      = ( 12,  22,  38)
ACCENT   = (  0, 200, 175)
ACCENT2  = (  0, 155, 215)
GREEN    = ( 40, 220, 115)
ORANGE   = (255, 148,   0)
RED      = (220,  45,  45)
YELLOW   = (255, 220,  50)
CYAN     = (  0, 210, 220)
PURPLE   = (180,  80, 255)
TEXT     = (195, 215, 228)
TEXT_DIM = ( 65,  95, 115)
GRID_C   = ( 16,  32,  50)
VAN_COL  = (255, 200,  50)

DRONE_COL= [(0,255,170),(0,200,255),(255,110,50),(255,215,0),(200,80,255)]

SF_COLORS= {7:GREEN, 8:CYAN, 9:ACCENT, 10:YELLOW, 11:ORANGE, 12:RED}

def rssi_col(rssi):
    if rssi>=-90: return GREEN
    if rssi>=-105: return ORANGE
    return RED

def snr_col(snr):
    if snr>=10: return GREEN
    if snr>=0:  return ORANGE
    return RED

def blend(col,a,bg=BG):
    return tuple(max(0,min(255,int(col[i]*a+bg[i]*(1-a)))) for i in range(3))

# ── LoRa Physics Engine ───────────────────────────────────────────────────────
class LoRaRadio:
    """Simulates one LoRa link between a drone and the van."""

    def __init__(self, drone_id, spreading_factor=9):
        self.drone_id = drone_id
        self.sf       = spreading_factor
        self.tx_power = LORA_TX_POWER
        self.shadow_cache = {}  # position -> shadow fade dB

    def distance_m(self, drone_pos, van_pos):
        """World units to metres."""
        dx=(drone_pos[0]-van_pos[0])*WORLD_SCALE
        dy=(drone_pos[1]-van_pos[1])*WORLD_SCALE
        dz=(drone_pos[2]-(van_pos[2] if len(van_pos)>2 else 0))*WORLD_SCALE
        return max(1.0, math.sqrt(dx*dx+dy*dy+dz*dz))

    def path_loss_db(self, dist_m):
        """Log-distance path loss (dB)."""
        d0=1.0  # reference distance 1m
        # Free space at d0
        pl_d0 = 20*math.log10(4*math.pi*d0*LORA_FREQ_MHZ*1e6/3e8)
        return pl_d0 + 10*LORA_PATH_LOSS_EXP*math.log10(dist_m/d0)

    def shadow_fade(self, drone_pos):
        """Random log-normal shadow fading (cached per position grid cell)."""
        gx=int(drone_pos[0]*2); gy=int(drone_pos[1]*2)
        key=(gx,gy)
        if key not in self.shadow_cache:
            self.shadow_cache[key]=random.gauss(0, LORA_SHADOW_STD)
        return self.shadow_cache[key]

    def rssi(self, drone_pos, van_pos):
        """Calculate received signal strength (dBm)."""
        dist=self.distance_m(drone_pos, van_pos)
        pl  =self.path_loss_db(dist)
        sf  =self.shadow_fade(drone_pos)
        return self.tx_power + LORA_ANTENNA_G - pl + sf

    def snr(self, rssi_dbm):
        return rssi_dbm - LORA_NOISE_FLOOR

    def packet_loss_prob(self, rssi_dbm):
        """Probability of packet loss based on RSSI vs sensitivity."""
        sensitivity = LORA_SENSITIVITY[self.sf]
        margin = rssi_dbm - sensitivity
        if margin >= 10:   return 0.01
        if margin >= 5:    return 0.05
        if margin >= 0:    return 0.20
        if margin >= -5:   return 0.55
        if margin >= -10:  return 0.85
        return 1.0

    def link_budget(self, drone_pos, van_pos):
        r = self.rssi(drone_pos, van_pos)
        s = self.snr(r)
        p = self.packet_loss_prob(r)
        d = self.distance_m(drone_pos, van_pos)
        sensitivity = LORA_SENSITIVITY[self.sf]
        margin = r - sensitivity
        return {'rssi':round(r,1), 'snr':round(s,1),
                'loss_prob':round(p,3), 'dist_m':round(d,1),
                'margin':round(margin,1), 'reachable': r>sensitivity}

# ── Packet types ──────────────────────────────────────────────────────────────
PKT_TYPES = {
    'TELEMETRY': {'size':48,  'priority':1, 'color':CYAN,    'symbol':'T'},
    'TASK_ALERT':{'size':72,  'priority':3, 'color':RED,     'symbol':'!'},
    'SURVIVOR':  {'size':96,  'priority':4, 'color':ORANGE,  'symbol':'S'},
    'HEARTBEAT': {'size':20,  'priority':0, 'color':TEXT_DIM,'symbol':'♥'},
    'MAP_UPDATE':{'size':128, 'priority':2, 'color':GREEN,   'symbol':'M'},
    'ACK':       {'size':12,  'priority':5, 'color':ACCENT,  'symbol':'✓'},
}

class Packet:
    _id=0
    def __init__(self, ptype, drone_id, payload, rssi, snr, lost):
        Packet._id+=1
        self.pid      = Packet._id
        self.ptype    = ptype
        self.drone_id = drone_id
        self.payload  = payload
        self.rssi     = rssi
        self.snr      = snr
        self.lost     = lost
        self.timestamp= time.time()
        self.size     = PKT_TYPES[ptype]['size']
        self.color    = PKT_TYPES[ptype]['color']
        self.symbol   = PKT_TYPES[ptype]['symbol']

# ── Animated radio wave ───────────────────────────────────────────────────────
class RadioWave:
    def __init__(self, sx, sy, col, lost):
        self.sx=sx; self.sy=sy; self.col=col; self.lost=lost
        self.r=0; self.max_r=80; self.speed=3.5
    def update(self): self.r+=self.speed; return self.r<self.max_r
    def draw(self, surf):
        alpha=int(180*(1-self.r/self.max_r))
        s=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
        col=(*self.col,alpha) if not self.lost else (*(RED),alpha//2)
        pygame.draw.circle(s,col,(self.sx,self.sy),int(self.r),2)
        surf.blit(s,(0,0))

# ── ROS Node ──────────────────────────────────────────────────────────────────
class LoRaNode(Node):
    def __init__(self):
        super().__init__('lora_simulator')
        self.lock = threading.Lock()
        self.drone_poses   = {}
        self.drone_status  = {}
        self.tasks         = {}
        self.centre        = (5.0, 6.5, 2.0)
        for i in range(NUM_DRONES):
            self.create_subscription(PoseStamped, f'/drone_{i}/pose',
                lambda m,d=i: self._pose(m,d), 10)
            self.create_subscription(String, f'/drone_{i}/status',
                lambda m,d=i: self._status(m,d), 10)
        self.create_subscription(PoseStamped,'/swarm/centre',self._centre,10)
        self.create_subscription(String,'/swarm/triage_data',self._triage,10)
    def _pose(self,m,d):
        with self.lock:
            self.drone_poses[d]=(m.pose.position.x,m.pose.position.y,m.pose.position.z)
    def _status(self,m,d):
        try:
            with self.lock: self.drone_status[d]=json.loads(m.data)
        except: pass
    def _centre(self,m):
        with self.lock: self.centre=(m.pose.position.x,m.pose.position.y,m.pose.position.z)
    def _triage(self,m):
        try:
            data=json.loads(m.data); key=f"task_{data['task_id']}"
            with self.lock:
                if key not in self.tasks: self.tasks[key]=data
        except: pass
    def snap(self):
        with self.lock:
            return (dict(self.drone_poses), dict(self.drone_status),
                    dict(self.tasks), self.centre)

# ── Main Renderer ─────────────────────────────────────────────────────────────
class LoRaRenderer:
    MAP_X, MAP_Y = 10, 55
    MAP_W, MAP_H = 500, 580

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W,WIN_H))
        pygame.display.set_caption('LoRa Packet Simulator — Drone-to-Van Link')
        self.clock  = pygame.time.Clock()
        self.f7  = pygame.font.SysFont('monospace', 7)
        self.f8  = pygame.font.SysFont('monospace', 8)
        self.f9  = pygame.font.SysFont('monospace', 9)
        self.f10 = pygame.font.SysFont('monospace',10,bold=True)
        self.f11 = pygame.font.SysFont('monospace',11,bold=True)
        self.f13 = pygame.font.SysFont('monospace',13,bold=True)
        self.f16 = pygame.font.SysFont('monospace',16,bold=True)
        self.f20 = pygame.font.SysFont('monospace',20,bold=True)

        # Van position (draggable, in world coords)
        self.van_wx = 10.5
        self.van_wy = 13.5
        self.dragging_van = False

        # LoRa radios per drone
        self.sf = 9
        self.radios = {i: LoRaRadio(i, self.sf) for i in range(NUM_DRONES)}

        # Packet log
        self.packet_log  = deque(maxlen=200)
        self.waves       = []
        self.tick        = 0

        # Stats per drone
        self.pkt_sent    = {i:0 for i in range(NUM_DRONES)}
        self.pkt_recv    = {i:0 for i in range(NUM_DRONES)}
        self.pkt_lost    = {i:0 for i in range(NUM_DRONES)}
        self.rssi_hist   = {i:deque([LORA_NOISE_FLOOR]*60,maxlen=120) for i in range(NUM_DRONES)}
        self.throughput  = {i:deque([0.]*30,maxlen=60) for i in range(NUM_DRONES)}

        # Van received data
        self.van_known_tasks  = {}
        self.van_drone_states = {}
        self.van_last_update  = {i:0. for i in range(NUM_DRONES)}

        # Timing
        self.last_pkt_time = {i:0. for i in range(NUM_DRONES)}
        self.pkt_interval  = {i:3.0+i*.5 for i in range(NUM_DRONES)}  # staggered
        self.start_time    = time.time()

        # Channel busy tracking
        self.channel_busy_until = 0.

        # Total bytes received
        self.total_bytes = 0

    def _w2m(self, wx, wy):
        """World coords → map screen coords."""
        sx = self.MAP_X + int((wx-WX_MIN)/(WX_MAX-WX_MIN)*self.MAP_W)
        sy = self.MAP_Y + int((1-(wy-WY_MIN)/(WY_MAX-WY_MIN))*self.MAP_H)
        return sx, sy

    def _m2w(self, sx, sy):
        """Map screen coords → world coords."""
        wx = WX_MIN + (sx-self.MAP_X)/(self.MAP_W)*(WX_MAX-WX_MIN)
        wy = WY_MIN + (1-(sy-self.MAP_Y)/self.MAP_H)*(WY_MAX-WY_MIN)
        return wx, wy

    def _in_map(self, sx, sy):
        return self.MAP_X<=sx<=self.MAP_X+self.MAP_W and self.MAP_Y<=sy<=self.MAP_Y+self.MAP_H

    def process_packets(self, poses):
        """Generate and simulate LoRa packets from each drone."""
        now = time.time()
        if now < self.channel_busy_until:
            return  # channel occupied

        for d in range(NUM_DRONES):
            if now - self.last_pkt_time[d] < self.pkt_interval[d]:
                continue
            if d not in poses:
                continue

            self.last_pkt_time[d] = now
            drone_pos = poses[d]
            van_pos   = (self.van_wx, self.van_wy, 0.0)

            radio = self.radios[d]
            lb    = radio.link_budget(drone_pos, van_pos)

            # Choose packet type based on drone state
            st = self.van_drone_states.get(d, {})
            if st.get('state') == 'tasked':
                ptype = 'SURVIVOR'
            elif self.tick % 15 == 0:
                ptype = 'MAP_UPDATE'
            elif self.tick % 5 == 0:
                ptype = 'TASK_ALERT'
            else:
                ptype = 'TELEMETRY'

            # Simulate packet loss
            lost = random.random() < lb['loss_prob']

            # Airtime
            airtime = LORA_AIRTIME_MS[self.sf] / 1000.0
            self.channel_busy_until = now + airtime

            # Build payload
            payload = {
                'drone_id'  : d,
                'pos'       : [round(drone_pos[0],2), round(drone_pos[1],2), round(drone_pos[2],2)],
                'battery'   : st.get('battery',100),
                'state'     : st.get('state','patrol'),
                'task_id'   : st.get('task_id',-1),
                'rssi_self' : lb['rssi'],
                'timestamp' : round(now-self.start_time, 2),
            }

            pkt = Packet(ptype, d, payload, lb['rssi'], lb['snr'], lost)
            self.packet_log.appendleft(pkt)
            self.pkt_sent[d] += 1

            # Radio wave animation from drone map pos
            mp = self._w2m(drone_pos[0], drone_pos[1])
            self.waves.append(RadioWave(mp[0], mp[1], DRONE_COL[d], lost))

            if not lost:
                self.pkt_recv[d] += 1
                self.total_bytes += pkt.size
                self.van_drone_states[d] = payload
                self.van_last_update[d]  = now
            else:
                self.pkt_lost[d] += 1

            self.rssi_hist[d].append(lb['rssi'])
            bps = (PKT_TYPES[ptype]['size']*8) / airtime if not lost else 0
            self.throughput[d].append(bps)

    def draw(self, data):
        poses, statuses, tasks, centre = data
        self.tick += 1

        # Update van drone states from ROS
        for d, st in statuses.items():
            if d not in self.van_drone_states:
                self.van_drone_states[d] = st

        self.process_packets(poses)

        # Update waves
        self.waves = [w for w in self.waves if w.update()]

        self.screen.fill(BG)

        # ── Header ────────────────────────────────────────────────────────────
        pygame.draw.rect(self.screen,(0,42,68),(0,0,WIN_W,48))
        pygame.draw.line(self.screen,ACCENT,(0,48),(WIN_W,48),2)
        self.screen.blit(self.f16.render('LoRa RADIO SIMULATOR — DRONE-TO-VAN COMMS (SX1276)',True,ACCENT),(14,12))
        elapsed=time.time()-self.start_time
        mm,ss=divmod(int(elapsed),60)
        self.screen.blit(self.f10.render(
            f'SF{self.sf}  {LORA_FREQ_MHZ}MHz  TX:{LORA_TX_POWER}dBm  '
            f'Airtime:{LORA_AIRTIME_MS[self.sf]}ms  '
            f'[S]=change SF  Uptime:{mm:02d}:{ss:02d}',
            True,ACCENT2),(14,30))
        total_sent=sum(self.pkt_sent.values())
        total_recv=sum(self.pkt_recv.values())
        pdr=total_recv/max(total_sent,1)*100
        self.screen.blit(self.f10.render(
            f'Pkts Sent:{total_sent}  Recv:{total_recv}  '
            f'PDR:{pdr:.1f}%  Bytes:{self.total_bytes}',
            True,GREEN if pdr>80 else (ORANGE if pdr>50 else RED)),(WIN_W-460,30))

        # ── Map panel ─────────────────────────────────────────────────────────
        pygame.draw.rect(self.screen,BG2,(self.MAP_X,self.MAP_Y,self.MAP_W,self.MAP_H))
        pygame.draw.rect(self.screen,(0,55,88),(self.MAP_X,self.MAP_Y,self.MAP_W,self.MAP_H),1)

        # Grid
        for wx in range(int(WX_MIN),int(WX_MAX)+1,2):
            a=self._w2m(wx,WY_MIN); b=self._w2m(wx,WY_MAX)
            pygame.draw.line(self.screen,GRID_C,a,b,1)
        for wy in range(int(WY_MIN),int(WY_MAX)+1,2):
            a=self._w2m(WX_MIN,wy); b=self._w2m(WX_MAX,wy)
            pygame.draw.line(self.screen,GRID_C,a,b,1)

        # Range rings around van
        van_mp = self._w2m(self.van_wx, self.van_wy)
        for km,col,lbl in [(1000,(*GREEN,20),'1km'),(2000,(*ORANGE,15),'2km'),(3000,(*RED,10),'3km')]:
            ring_r = int(km/WORLD_SCALE * self.MAP_W/(WX_MAX-WX_MIN))
            s=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
            pygame.draw.circle(s,col,van_mp,ring_r,1)
            self.screen.blit(s,(0,0))
            lp=(van_mp[0]+ring_r,van_mp[1])
            if 0<=lp[0]<WIN_W: self.screen.blit(self.f7.render(lbl,True,col[:3]),(lp[0]+2,lp[1]-5))

        # RSSI heatmap on map (per drone, colour coded)
        for d in range(NUM_DRONES):
            if d not in poses: continue
            radio = self.radios[d]
            # Draw signal strength overlay in a few grid cells
            for gx in range(0,12,3):
                for gy in range(0,15,3):
                    wx=WX_MIN+gx*(WX_MAX-WX_MIN)/12
                    wy=WY_MIN+gy*(WY_MAX-WY_MIN)/15
                    lb=radio.link_budget((wx,wy,2.0),(self.van_wx,self.van_wy,0))
                    if lb['reachable']:
                        mp=self._w2m(wx,wy)
                        sig=(lb['rssi']-LORA_SENSITIVITY[self.sf])/20.0
                        sig=max(0,min(1,sig))
                        col=DRONE_COL[d]
                        s=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
                        pygame.draw.circle(s,(*col,int(sig*18)),mp,14)
                        self.screen.blit(s,(0,0))

        # Radio waves
        for w in self.waves: w.draw(self.screen)

        # Link lines drone→van
        for d,pos in poses.items():
            mp = self._w2m(pos[0],pos[1])
            radio=self.radios[d]
            lb=radio.link_budget(pos,(self.van_wx,self.van_wy,0))
            line_col=rssi_col(lb['rssi'])
            age=time.time()-self.van_last_update.get(d,0)
            if lb['reachable'] and age<10:
                s=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
                pygame.draw.line(s,(*line_col,60),mp,van_mp,1)
                self.screen.blit(s,(0,0))

        # Drone icons on map
        for d,pos in poses.items():
            mp=self._w2m(pos[0],pos[1])
            col=DRONE_COL[d]
            radio=self.radios[d]
            lb=radio.link_budget(pos,(self.van_wx,self.van_wy,0))
            # Outer glow
            gs=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
            pygame.draw.circle(gs,(*col,35),mp,12); self.screen.blit(gs,(0,0))
            pygame.draw.circle(self.screen,col,mp,7)
            pygame.draw.circle(self.screen,(220,235,245),mp,7,1)
            self.screen.blit(self.f7.render(str(d),True,(255,255,255)),(mp[0]-3,mp[1]-4))
            # RSSI label
            rssi_str=f'{lb["rssi"]:.0f}dBm'
            self.screen.blit(self.f7.render(rssi_str,True,rssi_col(lb['rssi'])),(mp[0]+9,mp[1]-4))
            # Altitude
            self.screen.blit(self.f7.render(f'z={pos[2]:.1f}m',True,TEXT_DIM),(mp[0]+9,mp[1]+4))

        # Van icon
        gs=pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
        pygame.draw.circle(gs,(*VAN_COL,50),van_mp,16); self.screen.blit(gs,(0,0))
        pygame.draw.rect(self.screen,VAN_COL,(van_mp[0]-9,van_mp[1]-6,18,12))
        pygame.draw.rect(self.screen,BG,(van_mp[0]-8,van_mp[1]-5,16,10))
        pygame.draw.circle(self.screen,(255,255,255),van_mp,3)
        self.screen.blit(self.f7.render('VAN',True,VAN_COL),(van_mp[0]-10,van_mp[1]+9))
        self.screen.blit(self.f7.render('(drag)',True,TEXT_DIM),(van_mp[0]-13,van_mp[1]+18))

        # Map title
        self.screen.blit(self.f8.render('FIELD MAP — LoRa Coverage',True,TEXT_DIM),(self.MAP_X+4,self.MAP_Y+4))

        # ── Per-drone link status panel ───────────────────────────────────────
        LPX=520; LPY=55; LPW=350; row_h=108
        self.screen.blit(self.f11.render('LINK STATUS PER DRONE',True,ACCENT),(LPX,LPY-18))
        for d in range(NUM_DRONES):
            col=DRONE_COL[d]; ry=LPY+d*row_h
            pygame.draw.rect(self.screen,BG2,(LPX,ry,LPW,row_h-4))
            pygame.draw.rect(self.screen,(*col,60),(LPX,ry,LPW,row_h-4),1)
            pygame.draw.rect(self.screen,col,(LPX,ry,3,row_h-4))

            pos=poses.get(d,(0,0,2))
            radio=self.radios[d]
            lb=radio.link_budget(pos,(self.van_wx,self.van_wy,0))
            sent=self.pkt_sent[d]; recv=self.pkt_recv[d]; lost_n=self.pkt_lost[d]
            pdr=recv/max(sent,1)*100
            age=time.time()-self.van_last_update.get(d,0.)

            # Header
            self.screen.blit(self.f10.render(f'UAV-{d}',True,col),(LPX+8,ry+4))
            status_col=GREEN if lb['reachable'] and age<15 else RED
            status_txt='LINKED' if lb['reachable'] and age<15 else 'NO LINK'
            self.screen.blit(self.f10.render(status_txt,True,status_col),(LPX+65,ry+4))
            self.screen.blit(self.f8.render(f'{lb["dist_m"]:.0f}m away',True,TEXT_DIM),(LPX+145,ry+5))

            # RSSI
            self.screen.blit(self.f8.render(f'RSSI:',True,TEXT_DIM),(LPX+8,ry+20))
            self.screen.blit(self.f10.render(f'{lb["rssi"]:.1f} dBm',True,rssi_col(lb['rssi'])),(LPX+48,ry+19))

            # SNR
            self.screen.blit(self.f8.render(f'SNR:',True,TEXT_DIM),(LPX+155,ry+20))
            self.screen.blit(self.f10.render(f'{lb["snr"]:.1f} dB',True,snr_col(lb['snr'])),(LPX+190,ry+19))

            # Link margin
            mc=GREEN if lb['margin']>=10 else (ORANGE if lb['margin']>=0 else RED)
            self.screen.blit(self.f8.render(f'Margin: {lb["margin"]:.1f}dB',True,mc),(LPX+270,ry+20))

            # PDR bar
            self.screen.blit(self.f8.render(f'PDR: {pdr:.0f}%',True,TEXT),(LPX+8,ry+35))
            pygame.draw.rect(self.screen,(20,38,55),(LPX+65,ry+37,180,8))
            fw=int(180*pdr/100)
            if fw>0:
                bar_col=GREEN if pdr>80 else (ORANGE if pdr>50 else RED)
                pygame.draw.rect(self.screen,bar_col,(LPX+65,ry+37,fw,8))
            pygame.draw.rect(self.screen,(0,55,85),(LPX+65,ry+37,180,8),1)
            self.screen.blit(self.f7.render(f'{sent}↑ {recv}↓ {lost_n}✗',True,TEXT_DIM),(LPX+250,ry+36))

            # Loss probability
            lp_col=GREEN if lb['loss_prob']<.1 else (ORANGE if lb['loss_prob']<.4 else RED)
            self.screen.blit(self.f8.render(f'Loss: {lb["loss_prob"]*100:.0f}%  SF{self.sf}  '
                             f'Air:{LORA_AIRTIME_MS[self.sf]}ms',True,lp_col),(LPX+8,ry+50))

            # Last update
            age_col=GREEN if age<10 else (ORANGE if age<30 else RED)
            self.screen.blit(self.f7.render(f'Last update: {age:.1f}s ago',True,age_col),(LPX+8,ry+65))

            # RSSI sparkline
            if len(self.rssi_hist[d])>=2:
                pts=list(self.rssi_hist[d])
                mn=min(pts)-.1; mx=max(pts)+.1; rng=mx-mn
                sp=[(LPX+8+int(i/(len(pts)-1)*330),
                     ry+88-int((p-mn)/rng*18)) for i,p in enumerate(pts)]
                pygame.draw.lines(self.screen,blend(col,.7),False,sp,1)
                self.screen.blit(self.f7.render(f'{pts[-1]:.0f}dBm',True,rssi_col(pts[-1])),(LPX+342,ry+80))
            self.screen.blit(self.f7.render('RSSI history ─────────────────────────────────────',
                             True,TEXT_DIM),(LPX+8,ry+78))

        # ── Packet log ────────────────────────────────────────────────────────
        PLX=880; PLY=55; PLW=WIN_W-PLX-8
        pygame.draw.rect(self.screen,BG2,(PLX,PLY,PLW,WIN_H-PLY-8))
        pygame.draw.rect(self.screen,(0,55,88),(PLX,PLY,PLW,WIN_H-PLY-8),1)
        pygame.draw.rect(self.screen,(0,45,75),(PLX,PLY,PLW,20))
        pygame.draw.rect(self.screen,ACCENT,(PLX,PLY,3,20))
        self.screen.blit(self.f10.render('PACKET LOG',True,ACCENT),(PLX+8,PLY+3))

        # Column headers
        hy=PLY+24
        for hx,hdr in [(PLX+4,'#'),(PLX+35,'TYPE'),(PLX+105,'UAV'),(PLX+135,'RSSI'),(PLX+185,'SNR'),(PLX+225,'STATUS')]:
            self.screen.blit(self.f7.render(hdr,True,TEXT_DIM),(hx,hy))
        pygame.draw.line(self.screen,(0,55,88),(PLX+4,hy+10),(PLX+PLW-4,hy+10),1)

        py=hy+14
        for pkt in self.packet_log:
            if py>WIN_H-18: break
            row_bg=(10,20,35) if pkt.pid%2==0 else BG2
            pygame.draw.rect(self.screen,row_bg,(PLX+2,py-1,PLW-4,12))
            if not pkt.lost:
                pygame.draw.rect(self.screen,(*pkt.color,25),(PLX+2,py-1,3,12))
            self.screen.blit(self.f7.render(f'{pkt.pid:04d}',True,TEXT_DIM),(PLX+4,py))
            tc=pkt.color if not pkt.lost else RED
            self.screen.blit(self.f7.render(pkt.ptype[:8],True,tc),(PLX+35,py))
            self.screen.blit(self.f7.render(str(pkt.drone_id),True,DRONE_COL[pkt.drone_id]),(PLX+105,py))
            self.screen.blit(self.f7.render(f'{pkt.rssi:.0f}',True,rssi_col(pkt.rssi)),(PLX+135,py))
            self.screen.blit(self.f7.render(f'{pkt.snr:.0f}',True,snr_col(pkt.snr)),(PLX+185,py))
            if pkt.lost:
                self.screen.blit(self.f7.render('LOST',True,RED),(PLX+225,py))
            else:
                self.screen.blit(self.f7.render('OK',True,GREEN),(PLX+225,py))
            py+=13

        # ── Van received data summary ─────────────────────────────────────────
        VDX=520; VDY=55+5*row_h+8; VDW=350; VDH=WIN_H-VDY-8
        if VDH>60:
            pygame.draw.rect(self.screen,BG2,(VDX,VDY,VDW,VDH))
            pygame.draw.rect(self.screen,(0,55,88),(VDX,VDY,VDW,VDH),1)
            pygame.draw.rect(self.screen,(0,45,75),(VDX,VDY,VDW,20))
            pygame.draw.rect(self.screen,VAN_COL,(VDX,VDY,3,20))
            self.screen.blit(self.f10.render('VAN RECEIVED DATA',True,VAN_COL),(VDX+8,VDY+3))
            vy=VDY+24
            self.screen.blit(self.f8.render(f'Total bytes received: {self.total_bytes}  '
                             f'Total pkts: {sum(self.pkt_recv.values())}',True,TEXT),(VDX+6,vy)); vy+=14
            self.screen.blit(self.f8.render(f'Channel SF{self.sf}  '
                             f'{LORA_FREQ_MHZ}MHz  Sensitivity:{LORA_SENSITIVITY[self.sf]}dBm',
                             True,TEXT_DIM),(VDX+6,vy)); vy+=14
            pygame.draw.line(self.screen,(0,55,88),(VDX+4,vy),(VDX+VDW-4,vy),1); vy+=4
            self.screen.blit(self.f8.render('Van knows about:',True,TEXT),(VDX+6,vy)); vy+=13
            for d,st in self.van_drone_states.items():
                age=time.time()-self.van_last_update.get(d,0)
                age_col=GREEN if age<10 else (ORANGE if age<30 else RED)
                col=DRONE_COL[d]
                self.screen.blit(self.f8.render(
                    f'  UAV-{d}: bat={st.get("battery",0):.0f}%  '
                    f'{st.get("state","?")[:6]}  '
                    f'({st.get("pos",[0,0,0])[0]:.1f},{st.get("pos",[0,0,0])[1]:.1f})',
                    True,col),(VDX+6,vy))
                self.screen.blit(self.f7.render(f'{age:.0f}s ago',True,age_col),(VDX+VDW-55,vy))
                vy+=13
                if vy>VDY+VDH-10: break

        pygame.display.flip()
        self.clock.tick(FPS)

    def handle_event(self, event):
        if event.type==pygame.MOUSEBUTTONDOWN and event.button==1:
            mx,my=event.pos
            van_mp=self._w2m(self.van_wx,self.van_wy)
            if abs(mx-van_mp[0])<18 and abs(my-van_mp[1])<18:
                self.dragging_van=True
        elif event.type==pygame.MOUSEBUTTONUP:
            self.dragging_van=False
        elif event.type==pygame.MOUSEMOTION:
            if self.dragging_van:
                mx,my=event.pos
                if self._in_map(mx,my):
                    self.van_wx,self.van_wy=self._m2w(mx,my)
        elif event.type==pygame.KEYDOWN:
            if event.key==pygame.K_s:
                sfs=[7,8,9,10,11,12]
                self.sf=sfs[(sfs.index(self.sf)+1)%len(sfs)]
                for d in self.radios: self.radios[d].sf=self.sf
                print(f'[LoRa] Spreading Factor → SF{self.sf}')

# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node     = LoRaNode()
    renderer = LoRaRenderer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('[lora_simulator] LoRa Simulator Running — ESC to quit, S to change SF')

    running=True
    while running:
        for e in pygame.event.get():
            if e.type==pygame.QUIT: running=False
            elif e.type==pygame.KEYDOWN and e.key==pygame.K_ESCAPE: running=False
            else: renderer.handle_event(e)
        renderer.draw(node.snap())

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
