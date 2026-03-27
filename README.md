# 🚁 Swarm Drone — Flood Zone Triage System

A multi-drone autonomous rescue coordination system built on **ROS2**.  
5 UAVs patrol a flood zone, detect survivors, calculate survival probability,  
and autonomously dispatch the highest-priority drone to each rescue task.

---

## 📁 Project Files

| File | Purpose |
|------|---------|
| `swarm_manager.py` | Central coordinator — formation control + triage-aware auction |
| `drone_controller.py` | Per-drone flight controller — potential field + wind + payload |
| `task_generator.py` | Spawns flood victims with survival probability + urgency scoring |
| `swarm_visualizer_.py` | Main 3D visualizer — pygame fake-3D with charts and dashboard |
| `lora_simulator.py` | Optional — simulates LoRa radio link between drones and ground van |
| `run.sh` | One-command launcher for all nodes |

---

## ⚙️ Requirements (Install these to run the PROJECT)-

### Operating System
- Ubuntu 22.04 (native or WSL2 on Windows)

### ROS2
#Note- Please run the following codes in the Ubuntu Terminal
```bash
# Install ROS2 Humble
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Python Dependencies
```bash
sudo apt install python3-pip -y
pip3 install pygame PyOpenGL PyOpenGL_accelerate
```

---

## 🚀 How to Run

pkill -f drone_controller.py 2>/dev/null
pkill -f swarm_manager.py 2>/dev/null
pkill -f task_generator.py 2>/dev/null
pkill -f swarm_visualizer 2>/dev/null
pkill -f lora_simulator.py 2>/dev/null
pkill -f analytics_dashboard.py 2>/dev/null
sleep 2
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/saurabh/Pictures/swarm_folder  #NOTE- PLEASE CHANGE Location According to your PC

python3 swarm_manager.py &
sleep 1
python3 task_generator.py &
sleep 1
for i in 0 1 2 3 4; do
  python3 drone_controller.py --ros-args -p drone_id:=$i -p start_x:=$(python3 -c "print($i*1.5)") -p start_y:=0.0 -p start_z:=2.0 &
done
sleep 1
python3 swarm_visualizer.py

# Run LoRa Radio Simulator (optional, extra terminal)
```bash
source /opt/ros/humble/setup.bash
cd /path/to/swarm_folder    
python3 lora_simulator.py
```
#Example- 
cd /mnt/c/Users/saurabh/Pictures/swarm_folder
python3 lora_simulator.py


# Run ANALYTICS DASHBOARD (optional, extra terminal)
```bash
source /opt/ros/humble/setup.bash
cd /path/to/swarm_folder   
python3 analytics_dashboard.py
```
#Example-
cd /mnt/c/Users/saurabh/Pictures/swarm_folder
python3 analytics_dashboard.py

## 🎮 Visualizer Controls

| Input | Action |
|-------|--------|
| Left drag | Rotate camera |
| Scroll wheel | Zoom in/out |
| `+` / `-` keys | Zoom in/out (keyboard) |
| `R` | Reset camera to default |
| Left click on map | Spawn manual rescue task |
| `ESC` | Quit |

---

## 🧠 How It Works

### 1. Patrol & Detection
Drones fly in formation following a boustrophedon (lawnmower) path over the flood zone.  
The camera detects survivors and classifies them by profile (infant, elderly, cardiac, etc).

### 2. Triage Scoring
Each detected person gets a survival probability and urgency score:
```
urgency = 0.4×water_level + 0.3×condition + 0.2×(1−survival) + 0.1×time_elapsed
```

Priority labels: `CRITICAL (≥0.75)` → `HIGH (≥0.50)` → `MEDIUM (≥0.30)` → `LOW`

### 3. Auction-Based Dispatch
When a task appears, every available drone bids:
```
score = 0.5×urgency + 0.4×(1/distance) + 0.1×(battery/100)
```
Wind direction and payload weight also affect the score.  
Highest score wins — most critical person always gets a drone first.

### 4. Rescue
The dispatched drone flies to the survivor, drops supplies (life jacket, medicine, rope, food),  
relays comms, and precision-marks the location for the boat crew.

---

## 📡 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_N/pose` | PoseStamped | Drone position (x,y,z) |
| `/drone_N/status` | String (JSON) | Battery, state, task_id, payload, kit |
| `/drone_N/cmd_vel` | Twist | Velocity command |
| `/drone_N/formation_goal` | PoseStamped | Formation target from manager |
| `/drone_N/task_goal` | PoseStamped | Rescue task assignment |
| `/swarm/centre` | PoseStamped | Swarm formation centre point |
| `/swarm/detected_task` | PoseStamped | New task detected / spawned |
| `/swarm/triage_data` | String (JSON) | Full triage data for visualizer |
| `/swarm/change_formation` | String | Change formation shape |

---

## 🌬️ Wind & Payload Model

**Wind** — constant base wind + random gusts every ~2 seconds:
```
WIND_X = 0.25 m/s (eastward)
WIND_Y = 0.10 m/s (northward)
WIND_GUST = ±0.12 m/s random turbulence
```

**Rescue Kit** — 0.80kg total, depletes per rescue:
| Item | Weight | Used for |
|------|--------|---------|
| Life jacket | 0.28 kg | Flotation |
| Medicine box | 0.18 kg | Insulin, BP meds |
| Rescue rope 25m | 0.22 kg | Extraction |
| Food + water | 0.12 kg | Emergency ration |

Loaded drone: max speed reduced from `1.5 m/s → 0.83 m/s`

---

## 🏗️ Algorithms Used

| Algorithm | File | Purpose |
|-----------|------|---------|
| Weighted Greedy Auction | swarm_manager.py | Task allocation |
| Artificial Potential Field | drone_controller.py | Motion + collision avoidance |
| Boustrophedon Coverage | swarm_manager.py | Systematic area sweep |
| Fixed Offset Formation | swarm_manager.py | Swarm geometry |
| Linear Survival Decay | task_generator.py | Survival probability model |
| Euler Integration | drone_controller.py | Position simulation |

---

## 📊 Dashboard Panels

**Left — 3D Map View**
- Real-time drone positions with altitude poles and spinning rotors
- Flood zones as water volumes
- Survivor beacon pillars (height = urgency)
- Wind arrow + payload KIT indicator
- Rain, debris, ripple effects

**Centre — Analytics**
- Survival probability over time (Infant / Elderly / All)
- Search grid coverage heat map
- UAV deployment vs rescues chart
- Case status donut chart

**Right — Command Panel**
- Fleet status (battery bar + kit bar + rescue count per drone)
- Situation distribution
- Priority queue (pending tasks sorted by urgency)
- Rescued log

---

## ⚠️ Troubleshooting

**`Command 'python' not found`**
```bash
sudo apt install python-is-python3 -y
```

**`pip3: command not found`**
```bash
sudo apt install python3-pip -y
```
**ROS2 source error**
```bash
source /opt/ros/humble/setup.bash
```

**Old processes still running (ghost nodes)**
```bash
pkill -f drone_controller.py
pkill -f swarm_manager.py
pkill -f task_generator.py
pkill -f swarm_visualizer
pkill -f lora_simulator.py 
pkill -f analytics_dashboard.py
sleep 2
bash run.sh
```

**Paste not working in WSL terminal**
- Right-click to paste
- Or: `bind 'set enable-bracketed-paste off'`

---

## Team-
Flood Zone UAV Swarm Triage System  
Built with ROS2 Humble + Python 3 + Pygame
