# Intelligent Bus Race: Hybrid AI (Road to Intelligence: The Agent’s Race)

A real-time **2-bus racing simulation** built with **Python + Pygame**, where two agents compete on a **5-lane road** filled with **dynamic obstacles** and **pedestrians**.  
Each bus is controlled by a **Hybrid AI** that combines:

- **BFS / DFS** for lane/path decisions  
- **Minimax + Alpha–Beta pruning** for opponent-aware choices  
- **Fuzzy logic** to brake near road edges

---

## 🎮 Gameplay Overview

- Two buses race to the finish line (target distance: **2000m**)
- The road has **5 lanes**
- Obstacles and pedestrians spawn over time
- Collisions reduce **life** and **score**
- Going **off-track** ends the run immediately

### Win Conditions
A bus wins if it:
1. Reaches the finish distance first, **or**
2. The opponent loses (life reaches 0) / gets forced off-track

---

## 🧠 Hybrid AI: How It Works

### 1) Hazard detection (lookahead scan)
The agent scans hazards ahead and marks lanes as safe/unsafe using a **safe distance threshold**.

### 2) BFS lane planning (safe route)
If safe lanes exist, the agent runs **BFS** to find the **shortest lane-change sequence** to a safe lane.

### 3) DFS fallback (limited depth)
If BFS can’t find a good option (or the situation is tight), it switches to **depth-limited DFS** (aggressive exploration).

### 4) Minimax + Alpha–Beta (opponent modeling)
When the opponent is near, the agent uses **Minimax** (with **Alpha–Beta pruning**) to predict opponent moves and choose counter-actions.

### 5) Fuzzy edge control (brake near boundaries)
A fuzzy membership score estimates how close the bus is to the road edge:

\[
\mu_{danger}(x) = \max\left(\min\left(\frac{d_{left}}{60}, 1\right), \min\left(\frac{d_{right}}{60}, 1\right)\right)
\]

If `μdanger > 0.80`, the bus **brakes** to avoid going out of bounds.

---

## 💥 Scoring & Collision Rules

| Event | Effect |
|------|--------|
| Obstacle collision | Life −10%, Score −10 |
| Pedestrian collision | Life −25%, Score −25 |
| Bus-to-bus collision | Life −30%, Score −30 |
| Off-track | Game Over |
| Survival bonus | +10 score / second |
| Distance bonus | +1 score / 30m |
| Health regen | +2% life / second (max 100%) |

---

## ⌨️ Controls

### In-game keys
- **P** → Pause  
- **S** → Resume  
- **R** → Reset  
- **Q** / **ESC** → Quit  

### Human vs AI mode (if using `app.py`)
- **Arrow Left / Right** → Change lane  
- **Arrow Up** → Accelerate  
- **Arrow Down** → Brake  

---

## 🛠️ Requirements

- **Python 3.9+**
- **Pygame 2.x**

---

## 🚀 Installation

### 1) Create a virtual environment
```bash
python -m venv .venv
```
### 2) Create a virtual environment(Windows)
```
.\.venv\Scripts\activate
pip install pygame
```
### 3)▶️ Run the Game
Option A (Recommended): Mode selection (Human vs AI / AI vs AI)
```
python app.py
```
### 4)Option B: Direct AI vs AI run
```
python main.py
```

###5)📁 Project Structure (typical)
```
app.py — Mode selection + game loop (Human vs AI / AI vs AI)

main.py — Hybrid AI race (AI vs AI)

two_buses_game.py / final.py — Alternative versions / experiments

AI_2_Bus_Game (1).pdf — Project report
```
###6 🖼️ Screenshots / Demo

<img width="1247" height="841" alt="UI" src="https://github.com/user-attachments/assets/605b5d35-db24-410f-8d04-7b7de2535e38" />

<img width="1248" height="871" alt="A win" src="https://github.com/user-attachments/assets/58d1de3f-ee28-491b-9840-c180878b5bd0" />
<img width="1242" height="847" alt="B bus Wins" src="https://github.com/user-attachments/assets/bd23f675-846b-4dd0-a6ee-1f76b9341218" />
<img width="1243" height="827" alt="inter" src="https://github.com/user-attachments/assets/1d4f3b5c-6e24-4d04-b23c-fc2e80feda72" />
<img width="1248" height="871" alt="Screenshot 2025-11-01 170322" src="https://github.com/user-attachments/assets/3168a28c-a27b-45a6-8288-6c693438dd47" />
<img width="1245" height="855" alt="Screenshot 2025-11-01 174305" src="https://github.com/user-attachments/assets/0b251dbd-c0a5-4cde-8220-1f916720978b" />

<img width="696" height="470" alt="fuzzy" src="https://github.com/user-attachments/assets/f5e6528e-b8ff-42b4-a9eb-56f7c53e0208" />
<img width="989" height="590" alt="fuzzy logics" src="https://github.com/user-attachments/assets/9250c40b-35bb-46e8-9bc2-56c4c9017b8e" />


###7 ⚙️ Key Parameters (Defaults) :
```
Screen: 1000×700

Lanes: 5 (road width 700)

FPS: 60

Finish distance: 2000m

Obstacle spawn: 2.5s

Pedestrian spawn: 3.5s

Max speed: 120 km/h

Safe distance threshold: 120 px

DFS depth: 2

Minimax depth: 2 ply

Fuzzy brake threshold: 0.80
```
### 8 👥 Authors:
    Sourav Debnath (Roll: 2007109)

    Tirtho Mondal (Roll: 2007117)

Course: CSE 4110 — Artificial Intelligence Laboratory
Khulna University of Engineering & Technology (KUET)
