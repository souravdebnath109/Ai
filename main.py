#!/usr/bin/env python3
"""
Main entry point for Intelligent Bus Race - Hybrid AI
Runs the game with Hybrid AI (BFS/DFS/Minimax/Fuzzy Logic)
"""

import pygame
import sys
import random
import math
from collections import deque

# --- Constants ---
SCREEN_W, SCREEN_H = 1000, 700
FPS = 60
LANES = 5
ROAD_W = 700
LANE_W = ROAD_W // LANES
ROAD_X = (SCREEN_W - ROAD_W) // 2
KMH_TO_MPS = 1000.0 / 3600.0
BUS_W = int(LANE_W * 0.8)
BUS_H = 60
OBSTACLE_SPAWN_INTERVAL = 2.5
PEDESTRIAN_SPAWN_INTERVAL = 3.5
FINISH_DIST = 2000.0

def fuzzy_outside(self_x):
    left_dist = (self_x - ROAD_X)
    right_dist = (ROAD_X + ROAD_W - self_x)
    return max(min(left_dist / 60.0, 1.0), min(right_dist / 60.0, 1.0))

class Bus:
    def __init__(self, lane_idx, color, name, agent_cls):
        self.x = ROAD_X + lane_idx * LANE_W + (LANE_W - BUS_W) / 2
        self.y = SCREEN_H - 180 + random.uniform(-18.0, 18.0)
        self.w = BUS_W
        self.h = BUS_H
        self.color = color
        self.name = name
        self.max_speed = 120.0
        self.speed = 0.0
        self.accel = 40.0
        self.brake_power = 100.0
        self.lat_speed = 8.0
        self.progress_m = 0.0
        self.life = 100.0
        self.score = 0
        self.dead = False
        self.off_track = False
        self.agent = agent_cls(self)
        self.time_alive = 0.0
        self.last_progress_score = 0

    def update(self, dt, world, opponent):
        if self.dead or self.off_track:
            self.speed = 0
            return
        self.time_alive += dt
        if self.time_alive >= 1.0:
            self.score += 10
            self.life = min(100.0, self.life + 2)
            self.time_alive = 0
        controls = self.agent.decide(world, opponent)
        if controls.get("accel"):
            self.speed += self.accel * dt
        elif controls.get("brake"):
            self.speed -= self.brake_power * dt
        else:
            self.speed -= 5.0 * dt
        if controls.get("left"):
            self.x -= self.lat_speed * dt * 60.0
        if controls.get("right"):
            self.x += self.lat_speed * dt * 60.0
        left_bound = ROAD_X + 6
        right_bound = ROAD_X + ROAD_W - self.w - 6
        if self.x < left_bound or self.x > right_bound:
            self.off_track = True
        self.x = max(left_bound, min(self.x, right_bound))
        self.y = max(100, min(self.y, SCREEN_H - 100))
        self.speed = max(0.0, min(self.speed, self.max_speed))
        self.progress_m += self.speed * KMH_TO_MPS * dt
        dist_score = int(self.progress_m // 30)
        if dist_score > self.last_progress_score:
            self.score += dist_score - self.last_progress_score
            self.last_progress_score = dist_score
        self.life = max(0.0, self.life)
        if self.life <= 0.0:
            self.dead = True
            self.speed = 0
            self.life = 0

    def crash(self, obstacle=False, pedestrian=False, bus=False):
        if obstacle:
            self.life -= 10
            self.score = max(0, self.score - 10)
        if pedestrian:
            self.life -= 25
            self.score = max(0, self.score - 25)
        if bus:
            self.life -= 30
            self.score = max(0, self.score - 30)
        self.life = max(0.0, self.life)
        if self.life <= 0.0:
            self.dead = True
            self.speed = 0
            self.life = 0

    def lane_index(self):
        idx = int((self.x - ROAD_X) // LANE_W)
        return max(0, min(idx, LANES - 1))

    def draw(self, surf):
        rect = pygame.Rect(int(self.x), int(self.y), self.w, self.h)
        color = (100, 100, 100) if self.dead or self.off_track else self.color
        pygame.draw.rect(surf, color, rect, border_radius=6)
        pygame.draw.rect(surf, (220, 240, 255), (rect.x + 8, rect.y + 8, rect.w - 16, int(rect.h / 2)), border_radius=3)
        font = getattr(self, "_label_font", None)
        if font is None:
            font = pygame.font.SysFont("arial", 18)
        lbl = font.render(self.name, True, (255, 255, 255))
        surf.blit(lbl, (rect.centerx - lbl.get_width() // 2, rect.y - 18))

    def rect(self):
        return pygame.Rect(int(self.x), int(self.y), self.w, self.h)

class Obstacle:
    def __init__(self, lane_idx, y=-100):
        self.x = ROAD_X + lane_idx * LANE_W + (LANE_W - 40) / 2
        self.y = y
        self.w = 40
        self.h = 40
    def update(self, scroll_speed, dt):
        self.y += scroll_speed * dt * 0.3
    def draw(self, surf):
        pygame.draw.rect(surf, (120, 80, 20), pygame.Rect(int(self.x), int(self.y), self.w, self.h))

class Pedestrian:
    def __init__(self, lane_idx, y=-60):
        self.x = ROAD_X + lane_idx * LANE_W + LANE_W * 0.2
        self.y = y
        self.w = 16
        self.h = 28
        self.vx = random.choice([-1.0, 1.0])
        self.vy = 1.0
    def update(self, dt, scroll_speed):
        self.x += self.vx * dt * 50
        self.y += self.vy * dt * 50 + scroll_speed * dt
    def draw(self, surf):
        pygame.draw.rect(surf, (0, 200, 0), pygame.Rect(int(self.x), int(self.y), self.w, self.h))

def bfs_lane(safe_lanes, cur):
    queue = deque([(cur, [])])
    visited = set([cur])
    while queue:
        lane, path = queue.popleft()
        if lane in safe_lanes:
            return path
        for shift in [-1, 1]:
            next_lane = lane + shift
            if 0 <= next_lane < LANES and next_lane not in visited:
                visited.add(next_lane)
                queue.append((next_lane, path + [next_lane]))
    return []

def dfs_lane(safe_lanes, cur, max_depth=2):
    stack = [(cur, [])]
    visited = set([cur])
    while stack:
        lane, path = stack.pop()
        if lane in safe_lanes:
            return path
        if len(path) > max_depth:
            continue
        for shift in [-1, 1]:
            next_lane = lane + shift
            if 0 <= next_lane < LANES and next_lane not in visited:
                visited.add(next_lane)
                stack.append((next_lane, path + [next_lane]))
    return []

def minimax_lane(bus, opponent, lane_hazards, depth=2, maximizing=True, alpha=-math.inf, beta=math.inf):
    if depth == 0 or bus.life <= 0 or opponent.life <= 0:
        my_reward = -sum([len(h) for h in lane_hazards])
        return my_reward
    possible = []
    for shift in [-1, 0, 1]:
        l = bus.lane_index() + shift
        if l < 0 or l >= LANES:
            continue
        reward = -len(lane_hazards[l])
        possible.append((reward, l))
    if maximizing:
        max_eval = -math.inf
        for _, l in possible:
            eval = minimax_lane(opponent, bus, lane_hazards, depth-1, not maximizing, alpha, beta)
            max_eval = max(max_eval, eval)
            alpha = max(alpha, eval)
            if beta <= alpha:
                break
        return max_eval
    else:
        min_eval = math.inf
        for _, l in possible:
            eval = minimax_lane(opponent, bus, lane_hazards, depth - 1, not maximizing, alpha, beta)
            min_eval = min(min_eval, eval)
            beta = min(beta, eval)
            if beta <= alpha:
                break
        return min_eval

class HybridAIBus:
    def __init__(self, bus):
        self.bus = bus
        self.target_speed = 120.0
    def decide(self, world, opponent):
        controls = dict(left=False, right=False, accel=False, brake=False)
        current_lane = self.bus.lane_index()
        my_y = self.bus.y
        LOOKAHEAD = 1000
        SAFE_DIST = 120
        opp_y = getattr(opponent, 'y', None) if opponent is not None else None
        opp_dead = getattr(opponent, 'dead', False) if opponent is not None else True
        opp_lane = None
        if hasattr(opponent, 'lane_index'):
            try:
                opp_lane = opponent.lane_index()
            except Exception:
                pass
        elif isinstance(opponent, dict):
            opp_lane = opponent.get('lane')
        lane_hazards = [[] for _ in range(LANES)]
        for obj in world.get("obstacles", []):
            lane = int((obj.x - ROAD_X) // LANE_W)
            lane = max(0, min(lane, LANES - 1))
            dy = my_y - obj.y
            if 0 < dy < LOOKAHEAD:
                lane_hazards[lane].append((dy, "obstacle"))
        for obj in world.get("pedestrians", []):
            lane = int((obj.x - ROAD_X) // LANE_W)
            lane = max(0, min(lane, LANES - 1))
            dy = my_y - obj.y
            if 0 < dy < LOOKAHEAD:
                lane_hazards[lane].append((dy, "pedestrian"))
        safe_lanes = [l for l in range(LANES) if all(dy > SAFE_DIST for dy, _ in lane_hazards[l])]
        path = bfs_lane(safe_lanes, current_lane) if safe_lanes else []
        if (not opp_dead) and (opp_y is not None) and abs(my_y - opp_y) < 300:
            for i in range(LANES):
                if opp_lane is not None and i == opp_lane:
                    lane_hazards[i] = [(dy, "opponent") for dy, _ in lane_hazards[i]]
            if not path and safe_lanes:
                path = dfs_lane(safe_lanes, current_lane)
        next_lane = current_lane
        if path:
            next_lane = path[0]
        elif safe_lanes:
            next_lane = safe_lanes[0]
        if next_lane < current_lane:
            controls["left"] = True
        elif next_lane > current_lane:
            controls["right"] = True
        edge_val = fuzzy_outside(self.bus.x)
        if edge_val > 0.80:
            controls["brake"] = True
        if not any(lane_hazards[current_lane]) or min([dy for dy, _ in lane_hazards[current_lane]], default=999) > SAFE_DIST * 2:
            controls["accel"] = True
        else:
            controls["brake"] = True
        return controls

def clean_bus_collision(b1, b2):
    if b1.lane_index() == b2.lane_index() and abs(b1.y - b2.y) < BUS_H:
        b1.crash(bus=True)
        b2.crash(bus=True)
        overlap = BUS_H - abs(b1.y - b2.y)
        if overlap > 0:
            shift = max(4, int(overlap // 2))
            b1.y += shift
            b2.y -= shift
        if not b1.dead:
            b1.speed = max(0.0, b1.speed - 10.0)
        if not b2.dead:
            b2.speed = max(0.0, b2.speed - 10.0)

class Game:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Intelligent Bus Race: Hybrid AI (BFS/DFS/Minimax/Fuzzy)")
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("arial", 24)
        self.bigfont = pygame.font.SysFont("arial", 64)
        self.small_font = pygame.font.SysFont("arial", 18)
        self.running = True
        self.paused = False
        self.reset()

    def reset(self):
        self.game_time, self.time = 0, 0
        self.buses = [
            Bus(lane_idx=0, color=(30,144,255), name="Bus A", agent_cls=HybridAIBus),
            Bus(lane_idx=LANES-1, color=(200,60,60), name="Bus B", agent_cls=HybridAIBus)
        ]
        for b in self.buses:
            b._label_font = self.small_font
        self.obstacles, self.peds = [], []
        self.last_obs, self.last_ped = 0, 0
        self.winner = None

    def spawn_obstacle(self):
        lane = random.randrange(LANES)
        self.obstacles.append(Obstacle(lane_idx=lane, y=-80))
    def spawn_pedestrian(self):
        lane = random.randrange(LANES)
        self.peds.append(Pedestrian(lane_idx=lane, y=-40))
    def draw_road(self):
        pygame.draw.rect(self.screen, (50,50,50), (ROAD_X,0,ROAD_W,SCREEN_H))
        for i in range(1, LANES):
            lx = ROAD_X + i * LANE_W
            for y in range(0, SCREEN_H, 40):
                pygame.draw.line(self.screen, (255,255,100), (lx, y), (lx, y + 20), 2)

    def update(self, dt):
        self.time += dt
        self.game_time += dt
        if self.time - self.last_obs > OBSTACLE_SPAWN_INTERVAL:
            self.spawn_obstacle(); self.last_obs = self.time
        if self.time - self.last_ped > PEDESTRIAN_SPAWN_INTERVAL:
            self.spawn_pedestrian(); self.last_ped = self.time
        world = {"obstacles": self.obstacles, "pedestrians": self.peds}
        for i in range(2):
            other_bus = self.buses[1-i]
            self.buses[i].update(dt, world, other_bus)
        alive_buses = [b for b in self.buses if not b.dead and not b.off_track]
        avg_speed = sum(b.speed for b in alive_buses)/len(alive_buses) if alive_buses else 0
        scroll_speed = avg_speed * KMH_TO_MPS * 20 * dt
        for o in self.obstacles: o.update(scroll_speed, 1)
        for p in self.peds: p.update(dt, scroll_speed)
        obstacles_to_remove = set()
        peds_to_remove = set()
        for b in self.buses:
            for o in self.obstacles:
                if b.rect().colliderect(pygame.Rect(int(o.x), int(o.y), o.w, o.h)):
                    b.crash(obstacle=True)
                    obstacles_to_remove.add(o)
            for p in self.peds:
                if b.rect().colliderect(pygame.Rect(int(p.x), int(p.y), p.w, p.h)):
                    b.crash(pedestrian=True)
                    peds_to_remove.add(p)
        self.obstacles = [o for o in self.obstacles if o not in obstacles_to_remove and o.y < SCREEN_H + 100]
        self.peds = [p for p in self.peds if p not in peds_to_remove and p.y < SCREEN_H + 100]
        if not any(b.dead or b.off_track for b in self.buses):
            clean_bus_collision(self.buses[0], self.buses[1])
        for b in self.buses:
            if b.progress_m >= FINISH_DIST:
                self.winner = b
        if not self.winner:
            if all(b.dead or b.off_track for b in self.buses):
                best = max(self.buses, key=lambda b: b.score)
                self.winner = best
        if not self.winner:
            pass

    def draw(self):
        self.screen.fill((32,128,76))
        self.draw_road()
        for o in self.obstacles: o.draw(self.screen)
        for p in self.peds: p.draw(self.screen)
        for b in self.buses: b.draw(self.screen)

        hud_panel = pygame.Surface((SCREEN_W, 80))
        hud_panel.set_alpha(220)
        hud_panel.fill((40,57,75))
        self.screen.blit(hud_panel, (0,0))

        font_big = pygame.font.SysFont("arial", 28)
        font_small = pygame.font.SysFont("arial", 20)
        for i, b in enumerate(self.buses):
            x_pos = 20 + i * 500
            name_lbl = font_big.render(b.name, True, b.color)
            self.screen.blit(name_lbl, (x_pos, 8))
            life_lbl = font_small.render(f"Life: {int(b.life)}%", True, (255,200,0))
            self.screen.blit(life_lbl, (x_pos, 38))
            score_lbl = font_small.render(f"Score: {b.score}", True, (0,255,0))
            self.screen.blit(score_lbl, (x_pos, 58))

        font_timer = pygame.font.SysFont("arial", 22)
        time_lbl = font_timer.render(f"Time: {int(self.game_time)}s", True, (255,255,0))
        self.screen.blit(time_lbl, (SCREEN_W-140,12))

        if self.paused:
            paused_surf = pygame.Surface((SCREEN_W, SCREEN_H))
            paused_surf.fill((0, 0, 0))
            paused_surf.set_alpha(128)
            self.screen.blit(paused_surf, (0, 0))
            paused_lbl = self.bigfont.render("PAUSED", True, (255, 255, 255))
            self.screen.blit(paused_lbl, (SCREEN_W//2 - paused_lbl.get_width()//2, SCREEN_H//2 - 100))

        if self.winner:
            winner_surf = pygame.Surface((SCREEN_W, SCREEN_H))
            winner_surf.fill((0, 0, 0))
            winner_surf.set_alpha(200)
            self.screen.blit(winner_surf, (0, 0))
            winner_lbl = self.bigfont.render(f"{self.winner.name} Wins!", True, self.winner.color)
            self.screen.blit(winner_lbl, (SCREEN_W//2 - winner_lbl.get_width()//2, SCREEN_H//2 - 100))
            score_lbl = font_big.render(f"Final Score: {self.winner.score}", True, (255, 255, 255))
            self.screen.blit(score_lbl, (SCREEN_W//2 - score_lbl.get_width()//2, SCREEN_H//2 + 20))
            restart_lbl = font_small.render("Press R to Restart or Q to Quit", True, (200, 200, 200))
            self.screen.blit(restart_lbl, (SCREEN_W//2 - restart_lbl.get_width()//2, SCREEN_H//2 + 80))

        pygame.display.flip()

    def start_screen(self):
        self.screen.fill((25, 30, 56))
        msg = self.bigfont.render("Intelligent Bus Race: Hybrid AI", True, (255,220,40))
        start = self.font.render("Press ENTER to Start", True, (255,255,255))
        rules_font = pygame.font.SysFont("arial", 22)
        rules = [
            "Hybrid AI: BFS/DFS for paths, Minimax/Alpha-Beta for opponent, Fuzzy logic for edges.",
            "Obstacles = -10% life, Pedestrians = -25%, Bus collision = -30%.",
            "Press 'P' to pause, 'S' to resume, 'R' to reset, 'Q' to quit."
        ]
        self.screen.blit(msg, (SCREEN_W//2 - msg.get_width()//2, SCREEN_H//3))
        self.screen.blit(start, (SCREEN_W//2 - start.get_width()//2, SCREEN_H//3 + 80))
        for i, rule in enumerate(rules):
            rule_lbl = rules_font.render(rule, True, (200, 220, 255))
            self.screen.blit(rule_lbl, (20, SCREEN_H//2 + i * 30))
        pygame.display.flip()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        return

    def run(self):
        self.start_screen()
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.reset()
                    if event.key == pygame.K_q:
                        self.running = False
                    if event.key == pygame.K_p:
                        self.paused = True
                    if event.key == pygame.K_s:
                        self.paused = False
            if not self.paused:
                self.update(dt)
            self.draw()
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    Game().run()
