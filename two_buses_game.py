import pygame
import random
import sys
from dataclasses import dataclass

# CONFIG & CONSTANTS
SCREEN_W, SCREEN_H = 1000, 700
FPS = 60
LANES = 5
ROAD_W = 700
LANE_W = ROAD_W // LANES
ROAD_X = (SCREEN_W - ROAD_W) // 2
KMH_TO_MPS = 1000.0 / 3600.0
BUS_W = int(LANE_W * 0.8)
BUS_H = 60
OBSTACLE_SPAWN_INTERVAL = 2.0
PEDESTRIAN_SPAWN_INTERVAL = 3.0
FINISH_DIST = 2000.0

@dataclass
class RectObj:
    x: float
    y: float
    w: int
    h: int
    def rect(self):
        return pygame.Rect(int(self.x), int(self.y), self.w, self.h)

class Bus:
    def __init__(self, lane_idx, color, name, agent_cls):
        self.x = ROAD_X + lane_idx * LANE_W + (LANE_W - BUS_W) / 2
        self.y = SCREEN_H - 180
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
        self.life = 100.0      # Life (%) from 100 to 0
        self.score = 0
        self.dead = False
        self.off_track = False
        self.agent = agent_cls(self)
        self.time_alive = 0.0

    def update(self, dt, world, opponent):
        if self.dead or self.off_track:
            self.speed = 0
            return
        self.time_alive += dt
        if self.time_alive >= 1.0:
            self.score += 10
            self.time_alive = 0
        controls = self.agent.decide(world, opponent)
        if controls.get("accel"): self.speed += self.accel * dt
        elif controls.get("brake"): self.speed -= self.brake_power * dt
        else: self.speed -= 5.0 * dt
        if controls.get("left"): self.x -= self.lat_speed
        if controls.get("right"): self.x += self.lat_speed
        if controls.get("forward_adjust"): self.y -= 2
        if controls.get("backward_adjust"): self.y += 2
        left_bound = ROAD_X + 6
        right_bound = ROAD_X + ROAD_W - self.w - 6
        if self.x < left_bound or self.x > right_bound: self.off_track = True
        self.x = max(left_bound, min(self.x, right_bound))
        self.y = max(100, min(self.y, SCREEN_H - 100))
        self.speed = max(0, min(self.speed, self.max_speed))
        self.progress_m += self.speed * KMH_TO_MPS * dt
        if self.life <= 0:
            self.dead = True
            self.speed = 0
            self.life = 0

    def crash(self, obstacle=False, pedestrian=False, bus=False):
        if self.dead:
            return
        if obstacle:
            self.life -= 20
            self.score = max(0, self.score - 20)
        if pedestrian:
            self.life -= 40
            self.score = max(0, self.score - 40)
        if bus:
            self.life -= 30
            self.score = max(0, self.score - 30)
        if self.life <= 0:
            self.dead = True
            self.speed = 0
            self.life = 0

    def lane_index(self):
        return int((self.x - ROAD_X) // LANE_W)

    def draw(self, surf):
        rect = pygame.Rect(int(self.x), int(self.y), self.w, self.h)
        color = (100, 100, 100) if self.dead or self.off_track else self.color
        pygame.draw.rect(surf, color, rect, border_radius=6)
        pygame.draw.rect(surf, (220, 240, 255),
                        (rect.x + 8, rect.y + 8, rect.w - 16, int(rect.h / 2)),
                        border_radius=3)
        font = pygame.font.SysFont("arial", 18)
        lbl = font.render(self.name, True, (255,255,255))
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
        self.y += scroll_speed * dt * 0.45   # SLOWER OBSTACLE SPEED

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

class GoalBasedBusAI:
    def __init__(self, bus):
        self.bus = bus
        self.target_speed = 100.0
    def decide(self, world, opponent):
        controls = dict(left=False, right=False, accel=False, brake=False,forward_adjust=False, backward_adjust=False)
        current_lane = self.bus.lane_index()
        for obj in world["obstacles"] + world["pedestrians"]:
            olane = int((obj.x - ROAD_X) // LANE_W)
            if olane == current_lane and 0 < self.bus.y - obj.y < 500:
                left_lane, right_lane = current_lane - 1, current_lane + 1
                for l in [left_lane, right_lane]:
                    if 0 <= l < LANES and self.lane_free(l, world, opponent):
                        if l < current_lane: controls["left"] = True
                        else: controls["right"] = True
                        break
                else: controls["brake"] = True
        if self.bus.speed < self.target_speed: controls["accel"] = True
        return controls
    def lane_free(self, lane, world, opponent):
        for obj in world["obstacles"] + world["pedestrians"]:
            olane = int((obj.x - ROAD_X) // LANE_W)
            if olane == lane and abs(self.bus.y - obj.y) < 150: return False
        if opponent and not opponent.dead and opponent.lane_index() == lane and abs(self.bus.y - opponent.y) < 150: return False
        return True

class AdversarialBusAI:
    def __init__(self, bus):
        self.bus = bus
        self.target_speed = 100.0
    def decide(self, world, opponent):
        controls = dict(left=False, right=False, accel=False, brake=False,forward_adjust=False, backward_adjust=False)
        current_lane = self.bus.lane_index()
        if opponent and not opponent.dead:
            if abs(self.bus.y - opponent.y) < 80 and abs(self.bus.x - opponent.x) < 60:
                if current_lane < LANES - 1: controls["right"] = True
                elif current_lane > 0: controls["left"] = True
            elif opponent.lane_index() < current_lane and self.lane_free(current_lane-1, world, opponent):
                controls["left"] = True
            elif opponent.lane_index() > current_lane and self.lane_free(current_lane+1, world, opponent):
                controls["right"] = True
        for obj in world["obstacles"] + world["pedestrians"]:
            olane = int((obj.x - ROAD_X) // LANE_W)
            if olane == current_lane and 0 < self.bus.y - obj.y < 500:
                left_lane, right_lane = current_lane - 1, current_lane + 1
                for l in [left_lane, right_lane]:
                    if 0 <= l < LANES and self.lane_free(l, world, opponent):
                        if l < current_lane: controls["left"] = True
                        else: controls["right"] = True
                        break
                else: controls["brake"] = True
        if self.bus.speed < self.target_speed: controls["accel"] = True
        return controls
    def lane_free(self, lane, world, opponent):
        for obj in world["obstacles"] + world["pedestrians"]:
            olane = int((obj.x - ROAD_X) // LANE_W)
            if olane == lane and abs(self.bus.y - obj.y) < 150: return False
        if opponent and not opponent.dead and opponent.lane_index() == lane and abs(self.bus.y - opponent.y) < 150: return False
        return True

class Game:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Intelligent Bus Race: AI vs AI")
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("arial", 24)
        self.bigfont = pygame.font.SysFont("arial", 64)
        self.running = True
        self.reset()

    def reset(self):
        self.game_time, self.time = 0, 0
        self.buses = [
            Bus(lane_idx=0, color=(30,144,255), name="Bus A (Goal Agent)", agent_cls=GoalBasedBusAI),
            Bus(lane_idx=LANES-1, color=(200,60,60), name="Bus B (Adv Agent)", agent_cls=AdversarialBusAI)
        ]
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
        pygame.draw.rect(self.screen, (50, 50, 50), (ROAD_X, 0, ROAD_W, SCREEN_H))
        for i in range(1, LANES):
            lx = ROAD_X + i * LANE_W
            for y in range(0, SCREEN_H, 40):
                pygame.draw.rect(self.screen, (230, 230, 230), (lx - 2, y + 10, 4, 20))

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
        scroll_speed = avg_speed * KMH_TO_MPS * 18 * dt  # MUCH SLOWER OBSTACLE SPEED

        for o in self.obstacles: o.update(scroll_speed, 1)
        for p in self.peds: p.update(dt, scroll_speed)
        for b in self.buses:
            if b.dead or b.off_track: continue
            for o in self.obstacles:
                if b.rect().colliderect(pygame.Rect(o.x, o.y, o.w, o.h)):
                    b.crash(obstacle=True)
            for p in self.peds:
                if b.rect().colliderect(pygame.Rect(p.x, p.y, p.w, p.h)):
                    b.crash(pedestrian=True)
        if not self.buses[0].dead and not self.buses[0].off_track and not self.buses[1].dead and not self.buses[1].off_track:
            if self.buses[0].rect().colliderect(self.buses[1].rect()):
                self.buses[0].crash(bus=True)
                self.buses[1].crash(bus=True)
        for b in self.buses:
            if b.progress_m >= FINISH_DIST and not self.winner and not b.dead and not b.off_track:
                self.winner = b.name
        if not self.winner:
            pushed = [b for b in self.buses if b.off_track]
            if len(pushed) == 1:
                self.winner = self.buses[1 - self.buses.index(pushed[0])].name
        if not self.winner:
            alive = [b for b in self.buses if not b.dead and not b.off_track]
            if len(alive) == 1: self.winner = alive[0].name
            elif len(alive) == 0: self.winner = "Nobody"

    def draw(self):
        self.screen.fill((32,128,76))
        self.draw_road()
        for o in self.obstacles: o.draw(self.screen)
        for p in self.peds: p.draw(self.screen)
        for b in self.buses: b.draw(self.screen)
        hud_panel = pygame.Surface((SCREEN_W, 80)); hud_panel.set_alpha(220)
        hud_panel.fill((40,57,75)); self.screen.blit(hud_panel, (0,0))
        font_big = pygame.font.SysFont("arial", 28)
        font_small = pygame.font.SysFont("arial", 20)
        for i, b in enumerate(self.buses):
            panel_x = 60 + i * 480
            rect = pygame.Rect(panel_x, 10, 400, 60)
            pygame.draw.rect(self.screen, b.color, rect, 0, border_radius=14)
            pygame.draw.rect(self.screen, (200,200,230), rect, 2, border_radius=14)
            lbl1 = font_big.render(f"{b.name}", True, (255,255,255))
            lbl2 = font_small.render(f"Dist Left: {max(0, int(FINISH_DIST - b.progress_m))}m", True, (255,255,255))
            lbl3 = font_small.render(f"Speed: {int(b.speed)} km/h  Score: {b.score}  Life: {int(b.life)}%", True, (255,255,255))
            self.screen.blit(lbl1, (rect.x + 12, rect.y + 4))
            self.screen.blit(lbl2, (rect.x + 12, rect.y + 32))
            self.screen.blit(lbl3, (rect.x + 180, rect.y + 32))
            prog_pct = min(1.0, b.progress_m / FINISH_DIST)
            px = rect.x + 12
            py = rect.y + 56
            bar_w = 370
            bar_h = 10
            pygame.draw.rect(self.screen, (70,80,80), (px, py, bar_w, bar_h), 0, border_radius=5)
            pygame.draw.rect(self.screen, b.color, (px, py, int(bar_w*prog_pct), bar_h), 0, border_radius=5)
            pygame.draw.rect(self.screen, (220,220,220), (px, py, bar_w, bar_h), 2, border_radius=5)
        font_timer = pygame.font.SysFont("arial", 22)
        time_lbl = font_timer.render(f"Time: {int(self.game_time)}s", True, (255,255,0))
        self.screen.blit(time_lbl, (SCREEN_W-140,12))
        if self.winner:
            overlay = pygame.Surface((SCREEN_W, SCREEN_H)); overlay.set_alpha(180)
            overlay.fill((0, 0, 0)); self.screen.blit(overlay, (0, 0))
            msg = self.bigfont.render(f"{self.winner} Wins!", True, (255,220,40))
            rect_msg = msg.get_rect(center=(SCREEN_W//2, SCREEN_H//2 - 40))
            self.screen.blit(msg, rect_msg)
            score_panel = pygame.Surface((520, 110)); score_panel.set_alpha(230)
            score_panel.fill((35, 36, 49)); self.screen.blit(score_panel, (SCREEN_W//2-260, SCREEN_H//2+10))
            for i, b in enumerate(self.buses):
                ps = font_big.render(f"{b.name}: Score {b.score}, Life {int(b.life)}%, Distance {int(b.progress_m)}m", True, b.color)
                self.screen.blit(ps, (SCREEN_W//2-240, SCREEN_H//2+20 + 38*i))
        pygame.display.flip()

    def start_screen(self):
        self.screen.fill((25, 30, 56))
        msg = self.bigfont.render("Intelligent Bus Race", True, (255,220,40))
        start = self.font.render("Press ENTER to Start", True, (255,255,255))
        rules_font = pygame.font.SysFont("arial", 22)
        rules = [
            "Goal: Reach the 2000m finish or push your opponent off road.",
            "Red Bus: Adversarial AI. Blue Bus: Goal-based AI.",
            "Obstacles = -20% life & points, Pedestrians = -40%, Bus collision = -30%",
            "Life bar is shown as percent (100% to 0%)",
            "5 lanes for advanced pathfinding.",
            "Press 'R' anytime to reset the game."
        ]
        self.screen.blit(msg, (SCREEN_W//2 - msg.get_width()//2, SCREEN_H//3))
        self.screen.blit(start, (SCREEN_W//2 - start.get_width()//2, SCREEN_H//3 + 80))
        for i, rule in enumerate(rules):
            rl = rules_font.render(rule, True, (230,230,230))
            self.screen.blit(rl, (SCREEN_W//2 - rl.get_width()//2, SCREEN_H//3 + 130 + 32*i))
        pygame.display.flip()
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT: pygame.quit(); sys.exit()
                if e.type == pygame.KEYDOWN and e.key == pygame.K_RETURN: return

    def run(self):
        self.start_screen()
        while self.running:
            dt = self.clock.tick(FPS)/1000.0
            for e in pygame.event.get():
                if e.type == pygame.QUIT: self.running = False
                if e.type == pygame.KEYDOWN:
                    if e.key in [pygame.K_ESCAPE, pygame.K_q]: self.running = False
                    if e.key == pygame.K_r: self.reset()
            if not self.winner: self.update(dt)
            self.draw()
        pygame.quit(); sys.exit()

if __name__ == "__main__":
    Game().run()
