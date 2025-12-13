import math
import random
from typing import List, Optional

import numpy as np
from OpenGL.GL import (
    glBegin,
    glBlendFunc,
    glColor3f,
    glColor4f,
    glDisable,
    glEnable,
    glEnd,
    glPointSize,
    glPopMatrix,
    glPushMatrix,
    glRotatef,
    glTranslatef,
    glVertex3f,
    GL_BLEND,
    GL_DEPTH_TEST,
    GL_ONE,
    GL_SRC_ALPHA,
    GL_POINTS,
    GL_TRIANGLE_STRIP,
    GL_TRIANGLES,
)

from sim.config import clamp, wrap_deg, WORLD_HALF
from sim.world.dem import DEM


class MovingTarget:
    def __init__(self, x=300.0, y=0.0, vmin=6.0, vmax=16.0, tau=1.2, sigma=2.0, world_half: float = WORLD_HALF):
        self.x, self.y, self.z = float(x), float(y), 0.0
        self.heading = random.uniform(0, 360.0)
        self.heading_rate = 0.0
        self.v = random.uniform(vmin, vmax)
        self.vmin, self.vmax = vmin, vmax
        self.tau, self.sigma = tau, sigma
        self.world_half = world_half
        self.v_tau, self.v_sigma = 4.0, 1.0
        self.color = (1.0, 0.4, 0.4)
        self.alive = True

    def step(self, dt: float, dem: Optional[DEM] = None):
        if not self.alive:
            return
        self.heading_rate += (-self.heading_rate / self.tau + self.sigma * random.gauss(0, 1)) * dt
        self.heading_rate = clamp(self.heading_rate, -90.0, 90.0)
        self.heading = wrap_deg(self.heading + self.heading_rate * dt)
        dv = (-(self.v - 0.5 * (self.vmin + self.vmax)) / self.v_tau + self.v_sigma * random.gauss(0, 1)) * dt
        self.v = clamp(self.v + dv, self.vmin, self.vmax)
        margin = 0.15 * self.world_half
        if abs(self.x) > self.world_half - margin or abs(self.y) > self.world_half - margin:
            desired = math.degrees(math.atan2(-self.y, -self.x))
            err = ((desired - self.heading + 540) % 360) - 180
            self.heading_rate += 35.0 * clamp(err / 90.0, -1.0, 1.0) * dt
        rad = math.radians(self.heading)
        self.x = clamp(self.x + math.cos(rad) * self.v * dt, -self.world_half, self.world_half)
        self.y = clamp(self.y + math.sin(rad) * self.v * dt, -self.world_half, self.world_half)
        self.z = dem.get_height(self.x, self.y) if dem is not None else 0.0

    def draw(self):
        glPushMatrix()
        glTranslatef(self.x, self.y, self.z)
        glRotatef(self.heading, 0, 0, 1)
        glDisable(GL_DEPTH_TEST)
        glColor3f(*self.color)
        glBegin(GL_TRIANGLES)
        glVertex3f(12, 0, 0.1)
        glVertex3f(-10, -6, 0.1)
        glVertex3f(-10, 6, 0.1)
        glEnd()
        glEnable(GL_DEPTH_TEST)
        glPopMatrix()


class Spark:
    def __init__(self, origin_xy, base_z):
        ang = random.uniform(0, 2 * math.pi)
        speed = random.uniform(80.0, 180.0)
        self.vx = math.cos(ang) * speed
        self.vy = math.sin(ang) * speed
        self.x, self.y = origin_xy
        self.z = base_z + random.uniform(0.0, 4.0)
        self.life = random.uniform(0.35, 0.6)
        self.age = 0.0

    def step(self, dt):
        self.age += dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z -= 25.0 * dt

    def alive(self):
        return self.age < self.life

    def color(self):
        t = self.age / self.life
        return (1.0, 0.5 * (1.0 - t) + 0.1 * t, 0.0, 1.0 - 0.8 * t)


class Missile:
    def __init__(self, x, y, z, target: MovingTarget, speed=350.0, blast_radius=12.0, max_life=15.0):
        self.pos = np.array([float(x), float(y), float(z)], dtype=float)
        self.target = target
        self.speed = speed
        self.blast_radius = blast_radius
        self.max_life = max_life
        self.life = 0.0
        self.active = True
        self.exploded = False
        self.explode_time = 0.0
        self.explode_visual_radius = 0.0
        self.sparks: List[Spark] = []

    def step(self, dt):
        if not self.active:
            if self.exploded:
                self.explode_time += dt
                self.explode_visual_radius += 120.0 * dt
                for s in list(self.sparks):
                    s.step(dt)
                    if not s.alive():
                        self.sparks.remove(s)
            return

        self.life += dt
        if self.life > self.max_life:
            self.active = False
            return

        tgt_pos = np.array([self.target.x, self.target.y, self.target.z], dtype=float)
        dir_vec = tgt_pos - self.pos
        dist = np.linalg.norm(dir_vec)
        if dist < 1e-6:
            self._explode()
            return

        dir_n = dir_vec / dist
        step_move = dir_n * self.speed * dt
        if np.linalg.norm(step_move) >= dist:
            self.pos = tgt_pos
            self._explode()
        else:
            self.pos += step_move

        if np.linalg.norm(np.array([self.target.x, self.target.y, self.target.z]) - self.pos) <= self.blast_radius:
            self._explode()

    def _explode(self):
        if self.exploded:
            return
        self.exploded = True
        self.active = False
        self.target.v = 0.0
        self.target.alive = False
        self.target.color = (0.0, 0.0, 0.0)
        self.explode_time = 0.0
        self.explode_visual_radius = 4.0
        base_z = self.target.z
        origin = (self.pos[0], self.pos[1])
        for _ in range(40):
            self.sparks.append(Spark(origin, base_z))

    def draw(self, dem: Optional[DEM] = None):
        if self.active and not self.exploded:
            glPushMatrix()
            glTranslatef(self.pos[0], self.pos[1], self.pos[2])
            glDisable(GL_DEPTH_TEST)
            glColor3f(1.0, 0.9, 0.2)
            glBegin(GL_TRIANGLES)
            glVertex3f(0, 0, 0.1)
            glVertex3f(2.5, 0, 0.1)
            glVertex3f(0, 2.5, 0.1)
            glEnd()
            glEnable(GL_DEPTH_TEST)
            glPopMatrix()
        elif self.exploded:
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE)
            r = self.explode_visual_radius
            base_z = dem.get_height(self.pos[0], self.pos[1]) if dem is not None else 0.0
            t = min(1.0, self.explode_time / 0.9)
            col = (1.0, 0.45 * (1.0 - t) + 0.1 * t, 0.0, 0.9 * (1.0 - t))
            glColor4f(*col)
            glBegin(GL_TRIANGLE_STRIP)
            for a in range(0, 361, 8):
                rad = math.radians(a)
                glVertex3f(self.pos[0] + (r - 2.0) * math.cos(rad), self.pos[1] + (r - 2.0) * math.sin(rad), base_z)
                glVertex3f(self.pos[0] + (r + 2.0) * math.cos(rad), self.pos[1] + (r + 2.0) * math.sin(rad), base_z)
            glEnd()
            glPointSize(3.0)
            glBegin(GL_POINTS)
            for s in self.sparks:
                r_col, g_col, b_col, a_col = s.color()
                glColor4f(r_col, g_col, b_col, a_col)
                glVertex3f(s.x, s.y, s.z)
            glEnd()
            glDisable(GL_BLEND)
            glEnable(GL_DEPTH_TEST)
