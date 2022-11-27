import pygame
import sys
import numpy as np
from numpy import sin, cos, pi
from numpy.linalg import inv
from pygame.locals import *
from spring import spring


#Solver functions (Runge_Kutta)

def G(y,t):
    x_d, theta_d, x, theta = y[0], y[1], y[2], y[3]
    
    m11, m12 = (cart_mass+pendulum_mass), -pendulum_mass*l*sin(theta)
    m21, m22 = -sin(theta), l
    m = np.array([[m11,m12],[m21,m22]])
    
    f1 = -spring_constant*x+pendulum_mass*l*cos(theta)*theta_d**2
    f2 = -g*sin(theta)
    f = np.array([f1,f2])

    accel = inv(m).dot(f)
    return np.array([accel[0],accel[1],x_d,theta_d])

def RK4_step(y, t, dt):
    k1 = G(y, t)
    k2 = G(y+0.5*k1*dt, t+0.5*dt)
    k3 = G(y+0.5*k2*dt, t+0.5*dt)
    k4 = G(y+k3*dt, t+dt)

    return dt * (k1 + 2*k2+ 2*k3 + k4)/6

def angle2pos(theta):
    pendulum_position = np.zeros(2)
    pendulum_position[0] = int(l*sin(theta))
    pendulum_position[1] = int(l*cos(theta))

    return pendulum_position

#Render functions and classes

def render():
    screen.fill(WHITE)
    render_statics()
    mass_spring.render()
    mass.render()
    pend.render()
    
def update(point2, point3):
    mass.update(point2)
    pend.update(point2, point3)
    mass_spring.update(point1, point2)

def render_statics():
    pygame.draw.line(screen, BLACK, (250, point1[1]), (1300, point1[1]), 7)
    pygame.draw.circle(screen, BLACK, point1, 10)

class Mass():
    def __init__(self, position, color, width, height):
        self.pos = position
        self.color = color
        self.w = width
        self.h = height
        self.left = self.pos[0] - self.w/2
        self.top = self.pos[1] - self.h/2
    def render(self):
        pygame.draw.rect(screen, self.color, (self.left, self.top, self.w, self.h))
    def update(self, position):
        self.pos = position
        self.left = self.pos[0] - self.w/2
        self.top = self.pos[1] - self.h/2

class Pendulum():
    def __init__(self, position_attachment, position_pendulum, color, radius):
        self.pos = position_pendulum
        self.att = position_attachment
        self.color = color
        self.rad = radius
        self.left = self.pos[0] 
        self.top = self.pos[1] 
    def render(self):
        pygame.draw.circle(screen, BLACK, self.att, 5)
        pygame.draw.line(screen, BLACK, self.att, self.pos, 5)
        pygame.draw.circle(screen, self.color, (self.left, self.top), self.rad)
    def update(self, position_attachment, position_pendulum):
        self.pos = position_pendulum
        self.left = self.pos[0] 
        self.top = self.pos[1]
        self.att = position_attachment

class Spring():
    def __init__(self, color, start, end, nodes, width, lead1, lead2):
        self.start = start
        self.end = end
        self.nodes = nodes
        self.width = width
        self.lead1 = lead1
        self.lead2 = lead2
        self.weight = 3
        self.color = color
    def update(self, start, end):
        self.start = start
        self.end = end
        self.x, self.y, self.p1, self.p2 = spring(self.start, self.end, self.nodes, self.width, self.lead1, self.lead2)
        self.p1 = (int(self.p1[0]), int(self.p1[1]))
        self.p2 = (int(self.p2[0]), int(self.p2[1]))
    def render(self):
        pygame.draw.line(screen, self.color, self.start, self.p1, self.weight)
        prev_point = self.p1
        for point in zip(self.x, self.y):
            pygame.draw.line(screen,self.color,prev_point,point,self.weight)
            prev_point = point
        pygame.draw.line(screen, self.color, self.p2, self.end, self.weight)
            
#pygame and objects setup
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (150, 150, 150)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
cart_spring_length = 250
scale = 1
pendulum_length = 400 * scale

screen = pygame.display.set_mode((0,0), pygame.FULLSCREEN)
screen.fill(WHITE)
trace = screen.copy()
clock = pygame.time.Clock()
pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 38)

point1 = (800, 45)
mass = Mass((800,300), RED, 160, 120)
pend = Pendulum((400,150),(800,300), BLUE, 60)
mass_spring = Spring(BLACK, (0,0), (0,0), 20, 50, 50, 90)

#parameters
cart_mass = 2.0
pendulum_mass = 4.0
spring_constant = 3.0
g = 9.81                    #gravitational force
l = pendulum_length         #pendulum length
t = 0                       #initial time
delta_t = 0.1               #time step
y = np.array([0,0,2,15])     #initial conditions [cart speed, pendulum angular speed, cart position, pendulum angle]

#main loop

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    pendulum_position = angle2pos(y[3])
    point2 = point1[0]  , point1[1] + cart_spring_length + y[2]
    point3 = point1[0]+pendulum_position[0]  , point1[1] + cart_spring_length + pendulum_position[1] + y[2]

    update(point2,point3)
    render()

    t += delta_t
    y = y + RK4_step(y, t, delta_t)

    time_string = 'Time: {} seconds'.format(round(t,1))
    text = myfont.render(time_string, False, (0,0,0))
    screen.blit(text,(10,55))
    clock.tick(60)
    pygame.display.update()
