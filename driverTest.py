import numpy as np

from AgenTrajectories import *


p0 = np.zeros(3)
v0 = np.zeros(3)
pf = np.array([2.01611612278146, 1.27868971359693, -0.577831661211734])
vf = np.zeros(3)
t0 = 0
tf = 10

cx, cy, cz = AgentTrajectories(p0, pf, v0, vf, t0, tf)


