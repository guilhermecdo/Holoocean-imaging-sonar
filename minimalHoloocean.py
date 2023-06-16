import holoocean
import numpy as np


env = holoocean.make("PierHarbor-Hovering")

command=np.array([10,10,0,0,0,0,10,10])

for _ in range(180000000):
    state = env.step(command)
