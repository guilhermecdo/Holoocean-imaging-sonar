import holoocean
import numpy as np

env = holoocean.make("OpenWater-HoveringCamera")

# The hovering AUV takes a command for each thruster
command = np.array([0,0,0,0,0,0,0,0])

while True:
   state = env.step(command)
