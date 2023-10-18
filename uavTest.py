import holoocean
import numpy as np
#from pynput import keyboard
'''
env=holoocean.make("waterTank")
command = np.array([0,0,0,0,0,0,0,0])
while True:
    state = env.step(command)
'''

env = holoocean.make("waterTankUAV")

for _ in range(10000):
    state = env.step(([0,0,0,35],[0,0,0,0]))

#for _ in range(200):
    #state = env.step(([0,0,0,100],[0,0,0,0]))
