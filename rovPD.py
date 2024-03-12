import holoocean
import numpy as np
import matplotlib.pyplot as plt
import modules.holoOceanUtils

#numberOfROVS=8
#auv=[]

#auv_matrix = [[column for column in range(numberOfROVS)] for row in range(numberOfROVS)]



command=[-2.0,0.0,-2.5,
         0.0,0.0,270.0]
scenario=modules.holoOceanUtils.scenario("ImagingSonarTank2","64tankMap1","simulationWorld2",200)
rov=modules.holoOceanUtils.ROV(0,1,[-2,0,-2.5],[0,0,0])
rov.addSensor("LocationSensor","Origin")
rov.addSensor("RotationSensor","Origin")
rov.addSonarImaging()
scenario.addAgent(rov.agent)

fineshed=0



#start Simulation
env=holoocean.make(scenario_cfg=scenario.cfg)

env.reset
env.move_viewport([1.23,-4.56,7.89],[0,0,180])
env.set_render_quality(3)
#env.move_viewport([26.25,-26.25,50],[0,0,180])
state = env.tick()
rov_pose=[]

while  not rov.reachedWaypoint(command,state[rov.name]["LocationSensor"],state[rov.name]["RotationSensor"],0.1):
        
        state = env.tick()
        env.act('auv0', command)
        state = env.tick()
        #print(state[rov.name]["LocationSensor"],state[rov.name]["RotationSensor"],end="\r")

print("Finished Simulation!")
#plt.ioff()
#plt.show()