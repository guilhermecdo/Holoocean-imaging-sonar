import holoocean
import numpy as np
import matplotlib.pyplot as plt
import modules.holoOceanUtils

numberOfROVS=1
#auv=[]

auv_matrix = [[column for column in range(numberOfROVS)] for row in range(numberOfROVS)]

waypoints=[[-2,0,-2,0,0,0],[-2,0,-4,0,0,0],
           [0,-2,-2,0,0,90],[0,-2,-4,0,0,90],
           [2,0,-2,0,0,180],[2,0,-4,0,0,180],
           [0,2,-2,0,0,270],[0,2,-4,0,0,270]]

command=[]
scenario=modules.holoOceanUtils.scenario("ImagingSonarTank2","64tankMap1","simulationWorld2",200)

for j in range(numberOfROVS):
    for i in range(numberOfROVS):
        auv_matrix[j][i]=(modules.holoOceanUtils.ROV(str(j)+str(i),1,[-2+(i*7.5),(j*-7.5),-2],[0,0,0]))
        auv_matrix[j][i].addSonarImaging()
        auv_matrix[j][i].addSensor("LocationSensor","Origin")
        auv_matrix[j][i].addSensor("RotationSensor","Origin")
        auv_matrix[j][i].imageViwer()
        #command.append([-2.5,-2.5,-2.5,-2.5,0,0,0,0])
        scenario.addAgent(auv_matrix[j][i].agent)

fineshed=0

#start Simulation
env=holoocean.make(scenario_cfg=scenario.cfg)
env.reset
env.set_render_quality(3)
env.move_viewport([26.25,-26.25,50],[0,0,180])
state = env.tick()

    
while fineshed<numberOfROVS:
    for points in waypoints:       
        state = env.tick()
        while np.linalg.norm(np.array(state[auv_matrix[j][i].name]["LocationSensor"])-np.array(points[0:3]))>=0.1:
            print("ROV Location:",state[auv_matrix[j][i].name]["LocationSensor"],"ROV Waipont:",points[0:3],end="\r")
            for j in range(numberOfROVS):
                for i in range(numberOfROVS):

                    if 'ImagingSonar' in state[auv_matrix[j][i].name]:
                        auv_matrix[j][i].updateImnage(state[auv_matrix[j][i].name]['ImagingSonar'])

                #if state[auv_matrix[j][i].name]["LocationSensor"][2]>-3:
                    env.act(auv_matrix[j][i].name, points)
                #else:
                    #env.act(auv_matrix[j][i].name, [0,0,0,0,0,0,0,0])
    
            state = env.tick()
        fineshed+=1

print("Finished Simulation!")
#plt.ioff()
#plt.show()