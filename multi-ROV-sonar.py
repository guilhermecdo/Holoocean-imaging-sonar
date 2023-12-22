import holoocean
import numpy as np
import matplotlib.pyplot as plt
import modules.holoOceanUtils

numberOfROVS=8
#auv=[]

auv_matrix = [[column for column in range(numberOfROVS)] for row in range(numberOfROVS)]

command=[]
scenario=modules.holoOceanUtils.scenario("ImagingSonarTank","64tankMap","simulationWorld",200)

for j in range(numberOfROVS):
    for i in range(numberOfROVS):
        auv_matrix[j][i]=(modules.holoOceanUtils.ROV(str(j)+str(i),[-2+(i*7.5),(j*-7.5),-2],[0,0,0]))
        auv_matrix[j][i].addSensor("LocationSensor","Origin")
        auv_matrix[j][i].addSensor("RotationSensor","Origin")
        auv_matrix[j][i].imageViwer()
        command.append([-2.5,-2.5,-2.5,-2.5,0,0,0,0])
        scenario.addAgent(auv_matrix[j][i].agent)

fineshed=0

#start Simulation
env=holoocean.make(scenario_cfg=scenario.cfg)
env.reset
env.set_render_quality(3)
env.move_viewport([26.25,-26.25,50],[0,0,180])
state = env.tick()

    
while fineshed<numberOfROVS:
        
        state = env.tick()
        for j in range(numberOfROVS):
            for i in range(numberOfROVS):

                if 'ImagingSonar' in state['auv'+str(j)+str(i)]:
                    auv_matrix[j][i].updateImnage(state['auv'+str(j)+str(i)]['ImagingSonar'])

                if state['auv'+str(j)+str(i)]["LocationSensor"][2]>-3:
                    env.act('auv'+str(j)+str(i), command[i])
                else:
                    env.act('auv'+str(j)+str(i), [0,0,0,0,0,0,0,0])
                    fineshed+=1
        state = env.tick()

print("Finished Simulation!")
#plt.ioff()
#plt.show()