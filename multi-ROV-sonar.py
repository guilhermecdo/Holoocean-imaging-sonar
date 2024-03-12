import holoocean
import numpy as np
import matplotlib.pyplot as plt
import modules.holoOceanUtils
import os

numberOfROVS=1

auv_matrix = [[column for column in range(numberOfROVS)] for row in range(numberOfROVS)]

waypoints=[]
angles=np.linspace(2*np.pi,0,5)
for angle in angles:
    x=2*np.cos(angle)
    y=2*np.sin(angle)
    waypoints.append([x,y,-2,0,0,np.rad2deg(angle+(np.pi))])

command=[]
scenario=modules.holoOceanUtils.scenario("ImagingSonarTank2","64tankMap1","simulationWorld2",200)

for j in range(numberOfROVS):
    for i in range(numberOfROVS):
        auv_matrix[j][i]=(modules.holoOceanUtils.ROV(str(j)+str(i),1,[2+(i*7.5),0+(j*-7.5),-2],[0,0,-180]))
        #auv_matrix[j][i].addSonarImaging()
        auv_matrix[j][i].addSensor("LocationSensor","Origin")
        auv_matrix[j][i].addSensor("RotationSensor","Origin")
        #auv_matrix[j][i].imageViwer()
        #command.append([-2.5,-2.5,-2.5,-2.5,0,0,0,0])
        scenario.addAgent(auv_matrix[j][i].agent)

fineshed=0

#start Simulation
env=holoocean.make(scenario_cfg=scenario.cfg)
env.reset
env.set_render_quality(3)
env.move_viewport([26.25,-26.25,50],[0,0,180])
state = env.tick()

for l in waypoints:
        env.draw_point([l[0], l[1], l[2]],[0,255,0], lifetime=0)
    
while fineshed<numberOfROVS:
    for points in waypoints[1:]:       
        state = env.tick()
        env._command_center.handle_buffer()
        while not auv_matrix[j][i].reachedWaypoint(points,state[auv_matrix[j][i].name]["LocationSensor"],state[auv_matrix[j][i].name]["RotationSensor"],0.1,1):
            os.system("clear")
            print("ROV Location:",state[auv_matrix[j][i].name]["LocationSensor"],"ROV Waipont:",points[0:3],"ROV Rotation Sensor:",state[auv_matrix[j][i].name]["RotationSensor"],"Angle:",points[3:],end="\r")
            for j in range(numberOfROVS):
                for i in range(numberOfROVS):
                    if 'ImagingSonar' in state[auv_matrix[j][i].name]:
                        auv_matrix[j][i].updateImnage(state[auv_matrix[j][i].name]['ImagingSonar'])
                    env.act(auv_matrix[j][i].name, points)
            state = env.tick()
    fineshed+=1

print("Finished Simulation!")