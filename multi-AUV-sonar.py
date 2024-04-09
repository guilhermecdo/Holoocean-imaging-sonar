import holoocean
import numpy as np
import modules.holoOceanUtils

numberOfROVS=2
auv_array=[]

auv_matrix = [[column for column in range(numberOfROVS)] for row in range(numberOfROVS)]

scenario=modules.holoOceanUtils.scenario("ImagingSonarTank2","64tankMap1","simulationWorld2",200)

for i in range(numberOfROVS):
    auv_array.append(modules.holoOceanUtils.AUV(str(i),2,[2+(i*7.5),0,-2.5],[0,0,180]))
    auv_array[i].addSonarImaging()
    auv_array[i].addSensor("LocationSensor","Origin")
    auv_array[i].addSensor("RotationSensor","Origin")
    auv_array[i].imageViwer()
    auv_array[i].createWaypoints(-1.7)
    scenario.addAgent(auv_array[i].agent)

"""
for j in range(numberOfROVS):
    for i in range(numberOfROVS):
        auv_matrix[j][i]=(modules.holoOceanUtils.ROV(str(j)+str(i),2,[2+(i*7.5),(j*-7.5),-2.5],[0,0,180]))
        auv_matrix[j][i].addSonarImaging()
        auv_matrix[j][i].addSensor("LocationSensor","Origin")
        auv_matrix[j][i].addSensor("RotationSensor","Origin")
        auv_matrix[j][i].imageViwer()
        scenario.addAgent(auv_matrix[j][i].agent)
"""

#start Simulation
env=holoocean.make(scenario_cfg=scenario.cfg)
env.reset
env.set_render_quality(3)
#env.move_viewport([26.25,-26.25,50],[0,0,180])
env.move_viewport([0,0,4],[0,0,180])


state = env.tick()
interactions=0
for auv in auv_array:
    for l in auv.waypoints:
        interactions+=1
        env.draw_point([l[0], l[1], l[2]],[0,255,0], lifetime=0)


print(interactions)
while True:
    #state = env.tick()
    for auv in auv_array:
        auv.updateState(state)
        if auv.fineshedMission():
            holoocean.command.RemoveSensorCommand(auv.name,'ImagingSonar')
        env.act(auv.name,auv.command)
    state = env.tick()
        
print("Finished Simulation!")