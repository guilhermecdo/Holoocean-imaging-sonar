import holoocean
import numpy as np
import modules.holoOceanUtils
import csv

with open('Dataset-metadata.csv', newline='') as f:
    reader = csv.reader(f)
    metadata = list(reader)



numberOfROVS=1
auv_array=[]

h=[1.01, #0
   1.04, #1
   1.88, #2
   1.52, #3
   0.87, #4
   1.15, #5
   2.00, #6
   2.29, #7
   ] 

N=7
ID=39

auv_matrix = [[column for column in range(numberOfROVS)] for row in range(numberOfROVS)]

scenario=modules.holoOceanUtils.scenario("Imaging_Sonar_Dataset","64tankMap","Dataset-world",200)

for i in range(numberOfROVS):
    auv_array.append(modules.holoOceanUtils.AUV(str(ID),2,[2+((i+N)*7.5),-30,-2.5],[0,0,180]))
    auv_array[i].addSonarImaging()
    auv_array[i].addSensor("LocationSensor","Origin")
    auv_array[i].addSensor("RotationSensor","Origin")
    auv_array[i].imageViwer()
    auv_array[i].createWaypoints((-2.5+h[(i+N)]))
    scenario.addAgent(auv_array[i].agent)

#start Simulation
env=holoocean.make(scenario_cfg=scenario.cfg)
env.reset
env.set_render_quality(3)
#env.move_viewport([26.25,-26.25,50],[0,0,180])
env.move_viewport([7.5*N,-30,10],[0,0,180])


state = env.tick()
interactions=0
for auv in auv_array:
    interactions=0
    for l in auv.waypoints:
        interactions+=1
        env.draw_point([l[0], l[1], l[2]],[0,255,0], lifetime=0)
    print(interactions)

fineshed=0

while numberOfROVS>=fineshed:
    for auv in auv_array:
        auv.updateState(state)
        if auv.fineshedMission():
            holoocean.command.RemoveSensorCommand(auv.name,'ImagingSonar')
            fineshed+=1
        env.act(auv.name,auv.command)
    state = env.tick()
        
print("Finished Simulation!")