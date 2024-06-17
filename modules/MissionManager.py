import holoocean
import modules.holoOceanUtils
import os
import time

def endMission():
    pass

def mission(mission_data:list,mission_id:int):
    data=mission_data
    scenario=modules.holoOceanUtils.scenario("Imaging_Sonar_Dataset","64-tank-Map-"+str(mission_id),"Dataset-world",200)

    auv=modules.holoOceanUtils.AUV(id=str(data[0]),location=[float(data[2]),-1*float(data[3]),float(data[4])],rotation=[0,0,180],mission=mission_id)
    auv.addSonarImaging()
    auv.addSensor("LocationSensor","Origin")
    auv.addSensor("RotationSensor","Origin")
    auv.createWaypoints((float(data[4])+float(data[5])))
    auv.imageViwer()
    scenario.addAgent(auv.agent)

    env=holoocean.make(scenario_cfg=scenario.cfg,verbose=True)
    env.reset

    for l in auv.waypoints:
        env.draw_point([l[0], l[1], l[2]],[0,255,0], lifetime=0)
    #start Simulation

    env.move_viewport([float(data[2]),-1*float(data[3]),6],[0,0,180])

    state=env.tick()
    auv.updateState(state)

    while not auv.fineshedMission():
        state=env.tick()
        auv.updateState(state)
        env.act(auv.name,auv.command)
    print("Finished Mission "+data[0])
    #env.reset
    os.system("killall -e Holodeck")
    time.sleep(10)
    #Holodeck