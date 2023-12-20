import holoocean
import numpy as np
import matplotlib.pyplot as plt
import modules.holoOceanUtils

#joystick=ManuallyControlling.joystick()

#scenario="ImagingSonar64Tank"


auv0=modules.holoOceanUtils.ROV(0,[-2,0,-2],[0,0,0])
auv0.addSensor("LocationSensor","Origin")
auv0.addSensor("RotationSensor","Origin")
auv0.imageViwer()

auv1=modules.holoOceanUtils.ROV(1,[5.5,0,-2],[0,0,0])
auv1.addSensor("LocationSensor","Origin")
auv1.addSensor("RotationSensor","Origin")
auv1.imageViwer()

command0=np.array([-2.5,-2.5,-2.5,-2.5,0,0,0,0])
command1=np.array([-2.5,-2.5,-2.5,-2.5,0,0,0,0])

scenario=modules.holoOceanUtils.scenario("ImagingSonarTank","64tankMap","simulationWorld",200)
scenario.addAgent(auv0.agent)
scenario.addAgent(auv1.agent)

fineshed=0

#start Simulation
env=holoocean.make(scenario_cfg=scenario.cfg)
env.reset
env.set_render_quality(3)
env.move_viewport([4,6,8],[-90,270,180])
state = env.tick()

    
while fineshed<2:
        state = env.tick()
        if state['auv0']["LocationSensor"][2] > -3:
            if 'ImagingSonar' in state['auv0']:
                auv0.updateImnage(state['auv0']['ImagingSonar'])
                
        else:
            fineshed+=1
            command0=np.array([0,0,0,0,0,0,0,0])


        if state['auv0']["LocationSensor"][2] > -3:
            if 'ImagingSonar' in state['auv0']:
                auv1.updateImnage(state['auv1']['ImagingSonar'])
        else:
            fineshed+=1
            command0=np.array([0,0,0,0,0,0,0,0])

        
        env.act("auv0", command0)
        env.act("auv1", command1)
        state = env.tick()

print("Finished Simulation!")
plt.ioff()
plt.show()