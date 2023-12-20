import holoocean
import modules.ManuallyControlling as ManuallyControlling
import numpy as np
import matplotlib.pyplot as plt

#joystick=ManuallyControlling.joystick()

scenario="ImagingSonar64Tank"
config = holoocean.packagemanager.get_scenario(scenario)
camera_pose=config['agents'][0]['location']
camera_pose[2]=6
config = config['agents'][0]['sensors'][-1]["configuration"]
azi = config['Azimuth']
minR = config['RangeMin']
maxR = config['RangeMax']
binsR = config['RangeBins']
binsA = config['AzimuthBins']

plt.ion()
fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(5,5))
ax.set_theta_zero_location("N")
ax.set_thetamin(-azi/2)
ax.set_thetamax(azi/2)

theta = np.linspace(-azi/2, azi/2, binsA)*np.pi/180
r = np.linspace(minR, maxR, binsR)
T, R = np.meshgrid(theta, r)
z = np.zeros_like(T)

plt.grid(False)
plot = ax.pcolormesh(T, R, z, cmap='CMRmap', shading='auto', vmin=0, vmax=1)
plt.tight_layout()
fig.canvas.draw()
fig.canvas.flush_events()

command=np.array([-2.5,-2.5,-2.5,-2.5,   # UP
                 0,0,0,0])     # OMNI

#client=holoocean.holooceanclient.HoloOceanClient('client1')


#start Simulation
with holoocean.make(scenario,verbose=False) as env:
    #ROV=holoocean.agents.HoloOceanAgent(client)
    env.set_render_quality(3)
    env.move_viewport(camera_pose,[-90,0,180])
    state = env.tick()
    
    
    if "LocationSensor" in state['auv0']:
        print(state['auv0']["LocationSensor"], end='\r')

    while state['auv0']["LocationSensor"][2] > -3:
        print(state['auv0']["LocationSensor"], end='\r')
        env.act("auv0", command)
        state = env.tick()

        if 'ImagingSonar' in state['auv0']:
            s = state['auv0']['ImagingSonar']
            plot.set_array(s.ravel())

            fig.canvas.draw()
            fig.canvas.flush_events()


print("Finished Simulation!")
plt.ioff()
plt.show()