import holoocean
import ManuallyControlling
import numpy as np
import matplotlib.pyplot as plt

#joystick=ManuallyControlling.joystick()

scenario="ImagingSonarTank"
config = holoocean.packagemanager.get_scenario(scenario)
config = config['agents'][0]['sensors'][-1]["configuration"]
azi = config['Azimuth']
minR = config['RangeMin']
maxR = config['RangeMax']
binsR = config['RangeBins']
binsA = config['AzimuthBins']

plt.ion()
fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8,5))
ax.set_theta_zero_location("N")
ax.set_thetamin(-azi/2)
ax.set_thetamax(azi/2)

theta = np.linspace(-azi/2, azi/2, binsA)*np.pi/180
r = np.linspace(minR, maxR, binsR)
T, R = np.meshgrid(theta, r)
z = np.zeros_like(T)

plt.grid(False)
plot = ax.pcolormesh(T, R, z, cmap='gray', shading='auto', vmin=0, vmax=1)
plt.tight_layout()
fig.canvas.draw()
fig.canvas.flush_events()

command=np.array([-5,-5,-5,-5   # UP
                 ,0,0,0,0])     # OMNI

#start Simulation
with holoocean.make(scenario) as env:
    env.move_viewport([0,0,7],[0,0,90])
    env.spawn_prop("sphere",[0,0,-2],[0,0,0],2,False,"brick")
    state = env.tick()
    print(state)
    
    if "LocationSensor" in state:
        print(state["LocationSensor"])

    while state["LocationSensor"][2] > -4:
        
        env.act("auv0", command)
        state = env.tick()

        if 'ImagingSonar' in state:
            s = state['ImagingSonar']
            plot.set_array(s.ravel())

            fig.canvas.draw()
            fig.canvas.flush_events()
        
print("Finished Simulation!")
plt.ioff()
plt.show()