import numpy as np
import matplotlib.pyplot as plt

class scenario:
    def __init__(self,name:str,world:str,package_name:str,ticks_per_sec:int) -> None:
        
        self.cfg={
            "name": name,
            "world": world,
            "package_name":package_name,
            "ticks_per_sec": ticks_per_sec,
            "frames_per_sec": True,
            "octree_min": 0.02,
            "octree_max": 5,
            "agents":[]
        }
    
    def addAgent(self, agent)->None:
        self.cfg["agents"].append(agent) 
        pass

class ROV:
    def __init__(self, id:str,control_scheme:int=1,location=[int,int,int],rotation=[int,int,int])->None:
        self.id=id
        self.name:str="auv"+str(id)
        self.control_scheme=control_scheme

        self.number_of_sensors:int=0
        self.sonar_ID:int
        self.agent={
            "agent_name": self.name,
            "agent_type": "HoveringAUV",
            "sensors":[
            ],
            "control_scheme":self.control_scheme,
            "location": location,
            "rotation": rotation
        }

    def addSensor(self,sensor:str,socket:str)->None:
        self.agent["sensors"].append({"sensor_type":sensor,
                                    "socket": socket})
        self.number_of_sensors+=1
    
    def addSonarImaging(self,hz=10,RangeBins=394,AzimuthBins=768,RangeMin=0.5,
                        RangeMax=10,Elevation=20,Azimuth=130,
                        AzimuthStreaks=-1,ScaleNoise=True,AddSigma=0.05,
                        MultSigma=0.05,RangeSigma=0.05,MultiPath=True,
                        ViewRegion=True,ViewOctree=-1)->None:
        
        self.agent["sensors"].append({"sensor_type":"ImagingSonar",
                                    "socket": "SonarSocket",
                                    "Hz": hz,
                                    "configuration":{}
                                    })
        
        self.sonar_ID=self.number_of_sensors

        self.agent["sensors"][self.sonar_ID]["configuration"]={
            "RangeBins": RangeBins,
            "AzimuthBins": AzimuthBins,
            "RangeMin": RangeMin,
            "RangeMax": RangeMax,
            "InitOctreeRange":50,
            "Elevation": Elevation,
            "Azimuth": Azimuth,
            "AzimuthStreaks": AzimuthStreaks,
            "ScaleNoise": ScaleNoise,
            "AddSigma": AddSigma,
            "MultSigma": MultSigma,
            "RangeSigma": RangeSigma,
            "MultiPath": MultiPath,
            "ViewRegion": ViewRegion,
            "ViewOctree": ViewOctree
        }

        self.number_of_sensors+=1
    
    def imageViwer(self)->None:    
        config = self.agent['sensors'][self.sonar_ID]["configuration"]
        azi = config['Azimuth']
        minR = config['RangeMin']
        maxR = config['RangeMax']
        binsR = config['RangeBins']
        binsA = config['AzimuthBins']
        plt.ion()
        self.fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(5,5))
        ax.set_theta_zero_location("N")
        ax.set_thetamin(-azi/2)
        ax.set_thetamax(azi/2)

        theta = np.linspace(-azi/2, azi/2, binsA)*np.pi/180
        r = np.linspace(minR, maxR, binsR)
        T, R = np.meshgrid(theta, r)
        z = np.zeros_like(T)

        plt.grid(False)
        self.plot = ax.pcolormesh(T, R, z, cmap='CMRmap', shading='auto', vmin=0, vmax=1)
        plt.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def updateImnage(self,sonarImage)->None:
        s = sonarImage
        self.plot.set_array(s.ravel())
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def reachedWaypoint(self,waypoint:list,rov_location:list,rov_rotation:list,tresh_hold_distance:float,tresh_hold_angle:float)->bool:
        x_rov=rov_location[0]
        y_rov=rov_location[1]
        z_rov=rov_location[2]
        roll_rov=rov_rotation[0]
        pitch_rov=rov_rotation[1]
        yaw_rov=rov_rotation[2]
        
        if roll_rov>180:
            roll_rov-=360
        if pitch_rov>180:
            pitch-=360
        if yaw_rov>180:
            yaw_rov-=360

        x=waypoint[0]
        y=waypoint[1]
        z=waypoint[2]
        roll=waypoint[3]
        pitch=waypoint[4]
        yaw=waypoint[5]

        if roll>180:
            roll-=360
        if pitch>180:
            pitch-=360
        if yaw>180:
            yaw-=360

        if np.linalg.norm(np.array([x,y,z])-np.array([x_rov,y_rov,z_rov]))<=tresh_hold_distance and np.linalg.norm(roll-roll_rov)<=tresh_hold_angle and np.linalg.norm(pitch-pitch_rov)<=tresh_hold_angle and np.linalg.norm(yaw-yaw_rov)<=tresh_hold_angle:
            return True
        else:
            return False
        
        