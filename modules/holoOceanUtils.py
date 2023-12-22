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
    def __init__(self, id:str,location=[int,int,int],rotation=[int,int,int])->None:
        self.id=id

        self.agent={
            "agent_name": "auv"+id,
            "agent_type": "HoveringAUV",
            "sensors":[
                {
                    "sensor_type": "ImagingSonar",
                    "socket": "SonarSocket",
                    "Hz": 25,
                    "configuration": {
                        "RangeBins": 768,
                        "AzimuthBins": 334,
                        "RangeMin": 0.5,
                        "RangeMax": 2,
                        "InitOctreeRange":50,
                        "Elevation": 20,
                        "Azimuth": 60,
                        "AzimuthStreaks": -1,
                        "ScaleNoise": True,
                        "AddSigma": 0.05,
                        "MultSigma": 0.05,
                        "RangeSigma": 0.05,
                        "MultiPath": True,
                        "ViewRegion": True,
                        "ViewOctree": -1
    
                    }
                },
            ],
            "control_scheme":1,
            "location": location,
            "rotation": rotation
        }

    def addSensor(self,sensor:str,socket:str)->None:
        self.agent["sensors"].append({"sensor_type":sensor,
                                    "socket": socket})
    
    def addSonarImaging(self,RangeBins=768,AzimuthBins=334,RangeMin=0.5,
                        RangeMax=2.0,Elevation=20,Azimuth=60,
                        AzimuthStreaks=-1,ScaleNoise=True,AddSigma=0.05,
                        MultSigma=0.05,RangeSigma=0.05,MultiPath=True,
                        ViewRegion=True,ViewOctree=-1)->None:
        self.agent["sensors"][0]["configuration"]
        {
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
    
    def imageViwer(self)->None:    
        config = self.agent['sensors'][0]["configuration"]
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