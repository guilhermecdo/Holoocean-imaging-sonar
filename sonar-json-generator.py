import numpy as np
import json

def addImagingSonar(self,configuration:dict=None,rotation:list=[0,0,0])->None:
        self.sensors.image_sonar_config=configuration

        self.agent["sensors"].append({"sensor_type":"ImagingSonar",
                                    "socket": "Origin",
                                    "rotation":rotation,
                                    "configuration":{}
                                    })
        
        self.sonar_ID=self.number_of_sensors
        self.agent["sensors"][self.sonar_ID]["configuration"]=configuration
        self.sensors.image_sonar_config=configuration
        self.number_of_sensors+=1

def addSonarCamera(self,rotation:list=[0,0,0])->None:
        
        CaptureHeight=((np.tan(np.deg2rad(self.sensors.image_sonar_config["Elevation"]/2))*2*self.sensors.image_sonar_config["RangeMax"]) /
                        ((np.tan(np.deg2rad(self.sensors.image_sonar_config["Azimuth"]/2))*2*self.sensors.image_sonar_config["RangeMax"])/
                         self.sensors.image_sonar_config["AzimuthBins"]))
        FovAngle=np.arctan((np.deg2rad(self.sensors.image_sonar_config["Azimuth"]/2))/(np.tan(np.deg2rad(self.sensors.image_sonar_config["Elevation"]/2))))

        self.depth_image=np.zeros(shape=(int(CaptureHeight),self.sensors.image_sonar_config["AzimuthBins"],1))

        self.agent["sensors"].append({"sensor_type":"Camera",
                                    "socket": "Origin",
                                    "rotation":rotation,
                                    "configuration":{
                                        "CaptureWidth":self.sensors.image_sonar_config["AzimuthBins"],
                                        "CaptureHeight":int(CaptureHeight),
                                        "FovAngle":np.rad2deg(FovAngle),
                                        "convertToDistance":True,
                                    }})
        
def addSonarGT(self,rotation:list=[0,0,0])->None:
        self.gt_matrix=np.zeros(shape=(int(self.sensors.image_sonar_config["AzimuthBins"] ), int(self.sensors.image_sonar_config["Elevation"])))
        for t in range(int(self.sensors.image_sonar_config["AzimuthBins"])):
            for p in range(int(self.sensors.image_sonar_config["Elevation"])):
                rotation=[0,rotation[1],(t*(self.sensors.image_sonar_config["Azimuth"]/self.sensors.image_sonar_config["AzimuthBins"]))-self.sensors.image_sonar_config["Azimuth"]/2]
                self.agent["sensors"].append({"sensor_type":"RangeFinderSensor",
                                                "sensor_name":(f"{t} {p}"),
                                                "socket": "Origin",
                                                "rotation":rotation,
                                                "configuration":{
                                                    "LaserMaxDistance": self.sensors.image_sonar_config["RangeMax"],
                                                    "LaserCount": 1,
                                                    "LaserAngle":p-int(self.sensors.image_sonar_config["Elevation"])/2,
                                                    "LaserDebug": True,
                                                }
                                            })
