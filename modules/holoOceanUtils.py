import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import json
import os

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

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class AUV:
    def __init__(self, id:str,control_scheme:int=2,location=[float,float,float],rotation=[int,int,int],root_folder:str='Sonar-Dataset')->None:
        self.files_folder='auv-'+id+'-data'
        self.root_folder=root_folder
        self.meta_data_file_name:str
        self.raw_sonar_data_file_name:str
        self.cartesian_image_file_name:str
        self.polar_image_file_name:str

        if os.path.exists(root_folder):
            os.system('mkdir '+root_folder+'/'+self.files_folder)
        else:
            os.system('mkdir '+root_folder)
            os.system('mkdir '+root_folder+'/'+self.files_folder)
        
        self.id=id
        self.name:str="auv"+str(id)
        self.control_scheme=control_scheme
        self.start_location=location
        self.start_rotation=rotation

        self.number_of_sensors:int=0
        self.sonar_ID:int
        self.sonar_ground_truth_ID:int
        self.agent={
            "agent_name": self.name,
            "agent_type": "HoveringAUV",
            "sensors":[
            ],
            "control_scheme":self.control_scheme,
            "location": self.start_location,
            "rotation": self.start_rotation
        }

        self.waypoints=[]
        self.number_of_waypoints:int=0
        self.reached_waypoints:int=0
        self.actual_waypoint=[]

        self.distance_tresh_hold=0.1
        self.angle_tresh_hold=2.0
        
        self.sonar_image=None
        self.actual_location=location
        self.actual_rotation=rotation

        self.pid_controller_linear = PIDController(2.5,0,0.01)
        self.pid_controller_angular = PIDController(0.01,0,0.001)
        self.dt=1/200

        self.command=None
        
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
    def addsonarGroundTruth(self)->None:
        self.sonar_ground_truth_ID=self.number_of_sensors
        self.agent["sensors"].append({"sensor_type":"RangeFinderSensor",
                                    "socket": "SonarSocket",
                                    "configuration":{
                                        "LaserMaxDistance":10,
                                        "LaserCount":2250,
                                        "LaserAngle":10,
                                        "LaserDebug":True
                                        }
                                    })

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
    
    def updateSonarImage(self)->None:
        self.polar_image_file_name=str(self.id)+'-polar-image-'+str(self.reached_waypoints)+'.png'
        s = self.sonar_image
        self.plot.set_array(s.ravel())
        self.fig.canvas.draw()
        
        self.fig.canvas.flush_events()
        plt.savefig(self.polar_image_file_name,transparent=False)
        
        os.system('mv '+self.polar_image_file_name+' '+self.root_folder+'/'+self.files_folder)
        
    
    def saveCartesianImage(self)->None:
        self.cartesian_image_file_name=str(self.id)+'-cartesian-image-'+str(self.reached_waypoints)+'.png'
        image=(self.sonar_image*255).astype(np.uint8)
        cartesian_image=Image.fromarray(image, mode='L').rotate(180)
        cartesian_image.save(self.cartesian_image_file_name,format='PNG')
        
        os.system('mv '+self.cartesian_image_file_name+' '+self.root_folder+'/'+self.files_folder)
    

    def saveSonarRawData(self)->None:
        self.raw_sonar_data_file_name=str(self.id)+'-raw-sonar-data-'+str(self.reached_waypoints)
        np.save(self.raw_sonar_data_file_name,self.sonar_image)
        os.system('mv '+self.raw_sonar_data_file_name+'.npy'+' '+self.root_folder+'/'+self.files_folder)

    def saveMetaDataFile(self)->None:
        
        sonar_specs=self.agent['sensors'][self.sonar_ID]["configuration"]
        sonar_data={
            "AUV_ID":str(self.id),
            "sonar_raw_data_file":self.raw_sonar_data_file_name,
            "sonar_cartesian_image_file":self.cartesian_image_file_name,
            "sonar_polar_image_file":self.polar_image_file_name,
            "x":float(self.actual_location[0]),
            "y":float(self.actual_location[1]),
            "z":float(self.actual_location[2]),
            "roll":float(self.actual_rotation[0]),
            "pitch":float(self.actual_rotation[1]),
            "yaw":float(self.actual_rotation[2]),
            "sonar_azimuth":int(sonar_specs['Azimuth']),
            "sonar_range_min":float(sonar_specs['RangeMin']),
            "sonar_range_max":float(sonar_specs['RangeMax']),
            "azimuth_bins":int(sonar_specs['RangeBins']),
            "range_bins":int(sonar_specs['AzimuthBins']),
        }
        self.meta_data_file_name=str(self.id)+'-sonar_meta_data-'+str(self.reached_waypoints)+'.json'
        with open(self.meta_data_file_name,'w') as fp:
            json.dump(sonar_data, fp)
        
        os.system('mv '+self.meta_data_file_name+' '+self.root_folder+'/'+self.files_folder)

    def updateState(self,state,save_data:True)->None:
        if 'ImagingSonar' in state[self.name]:    
            self.sonar_image=(state[self.name]['ImagingSonar'])
        if 'LocationSensor' in state[self.name]:
            self.actual_location=(state[self.name]['LocationSensor'])
        if 'RotationSensor' in state[self.name]:
            self.actual_rotation=(state[self.name]['RotationSensor'])
        
        if self.reachedWaypoint() and save_data==True:
            
            self.updateSonarImage()
            self.saveSonarRawData()
            self.saveCartesianImage()
            self.saveMetaDataFile()
        
        self.calculateVelocities()

    def createWaypoints(self,end_z)->None:
        angles=np.linspace(0,350,36)
        headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
        elevation=np.arange(-2.5,end_z,0.3 )
        for z in elevation:
            for angle, heading in zip(angles,headings):
                x=2*np.cos(np.deg2rad(angle))+self.start_location[0]-2
                y=2*np.sin(np.deg2rad(angle))+self.start_location[1]
                self.waypoints.append([x,y,z,0,0,heading])
        self.number_of_waypoints=len(self.waypoints)
        self.actual_waypoint=self.waypoints[0]

    def reachedWaypoint(self)->bool:
        tresh_hold_distance=self.distance_tresh_hold
        tresh_hold_angle=self.angle_tresh_hold
        x_rov=self.actual_location[0]
        y_rov=self.actual_location[1]
        z_rov=self.actual_location[2]
        roll_rov=self.actual_rotation[0]
        pitch_rov=self.actual_rotation[1]
        yaw_rov=self.actual_rotation[2]
        
        if roll_rov>180:
            roll_rov-=360
        if pitch_rov>180:
            pitch-=360
        if yaw_rov>180:
            yaw_rov-=360

        x=self.actual_waypoint[0]
        y=self.actual_waypoint[1]
        z=self.actual_waypoint[2]
        roll=self.actual_waypoint[3]
        pitch=self.actual_waypoint[4]
        yaw=self.actual_waypoint[5]

        if roll>180:
            roll-=360
        if pitch>180:
            pitch-=360
        if yaw>180:
            yaw-=360

        if np.linalg.norm(np.array([x,y,z])-np.array([x_rov,y_rov,z_rov]))<=tresh_hold_distance and np.linalg.norm(roll-roll_rov)<=tresh_hold_angle and np.linalg.norm(pitch-pitch_rov)<=tresh_hold_angle and np.linalg.norm(yaw-yaw_rov)<=tresh_hold_angle:
            self.reached_waypoints+=1
            if self.reached_waypoints<self.number_of_waypoints:
                self.actual_waypoint=self.waypoints[self.reached_waypoints]
                return True
            else:
                return False
        return False
    
    def calculateVelocities(self)->None:
        position_error = self.actual_waypoint[:3] - self.actual_location
        
        desired_linear_velocity = self.pid_controller_linear.update(np.linalg.norm(position_error), self.dt)
        linear_velocity = position_error / np.linalg.norm(position_error) * desired_linear_velocity

        erro_orientacao = self.actual_waypoint[3:] - self.actual_rotation

        erro_orientacao[erro_orientacao > 180] -= 360
        erro_orientacao[erro_orientacao < -180] += 360

        desired_angular_velocity = self.pid_controller_angular.update(np.linalg.norm(erro_orientacao), self.dt)
        angular_velocity = erro_orientacao / np.linalg.norm(erro_orientacao) * desired_angular_velocity

        self.command = np.concatenate((linear_velocity, angular_velocity), axis=None)

    def fineshedMission(self)->bool:
        if self.reached_waypoints-1>self.number_of_waypoints:
            self.command=[0,0,0,0,0,0]
            return True
        else:
            return False