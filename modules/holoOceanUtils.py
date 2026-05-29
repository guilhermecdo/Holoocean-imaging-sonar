import cv2
import holoocean
import holoocean.agents
import holoocean.sensors
import holoocean.holooceanclient
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import json
import os
import pickle
import math

#import cv2

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
            "agents":[],
            "window_width":  1080,
            "window_height": 720
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

class Sensors:
    def __init__(self,agent_name:str,agent_type:str) -> None:
        self.agent_name=agent_name
        self.agent_type=agent_type
        self.image_sonar=None
        self.image_sonar_config=None
        self.location_sensor=None
        self.rotation_sensor=None

    def addPositionSensor(self)->None:
        self.location_sensor=holoocean.sensors.SensorDefinition(
            agent_name=self.agent_name,
            agent_type=self.agent_type,
            sensor_name="LocationSensor",
            sensor_type="LocationSensor",
            socket="Origin")
        
        self.rotation_sensor=holoocean.sensors.SensorDefinition(
            agent_name=self.agent_name,
            agent_type=self.agent_type,
            sensor_name="RotationSensor",
            sensor_type="RotationSensor",
            socket="Origin")
        
    def addImagingSonar(self)->None:
        self.image_sonar=holoocean.sensors.SensorDefinition(
            agent_name=self.agent_name,
            agent_type=self.agent_type,
            sensor_name="ImagingSonar",
            sensor_type="ImagingSonar",
            socket="SonarSocket",
            config=self.image_sonar_config)
        
class AUV:
    def __init__(self,id:str,control_scheme:int=2,location=[float,float,float],rotation=[int,int,int],mission=1,waypoints=[],sonar_model:str="")->None:
        
        self.files_folder='auv-'+id
        self.pkl_folder='States'
        self.rgbd_image_folder='RGBD-images'
        self.cartesian_image_folder='Cartesian-images'
        self.polar_image_folder='Polar-images'
        self.raw_data_folder='Raw-data'
        self.meta_data_folder='Meta-data'
        self.root_folder="Sonar-Dataset-mission-"+str(mission)+"-"+sonar_model
        self.gt_folder="GT-folder"
        self.meta_data_file_name:str
        self.raw_sonar_data_file_name:str
        self.cartesian_image_file_name:str
        self.polar_image_file_name:str
        self.mission=mission
        if os.path.exists(self.root_folder):
            os.system('mkdir '+self.root_folder+'/'+self.files_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.pkl_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.cartesian_image_folder)
            #os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.rgbd_image_folder)
            os.system(f"mkdir {self.root_folder}/{self.files_folder}/GT-images/")
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.gt_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.polar_image_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.raw_data_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.meta_data_folder)
        else:
            os.system('mkdir '+self.root_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.pkl_folder)
            #os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.rgbd_image_folder)
            os.system(f"mkdir {self.root_folder}/{self.files_folder}/GT-images/")

            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.gt_folder)

            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.cartesian_image_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.polar_image_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.raw_data_folder)
            os.system('mkdir '+self.root_folder+'/'+self.files_folder+'/'+self.meta_data_folder)
        
        self.id=id
        self.name:str="auv"+str(id)
        self.type="HoveringAUV"
        self.control_scheme=control_scheme
        self.start_location=location
        self.start_rotation=rotation

        self.number_of_sensors:int=0
        self.sonar_ID:int
        self.agent={
            "agent_name": self.name,
            "agent_type": "HoveringAUV",
            "sensors":[],
            "control_scheme":self.control_scheme,
            "location": self.start_location,
            "rotation": self.start_rotation
        }

        self.waypoints=waypoints
        self.number_of_waypoints:int=len(waypoints)
        self.reached_waypoints:int=0
        self.actual_waypoint=waypoints[self.reached_waypoints]

        self.distance_tresh_hold=0.1
        self.angle_tresh_hold=0.01
        
        self.sonar_image=None
        self.actual_location=location
        self.actual_rotation=rotation

        self.pid_controller_x = PIDController(kp=20,ki=0.1,kd=10)
        self.pid_controller_y = PIDController(kp=20,ki=0.1,kd=10)
        self.pid_controller_z = PIDController(kp=20,ki=0.1,kd=10)

        self.pid_controller_angular = PIDController(kp=8,ki=0.0,kd=1)
        self.dt=1/20

        self.command=None
        self.sensors=Sensors(self.name,"TurtleAgent")
        self.sensors.addImagingSonar()
        self.sensors.addPositionSensor()
        
        self.agent_definition=holoocean.agents.AgentDefinition(
            agent_name=self.name,
            agent_type="TurtleAgent",
            sensors=[self.sensors.image_sonar,self.sensors.location_sensor,self.sensors.rotation_sensor],
            starting_loc=self.start_location,
            starting_rot=self.start_rotation)
        self.counter=0

    def addSensor(self,sensor:str,socket:str,rotation:list=[0,0,0])->None:
        self.agent["sensors"].append({"sensor_type":sensor,
                                    "socket": socket,
                                    "rotation":rotation})
        self.number_of_sensors+=1
    
    def addSonarGT(self,rotation)->None:
        self.gt_matrix=np.zeros(shape=(int(self.sensors.image_sonar_config["AzimuthBins"] ), int(self.sensors.image_sonar_config["Elevation"])))
        for t in range(int(self.sensors.image_sonar_config["AzimuthBins"])):
            #for p in range(int(self.sensors.image_sonar_config["Elevation"])):
            for p in range(int(self.sensors.image_sonar_config["AzimuthBins"])):
                rotation=[0,rotation[1],(t*(self.sensors.image_sonar_config["Azimuth"]/self.sensors.image_sonar_config["AzimuthBins"]))-self.sensors.image_sonar_config["Azimuth"]/2]
                self.agent["sensors"].append({"sensor_type":"RangeFinderSensor",
                                                "sensor_name":(f"{t} {p}"),
                                                "socket": "Origin",
                                                "rotation":rotation,
                                                "configuration":{
                                                    "LaserMaxDistance": self.sensors.image_sonar_config["RangeMax"],
                                                    "LaserCount": 1,
                                                    #"LaserAngle":p-int(self.sensors.image_sonar_config["Elevation"])/2,
                                                    "LaserAngle":(p*(self.sensors.image_sonar_config["Elevation"]/self.sensors.image_sonar_config["AzimuthBins"]))-int(self.sensors.image_sonar_config["Elevation"])/2,
                                                    "LaserDebug": True,
                                                }
                                            })

    def addRGBDCamera(self,rotation)->None:
        
        CaptureHeight=((np.tan(np.deg2rad(self.sensors.image_sonar_config["Elevation"]/2))*2*self.sensors.image_sonar_config["RangeMax"]) /
                        ((np.tan(np.deg2rad(self.sensors.image_sonar_config["Azimuth"]/2))*2*self.sensors.image_sonar_config["RangeMax"])/
                         self.sensors.image_sonar_config["AzimuthBins"]))
        FovAngle=np.arctan((np.deg2rad(self.sensors.image_sonar_config["Azimuth"]/2))/(np.tan(np.deg2rad(self.sensors.image_sonar_config["Elevation"]/2))))

        self.depth_image=np.zeros(shape=(int(CaptureHeight),self.sensors.image_sonar_config["AzimuthBins"],1))

        self.agent["sensors"].append({"sensor_type":"RGBDCamera",
                                    "socket": "CameraSocket",
                                    "rotation":rotation,
                                    "configuration":{
                                        "CaptureWidth":self.sensors.image_sonar_config["AzimuthBins"],
                                        "CaptureHeight":int(CaptureHeight),
                                        "FovAngle":np.rad2deg(FovAngle),
                                        #"MaxViewDistanceOverride":self.sensors.image_sonar_config["RangeMax"]*100,
                                        #"ShowDebugPoints":True,
                                        "convertToDistance":True,
                                        #"ViewRegion": True,
                                    }})
        
    def addSonarImaging(self,configuration:dict=None,rotation:list=[0,0,0],hz=10)->None:
        self.sensors.image_sonar_config=configuration

        #return 0
        self.agent["sensors"].append({"sensor_type":"ImagingSonar",
                                    "socket": "Origin",
                                    "rotation":rotation,
                                    #location":[self.actual_location[0]/100,self.actual_location[1]/100,self.actual_location[2]/100],
                                    "Hz": hz,
                                    "configuration":{}
                                    })
        
        self.sonar_ID=self.number_of_sensors
        self.agent["sensors"][self.sonar_ID]["configuration"]=configuration
        self.sensors.image_sonar_config=configuration
        self.number_of_sensors+=1

    def imageViwer(self)->None:    
        config = self.sensors.image_sonar_config
        azi = config['Azimuth']
        minR = config['RangeMin']
        maxR = config['RangeMax']
        binsR = config['RangeBins']
        binsA = config['AzimuthBins']
        if not hasattr(self, 'fig_sonar'):  # Initialize the figure if it doesn't exist
            plt.ion()

            self.fig_sonar, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(5,5))
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
        #if not hasattr(self, 'fig_depth'):  # Initialize the figure if it doesn't exist
        #    plt.ion()
        #    self.fig_depth, ax_depth = plt.subplots(figsize=(5,5))
        #    self.depth_plot = ax_depth.imshow(np.zeros_like(self.depth_image), cmap='gray')

        
        self.fig_sonar.canvas.draw()
        self.fig_sonar.canvas.flush_events()
        #self.fig_depth.canvas.draw()
        #self.fig_depth.canvas.flush_events()

    def updateSonarImage(self)->None:
        self.polar_image_file_name=str(self.mission)+"-"+str(self.counter)+'.png'
        s = self.sonar_image
        self.plot.set_array(s.ravel())
        self.fig_sonar.canvas.draw()
        
        self.fig_sonar.canvas.flush_events()

        self.fig_sonar.savefig(self.polar_image_file_name)
        os.system('mv '+self.polar_image_file_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.polar_image_folder)
         
    def saveCartesianImage(self)->None:
        self.cartesian_image_file_name=str(self.mission)+"-"+str(self.counter)+'.png'
        jet_image_name=str(self.mission)+"-"+str(self.counter)+'-jet.png'
        image=(self.sonar_image*255).astype(np.uint8)
        
        # Vertically flip the raw image
        image_flipped = cv2.flip(image, 0)
        
        # Apply the colormap to the flipped image
        image_jet_flipped = cv2.applyColorMap(image_flipped, cv2.COLORMAP_JET)

        # Save the flipped images
        cv2.imwrite(self.cartesian_image_file_name, image_flipped)
        cv2.imwrite(jet_image_name, image_jet_flipped)
        #cartesian_image=Image.fromarray(image, mode='L')
        #cartesian_image.save(self.cartesian_image_file_name,format='PNG')

        os.system('mv '+self.cartesian_image_file_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.cartesian_image_folder)
        os.system('mv '+jet_image_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.cartesian_image_folder)
    
    def saveSonarRawData(self)->None:
        self.raw_sonar_data_file_name=str(self.mission)+"-"+str(self.counter)
        np.save(self.raw_sonar_data_file_name,self.sonar_image)
        os.system('mv '+self.raw_sonar_data_file_name+'.npy'+' '+self.root_folder+'/'+self.files_folder+'/'+self.raw_data_folder)

    def saveMetaDataFile(self)->None:
        
        sonar_specs=self.sensors.image_sonar_config
        sonar_data={
            "AUV_ID":str(self.id),
            #"sonar_raw_data_file":self.raw_sonar_data_file_name,
            #"sonar_cartesian_image_file":self.cartesian_image_file_name,
            #"sonar_polar_image_file":self.polar_image_file_name,
            "x":float(self.actual_location[0]),
            "y":float(self.actual_location[1]),
            "z":float(self.actual_location[2]),
            "roll":float(self.actual_rotation[0]),
            "pitch":float(self.actual_rotation[1]),
            "yaw":float(self.actual_rotation[2]),
            "sonar_azimuth":int(sonar_specs['Azimuth']),
            "sonar_range_min":float(sonar_specs['RangeMin']),
            "sonar_range_max":float(sonar_specs['RangeMax']),
            "azimuth_bins":int(sonar_specs['AzimuthBins']),
            "range_bins":int(sonar_specs['RangeBins'])
        }
        self.meta_data_file_name=str(self.mission)+"-"+str(self.counter)+'.json'
        #sonar_data = json.dumps(sonar_data,indent=len(sonar_data))
        with open(self.meta_data_file_name,'w') as fp:
            json.dump(sonar_data, fp)
        
        os.system('mv '+self.meta_data_file_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.meta_data_folder)

    def saveState(self,state)->None:
        #if 'RGBDCamera' in state[self.name]:    
            #self.sonar_image=(state[self.name]['ImagingSonar'])
        with open(str(self.mission)+"-"+str(self.counter)+'.pkl', 'wb') as file:  
            pickle.dump(state, file)
            os.system('mv '+str(self.mission)+"-"+str(self.counter)+'.pkl'+' '+self.root_folder+'/'+self.files_folder+'/'+self.pkl_folder)
                #self.counter+=1
    
    def updateRGBDImage(self,state)->None:

        pixels = state["RGBDCamera"]
        
        self.depth_data = pixels[:, :, 4]
        #print(self.depth_data)
        #self.depth_plot.set_clim(vmin=self.depth_data.min(), vmax=self.depth_data.max())
        #print(self.depth_data.min())
        #print(self.depth_data.max())

        print("######################################")
        print(self.depth_data.min())
        print("######################################")
        self.depth_plot.set_clim(vmin=0, vmax=1000)
        
        self.depth_plot.set_data(self.depth_data)


        self.fig_depth.canvas.draw()
        self.fig_depth.canvas.flush_events()
        
        with open(str(self.mission)+"-"+str(self.counter)+'.pkl', 'wb') as file:  
            pickle.dump(pixels, file)
            os.system('mv '+str(self.mission)+"-"+str(self.counter)+'.pkl'+' '+self.root_folder+'/'+self.files_folder+'/'+self.rgbd_image_folder)
    
    def saveSonarGT(self,state)->None:
        gt_image=np.zeros(shape=(self.sensors.image_sonar_config["RangeBins"],self.sensors.image_sonar_config["AzimuthBins"]))
        #print(gt_image.shape)
        if '0 0' in state:
            for t in range(int(self.sensors.image_sonar_config["AzimuthBins"])):
                for p in range(int(self.sensors.image_sonar_config["Elevation"])):
                    self.gt_matrix[t][p]=state[(f"{t} {p}")]
                    
            theta, phi = self.gt_matrix.shape
            #print(self.gt_matrix.shape)
            for t in range(theta):
                for p in range(phi):
                    r_index = int(math.floor(((self.gt_matrix[t][p]-(self.sensors.image_sonar_config["RangeMin"]))*self.sensors.image_sonar_config["RangeBins"])/self.sensors.image_sonar_config["RangeMax"]))
                    #print(r_index,t)
                    gt_image[r_index][t]=p
                    
            image=(gt_image).astype(np.uint8)
            cartesian_gt_image=Image.fromarray(image, mode='L').rotate(180)
            cartesian_gt_image.save((f"{self.root_folder}/{self.files_folder}/GT-images/{self.mission}-{self.counter}.png"),format='PNG')
            np.save(str(self.mission)+"-"+str(self.counter)+'.npy',self.gt_matrix)
            os.system('mv '+str(self.mission)+"-"+str(self.counter)+'.npy'+' '+self.root_folder+'/'+self.files_folder+'/'+self.gt_folder)       
        #print(self.gt_matrix)

    def updateState(self,state)->None:
        if 'LocationSensor' in state:    
            self.sonar_image=(state['ImagingSonar'])
            #if self.reachedWaypoint():
            self.updateSonarImage()
                #self.updateRGBDImage(state)
            self.saveSonarRawData()
            self.saveCartesianImage()
            self.saveSonarGT(state)
            self.saveMetaDataFile()
            self.saveState(state)
            self.counter+=1
            self.actual_location=(state['LocationSensor'])
            self.actual_rotation=(state['RotationSensor'])
                #return self.counter
        #if 'LocationSensor' in state[self.name]:
        #    self.actual_location=(state[self.name]['LocationSensor'])
        #if 'RotationSensor' in state[self.name]:
        #    self.actual_rotation=(state[self.name]['RotationSensor'])
        
        #self.calculateVelocities()

    def createWaypoints(self, end_z)->None:
        if self.mission==1:
            angles=np.linspace(0,350,36)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            elevation=np.arange(-2.5,end_z,0.3 )
            for z in elevation:
                for angle, heading in zip(angles,headings):
                    x=2*np.cos(np.deg2rad(angle))+self.start_location[0]
                    y=2*np.sin(np.deg2rad(angle))+self.start_location[1]
                    self.waypoints.append([x,y,z,0,0,heading])
            self.number_of_waypoints=len(self.waypoints)
            self.actual_waypoint=self.waypoints[0]
        
        if self.mission==2:
            angles=np.linspace(0,350,36)
            radious=np.linspace(2,1,3)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            for r in radious:
                for angle, heading in zip(angles,headings):
                    x=r*np.cos(np.deg2rad(angle))+self.start_location[0]
                    y=r*np.sin(np.deg2rad(angle))+self.start_location[1]
                    self.waypoints.append([x,y,end_z+1,0,0,heading])
            
            self.number_of_waypoints=len(self.waypoints)
            self.actual_waypoint=self.waypoints[0]

        if self.mission==3:
            elevation=np.arange(-2.5,end_z,0.3 )
            for i,z in enumerate(elevation):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=2*np.cos(np.deg2rad(angle))+self.start_location[0]
                        y=2*np.sin(np.deg2rad(angle))+self.start_location[1]
                        self.waypoints.append([x,y,z,0,0,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=2*np.cos(np.deg2rad(angle))+self.start_location[0]
                        y=2*np.sin(np.deg2rad(angle))+self.start_location[1]
                        self.waypoints.append([x,y,z,0,0,heading])       
            self.number_of_waypoints=len(self.waypoints)
            self.actual_waypoint=self.waypoints[0]

        if self.mission==4:
            radious=np.linspace(2,1,3)
            for i,r in enumerate(radious):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=r*np.cos(np.deg2rad(angle))+self.start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+self.start_location[1]
                        self.waypoints.append([x,y,end_z+1,0,0,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=r*np.cos(np.deg2rad(angle))+self.start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+self.start_location[1]
                        self.waypoints.append([x,y,end_z+1,0,0,heading])       
            self.number_of_waypoints=len(self.waypoints)
            self.actual_waypoint=self.waypoints[0]
        
        self.agent["location"]=self.actual_waypoint[0:3]
        self.agent["rotation"]=self.actual_waypoint[3:]

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
       
        position_error_x = self.actual_waypoint[0] - self.actual_location[0]
        position_error_y = self.actual_waypoint[1] - self.actual_location[1]
        position_error_z = self.actual_waypoint[2] - self.actual_location[2]

        desired_x_velocity = self.pid_controller_x.update(np.linalg.norm(position_error_x), self.dt)
        desired_y_velocity = self.pid_controller_y.update(np.linalg.norm(position_error_y), self.dt)
        desired_z_velocity = self.pid_controller_z.update(np.linalg.norm(position_error_z), self.dt)

        desired_linear_velocity=[desired_x_velocity,desired_y_velocity,desired_z_velocity]
        position_error=[position_error_x,position_error_y,position_error_z]

        #desired_linear_velocity = self.pid_controller_linear.update(np.linalg.norm(position_error), self.dt)
        linear_velocity = position_error / np.linalg.norm(position_error) * desired_linear_velocity

        erro_orientacao = self.actual_waypoint[3:] - self.actual_rotation

        erro_orientacao[erro_orientacao > 180] -= 360
        erro_orientacao[erro_orientacao < -180] += 360

        desired_angular_velocity = self.pid_controller_angular.update(np.linalg.norm(erro_orientacao), self.dt)
        angular_velocity = erro_orientacao / np.linalg.norm(erro_orientacao) * desired_angular_velocity
        angular_velocity=[0,0,angular_velocity[2]]
        self.command = np.concatenate((linear_velocity, angular_velocity), axis=None)
        #print(self.command)
        #self.command = [0,0,-0.3,0,0,0]
        
        #self.command=self.actual_waypoint

    def fineshedMission(self)->bool:
        if self.reached_waypoints-1>self.number_of_waypoints:
            self.command=[0]
            plt.close('all')
            return True
        else:
            return False