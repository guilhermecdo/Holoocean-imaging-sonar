import holoocean
import modules.holoOceanUtils
import numpy as np
import os
import json
import time

class mission():
    def __init__(self,mission_data:dict,mission_id:int,sonar:str,package:str, world:str, sensor_rotations:list=[0,0,0]) -> None:
        self.mission_id=mission_id
        self.mission_data=mission_data
        self.sonar_model=sonar
        self.package=package
        self.world=world
        self.sensor_rotations=sensor_rotations

        self.mission_waypoints=[]
        self.number_of_waypoints:int=0
        self.reached_waypoints:int=0
        self.actual_waypoint=[]
        self.distance_tresh_hold=0.1
        self.angle_tresh_hold=2.0

    def createWaypoints(self)->None:

        start_location=[self.mission_data["target"]["x"],-1*(self.mission_data["target"]["y"]),self.mission_data["target"]["z"]]
        end_z=(float(self.mission_data["target"]["z"])+float(self.mission_data["target"]["h"]))
        

        if self.mission_id==1:
            pitch=-1*float(self.mission_data["sonar"]["pitch"])
            angles=np.linspace(0,350,36)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            elevation=np.arange(-2.5,end_z,0.3 )
            
            for z in elevation:
                for angle, heading in zip(angles,headings):

                        x=2*np.cos(np.deg2rad(angle))+start_location[0]
                        y=2*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,z,0,pitch,heading])
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]
        
        if self.mission_id==2:
            pitch=-1*float(self.mission_data["sonar"]["pitch"])
            angles=np.linspace(0,350,36)
            radious=np.linspace(2,1,3)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            for r in radious:
                for angle, heading in zip(angles,headings):

                        x=r*np.cos(np.deg2rad(angle))+start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,end_z+1,0,pitch,heading])
            
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

        if self.mission_id==3:
            pitch=-1*float(self.mission_data["sonar"]["pitch"])
            elevation=np.arange(-2.5,end_z,0.3 )
            
            for i,z in enumerate(elevation):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):

                            x=2*np.cos(np.deg2rad(angle))+start_location[0]
                            y=2*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,z,0,pitch,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):

                            x=2*np.cos(np.deg2rad(angle))+start_location[0]
                            y=2*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,z,0,pitch,heading])       
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

        if self.mission_id==4:
            pitch=-1*float(self.mission_data["sonar"]["pitch"])
            radious=np.linspace(2,1,3)
            
            for i,r in enumerate(radious):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):

                            x=r*np.cos(np.deg2rad(angle))+start_location[0]
                            y=r*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,end_z+1,0,pitch,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):

                            x=r*np.cos(np.deg2rad(angle))+start_location[0]
                            y=r*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,end_z+1,0,pitch,heading])       
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]
        
        if self.mission_id==5:
            
            angles=np.linspace(0,350,36)
            radious=np.linspace(4,2,4)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            for r in radious:
                for angle, heading in zip(angles,headings):
                    for p in self.mission_data["sonar"]["pitch"]:
                        x=r*np.cos(np.deg2rad(angle))+start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,0.75,0,p,heading])
            
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]
        
    def generate_orbit_waypoints(self, num_waypoints=36):
        target = self.mission_data["target"]
        initial_sonar = self.mission_data["sonar"]

        offset_deg=90

        t_x, t_y = target["x"], target["y"]
        s_x, s_y, s_z = initial_sonar["x"], initial_sonar["y"], initial_sonar["z"]
    
        # 1. Calculate the exact radius in the XY plane
        radius = np.sqrt((s_x - t_x)**2 + (s_y - t_y)**2)
    
        # 2. Calculate the base mathematical angle from target to sonar
        start_angle = np.arctan2(s_y - t_y, s_x - t_x)
    
        # 3. Generate angles moving in the correct direction for your coordinate system
        # If points felt shifted, we change the rotation direction (-2 * np.pi) 
        # to match a clockwise/navigation frame.
        angles = start_angle - np.linspace(0, 2 * np.pi, num_waypoints, endpoint=False)
    
        # 4. Calculate X and Y positions
        wp_y = t_x + radius * np.cos(angles)
        wp_x = t_y + radius * np.sin(angles)
    
        # Hard lock WP 1 to the exact initial position to prevent floating point shift
        wp_x[0], wp_y[0] = s_x, s_y
    
        # 5. Broadcast constant Z, roll, and pitch values
        wp_z = np.full(num_waypoints, s_z)
        roll = np.full(num_waypoints, initial_sonar["roll"])
        pitch = np.full(num_waypoints, initial_sonar["pitch"])
    
        # 6. Calculate headings relative to your 90-degree starting yaw frame
        # Every step around the circle shifts the required yaw by the step angle
        step_angles_deg = np.linspace(0, 360, num_waypoints, endpoint=False)
    
        # Start at 90 deg, and step sequentially around the circle
        heading_deg = (90.0 + step_angles_deg) % 360
    
        # 7. Stack arrays column-wise
        waypoints_matrix = np.column_stack((wp_x, wp_y, wp_z, roll, pitch, heading_deg))
    
        # Round positions to 4 decimals, orientations to 2 decimals
        waypoints_matrix = np.round(waypoints_matrix, decimals=4)
        waypoints_matrix[:, 3:] = np.round(waypoints_matrix[:, 3:], decimals=2)

        self.mission_waypoints=waypoints_matrix.tolist()    
        self.number_of_waypoints=len(self.mission_waypoints)
        self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]
        
    def saveState(self,auv):
        self.mission_waypoints=auv.waypoints[auv.reached_waypoints:]

    def start(self):

        if "name" in self.mission_data.keys():
            mission_id=(f"{self.mission_id}-{self.mission_data['name']}")
            self.createWaypoints()
            print(f"waypoints:{self.actual_waypoint}")
        else:
            mission_id=self.package
            self.generate_orbit_waypoints(36)

        scenario=modules.holoOceanUtils.scenario("ExampleLevel1",self.package,self.world,10)

        auv=modules.holoOceanUtils.AUV(id=mission_id,location=self.actual_waypoint[0:3],rotation=self.actual_waypoint[3:],mission=mission_id,waypoints=self.mission_waypoints,sonar_model=self.sonar_model,root_folder=self.world)
        
        sonar_configuration = json.load(open('sonar-configuration.json'))
        
        sonar_model=sonar_configuration[self.sonar_model]
        sonar_model_denoise=sonar_configuration[f"{self.sonar_model}-denoise"]

        auv.addSonarImaging(configuration=sonar_model,rotation=self.sensor_rotations,name=self.sonar_model)
        #auv.addSonarImaging(configuration=sonar_model_denoise,rotation=self.sensor_rotations,name="denoise")

        #auv.addRGBcamera(self.sensor_rotations)

        auv.addSensor("PoseSensor","Origin",self.sensor_rotations)
        
        auv.addSensor("LocationSensor","Origin")
        auv.addSensor("RotationSensor","Origin")
        
        auv.addRaycastlidar(self.sensor_rotations)

        #auv.imageViwer()
        scenario.addAgent(auv.agent)

        # with open("Config.json",'w') as fp:
        #     json.dump(scenario.cfg, fp)
        #     os.system('mv '+'Config.json'+' '+auv.root_folder+'/'+auv.files_folder)
        
        env=holoocean.make(scenario_cfg=scenario.cfg,verbose=False, show_viewport=True)
        env.set_render_quality(1)
        env.reset
        
        # for l in self.mission_waypoints:
        #     env.draw_point([l[0], l[1], l[2]],[0,255,0], lifetime=0)
        
        #start Simulation

        env.move_viewport([self.mission_data["target"]["x"],-1*self.mission_data["target"]["y"],self.mission_data["target"]["z"]+8],[0,0,180])
        state=env.tick()
        auv.updateState(state)


        while auv.counter < len(self.mission_waypoints):
            state=env.tick()
            time.sleep(1)
            auv.updateState(state)
            if auv.counter < len(self.mission_waypoints):
               env.agents[auv.name].teleport(location=self.mission_waypoints[auv.counter][0:3],rotation=self.mission_waypoints[auv.counter][3:])
               env.act(auv.name,[0,0,0,0,0,0,0,0])
            else:
               break
        
        #auv.fineshedMission()
        #print("Finished Mission "+data[0])
        os.system("killall -e Holodeck")