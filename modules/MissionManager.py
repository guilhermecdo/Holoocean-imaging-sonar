import holoocean
import holoocean.agents
import holoocean.exceptions
import holoocean.holooceanclient
import modules.holoOceanUtils
import numpy as np
import os
import json

class mission():
    def __init__(self,mission_data:list,mission_id:int,sonar:str):
        self.mission_id=mission_id
        self.mission_data=mission_data
        self.sonar_model=sonar
        
        self.mission_waypoints=[]
        self.number_of_waypoints:int=0
        self.reached_waypoints:int=0
        self.actual_waypoint=[]
        self.distance_tresh_hold=0.1
        self.angle_tresh_hold=2.0

    def createWaypoints(self)->None:

        start_location=[float(self.mission_data[2]),-1*float(self.mission_data[3]),float(self.mission_data[4])]
        end_z=(float(self.mission_data[4])+float(self.mission_data[5]))
        pitchs=[0]

        if self.mission_id==1:
            angles=np.linspace(0,350,36)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            elevation=np.arange(-2.5,end_z,0.3 )
            
            for z in elevation:
                for angle, heading in zip(angles,headings):
                    for pitch in pitchs:
                        x=2*np.cos(np.deg2rad(angle))+start_location[0]
                        y=2*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,z,0,pitch,heading])
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]
        
        if self.mission_id==2:
            
            angles=np.linspace(0,350,36)
            radious=np.linspace(2,1,3)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            for r in radious:
                for angle, heading in zip(angles,headings):
                    for pitch in pitchs:
                        x=r*np.cos(np.deg2rad(angle))+start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,end_z+1,0,pitch,heading])
            
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

        if self.mission_id==3:
            elevation=np.arange(-2.5,end_z,0.3 )
            
            for i,z in enumerate(elevation):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        for pitch in pitchs:
                            x=2*np.cos(np.deg2rad(angle))+start_location[0]
                            y=2*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,z,0,pitch,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        for pitch in pitchs:
                            x=2*np.cos(np.deg2rad(angle))+start_location[0]
                            y=2*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,z,0,pitch,heading])       
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

        if self.mission_id==4:
            radious=np.linspace(2,1,3)
            
            for i,r in enumerate(radious):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        for pitch in pitchs:
                            x=r*np.cos(np.deg2rad(angle))+start_location[0]
                            y=r*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,end_z+1,0,pitch,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        for pitch in pitchs:
                            x=r*np.cos(np.deg2rad(angle))+start_location[0]
                            y=r*np.sin(np.deg2rad(angle))+start_location[1]
                            self.mission_waypoints.append([x,y,end_z+1,0,pitch,heading])       
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]
        
        if self.mission_id>=5:
            
            angles=np.linspace(0,350,36)
            radious=np.linspace(4.5,3,3)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            for r in radious:
                for angle, heading in zip(angles,headings):
                    for pitch in pitchs:
                        x=r*np.cos(np.deg2rad(angle))+start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,end_z+1,0,pitch,heading])
            
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

    def saveState(self,auv):
        self.mission_waypoints=auv.waypoints[auv.reached_waypoints:]

    def start(self):
        data=self.mission_data
        mission_id=self.mission_id
        self.createWaypoints()

        scenario=modules.holoOceanUtils.scenario("ExampleLevel-1","ExampleLevel","UM",10)

        auv=modules.holoOceanUtils.AUV(id=str(data[0]),location=self.actual_waypoint[0:3],rotation=self.actual_waypoint[3:],mission=mission_id,waypoints=self.mission_waypoints,sonar_model=self.sonar_model)
        #auv.reached_waypoints=self.reached_waypoints
        sonar_configuration = json.load(open('sonar-configuration.json'))
        
        sonar_model=sonar_configuration[self.sonar_model]

        if mission_id == 1 or mission_id == 3: 

            auv.addSonarImaging(configuration=sonar_model,rotation=[0,0,0])
            auv.addSensor("PoseSensor","Origin",[0,0,0])
            #auv.addRGBDCamera([0,0,0])
            auv.addSonarGT([0,0,0])
        
        elif mission_id == 5:
            auv.addSonarImaging(configuration=sonar_model,rotation=[0,45,0])
            auv.addSensor("PoseSensor","Origin",[0,45,0])
            #auv.addRGBDCamera([0,0,0])
            auv.addSonarGT([0,45,0])

        elif mission_id == 6:
            auv.addSonarImaging(configuration=sonar_model,rotation=[0,30,0])
            auv.addSensor("PoseSensor","Origin",[0,30,0])
            #auv.addRGBDCamera([0,0,0])
            auv.addSonarGT([0,30,0])

        elif mission_id == 7:
            auv.addSonarImaging(configuration=sonar_model,rotation=[0,15,0])
            auv.addSensor("PoseSensor","Origin",[0,15,0])
            #auv.addRGBDCamera([0,0,0])
            auv.addSonarGT([0,15,0])

        else:
            auv.addSonarImaging(configuration=sonar_model,rotation=[0,45,0])
            auv.addSensor("PoseSensor","Origin",[0,45,0])
            #auv.addRGBDCamera([0,45,0])
            auv.addSonarGT([0,45,0])

        auv.addSensor("LocationSensor","Origin")
        auv.addSensor("RotationSensor","Origin")
        
        auv.imageViwer()
        scenario.addAgent(auv.agent)

        with open("Config.json",'w') as fp:
            json.dump(scenario.cfg, fp)
            os.system('mv '+'Config.json'+' '+auv.root_folder+'/'+auv.files_folder)
        
        env=holoocean.make(scenario_cfg=scenario.cfg,verbose=False)

        env.reset
        
        for l in self.mission_waypoints:
            env.draw_point([l[0], l[1], l[2]],[0,255,0], lifetime=0)
        
        #start Simulation

        env.move_viewport([float(data[2]),-1*float(data[3]),(float(data[4]))+8],[0,0,180])
        state=env.tick()
        auv.updateState(state)
        #counter=0
        #env.agents[auv.name].teleport(location=self.mission_waypoints[0][0:3],rotation=self.mission_waypoints[0][3:])
        #while not auv.fineshedMission():
        while auv.counter < len(self.mission_waypoints):
            state=env.tick()
            auv.updateState(state)
            if auv.counter < len(self.mission_waypoints):
                env.agents[auv.name].teleport(location=self.mission_waypoints[auv.counter][0:3],rotation=self.mission_waypoints[auv.counter][3:])
                #env.act(auv.name,auv.command)
                env.act(auv.name,[0,0])
            else:
                break


        print("Finished Mission "+data[0])
        os.system("killall -e Holodeck")