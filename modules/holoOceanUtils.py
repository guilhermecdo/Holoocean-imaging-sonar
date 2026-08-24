from turtle import width

import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import json
import os
import pickle
import math
from scipy.spatial.transform import Rotation as R


class scenario:
    def __init__(self,name:str,world:str,package_name:str,ticks_per_sec:int) -> None:
        
        self.cfg={
            "name": name,
            "world": world,
            "package_name":package_name,
            "ticks_per_sec": ticks_per_sec,
            "frames_per_sec": True,
            "octree_min": 0.02,
            "octree_max": 0.2,
            "agents":[],
            "window_width":  640,
            "window_height": 480
        }
    def addAgent(self, agent)->None:
        self.cfg["agents"].append(agent) 
        pass
        
class AUV:
    def __init__(self,id:str,control_scheme:int=2,location=[float,float,float],rotation=[int,int,int],mission=1,waypoints=[],sonar_model:str="", root_folder:str="SEE")->None:
        
        self.files_folder=root_folder
        self.pkl_folder='states'
        self.rgbd_image_folder='RGBD-images'

        self.sonar_image_folder='sonar-images'
        self.sonar_image_blur_folder='sonar-images-blur'
        self.sonar_image_gauss_blur_folder='sonar-images-gauss-blur'
        self.sonar_image_denoise_folder='sonar-images-denoise'

        self.polar_image_folder='polar-images'
        self.raw_data_folder='raw-data'
        self.meta_data_folder='Meta-data'
        self.root_folder="Sonar-Dataset-mission-"+str(mission)+"-"+sonar_model
        self.gt_folder="GT-folder"

        self.lidar_data_local_folder="lidar-plc-local"
        self.lidar_data_world_folder="lidar-plc-world"
        self.lidar_image_folder="lidar-images"
        self.raw_lidar_data_folder="lidar-raw-data"

        self.pose_data_folder="poses"
        self.normalized_pcl_folder="normalized-point-clouds"
        self.rgb_camera_folder="rgb-images"


        self.meta_data_file_name:str
        self.raw_sonar_data_file_name:str
        self.cartesian_image_file_name:str
        self.polar_image_file_name:str
        self.mission=mission

        self.base_filename=(f"{sonar_model}-{mission}")

        self.sonar_name=sonar_model
        
        #create root SEE folder
        os.makedirs(f"{self.files_folder}", exist_ok=True)

        #create sonar images folders
        os.makedirs(f"{self.files_folder}/{self.sonar_image_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.sonar_image_blur_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.sonar_image_gauss_blur_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.sonar_image_denoise_folder}", exist_ok=True)

        #create folder for lidar data
        os.makedirs(f"{self.files_folder}/{self.lidar_data_local_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.lidar_data_world_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.lidar_image_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.normalized_pcl_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.raw_lidar_data_folder}", exist_ok=True)


        #create folder for camera data
        os.makedirs(f"{self.files_folder}/{self.rgb_camera_folder}", exist_ok=True)

        #create folder for pose GT
        os.makedirs(f"{self.files_folder}/{self.pose_data_folder}", exist_ok=True)

        os.makedirs(f"{self.files_folder}/{self.pkl_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.polar_image_folder}", exist_ok=True)
        os.makedirs(f"{self.files_folder}/{self.meta_data_folder}", exist_ok=True)
        

        self.id=id
        self.name=f"auv-{id}"
        self.type="HoveringAUV"
        self.control_scheme=control_scheme
        self.start_location=location
        self.start_rotation=rotation
        self.image_sonar_config:dict

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

        self.sonar_image=None
        self.actual_location=location
        self.actual_rotation=rotation

        self.counter=0

    def addSensor(self,sensor:str,socket:str,rotation:list=[0,0,0])->None:
        self.agent["sensors"].append({"sensor_type":sensor,
                                    "socket": socket,
                                    "rotation":rotation})
        self.number_of_sensors+=1
    
    def addSonarGT(self,rotation)->None:
        self.gt_matrix=np.zeros(shape=(int(self.image_sonar_config["AzimuthBins"] ), int(self.image_sonar_config["Elevation"])))
        for t in range(int(self.image_sonar_config["AzimuthBins"])):
            #for p in range(int(self.image_sonar_config["Elevation"])):
            for p in range(int(self.image_sonar_config["AzimuthBins"])):
                rotation=[0,rotation[1],(t*(self.image_sonar_config["Azimuth"]/self.image_sonar_config["AzimuthBins"]))-self.image_sonar_config["Azimuth"]/2]
                self.agent["sensors"].append({"sensor_type":"RangeFinderSensor",
                                                "sensor_name":(f"{t} {p}"),
                                                "socket": "Origin",
                                                "location": [self.image_sonar_config["RangeMin"],0,0],
                                                "rotation":rotation,
                                                "configuration":{
                                                    "LaserMaxDistance": self.image_sonar_config["RangeMax"]-self.image_sonar_config["RangeMin"],
                                                    "LaserCount": 1,
                                                    #"LaserAngle":p-int(self.image_sonar_config["Elevation"])/2,
                                                    "LaserAngle":(p*(self.image_sonar_config["Elevation"]/self.image_sonar_config["AzimuthBins"]))-int(self.image_sonar_config["Elevation"])/2,
                                                    "LaserDebug": True,
                                                }
                                            })

    def addRaycastlidar(self,rotation)->None:
        self.agent["sensors"].append({"sensor_type":"RaycastSemanticLidar",
                                    "socket": "Origin",
                                    #"location": [self.image_sonar_config["RangeMin"],0,0],
                                    "rotation":rotation,
                                    "configuration":{
                                        "Range": self.image_sonar_config["RangeMax"],
                                        "Channels":48,
                                        "PointsPerSecond": 48*960,
                                        "RotationFrequency": 10,
                                        "UpperFovLimit": 14.4,
                                        "LowerFovLimit": -14.4,
                                        "HorizontalFov": 28.8,
                                        "AtmospAttenRate": 0.0,
                                        "RandomSeed": 0,
                                        "DropOffGenRate": 0.0,
                                        "DropOffIntensityLimit": 0.0,
                                        "DropOffAtZeroIntensity": 0.0,
                                        "ShowDebugPoints": True,
                                        "NoiseStdDev": 0.0
                                    },
                                    "Hz": 10 #TicksPerCapture
                                })

    def addRGBcamera(self,rotation)->None:

        camera_config={
            "CaptureWidth":720,
            "CaptureHeight":720,
            "FovAngle":90, 
            "ExposureCompensation": 2,
            "TargetGamma":0.3,
            "ExposureMethod":"AEM_Basic",
 
        }

        self.agent["sensors"].append({"sensor_type":"CameraSensor",
                                    "socket": "Origin",
                                    "location": [0.8,0,0],
                                    "rotation":rotation,

                                    "configuration":camera_config
        })

    def addRGBDCamera(self,rotation)->None:
        
        CaptureHeight=((np.tan(np.deg2rad(self.image_sonar_config["Elevation"]/2))*2*self.image_sonar_config["RangeMax"]) /
                        ((np.tan(np.deg2rad(self.image_sonar_config["Azimuth"]/2))*2*self.image_sonar_config["RangeMax"])/
                         self.image_sonar_config["AzimuthBins"]))
        FovAngle=np.arctan((np.deg2rad(self.image_sonar_config["Azimuth"]/2))/(np.tan(np.deg2rad(self.image_sonar_config["Elevation"]/2))))

        self.depth_image=np.zeros(shape=(int(CaptureHeight),self.image_sonar_config["AzimuthBins"],1))

        self.agent["sensors"].append({"sensor_type":"RGBDCamera",
                                    "socket": "CameraSocket",
                                    "rotation":rotation,
                                    "configuration":{
                                        "CaptureWidth":self.image_sonar_config["AzimuthBins"],
                                        "CaptureHeight":int(CaptureHeight),
                                        "FovAngle":np.rad2deg(FovAngle),
                                        #"MaxViewDistanceOverride":self.image_sonar_config["RangeMax"]*100,
                                        #"ShowDebugPoints":True,
                                        "convertToDistance":True,
                                        #"ViewRegion": True,
                                    }})
        
    def addSonarImaging(self,configuration:dict=None,rotation:list=[0,0,0],hz=10,name:str="")->None:
        self.image_sonar_config=configuration

        #return 0
        # self.agent["sensors"].append({"sensor_type":"ImagingSonar",
        #                               "sensor_name":name,
        #                             "socket": "Origin",
        #                             "rotation":rotation,
        #                             #location":[self.actual_location[0]/100,self.actual_location[1]/100,self.actual_location[2]/100],
        #                             "Hz": hz,
        #                             "configuration":{}
        #                             })
        
        # self.sonar_ID=self.number_of_sensors
        # self.agent["sensors"][self.sonar_ID]["configuration"]=configuration
        # self.image_sonar_config=configuration
        # self.number_of_sensors+=1

    def imageViwer(self)->None:    
        config = self.image_sonar_config
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
        self.polar_image_file_name=(f"{self.files_folder}/{self.polar_image_folder}/{self.id}-{self.counter}.png")
        s = self.sonar_image
        self.plot.set_array(s.ravel())
        self.fig_sonar.canvas.draw()
        
        self.fig_sonar.canvas.flush_events()

        self.fig_sonar.savefig(self.polar_image_file_name)
        #os.system('mv '+self.polar_image_file_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.polar_image_folder)
         
    def saveCartesianImage(self,state)->None:

        cartesian_image_file_name=(f"{self.files_folder}/{self.sonar_image_folder}/{self.base_filename}-{self.counter}.png")
        blur_image_name=(f"{self.files_folder}/{self.sonar_image_blur_folder}/{self.base_filename}-{self.counter}.png")
        gaus_blur_image_name=(f"{self.files_folder}/{self.sonar_image_gauss_blur_folder}/{self.base_filename}-{self.counter}.png")

        denoise_image_name=(f"{self.files_folder}/{self.sonar_image_denoise_folder}/{self.base_filename}-{self.counter}.png")

        denoise_image=((state["denoise"])*255).astype(np.uint8)

        denoise_fliped=cv2.flip(denoise_image,-1)
        
        image=(self.sonar_image*255).astype(np.uint8)
        
        # Vertically flip the raw image
        image_flipped = cv2.flip(image, -1)

        #blur images for more data and diferent types of noise:

        blur_image_gauss=cv2.GaussianBlur(image_flipped, (3, 3), 0)
        blur_image=cv2.blur(image_flipped, (3, 3), 0)


        # Save the flipped images
        cv2.imwrite(cartesian_image_file_name, image_flipped)
        cv2.imwrite(blur_image_name, blur_image)
        cv2.imwrite(gaus_blur_image_name, blur_image_gauss)
        cv2.imwrite(denoise_image_name,denoise_fliped)
    
    def saveSonarRawData(self)->None:
        self.raw_sonar_data_file_name=(f"{self.files_folder}/{self.raw_data_folder}/{self.id}-{self.counter}.npy")
        #np.save(self.raw_sonar_data_file_name,self.sonar_image)
        #os.system('mv '+self.raw_sonar_data_file_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.raw_data_folder)

    def saveMetaDataFile(self)->None:
        
        sonar_specs=self.image_sonar_config
        sonar_data={
            "AUV_ID":str(self.id),
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
        self.meta_data_file_name=(f"{self.files_folder}/{self.meta_data_folder}/{self.base_filename}-{self.counter}.json")
        #sonar_data = json.dumps(sonar_data,indent=len(sonar_data))
        with open(self.meta_data_file_name,'w') as fp:
            json.dump(sonar_data, fp)
        
        #os.system('mv '+self.meta_data_file_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.meta_data_folder)

    def saveState(self,state)->None:
        self.pkl_file_name=(f"{self.files_folder}/{self.pkl_folder}/{self.base_filename}-{self.counter}.pkl")
        pose_file_name=(f"{self.files_folder}/{self.pose_data_folder}/{self.base_filename}-{self.counter}.npy")

        pose_matrix = np.array(state['PoseSensor'], dtype=np.float32)
        np.save(pose_file_name, pose_matrix)



        with open(self.pkl_file_name, 'wb') as file:  
            pickle.dump(state, file)
            #os.system('mv '+self.pkl_file_name+' '+self.root_folder+'/'+self.files_folder+'/'+self.pkl_folder)
    
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
        
        with open(str(self.id)+"-"+str(self.counter)+'.pkl', 'wb') as file:  
            pickle.dump(pixels, file)
            os.system('mv '+str(self.id)+"-"+str(self.counter)+'.pkl'+' '+self.root_folder+'/'+self.files_folder+'/'+self.rgbd_image_folder)
    
    def saveSonarGT(self,state)->None:
        gt_image=np.zeros(shape=(self.image_sonar_config["RangeBins"],self.image_sonar_config["AzimuthBins"]))
        #print(gt_image.shape)
        if '0 0' in state:
            for t in range(int(self.image_sonar_config["AzimuthBins"])):
                for p in range(int(self.image_sonar_config["Elevation"])):
                    self.gt_matrix[t][p]=state[(f"{t} {p}")]
                    
            theta, phi = self.gt_matrix.shape
            #print(self.gt_matrix.shape)
            for t in range(theta):
                for p in range(phi):
                    r_index = int(math.floor(((self.gt_matrix[t][p]-(self.image_sonar_config["RangeMin"]))*self.image_sonar_config["RangeBins"])/self.image_sonar_config["RangeMax"]))
                    #print(r_index,t)
                    gt_image[r_index][t]=p
                    
            image=(gt_image).astype(np.uint8)
            cartesian_gt_image=Image.fromarray(image, mode='L').rotate(180)
            cartesian_gt_image.save((f"{self.root_folder}/{self.files_folder}/GT-images/{self.id}-{self.counter}.png"),format='PNG')
            np.save(str(self.id)+"-"+str(self.counter)+'.npy',self.gt_matrix)
            os.system('mv '+str(self.id)+"-"+str(self.counter)+'.npy'+' '+self.root_folder+'/'+self.files_folder+'/'+self.gt_folder)       
        #print(self.gt_matrix)

    def saveRaycastLidar(self,state)->None:
        if 'RaycastSemanticLidar' in state:

            xyz_data = state['RaycastSemanticLidar'][:, :3]

            raw_sonar_data_file_name=(f"{self.files_folder}/{self.raw_lidar_data_folder}/{self.base_filename}-{self.counter}.npy")
            raw_sonar_data=state['RaycastSemanticLidar']
            np.save(raw_sonar_data_file_name,raw_sonar_data)

            points=xyz_data
            num_points = points.shape[0]
            if num_points == 0:
                return None
            if num_points > 1024:
                indices = np.random.choice(num_points, 1024, replace=False)
                points = points[indices]
            elif num_points < 1024:
                indices = np.random.choice(num_points, 1024, replace=True)
                points = points[indices]
            
            np.save((f"{self.files_folder}/{self.normalized_pcl_folder}/{self.base_filename}-{self.counter}.npy"),points)


            np.savetxt(f"{self.files_folder}/{self.lidar_data_local_folder}/{self.base_filename}-{self.counter}.xyz", xyz_data, fmt="%.6f", delimiter=" ")

            sensor_position = state['LocationSensor']
            sensor_orientation = state['RotationSensor']
            
            rotation = R.from_euler('xyz', sensor_orientation, degrees=True)
            rotation_matrix = rotation.as_matrix()

            world_points = (xyz_data @ rotation_matrix.T) + sensor_position


            np.savetxt(f"{self.files_folder}/{self.lidar_data_world_folder}/{self.base_filename}-{self.counter}.xyz", world_points, fmt="%.6f", delimiter=" ")

            self.point_cloud_to_restricted_spherical_image(state)

            #self.lidar_to_polar_tag_image(state['RaycastSemanticLidar'])

    def point_cloud_to_restricted_spherical_image(self,state)->None:

        h_fov_deg=self.image_sonar_config["Azimuth"]
        v_fov_deg=self.image_sonar_config["Elevation"]
        
        # 1. Convert FOV degrees to radians
        h_fov_rad = np.radians(h_fov_deg)
        v_fov_rad = np.radians(v_fov_deg)

        width=int(self.image_sonar_config["AzimuthBins"])
        height=int(self.image_sonar_config["AzimuthBins"]*v_fov_deg/h_fov_deg)
        
        # 2. Extract local x, y, z coordinates
        x = state['RaycastSemanticLidar'][:, 0]
        y = state['RaycastSemanticLidar'][:, 1]
        z = state['RaycastSemanticLidar'][:, 2]

        # x=world_points[:, 0]
        # y=world_points[:, 1]
        # z=world_points[:, 2]
    
        # # 3. Calculate spherical coordinates
        r = np.sqrt(x**2 + y**2 + z**2)
        theta = np.arctan2(y, x)         # Azimuth (Horizontal)
        phi = np.arcsin(z / (r + 1e-6))  # Elevation (Vertical)
    
        # 4. Auto-Center the window on your points
        # We find the median angle of your actual data to center the FOV
        theta_center = np.median(theta)
        phi_center = np.median(phi)
    
        # Define dynamic boundaries around that center
        h_bound_min = theta_center - (h_fov_rad / 2.0)
        h_bound_max = theta_center + (h_fov_rad / 2.0)
        v_bound_min = phi_center - (v_fov_rad / 2.0)
        v_bound_max = phi_center + (v_fov_rad / 2.0)
    
        # 5. Filter points using the auto-centered mask
        valid_mask = (theta >= h_bound_min) & (theta <= h_bound_max) & (phi >= v_bound_min) & (phi <= v_bound_max)
    
        if not np.any(valid_mask):
            print("Warning: No points fit inside the calculated FOV window!")
            cv2.imwrite(f"{self.files_folder}/{self.lidar_image_folder}/{self.base_filename}-{self.counter}.png", np.zeros((height, width), dtype=np.uint8))
            return False
        
        r = r[valid_mask]
        theta = theta[valid_mask]
        phi = phi[valid_mask]
    
        # 6. Map Theta and Phi to image pixels [0, 1] relative to the new window boundaries
        u = np.floor((theta - h_bound_min) / h_fov_rad * (width - 1)).astype(int)
        v = np.floor((phi - v_bound_min) / v_fov_rad * (height - 1)).astype(int)
    
        # Flip vertical index so positive elevation points to the top of the image canvas
        v = (height - 1) - v 
    
        # Keep pixel indices strictly inside the image bounds
        u = np.clip(u, 0, width - 1)
        v = np.clip(v, 0, height - 1)
    
        # 7. Normalize Radius to 0-255 grayscale
        r_min, r_max = r.min(), r.max()
        if r_max - r_min > 1e-6:
            r_normalized = 255 * (r - r_min) / (r_max - r_min)
        else:
            r_normalized = np.zeros_like(r)
        r_grayscale = r_normalized.astype(np.uint8)
    
        # 8. Create canvas and project points (handling occlusions)
        image = np.zeros((height, width), dtype=np.uint8)
        sort_indices = np.argsort(r)[::-1]
    
        image[v[sort_indices], u[sort_indices]] = r_grayscale[sort_indices]
        cv2.imwrite(f"{self.files_folder}/{self.lidar_image_folder}/{self.base_filename}-{self.counter}.png", image)

    def lidar_to_polar_tag_image(self,data, width=96, height=512, h_fov_deg=28.8,):
        """
        Converts Semantic LiDAR data into a Polar RGB Image.
        Origin (0,0) is located at the BOTTOM-LEFT of the image.
        """

        max_range=self.image_sonar_config["RangeMax"]

        h_fov_rad = np.radians(h_fov_deg)
        h_bound = h_fov_rad / 2.0
    
        # 1. Extract coordinates and semantic tags
        x = data[:, 0]
        y = data[:, 1]
        object_tags = data[:, 5].astype(np.uint16)

        print(object_tags)
    
        # 2. Calculate 2D range and horizontal angle
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
    
        # 3. Filter points within boundaries
        valid_mask = (r > 0) & (r <= max_range) & (theta >= -h_bound) & (theta <= h_bound)
        if not np.any(valid_mask):
            print("Warning: No lidar points fell inside the polar image boundaries.")
            return np.zeros((height, width, 3), dtype=np.uint8)
        
        r = r[valid_mask]
        theta = theta[valid_mask]
        object_tags = object_tags[valid_mask]
    
        # 4. Map to pixel coordinates
        # Horizontal: Leftmost beam (-h_bound) maps to 0 (Left side)
        u = np.floor((theta + h_bound) / h_fov_rad * (width - 1)).astype(int)
    
        # Vertical: 0 meters maps to 0 (Bottom side)
        # To put (0,0) at the bottom-left, index 0 must represent the bottom row of the matrix.
        # In standard image arrays, the bottom row is index (height - 1). 
        v = np.floor((r / max_range) * (height - 1)).astype(int)
        v_flipped = (height - 1) - v  # Converts 0 meters to the bottom row index
    
        # 5. Handle Tag Colors and File Tracking
        unique_tags = np.unique(object_tags)
        color_dict = self.get_or_create_tag_colors(unique_tags=unique_tags)
    
        # 6. Initialize Canvas (Height, Width, 3 Channels for RGB)
        rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
    
        # 7. Render handling occlusions (Further points rendered first, closer points overwrite them)
        sort_indices = np.argsort(r)[::-1]
    
        for idx in sort_indices:
            px_u = u[idx]
            px_v = v_flipped[idx]
            tag = object_tags[idx]
        
            # INDENTED: Now this runs for EVERY point in the loop
            rgb_image[px_v, px_u] = color_dict[tag]

        # OUTSIDE LOOP: Save the image only AFTER all points are painted
        cv2.imwrite(f"{self.files_folder}/{self.lidar_image_folder}/semantic-{self.id}-{self.counter}.png", rgb_image)

    
       #return polar_image

    def get_or_create_tag_colors(self,unique_tags, map_filepath="tag_color_map.json"):
        """
        Loads an existing tag-to-color mapping from a JSON file.
        If new tags are found, generates a random unique RGB color and saves it.
        """
        # Load existing map or initialize an empty one
        if os.path.exists(map_filepath):
            with open(map_filepath, "r") as f:
                # JSON keys are always strings; convert them back to integers later
                color_map = json.load(f)
        else:
            color_map = {}

        # Background/Empty space (Tag 0) should always be black
        if "0" not in color_map:
            color_map["0"] = [0, 0, 0]

        updated = False
        for tag in unique_tags:
            tag_str = str(int(tag))
            if tag_str not in color_map:
                # Generate a random, bright RGB color (avoiding pure black)
                # We use a seed based on the tag so it's reproducible if the file is lost
                np.random.seed(int(tag) + 42)
                random_color = np.random.randint(50, 256, size=3).tolist()
                color_map[tag_str] = random_color
                updated = True

        # Save the file if we added new tags
        if updated:
            with open(map_filepath, "w") as f:
                json.dump(color_map, f, indent=4)

        # Convert string keys back to integer keys for fast NumPy lookup
        return {int(k): v for k, v in color_map.items()}

    def saveRGBcamera(self,state):
        rgb_image_name=(f"{self.files_folder}/{self.rgb_camera_folder}/{self.base_filename}-{self.counter}.png")
        rgb_image=state["CameraSensor"]
        cv2.imwrite(rgb_image_name,rgb_image)

    def updateState(self,state)->None:
        if 'LocationSensor' in state:    
            #self.sonar_image=(state[self.sonar_name])
            #self.updateSonarImage()
            #self.updateRGBDImage(state)
            #self.saveCartesianImage(state)
            #self.saveMetaDataFile()
            #self.saveState(state)
            self.saveRaycastLidar(state)
            #self.saveRGBcamera(state)
            self.counter+=1
            self.actual_location=(state['LocationSensor'])
            self.actual_rotation=(state['RotationSensor'])