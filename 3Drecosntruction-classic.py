import numpy as np
from PIL import Image
import json
import tqdm
import csv

def matrix2xyz(matrix,output_xyz_filepath,index,mission,auv,mission_metadata):
        sonar_configuration = json.load(open('sonar-configuration.json'))
        sonar_model=sonar_configuration["P900"]
        auv_metadata=json.load(open(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission}-P900/auv-{auv}/Meta-data/{index}.json"))
        radius, theta = matrix.shape
        max_value = np.max(matrix)
        try:
            with open(output_xyz_filepath, 'a') as outfile:
                for t in range(theta):
                    for r in range(radius):
                        if matrix[r][t]>=max_value*0.95:
                            
                            rad = (r*sonar_model["RangeMax"])/sonar_model["RangeBins"] + sonar_model["RangeMin"]
                            phi_= (0)
                            theta_= ((t*(sonar_model["Azimuth"])/theta)-sonar_model["Azimuth"]/2) + auv_metadata["yaw"]
                            
                            x=(rad*np.cos(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["x"]-float(mission_metadata[2]))
                            y=(rad*np.sin(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["y"]+float(mission_metadata[3]))
                            z=(rad*np.sin(np.deg2rad(phi_))) + auv_metadata["z"]
                            outfile.write(f"{x} {y} {z}\n")
        except:
            pass

m=1
auv=12

with open(f"mission{m}.csv", newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)

mission=mission_metadata[auv]
output_xyz_filepath=(f"classic-{m}-auv-{auv}.xyz")
#image_file_path = "teste.png"
for i in tqdm.tqdm(range(int(mission[-1])-1)):
    #image_file_path = (f"/home/guilherme/Documents/Holoocean-imaging-sonar/experiments-elevatenet/{m}-{auv}/{i}.png")
    matrix=np.load((f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{m}-P900/auv-{auv}/Raw-data/{i}.npy"))
    matrix2xyz(matrix=matrix,output_xyz_filepath=output_xyz_filepath,index=i,mission=m,auv=auv,mission_metadata=mission)
# Example usage:
#image_file_path = "your_image.png"  # Replace with your image file path
#grayscale_matrix = png_to_grayscale_numpy(image_file_path)

