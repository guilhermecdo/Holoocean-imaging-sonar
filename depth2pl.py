import pickle
import numpy as np
import tqdm
import csv
import os

def pkl_to_xyz(pkl_filepath, output_xyz_filepath):
    """
    Converts a .pkl file (tuple of shape (H, W, 5) representing RGBD) to a .xyz point cloud.

    Args:
        pkl_filepath: Path to the input .pkl file.
        output_xyz_filepath: Path to save the .xyz point cloud.
    """
    try:
        with open(pkl_filepath, 'rb') as f:
            rgbd_data = pickle.load(f)

        if  rgbd_data.shape[2] == 5:  # Check shape
            image_data = rgbd_data
            height, width, _ = image_data.shape
            rgb_image = image_data[:, :, :3]  # RGB channels
            depth_image = image_data[:, :, 4]  # Depth channel (5th)
            # The 5th channel is ignored in this version. If it's important, let me know.

        else:
            raise ValueError("Unsupported data format. Expected a tuple of shape (H, W, 5).")

        fx = fy = 1000  # Replace with your camera's focal length.  CRUCIAL!
        cx = width / 2
        cy = height / 2

        with open(output_xyz_filepath, 'w') as outfile:
            for v in range(height):
                for u in range(width):
                    z = depth_image[v, u]/100

                    if z > 0:  # Or some other depth threshold
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        r, g, b = rgb_image[v, u]


                                                # Apply pitch rotation around the x-axis (horizontal axis)
                        y = y * np.cos(np.deg2rad(45)) + z * np.sin(np.deg2rad(45))
                        z = -y * np.sin(np.deg2rad(45)) + z * np.cos(np.deg2rad(45))
                        x = x  # x-coordinate remains unchanged


                        outfile.write(f"{x} {y} {z} {r} {g} {b}\n")

        #print(f"Point cloud saved to: {output_xyz_filepath}")

    except Exception as e:
        print(f"An error occurred: {e}")

missions=[4]
sonar="P900"
for id in tqdm.tqdm(missions):


    mission_id=id

    with open('mission'+str(mission_id)+'.csv', newline='') as f:
        reader = csv.reader(f)
        mission_metadata = list(reader)
        mission_metadata.pop(0)

       
    for mission in tqdm.tqdm(mission_metadata):
        for i in tqdm.tqdm(range(int(mission[6]))):
            pkl_file = (f"Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/RGBD-images/{i}.pkl")  # Replace with your .pkl file path
            try:
                os.mkdir(f"Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/Point-cloud")
            except FileExistsError:
                pass
            xyz_file = (f"Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/Point-cloud/{i}.xyz")  # Replace with desired output path
            pkl_to_xyz(pkl_file, xyz_file)