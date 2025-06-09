import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image  # For saving as image files

def visualize_and_save_point_cloud(xyz_file, output_image_path, dpi=300):
    """
    Visualizes an XYZ point cloud from a file and saves it as an image.

    Args:
        xyz_file (str): Path to the XYZ point cloud file.
        output_image_path (str): Path to save the output image.
        dpi (int): Dots per inch for the saved image.
    """
    try:
        data = np.loadtxt(xyz_file)
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]

        fig = plt.figure(figsize=(10, 10))  # Adjust figure size as needed
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z, s=1, c='blue')  # Adjust point size (s) and color (c)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(os.path.basename(xyz_file)) #File name as title

        #Remove the grid and axis for a cleaner image.
        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])
        
        #Adjust the view angle, if necessary
        #ax.view_init(elev=20, azim=45) #Example angles

        plt.savefig(output_image_path, dpi=dpi, bbox_inches='tight', pad_inches=0.1) #Save the image
        plt.close(fig) #Close the figure to free up memory

        print(f"Saved image: {output_image_path}")

    except FileNotFoundError:
        print(f"Error: File not found - {xyz_file}")
    except Exception as e:
        print(f"An error occurred: {e}")

def process_point_cloud_files(input_dir, output_dir, dpi=300):
    """
    Processes all XYZ point cloud files in a directory and saves images.

    Args:
        input_dir (str): Directory containing the XYZ files.
        output_dir (str): Directory to save the output images.
        dpi (int): Dots per inch for the saved images.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for filename in os.listdir(input_dir):
        if filename.endswith(".xyz"):
            input_file = os.path.join(input_dir, filename)
            output_file = os.path.join(output_dir, f"{os.path.splitext(filename)[0]}.png") #save as png.
            visualize_and_save_point_cloud(input_file, output_file, dpi)

# Example usage:
input_directory = "/home/guilherme/Documents/Holoocean-imaging-sonar/experiments-classic/" #Replace with the path to your xyz files.
output_directory = "/home/guilherme/Documents/Holoocean-imaging-sonar/experiments-classic/" #Replace with the desired output directory.
process_point_cloud_files(input_directory, output_directory, dpi = 600) #Adjust DPI as needed.