import cv2
import numpy as np

# Load the grayscale image
# Ensure the path to your image is correct
def addColorMap(image_path:str,output_path:str,color_map=cv2.COLORMAP_BONE):
    try:
        gray_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if gray_image is None:
            raise FileNotFoundError(f"Image not found at {image_path}")

        gray_image_8bit = cv2.normalize(gray_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # Apply a colormap (e.g., COLORMAP_JET)
        colorized_image = cv2.applyColorMap(gray_image_8bit, color_map)

        cv2.imwrite(output_path,colorized_image)


    except FileNotFoundError as e:
        print(e)
    except Exception as e:
        print(f"An error occurred: {e}")


HEAD_PATH= "/home/guilherme/Documents/SEE-Dataset/SEE-Synthetic-Data/Sonar-Dataset-mission-1-P900/auv-0/GT-images/"
image_path=f"{HEAD_PATH}0.png"
output_path="GT-1-P900-0-0.png"
addColorMap(image_path=image_path,output_path=output_path)
