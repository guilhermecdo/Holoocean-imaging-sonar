import cv2
import numpy as np

# Load the grayscale image
# Ensure the path to your image is correct
def addColorMap(image_path:str,output_path:str,color_map=cv2.COLORMAP_BONE):
    try:
        gray_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        #gray_image=np.load(image_path)
        #gray_image=(gray_image*255/20)
        #print(gray_image)
        if gray_image is None:
            raise FileNotFoundError(f"Image not found at {image_path}")

        gray_image_8bit = cv2.normalize(gray_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # Apply a colormap (e.g., COLORMAP_JET)
        colorized_image = cv2.applyColorMap(gray_image_8bit, color_map)

        # Display the original and colorized images
        cv2.imshow('Grayscale Image', gray_image)
        cv2.imshow('Colorized Image (JET)', colorized_image)
        cv2.imwrite(output_path,colorized_image)

        # You can try other colormaps as well
        # colorized_hot = cv2.applyColorMap(gray_image, cv2.COLORMAP_HOT)
        # cv2.imshow('Colorized Image (HOT)', colorized_hot)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except FileNotFoundError as e:
        print(e)
    except Exception as e:
        print(f"An error occurred: {e}")

image_path="/home/guilherme/Documents/Holoocean-imaging-sonar/Sonar-Dataset-mission-1-P900-pitch/auv-0/Cartesian-images/2.png"
output_path="teste2.png"
addColorMap(image_path=image_path,output_path=output_path)
