from unrealcv.automation import UE4Binary
from unrealcv.util import read_png, read_npy
from unrealcv import Client
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import time

def transform(depth_img):
            img_ = depth_img.copy()
            nan_location = np.isnan(img_)
            img_[nan_location] = np.nanmax(img_)
            norm_image =  (img_)*255./5.
            norm_image[0,0] = 255.
            norm_image = norm_image.astype('uint8')
            return cv2.cvtColor(norm_image, cv2.COLOR_GRAY2BGR)

binary_path = '/home/guilherme/develop-holoocean/UNREALCV/dist2/LinuxNoEditor/Holodeck.sh'
if os.path.isfile(binary_path):
    binary = UE4Binary(binary_path)
    pid=binary.start()
else:
    print("Can not find binary file %s \nWorking directory is %s" % (binary_path, os.path.abspath('.')))

client=Client(endpoint=('localhost', 9000))
client.connect()
print(client.request('vget /unrealcv/version'))
print(client.request('vget /unrealcv/status'))
client.request('vset /viewmode depth')
#print(client.request('vget /objects'))
#client.request('vset /camera/0/location -200 -0 -250')

res = client.request('vset /viewmode lit')
angles=np.linspace(0,350,36)
headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
elevation=np.arange(-2.5,0,0.3 )
for z in elevation:
    for angle, heading in zip(angles,headings):
        x=2*np.cos(np.deg2rad(angle))
        y=2*np.sin(np.deg2rad(angle))
        print(client.request('vset /camera/0/location '+str(x)+' '+str(y)+' '+str(z)))
        print(client.request('vset /camera/0/rotation 0 '+str(heading)+' 0'))
        
        res = client.request('vget /camera/0/depth npy')
        depth = read_npy(res)

        depth_data = np.frombuffer(depth, dtype=np.float32)
        depth_image = depth_data.reshape((394, 768))  
        
        
        #res = client.request('vget /camera/0/lit png')
        #rgb_image = cv2.imdecode(np.frombuffer(res, np.uint8), -1)
        #cv2.imwrite("rgb.png", rgb_image)
        #depth_norm=transform(depth_image)
        #Normalize depth values to the range [0, 255]
        #depth_image_normalized = cv2.normalize(depth_image, None, 255,0, cv2.NORM_MINMAX, cv2.CV_8U)


        #depth_image = cv2.imdecode(np.frombuffer(depth_image, np.float32), cv2.IMREAD_GRAYSCALE)
        cv2.imwrite("depth"+str(angle)+".png", depth_image)

        time.sleep(3)