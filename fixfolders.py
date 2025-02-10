import os
import shutil


sonar ="P900"

mission=[1,2,3,4]

for m in mission:
    for auv in range(0,40):
        rgbd_files_folder=(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{m}-{sonar}/auv-{auv}/RGBD-images/")
        point_cloud_files_folder=(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{m}-{sonar}/auv-{auv}/Point-cloud/")
        print(rgbd_files_folder)
        sonar_files_foler=(f"/home/guilherme/Documents/Holoocean-imaging-sonar/Sonar-Dataset-mission-{m}-{sonar}/auv-{auv}/")
        try:
            os.system("cp -r "+rgbd_files_folder+" "+sonar_files_foler)
            os.system("cp -r "+point_cloud_files_folder+" "+sonar_files_foler)
        except:
            print("num foi")