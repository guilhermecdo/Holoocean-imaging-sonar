import json
import csv

mission_id=3
path="/home/guilherme/Documents/Sonar-Dataset"

with open('mission'+str(mission_id)+'.csv', newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)


for data in mission_metadata: 

    id=int(data[0])
    interactions=int(data[-1])

    for j in range(1,interactions):
        full_path=path+"/mission-"+str(mission_id)+"/auv-"+str(id)+"-data/"
        file_name=full_path+str(id)+'-sonar_meta_data-'+str(j)+".json"
        print(file_name)
        with open(file_name,'r') as fp:
            meta_data=json.load(fp)
        range_bins=meta_data["azimuth_bins"]
        azimuth_bins=meta_data["range_bins"]

        meta_data["azimuth_bins"]=azimuth_bins
        meta_data["range_bins"]=range_bins
    

        json_object = json.dumps(meta_data, indent=len(meta_data))
        # Writing to sample.json
        with open(file_name, "w") as outfile:
            outfile.write(json_object)
