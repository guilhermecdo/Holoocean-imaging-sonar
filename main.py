from modules.MissionManager import mission
import csv

mission_id=1

with open('mission'+str(mission_id)+'.csv', newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)


for data in mission_metadata:
    m=mission(data,mission_id,"Aris3000")
    m.start()