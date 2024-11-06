from modules.MissionManager import mission
import csv

mission_id=4


with open('mission'+str(mission_id)+'.csv', newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)


#for data in mission_metadata[32:]:
    m=mission(mission_metadata[3],mission_id)
    m.start()