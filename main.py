from modules.MissionManager import mission
import csv
import tqdm

mission_id=1

with open('mission'+str(mission_id)+'.csv', newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)

mission_metadata=mission_metadata[1:]

for i in tqdm.tqdm(mission_metadata):
    m=mission(mission_metadata[int(i[0])],mission_id,"P900")
    m.start()