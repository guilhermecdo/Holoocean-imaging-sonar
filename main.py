from modules.MissionManager import mission
import csv
import tqdm
missions=[1,2,3,4]

for id in missions:


    mission_id=id

    with open('mission'+str(mission_id)+'.csv', newline='') as f:
        reader = csv.reader(f)
        mission_metadata = list(reader)
        mission_metadata.pop(0)

    #mission_met=mission_metadata[2]
    for i in tqdm.tqdm(mission_metadata):
        m=mission(mission_metadata[int(i[0])],mission_id,"Aris3000")
        m.start()