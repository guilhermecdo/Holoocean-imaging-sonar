from modules.MissionManager import mission
import csv
import tqdm
import os
missions=[1]

for j in missions:

    mission_id=j

    with open('mission'+str(mission_id)+'.csv', newline='') as f:
        reader = csv.reader(f)
        mission_metadata = list(reader)
        mission_metadata.pop(0)
    #voltar na m1 26 e 38
    mission_met=mission_metadata[26:27]
    #mission_metadata.pop(0)
    for i in tqdm.tqdm(mission_met):
        m=mission(mission_metadata[int(i[0])],mission_id,"P900")
        m.start()
    
    #os.system("killall -e Holodeck")