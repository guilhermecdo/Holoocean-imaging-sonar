from modules.MissionManager import mission
import json
import tqdm
import os
import sys
#SEE CONFIG
SONAR_MODEL="Didson"
PACKAGE_NAME="SEE"
WORLD_NAME=["64-tank-Map-1","64-tank-Map-2","64-tank-Map-3","64-tank-Map-4"]
MISSION_ID=[1,2,3,4]
MISSION_FILE=["see-1.json","see-2.json","see-3.json","see-4.json"]

# #MATLAB CONFIG
# SONAR_MODEL="Didson"
# PACKAGE_NAME="Rocks"
# WORLD_NAME=""
# MISSION_ID=5
# MISSION_FILE=["rocks.json"]


if len(sys.argv) != 3:  # Check if exactly two arguments (plus the script name) are provided
    print("Usage: python3 myscript.py <integer1> <integer2>")

try:
    MISSION_ID = int(sys.argv[1])  # Convert the first argument to an integer
    num2 = int(sys.argv[2])  # Convert the second argument to an integer

except:
    pass

for i in range(len(MISSION_FILE)):
    
    mission_metadata = json.load(open(MISSION_FILE[i]))

    #print(mission_metadata.keys())

    for j in tqdm.tqdm(mission_metadata.keys()):
        os.system("killall -e Holodeck")
        #print(mission_metadata[j])
        m=mission(mission_metadata[j],MISSION_ID[i],SONAR_MODEL,WORLD_NAME[i],PACKAGE_NAME,[0,0,0])
        m.start()