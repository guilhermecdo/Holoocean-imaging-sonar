from modules.MissionManager import mission
import json
import tqdm
import os
import sys

SONAR_MODEL="Didson"
PACKAGE_NAME="Matlab"
WORLD_NAME=None
MISSION_ID=[1]
MISSION_FILE=["matlab.json"]

if len(sys.argv) != 3:  # Check if exactly two arguments (plus the script name) are provided
    print("Usage: python3 myscript.py <integer1> <integer2>")

try:
    MISSION_ID = int(sys.argv[1])  # Convert the first argument to an integer
    num2 = int(sys.argv[2])  # Convert the second argument to an integer

except:
    pass

for i in range(len(MISSION_FILE)):
    
    mission_metadata = json.load(open(MISSION_FILE[i]))

    for j in tqdm.tqdm(mission_metadata.keys()):
        os.system("killall -e Holodeck")
        #print(mission_metadata[j])
        m=mission(mission_metadata[j],MISSION_ID[i],SONAR_MODEL,j,PACKAGE_NAME,[0,0,0])
        m.start()