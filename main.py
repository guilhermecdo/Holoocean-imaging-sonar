from modules.MissionManager import mission
import csv
import tqdm
import os
import sys

SONAR_MODEL="standart"
PACKAGE_NAME="Matlab"
WORLD_NAME=["coral_1","coral_2","woodTable","greenReef","redReef_1"]
MISSION_ID=1

if len(sys.argv) != 3:  # Check if exactly two arguments (plus the script name) are provided
    print("Usage: python3 myscript.py <integer1> <integer2>")

try:
    MISSION_ID = int(sys.argv[1])  # Convert the first argument to an integer
    num2 = int(sys.argv[2])  # Convert the second argument to an integer

except:
    pass

with open(f"mission{MISSION_ID}.csv", newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)

for i in tqdm.tqdm(mission_metadata):
        os.system("killall -e Holodeck")
        m=mission(mission_metadata[int(i[0])],MISSION_ID,SONAR_MODEL,WORLD_NAME,PACKAGE_NAME,[0,20,0])
        m.start()