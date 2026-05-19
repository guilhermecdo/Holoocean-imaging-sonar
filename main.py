from modules.MissionManager import mission
import csv
import tqdm
import os
import sys

#[23,27,34,35,36,37,38,39]

if len(sys.argv) != 3:  # Check if exactly two arguments (plus the script name) are provided
    print("Usage: python3 myscript.py <integer1> <integer2>")

try:
    num1 = int(sys.argv[1])  # Convert the first argument to an integer
    num2 = int(sys.argv[2])  # Convert the second argument to an integer

except ValueError: # Handle cases where the input isn't an integer
    print("Error: Input arguments must be integers.")
#issions=[2,3,4]

#for id in missions:
#args=sys.argv
#print(args)
mission_id=num1

with open('mission'+str(mission_id)+'.csv', newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)

    mission_met=mission_metadata[(num2):(num2+1)]
for i in tqdm.tqdm(mission_met):
        os.system("killall -e Holodeck")
        m=mission(mission_metadata[int(i[0])],mission_id,"Didson-denoise-2")
        m.start()