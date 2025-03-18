import shutil
import os
import csv

def copy_and_rename_file(source_folder, destination_folder, source_filename, new_filename):
    """
    Copies a file from a source folder to a destination folder and renames it.

    Args:
        source_folder: The path to the folder containing the source file.
        destination_folder: The path to the folder where the file will be copied.
        source_filename: The name of the file to be copied.
        new_filename: The new name for the copied file.  Make sure to include the file extension.
    
    Returns:
        True if the file was copied and renamed successfully, False otherwise.
        Prints informative messages to the console about success or failure.
    """

    source_path = os.path.join(source_folder, source_filename)
    destination_path = os.path.join(destination_folder, new_filename)

    try:
        # Check if the source file exists
        if not os.path.exists(source_path):
            print(f"Error: Source file '{source_filename}' not found in '{source_folder}'.")
            return False

        # Check if the destination folder exists, and if not, create it:
        if not os.path.exists(destination_folder):
            try:
                os.makedirs(destination_folder)  # Create the directory recursively
                print(f"Destination folder '{destination_folder}' created.")
            except OSError as e:
                print(f"Error creating destination folder: {e}")
                return False

        # Copy and rename the file
        shutil.copy2(source_path, destination_path)  # copy2 preserves metadata
        print(f"File '{source_filename}' copied and renamed to '{new_filename}' in '{destination_folder}'.")
        return True

    except Exception as e:
        print(f"An error occurred: {e}")
        return False



sonar="P900"
missions=[1,2,3,4]
#samples=142




#a=0

for m in missions:
    with open('mission'+str(m)+'.csv', newline='') as f:
        reader = csv.reader(f)
        mission_metadata = list(reader)
        mission_metadata.pop(0)

    for mission in mission_metadata:
        for i in range((int(mission[6]))):
                source_folder1 = (f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{m}-{sonar}/auv-{mission[0]}/GT-images")
                destination_folder1 = (f"/home/guilherme/Documents/Pytorch-UNet/data/masks")
                source_folder2 = (f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{m}-{sonar}/auv-{mission[0]}/Cartesian-images")
                destination_folder2 = (f"/home/guilherme/Documents/Pytorch-UNet/data/imgs")
                filename=(f"{i}.png")
                new_filename = (f"{m}-{sonar}-auv-{mission[0]}-{i}.png")
                try:
                    copy_and_rename_file(source_folder1, destination_folder1, filename, new_filename)
                    copy_and_rename_file(source_folder2, destination_folder2, filename, new_filename)
                    #a=a+1
                except:
                    pass
            
            