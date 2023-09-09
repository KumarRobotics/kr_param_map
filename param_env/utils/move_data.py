import os
import shutil
src_folder = '/home/yifei/ws/src/kr_autonomous_flight/kr_param_map/param_env/dataset/'
dest_folder = '/home/yifei/ws/src/kr_autonomous_flight/kr_param_map/param_env/dataset2/'
def move_and_rename(src_folder, dest_folder):
    # Counter to ensure unique filenames
    counter = 0
    
    # Walk through all files and folders within the source folder
    for dirpath, dirnames, filenames in os.walk(src_folder):
        print(f"Found directory: {dirpath}, filenames, {filenames}")
        for filename in filenames:
            # Generate source file path
            src_filepath = os.path.join(dirpath, filename)
            
            # Create a unique destination file name
            dest_filename = f"{counter}_{filename}"
            
            # Generate destination file path
            dest_filepath = os.path.join(dest_folder, dest_filename)
            
            # Move and rename the file
            shutil.move(src_filepath, dest_filepath)
            
            # Increment the counter
            counter += 1


# Create the destination folder if it doesn't exist
if not os.path.exists(dest_folder):
    os.makedirs(dest_folder)

move_and_rename(src_folder, dest_folder)
