#!/bin/bash

# filepath: /home/docker/ros_ws/src/gait_parameters_estimation/gait_parameter_estimation/iterate_folder_and_estimate_gait.bash

# This script iterates over all .bag files in a specified directory,
# runs toe detection to create an intermediate bag, and then runs
# gait parameter estimation on the result. All generated bags are
# saved to a separate output folder.

# --- Configuration ---
# Set the default folder to search for bags if no argument is provided.
INPUT_FOLDER="/home/docker/ros_ws/data"
OUTPUT_FOLDER="$INPUT_FOLDER/../gait"

# --- Script Logic ---
# Check if the provided input folder exists
if [ ! -d "$INPUT_FOLDER" ]; then
    echo "Error: Input directory '$INPUT_FOLDER' not found."
    exit 1
fi

echo "Searching for bag files in: $INPUT_FOLDER"

# Find all .bag files that do NOT end with _toe_output.bag or _gait_output.bag
# to avoid processing already processed files.
find "$INPUT_FOLDER" -type f -name "*.bag" ! -name "*_toe_output.bag" ! -name "*_gait_output.bag" | while read original_bag; do
    
    echo "------------------------------------------------------------"
    echo "Processing original file: $original_bag"
    
    # Get the base name of the original file to construct new output names
    original_basename=$(basename "$original_bag")
    
    # Define the output path for the toe detection step inside the output folder
    toe_output_bag="$OUTPUT_FOLDER/${original_basename%.bag}_toe_output.bag"
    
    # Create the output directory right before we need it.
    # 'mkdir -p' is safe to run multiple times.
    mkdir -p "$OUTPUT_FOLDER"
    
    # --- Step 1: Run Toe Detection with Kalman Filter ---
    echo "Running toe detection... Output will be: $toe_output_bag"
    roslaunch camera_lower_leg_tracking toe_detection_kalman_from_bag.launch input_bag_path:="$original_bag" output_bag_path:="$toe_output_bag"
    
    # Check if the toe detection was successful and created a non-empty file
    if [ ! -s "$toe_output_bag" ]; then
        echo "Error: Toe detection output file was not created or is empty. Skipping gait estimation."
        # Optional: remove the empty file if it exists
        [ -f "$toe_output_bag" ] && rm "$toe_output_bag"
        continue
    fi
    
    # --- Step 2: Run Gait Parameter Estimation ---
    # The python script will create its output file relative to the input path,
    # so the gait_output.bag will also be saved in the OUTPUT_FOLDER.
    echo "Running gait estimation on: $toe_output_bag"
    roslaunch gait_parameters_estimation gait_estimation_from_bag.launch input_bag_path:="$toe_output_bag"
    
    echo "Finished processing: $original_bag"
    echo "------------------------------------------------------------"
    
done

echo "All files processed."