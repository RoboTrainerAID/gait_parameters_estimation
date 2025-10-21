#!/bin/bash

# This script iterates over all .bag files in a specified directory
# and checks if they are valid and contain a significant number of messages.
# A bag is considered "invalid" or "empty" if `rosbag info` fails,
# or if the reported message count is 10 or less.

# --- Configuration ---
# Set the default folder to search for bags.
# You can change this to any folder you want to check.
INPUT_FOLDER="/home/docker/ros_ws/data/gait"

# --- Script Logic ---
# Check if the provided input folder exists
if [ ! -d "$INPUT_FOLDER" ]; then
    echo "Error: Input directory '$INPUT_FOLDER' not found."
    exit 1
fi

echo "Searching for bag files in: $INPUT_FOLDER"
echo "A bag is considered invalid if it has 10 or fewer messages or is corrupted."
echo "------------------------------------------------------------"

found_invalid=false

# Find all .bag files in the input folder.
find "$INPUT_FOLDER" -type f -name "*.bag" | while read bag_file; do
    
    # Run rosbag info with a 1-second timeout.
    # The timeout prevents the script from hanging on a corrupted bag file.
    info_output=$(timeout 1s rosbag info "$bag_file" 2>&1)
    
    # Check the exit status of the timeout command.
    # An exit status of 124 means the command timed out.
    if [ $? -eq 124 ]; then
        echo "Timeout error: 'rosbag info' took too long on file: $bag_file"
        found_invalid=true
        continue
    fi

    # Check for common error strings that indicate a corrupted or unreadable bag.
    if echo "$info_output" | grep -q -i -E "ERROR|does not exist|unindexed|empty"; then
        echo "Invalid/Corrupted bag file found: $bag_file"
        found_invalid=true
        continue
    fi
    
    # Extract the number of messages.
    # Use grep to find the line, then awk to get the second field.
    num_messages=$(echo "$info_output" | grep "^messages:" | awk '{print $2}')
    
    # Default to 0 if the messages line was not found
    num_messages=${num_messages:-0}

    # Check if the number of messages is 10 or less.
    if [ "$num_messages" -le 10 ]; then
        echo "Bag file with few or no messages found: $bag_file (Messages: $num_messages)"
        echo $info_output
        found_invalid=true
    fi
    
done

echo "------------------------------------------------------------"
if [ "$found_invalid" = false ]; then
    echo "All bag files in the directory are valid and contain more than 10 messages."
fi
echo "Check complete."