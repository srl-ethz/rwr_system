#!/bin/bash

# Set variables for file location and topics (use an empty list for all topics)
file_location="recordings/"  # Replace with desired save location
topics=("/ingress/mano" "/ingress/wrist")            # Replace with desired topics, or leave empty for all topics

# Construct command with topics if specified
if [ ${#topics[@]} -eq 0 ]; then
    python3 record_rosbag.py --file_location "$file_location"
else
    python3 record_rosbag.py --file_location "$file_location" --topics "${topics[@]}"
fi
