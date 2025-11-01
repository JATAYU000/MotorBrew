#!/bin/bash

# Check if commit message is provided
if [ $# -eq 0 ]; then
    echo "Error: Please provide a commit message"
    echo "Usage: $0 \"commit message\""
    exit 1
fi

# Store commit message from first argument
msg="$1"

# Git operations
git add NXP_AIM_INDIA_2025/
git commit -m "$msg"
git push origin hardware

# Check status of push
if [ $? -eq 0 ]; then
    echo "Successfully pushed to hardware branch"
else
    echo "Error pushing to hardware branch"
    exit 1
fi
