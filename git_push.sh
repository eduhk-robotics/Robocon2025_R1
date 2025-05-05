#!/bin/bash

# Navigate to the project directory
cd ~/Robocon2025_R1

# Stage all changes
git add *

# Commit with a message
git commit -m "update"

# Push changes to the repository
git push git@github.com:eduhk-robotics/Robocon2025_R1.git
