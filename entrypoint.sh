#!/bin/bash

# Path to a flag file indicating that the script has already run
FLAG_FILE="/$HOME/setup/.setup_done"

echo "=================================================================="
echo "Starting entrypoint.sh..."
echo "=================================================================="

# If the flag file does not exist, run the script and create the flag
if [ ! -f "$FLAG_FILE" ]; then
    echo "Running setup.sh for the first time..."
    bash "/$HOME/setup/setup.sh"
    if [ $? -eq 0 ]; then
        echo "setup.sh successfully completed."
        touch "$FLAG_FILE"
    else
        echo "Error when running setup.sh."
        exit 1
    fi
else
    echo "=================================================================="
    echo "Environment setup successfully completed!"
    echo "=================================================================="
fi

echo "=================================================================="
echo "Finishing entrypoint.sh."
echo "=================================================================="