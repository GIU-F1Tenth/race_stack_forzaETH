#!/bin/bash

# Script to copy all map files from source to install directory
SRC_DIR="/home/mohammedazab/ws/src/race_stack/stack_master/maps"
INSTALL_DIR="/home/mohammedazab/ws/src/race_stack/install/stack_master/share/stack_master/maps"

cd "$SRC_DIR"

for dir in */; do
    map_name=${dir%/}
    
    echo "=== Copying $map_name ==="
    
    # Create the directory in install location if it doesn't exist
    mkdir -p "$INSTALL_DIR/$map_name"
    
    # Copy all files from source to install
    cp -v "$SRC_DIR/$map_name"/* "$INSTALL_DIR/$map_name/"
    
    echo "Completed $map_name"
done

echo "=== All maps copied to install directory ==="
