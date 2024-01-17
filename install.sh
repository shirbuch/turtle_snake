#!/bin/bash

@echo off

original_dir=$(pwd)

echo ~/catkin_ws
cd ~/catkin_ws

echo catkin_make
catkin_make

echo cd "$original_dir/scripts"
cd "$original_dir/scripts"

for file in *; do
    if [ -f "$file" ]; then
        chmod u+x "$file"
        echo "chmod u+x $file"
    fi
done
