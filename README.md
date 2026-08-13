# octomap_testing
Repo for generation of octomaps from text files and saves into .bt format

# Usage
The .txt file in the maps folder contains the following information:

number of obstacles

location of obstacle vertices as a box for each obstacle x1 x2 y1 y2 z1 z2

The create_map.launch file creates the octomap with the input .txt file and desired output .bt octomap file

The visualise_map.launch visulises the octomap in rviz

# ROS 2 Conversion updates - Refer to Taj Foley for clarity
ROS 2 implementation for generating 3D Octomaps from text-based obstacle definitions and saving them in .bt format.

This repository is a ROS 2 Humble conversion of the original octomap_testing package.

## Usage
The .txt file in the maps folder defined the obstacles used to generate the Octomap.

The first line contains the:

number of obstacles

Each following line defines one rectangular obstacle using:

x1 x2 y1 y2 z1 z2

where the values define the minimum and maximum bounds of the obstacle in the X, Y and Z directions.

For example:

0.0 1.0 0.0 1.0 0.0 1.4

defines and obstacle from:

X: 0.0 to 1.0 m 
Y: 0.0 to 1.0 m 
Z: 0.0 to 1.4 m

The generated OctoMap uses a resolution of 0.2 m.

## Build

// Continue with this

