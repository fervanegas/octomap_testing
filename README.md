# octomap_testing
Repo for generation of octomaps from text files and saves into .bt format

# Usage
The .txt file in the maps folder contains the following information:

number of obstacles

location of obstacle vertices as a box for each obstacle x1 x2 y1 y2 z1 z2

The create_map.launch file creates the octomap with the input .txt file and desired output .bt octomap file

The visualise_map.launch visulises the octomap in rviz
