/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name notajet of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 # THis is the main file to change up

// ROS 2 CHANGE: ros/ros.h has been replaced by rclcpp.
#include <rclcpp/rclcpp.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string>
#include <cstdlib>                      // for exit
#include <fstream>                      // for ifstream, basic_istream, basic_istream<>::__istream_type
#include <iomanip>                      // for operator<<, setw
#include <iostream>                     // for cout
#include <memory>                       // ROS 2 CHANGE: for shared_ptr
#include <vector>                       // for vector

using namespace std;
using namespace octomap;


void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t "
         << node->getOccupancy() << endl;
  }
  else {
    cout << "occupancy probability at " << query
         << ":\t is unknown" << endl;
  }
}


int main(int argc, char** argv) {

  // ROS 2 CHANGE: ros::init has been replaced by rclcpp::init.
  rclcpp::init(argc, argv);

  // ROS 2 CHANGE: ros::NodeHandle has been replaced by an rclcpp node.
  auto node = std::make_shared<rclcpp::Node>("octomap_creator");

  // read the map of the environment and save obstacles location
  string mapPath;

  // ROS 2 CHANGE:
  node->declare_parameter<string>("mapFile", "");
  node->get_parameter("mapFile", mapPath);
  cout << "Map file path: " << mapPath << endl;

  string btFilename;

  // ROS 2 CHANGE:
  node->declare_parameter<string>("octomapFile", "");
  node->get_parameter("octomapFile", btFilename);
  cout << "Octomap file path: " << btFilename << endl;

  // string mapPath =
  // "/home/fernando/ABT_drone/Tapir14/tapir/problems/navtrack/maps/maparea.txt";

  /*
   * ROS 2 CHANGE:
   * Remove ROS-specific arguments before checking for a positional map-file argument - troubleshooting
   */
  vector<string> nonRosArguments =
    rclcpp::remove_ros_arguments(argc, argv);

  if (nonRosArguments.size() > 1)
  {
    mapPath = nonRosArguments[1];
    cout << "Using map file provided as command-line argument: "
         << mapPath << endl;
  }

  ifstream inFile;
  inFile.open(mapPath.c_str(), ifstream::in);

  if (!inFile.is_open()) {
    cout << "ERROR: Failed to open " << mapPath << endl;

    // ROS 2 CHANGE: shut ROS 2 down before exiting.
    rclcpp::shutdown();
    return 1;
  }

  int nObstacles = 0;

  // Read the number of obstacles from the map file
  if (!(inFile >> nObstacles)) {
    cout << "ERROR: Failed to read the number of obstacles from "
         << mapPath << endl;

    rclcpp::shutdown();
    return 1;
  }

  vector<vector<double>> obstacleMap(nObstacles);

  for (int r = 0; r < nObstacles; r++)
  {
    // it has 6 coordinates to read x1, x2, y1, y2, z1, z2
    obstacleMap[r].resize(6);

    if (!(inFile
      >> obstacleMap[r][0]
      >> obstacleMap[r][1]
      >> obstacleMap[r][2]
      >> obstacleMap[r][3]
      >> obstacleMap[r][4]
      >> obstacleMap[r][5]))
    {
      cout << "ERROR: Failed to read obstacle " << r + 1
           << " from " << mapPath << endl;

      rclcpp::shutdown(); //Shutdown ROS 2 before exiting.
      return 1;
    }
  }

  inFile.close();

  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree(0.2);  // create empty tree with resolution 0.2

  // insert some measurements of free cells
  for (int x = -50; x < 50; x++) {
    for (int y = -50; y < 50; y++) {
      for (int z = -10; z < 50; z++) {
        point3d endpoint(
          static_cast<float>(x) * 0.1f,
          static_cast<float>(y) * 0.1f,
          static_cast<float>(z) * 0.1f);

        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  // insert some measurements of occupied cells

  for (int i = 0; i < 3; i++) {
    for (int ob = 0; ob < nObstacles; ob++) {
      for (
        int x = static_cast<int>(obstacleMap[ob][0] * 10);
        x < static_cast<int>(obstacleMap[ob][1] * 10);
        x++)
      {
        for (
          int y = static_cast<int>(obstacleMap[ob][2] * 10);
          y < static_cast<int>(obstacleMap[ob][3] * 10);
          y++)
        {
          for (
            int z = static_cast<int>(obstacleMap[ob][4] * 10);
            z < static_cast<int>(obstacleMap[ob][5] * 10);
            z++)
          {
            point3d endpoint(
              static_cast<float>(x) * 0.1f,
              static_cast<float>(y) * 0.1f,
              static_cast<float>(z) * 0.1f);

            tree.updateNode(
              endpoint,
              true);  // integrate 'occupied' measurement
          }
        }
      }
    }
  }


  cout << endl;
  cout << "performing some queries:" << endl;

  point3d query(0., 0., 0.);
  OcTreeNode* result = tree.search(query);
  print_query_info(query, result);

  // These queries are outside the map, so they should return "unknown" occupancy.
  query = point3d(-1., -1., -1.);
  result = tree.search(query);
  print_query_info(query, result);

  query = point3d(-2., -1., 1.);
  result = tree.search(query);
  print_query_info(query, result);


  cout << endl;

  // Check whether the map was actually written successfully.
  if (!tree.writeBinary(btFilename)) {
    cout << "ERROR: Failed to write Octomap file: "
         << btFilename << endl;

    rclcpp::shutdown();
    return 1;
  }

  // Display the actual output filename instead of "simple_tree.bt".
  cout << "wrote Octomap file " << btFilename << endl << endl;

  cout << "now you can use octovis to visualize: octovis "
       << btFilename << endl;

  cout << "Hint: hit 'F'-key in viewer to see the freespace"
       << endl << endl;

  cout << "Now reading from binary" << endl;

  // string btFilename =
  // "/home/fernando/catkin_test_ws/scenario_8_obst.bt";  # Just using a different 

  OcTree other_tree(btFilename);

  cout << "read file " << btFilename << endl << endl;
  cout << "performing some queries:" << endl;

  query = point3d(0., 0., 0.);
  result = other_tree.search(query);
  print_query_info(query, result);

  query = point3d(-1., -1., -1.);
  result = other_tree.search(query);
  print_query_info(query, result);

  query = point3d(-2., -1., 1.);
  result = other_tree.search(query);
  print_query_info(query, result);

  // cleanly shut down the ROS 2 context - no big change here
  rclcpp::shutdown();

  return 0;
}