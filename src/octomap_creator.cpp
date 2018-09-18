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
 *     * Neither the name of the University of Freiburg nor the names of its
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
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string>    
#include <cstdlib>                      // for exit
#include <fstream>                      // for ifstream, basic_istream, basic_istream<>::__istream_type
#include <iomanip>                      // for operator<<, setw
#include <iostream>                     // for cout
#include <vector>                       // for vector

using namespace std;
using namespace octomap;


void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "octomap_creator");
    ros::NodeHandle nh;
	// Read the map from the file.
    //read the map of the environment and save obstacles location
	string mapPath = "/home/fernando/ABT_drone/Tapir14/tapir/problems/navtrack/maps/maparea.txt";
	if(argc > 1)
	{
		mapPath = argv[1];
	}
    ifstream inFile;
    inFile.open(mapPath.c_str(), ifstream::in);
    if (!inFile.is_open()) {
        cout << "ERROR: Failed to open " << mapPath;
        //debug::show_message(message.str());
        exit(1);
    }
	int nObstacles= 0;
    inFile >> nObstacles;
	vector< vector<double> > obstacleMap(nObstacles);

	for(int r = 0; r < nObstacles; r++)
	{
		obstacleMap[r].resize(6);//it has 6 coordinates to read x1, x2, y1, y2, z1, z2
		inFile >> obstacleMap[r][0] >> obstacleMap[r][1] >> obstacleMap[r][2] 
		>> obstacleMap[r][3] >> obstacleMap[r][4] >> obstacleMap[r][5];
	}

  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree (0.2);  // create empty tree with resolution 0.2

 // insert some measurements of free cells

  for (int x=-50; x<50; x++) {
    for (int y=-50; y<50; y++) {
      for (int z=-10; z<50; z++) {
        point3d endpoint ((float) x*0.1f, (float) y*0.1f, (float) z*0.1f);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }
  // insert some measurements of occupied cells
  for (int i = 0; i < 3; i++) {
	for (int ob = 0;ob < nObstacles; ob++) {
	  for (int x=obstacleMap[ob][0]*10; x<obstacleMap[ob][1]*10; x++) {
		for (int y=obstacleMap[ob][2]*10; y<obstacleMap[ob][3]*10; y++) {
		  for (int z=obstacleMap[ob][4]*10; z<obstacleMap[ob][5]*10; z++) {
			point3d endpoint ((float) x*0.1f, (float) y*0.1f, (float) z*0.1f);
			tree.updateNode(endpoint, true); // integrate 'occupied' measurement
		  }
		}
	  }
	}
}

 
  cout << endl;
  cout << "performing some queries:" << endl;
  
  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-2.,-1.,1.);
  result = tree.search (query);
  print_query_info(query, result);


  cout << endl;
  tree.writeBinary("scenario_8_obst.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  
  cout << "Now reading from binary" << endl;
  
  string btFilename = "/home/fernando/catkin_test_ws/scenario_8_obst.bt";
  OcTree* other_tree = new OcTree(btFilename);
  cout << "read file scenario_8_obst.bt" << endl << endl;  
  cout << "performing some queries:" << endl;
  
  query = point3d(0., 0., 0.);
  result = other_tree->search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = other_tree->search (query);
  print_query_info(query, result);

  query = point3d(-2.,-1.,1.);
  result = other_tree->search (query);
  print_query_info(query, result);
}
