#ifndef _ASTAR_H
#define _ASTAR_H

#include<bits/stdc++.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace navgraph_global_planner {
	// A structure to hold the neccesary parameters 
	struct cell 
	{ 
		// Node of parent
		// Note that 0 <= parent <= NODES-1
		int parent; 
		// f = g + h 
		double f, g, h; 
	}; 

	class AStar{
	public:
		AStar();

		~AStar();

		void initialize(const std::string& points, const std::string& adjacency);

		bool isValid(int node);

		double calculateDistance(int a, int b);

		bool isDestination(int src, int dest);

		void aStarSearch(int src, int dest, std::vector<geometry_msgs::Pose> &totalpath);

		int nearestnode(geometry_msgs::PoseStamped pose);

		double pathcost;
		int NODES;
		std::vector< std::vector<std::string> > dataList;

	private:
		void tracePath(cell *cellDetails, int dest, std::vector<geometry_msgs::Pose> &totalpath);
		
		int** adM;
		bool initialized;

	};
}
#endif
