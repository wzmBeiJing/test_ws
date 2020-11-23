#include "navgraph_solver/navgraph_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(navgraph_global_planner::NavgraphGlobalPlanner, nav_core::BaseGlobalPlanner)
using namespace visualization_msgs;

namespace navgraph_global_planner
{
	NavgraphGlobalPlanner::NavgraphGlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
	}

	NavgraphGlobalPlanner::NavgraphGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
	  initialize(name, costmap_ros);
	}

	NavgraphGlobalPlanner::~NavgraphGlobalPlanner()
	{
	}

	void NavgraphGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		if(!initialized_){
			ros::NodeHandle nh;
			ros::NodeHandle private_nh("~");
			file_sub2 = private_nh.subscribe("/panel_msgs", 10, &NavgraphGlobalPlanner::fileCallback, this);
			private_nh.param("points_file", points, std::string("navgraphs/points_rviz.csv"));
			private_nh.param("adjacency_file", adjacency, std::string("navgraphs/adjacency_rviz.csv"));
			private_nh.param("global_planner", global_planner, std::string("global_planner/GlobalPlanner"));
		    waypointfinder.initialize(points, adjacency);
			// ROS_INFO("Displaying Waypoints");
			
			// server.reset( new interactive_markers::InteractiveMarkerServer("waypoints") );
			// waypoint_service = nh.advertiseService("waypoint_service", &NavgraphGlobalPlanner::wp_from_index, this);
			// for(int i=0; i< waypointfinder.dataList.size(); i++){
			// 	float x = atof(waypointfinder.dataList[i][0].c_str());
			// 	float y = atof(waypointfinder.dataList[i][1].c_str());
			// 	int station = atoi(waypointfinder.dataList[i][8].c_str());
			// 	makeWaypoints(x,y,i, station);
			// }
			// server->applyChanges();

			ROS_INFO("Starting Navgraph Global Planner");
			pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> gp_loader_("nav_core", "nav_core::BaseGlobalPlanner");
			planner_ = gp_loader_.createInstance(global_planner);
			planner_->initialize(gp_loader_.getName(global_planner), costmap_ros);
			// globalplanner.initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
			initialized_ = true;
		}
	}
	    
    void NavgraphGlobalPlanner::fileCallback(const std_msgs::Int32ConstPtr& msg){
    	if(msg->data == 2){
    	    ros::NodeHandle private_nh("~");
			private_nh.param("points_file", points, std::string("navgraphs/points_rviz.csv"));
			private_nh.param("adjacency_file", adjacency, std::string("navgraphs/adjacency_rviz.csv"));
			waypointfinder.initialize(points, adjacency);
		}
    }

	bool NavgraphGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
  		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
	{
		int src = waypointfinder.nearestnode(start_pose);
		int dest = waypointfinder.nearestnode(goal);
		bool success = true;

		if(src != dest){
			success = success && connectpaths(src, dest, plan);
			plan.push_back(goal);
			}
		else
			success = planner_->makePlan(start_pose, goal, plan);
		return success;
	}

	bool NavgraphGlobalPlanner::connectpaths(int src, int dest, std::vector<geometry_msgs::PoseStamped>& plan){
		std::vector<geometry_msgs::PoseStamped> temp_plan;
		geometry_msgs::PoseStamped start_wp, goal_wp;
		waypointfinder.aStarSearch(src, dest, totalpath_);
		if(totalpath_.empty())
			return false;
		bool success = true;

		for(int node = 0; node < totalpath_.size() - 1; node++){
			start_wp.header.frame_id = "map";
			start_wp.header.stamp = ros::Time::now();
			start_wp.pose = totalpath_[node];
			goal_wp.header.frame_id = "map";
			goal_wp.header.stamp = ros::Time::now();
			goal_wp.pose = totalpath_[node+1];
			success = success && planner_->makePlan(start_wp, goal_wp, temp_plan);
			for(int i = 0; i< temp_plan.size(); i++)
				plan.push_back(temp_plan[i]);
		}
		return success;
	}

	void NavgraphGlobalPlanner::makeWaypoints(float x, float y, int i, int station){
		InteractiveMarker int_marker;
		int_marker.header.frame_id = "map";
		int_marker.scale = 0.5;

		int_marker.pose.position.x = x;
		int_marker.pose.position.y = y;
		int_marker.pose.position.z = 0.0;

		std::stringstream s;
	    s << i;
		int_marker.name = s.str();
		int_marker.description = "Button\n(Left Click)";

		InteractiveMarkerControl control;
		control.always_visible = true;
	  	control.interaction_mode = InteractiveMarkerControl::BUTTON;
		control.name = "button_control";

		Marker marker;

		marker.type = Marker::CUBE;
		marker.scale.x = int_marker.scale;
		marker.scale.y = int_marker.scale;
		marker.scale.z = int_marker.scale;
		marker.color.a = 1.0;
		if(station == 3){
			marker.color.r = 255;
			marker.color.g = 255;
			marker.color.b = 0;
		}
		else{
			marker.color.r = 0;
			marker.color.g = 255;
			marker.color.b = 0;
		}

		control.markers.push_back( marker );
		int_marker.controls.push_back( control );

		server->insert( int_marker );
		// server->setCallback(int_marker.name, &processFeedback);
	}

	bool NavgraphGlobalPlanner::wp_from_index(navgraph_solver::waypoint::Request  &req, navgraph_solver::waypoint::Response &res){
		if(req.index >= 0 && req.index < waypointfinder.dataList.size()){
			res.x = atof(waypointfinder.dataList[req.index][0].c_str());
			res.y = atof(waypointfinder.dataList[req.index][1].c_str());
			res.station = true;
			return true;
		}
		else{
			ROS_INFO("Invalid Index\n");
			return false;
		}
	}
}
