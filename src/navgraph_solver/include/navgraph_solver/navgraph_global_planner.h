#ifndef NAVGRAPH_GLOBAL_PLANNER_H
#define NAVGRAPH_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <navgraph_solver/my_astar.h>
#include <interactive_markers/interactive_marker_server.h>
#include "navgraph_solver/waypoint.h"
#include <pluginlib/class_loader.h>
#include <std_msgs/Int32.h>

namespace navgraph_global_planner
{
	class NavgraphGlobalPlanner : public nav_core::BaseGlobalPlanner
	{
	public:
		/**
     * @brief Default Constructor
     */
    NavgraphGlobalPlanner();

        /** subscriber callback 
    reinizialize nodes after modifications
    **/
    void fileCallback(const std_msgs::Int32ConstPtr& msg); 

    /**
     * @brief Constructor for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    NavgraphGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Default destructor
     */
    ~NavgraphGlobalPlanner();

    /**
     * @brief Initialization function for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start_pose The starting pose of the robot
     * @param goal The goal pose
     * @param plan The plan filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start_pose,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan);

    bool wp_from_index(navgraph_solver::waypoint::Request  &req,
         navgraph_solver::waypoint::Response &res);

    
    protected:

    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    costmap_2d::Costmap2D* costmap_;
    std::string frame_id_;
    bool initialized_, allow_unknown_;

    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    std::string global_planner;
    // global_planner::GlobalPlanner globalplanner;
    
    AStar waypointfinder;
    std::string points;
    std::string adjacency;
    std::vector<geometry_msgs::Pose> totalpath_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    ros::ServiceServer waypoint_service;

    ros::Subscriber file_sub2 ; 
    
    bool connectpaths(int src, int dest, std::vector<geometry_msgs::PoseStamped>& plan);

    void makeWaypoints(float x, float y, int i, int station);

	};
}

#endif  // NAVGRAPH_GLOBAL_PLANNER_H
