#include <bits/stdc++.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "navgraph_solver/waypoint.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <iterator>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <ros/time.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#define STATIONS 5
#define PI 3.14159265359

using namespace visualization_msgs;
using namespace std;

volatile bool reached_goal_ = true;
bool button_clicked = false;
int waypoint_count = 0;
std::string clicked_marker;
ros::Publisher next_station;
ros::Publisher localplanner;
ros::ServiceClient wp_client;
navgraph_solver::waypoint wp_srv;
std::vector<std::string> visited;;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server2;
interactive_markers::MenuHandler menu_handler;
interactive_markers::MenuHandler menu_handler2;
bool abort_mission = false;
bool initialize_move_base = false;
bool mission_complete = true;

int activate_proh = 0 ;

geometry_msgs::Pose robotPose ; 

double x1  ;
double yy1 ; 

void make6DofMarker(int waypoint_count, geometry_msgs::Pose pose, bool station);
void makelineMarker(std::string marker1, std::string marker2, int localplanner);

void missionPlanner(std::string mission){
	ROS_INFO("******************...EXECUTING NEW MISSION...***********************") ; 
	mission_complete = false;
	InteractiveMarker int_marker_point;
	geometry_msgs::PoseStamped goal;
	std_msgs::Bool planner_;
	goal.header.frame_id = "map";
	goal.pose.orientation.w = 1.0;
	std::vector<std::string> stations;
	std::vector<int> stay;
	std::vector<int> planner;
	std::vector<int> tasks;
	bool first_loop = true;
	std::string start_station = "";

    //parse stations, tasks, stay?, planner from message and save it in vectors 
	std::vector<std::string> vec, vec2;
	boost::algorithm::split(vec, mission, boost::is_any_of("/"));
	for(int i = 0; i<vec.size(); i++){
		boost::algorithm::split(vec2, vec[i], boost::is_any_of(" "));
		if(vec2[0].length()){
			ROS_INFO("Station: %d", atoi(vec2[0].c_str()));
			stations.push_back(vec2[0]);
			tasks.push_back(atoi(vec2[1].c_str()));
			stay.push_back(atoi(vec2[2].c_str()));
			planner.push_back(atoi(vec2[3].c_str()));
		}
	}

	//some code 
	visited.resize(stations.size(), "-1");
	for(int i=0; i<visited.size(); i++){
		if(visited[i].compare("-1") == 0){
			for(int j=i+1; j<visited.size(); j++)
				visited[j] = "-1";
		}
	}

	for(int i =0; ; i = (i+1)%stations.size()){
		if(visited[i].compare(stations[i]) == 0 && first_loop){
			// visited[i] = "-1";
			if(abort_mission){
				abort_mission = false;
				ROS_INFO("Aborting Mission");
				mission_complete = true;
				reached_goal_ = true;
				return;
			}
			if(i == stations.size()-1)
				first_loop = false;
			ROS_INFO("Skipping %s", stations[i].c_str());
			continue;
		}
		else
			first_loop = false;

		if(stations[i].compare("0") == 0){
			visited[i] = stations[i];
			continue;
		}

		//set goal -> next station  
		if(server->get(stations[i], int_marker_point)){
			ROS_INFO("Goal: %f,%f", int_marker_point.pose.position.x, int_marker_point.pose.position.y);
			goal.pose = int_marker_point.pose;
			planner_.data = planner[i];

			// stringstream s;
			// s << start_station << "," << start_station;
			// if(server2->get(s.str(), int_marker_point)){
			// 	if(int_marker_point.description.compare())
			// }

			if(reached_goal_){
				goal.header.stamp = ros::Time::now();
				//publish gosl to next station 
				next_station.publish(goal);
				localplanner.publish(planner_);
				ROS_INFO("Publishing Goal");
				if(mission_complete)
					ROS_INFO("MISSION COMPLETE IS SET");
				reached_goal_ = false;
				//waiting for goal to reach the station or the mission to be aborted 
				while(!reached_goal_){
					if(abort_mission){

						abort_mission = false;
						ROS_INFO("Aborting Mission");
						mission_complete = true;
						reached_goal_ = true;
						return;
					}
					
					sleep(1) ; 
				}
				visited[i] = stations[i];
				start_station = stations[i];
				//ROS_INFO("Executing task %d", tasks[i]);
				if(stay[i]){
					ROS_INFO("Waiting for continuation command from client..");
					mission_complete = true;
					// reached_goal_ = true;
					return;
				}
			}
			else{
				ROS_INFO("Waiting for Goal completion");
				if(abort_mission){
					abort_mission = false;
					ROS_INFO("Aborting Mission");
					mission_complete = true;
					reached_goal_ = true;
					return;
				}
			}
		}
		else{
			ROS_INFO("Incorrect Station Indices");
			mission_complete = true;
			// reached_goal_ = true;
			return;
		}
	}
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	std::ostringstream s;
	s << "Feedback from marker " << feedback->marker_name;

	switch ( feedback->event_type )
	{
		case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM( s.str() << ": button click" << "." );
		break;

		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		{
			ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << "." );
			if(feedback->menu_entry_id == 4){
				if(button_clicked){
					if(clicked_marker == feedback->marker_name){
						ROS_INFO("Cannot connect marker to itself");
						break;
					}
					else{
						ROS_INFO_STREAM("Connecting " << clicked_marker << " and " << feedback->marker_name);
						makelineMarker(clicked_marker, feedback->marker_name, 0);
						server2->applyChanges();
						button_clicked = false;
						clicked_marker = "";
					}
				}
				else{
					button_clicked = true;
					clicked_marker = feedback->marker_name;
					ROS_INFO_STREAM("Marked " << clicked_marker);
				}
				break;
			}
			else if(feedback->menu_entry_id == 1){
				server->erase(feedback->marker_name);
				make6DofMarker(atoi(feedback->marker_name.c_str()), feedback->pose, false );
			}
			else if(feedback->menu_entry_id == 2){
				server->erase(feedback->marker_name);
				make6DofMarker(atoi(feedback->marker_name.c_str()), feedback->pose, true );
			}
			else if(feedback->menu_entry_id == 3){
				ROS_INFO("Deleting point.");
				server->erase(feedback->marker_name);
				server->applyChanges();
				int k =1;
				InteractiveMarker marker;
				for(int i = 0; i<server->size(); i++){
					std::stringstream s, s1, s2;
					s1 << i+k << "," << feedback->marker_name;
					s2 << feedback->marker_name << "," << i+k;
					s << i+k;
					if(server->get(s.str(), marker)){
						if(server2->get(s1.str(), marker)){
							server2->erase(s1.str());
							server2->applyChanges();
						}
						if(server2->get(s2.str(), marker)){
							server2->erase(s2.str());
							server2->applyChanges();
						}
					}
					else{
						i -= 1;
						k += 1;
					}
				}
			}
			break;
		}

		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		{
			ROS_INFO_STREAM( s.str() << ": pose changed"
				<< "\nposition = "
				<< feedback->pose.position.x
				<< ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z
				<< "\norientation = "
				<< feedback->pose.orientation.w
				<< ", " << feedback->pose.orientation.x
				<< ", " << feedback->pose.orientation.y
				<< ", " << feedback->pose.orientation.z);

			int k =1;
			InteractiveMarker marker;
			server->applyChanges();
			for(int i = 0; i<server->size(); i++){
				std::stringstream s, s1, s2;
				s1 << i+k << "," << feedback->marker_name;
				s2 << feedback->marker_name << "," << i+k;
				s << i+k;
				if(server->get(s.str(), marker)){
					if(server2->get(s1.str(), marker)){
						makelineMarker(s.str(), feedback->marker_name, 0);
						server2->applyChanges();
					}
					if(server2->get(s2.str(), marker)){
						makelineMarker(feedback->marker_name, s.str(), 0);
						server2->applyChanges();
					}
				}
				else{
					i -= 1;
					k += 1;
				}
			}
			break;
		}

		case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      // ROS_INFO_STREAM( s.str() << ": mouse down" << "." );
		break;

		case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      // ROS_INFO_STREAM( s.str() << ": mouse up" << "." );
		break;

	}
	server->applyChanges();

}

void processFeedback2( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	std::ostringstream s1;
	s1 << "Feedback from marker " << feedback->marker_name;
	std::vector<std::string> vec;
	boost::algorithm::split(vec, feedback->marker_name, boost::is_any_of(","));
	int marker1 = atoi(vec[0].c_str());
	int marker2 = atoi(vec[1].c_str());
	stringstream s, ss;
	s << marker1;
	ss << marker2;

	switch ( feedback->event_type )
	{
		case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		break;

		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		{
			ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << "." );
			if(feedback->menu_entry_id == 1)
				ROS_INFO("Connection from Marker %d to Marker %d", marker1, marker2);
			else if(feedback->menu_entry_id == 2)
				server2->erase(feedback->marker_name);
			else if(feedback->menu_entry_id == 3){
				makelineMarker(ss.str(), s.str(), 0);
				server2->applyChanges();
			}
			else if(feedback->menu_entry_id == 4){
				InteractiveMarker marker;
				server2->get(feedback->marker_name, marker);
				server2->erase(feedback->marker_name);
				if(marker.description.compare("teb") == 0)
					makelineMarker(s.str(), ss.str(), 1);
				else
					makelineMarker(s.str(), ss.str(), 0);


			}
			break;
		}

		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		break;

		case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
		break;

		case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
		break;
	}

	server2->applyChanges();
}


Marker makeLine( InteractiveMarker &msg, bool bidirectional){
	Marker marker;

	marker.type = Marker::LINE_LIST;
	marker.scale.x = msg.scale * 0.45;
	marker.scale.y = msg.scale * 0.45;
	marker.scale.z = msg.scale * 0.45;
	if(!bidirectional){
		marker.color.r = 100;
		marker.color.g = 0;
		marker.color.b = 0;
	}
	else{
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 100;
	}
	marker.color.a = 1.0;

	return marker;
}

Marker makeBox( InteractiveMarker &msg, bool station){
	Marker marker;

	marker.type = Marker::CUBE;
	marker.scale.x = msg.scale * 0.45;
	marker.scale.y = msg.scale * 0.45;
	marker.scale.z = msg.scale * 0.45;
	if(!station){
		marker.color.r = 255;
		marker.color.g = 255;
		marker.color.b = 0;
	}
	else{
		marker.color.r = 0;
		marker.color.g = 255;
		marker.color.b = 0;
	}
	marker.color.a = 1.0;

	return marker;
}

Marker makeText( InteractiveMarker &msg, std::string name){
	Marker marker;

	marker.type = Marker::TEXT_VIEW_FACING;
	marker.scale.z = msg.scale;
	marker.color.r = 100;
	marker.color.g = 150;
	marker.color.b = 0;
	marker.color.a = 1.0;
	marker.text = name;
	marker.pose.position.z = 1.0;

	return marker;
}

void makelineMarker(std::string marker1, std::string marker2, int localplanner){
	InteractiveMarker int_marker;
	InteractiveMarker int_marker_point;
	int_marker.header.frame_id = "map";
	int_marker.scale = 0.25;

	std::stringstream s, ss1, ss2;
	if(localplanner)
		s << "dwa";
	else
		s << "teb";
	ss1 << marker1 << "," << marker2;
	ss2 << marker2 << "," << marker1;
	int_marker.name = ss1.str();
	int_marker.description = s.str();

	geometry_msgs::Point p;
	InteractiveMarkerControl control3;
	control3.always_visible = true;
	control3.interaction_mode = InteractiveMarkerControl::MENU;
	control3.name = "menu_only_control";
	if(server2->get(ss2.str(), int_marker_point)){
		control3.markers.push_back( makeLine(int_marker, true) );
		p.z = 0.10;
	}
	else{
		control3.markers.push_back( makeLine(int_marker, false) );
		p.z = 0.05;
	}
	server->get(marker1, int_marker_point);
	p.x = int_marker_point.pose.position.x;
	p.y = int_marker_point.pose.position.y;
	control3.markers[0].points.push_back(p);
	server->get(marker2, int_marker_point);
	p.x = int_marker_point.pose.position.x;
	p.y = int_marker_point.pose.position.y;
	control3.markers[0].points.push_back(p);
	int_marker.controls.push_back(control3);

	server2->insert(int_marker);
	server2->setCallback(int_marker.name, &processFeedback2);
	menu_handler2.apply( *server2, int_marker.name );
}

void make6DofMarker(int waypoint_count, geometry_msgs::Pose pose, bool station){
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "map";
	int_marker.pose = pose;
	int_marker.scale = 1;

	std::stringstream s, ss;
	s << "Point " << waypoint_count;
	ss << waypoint_count;
	int_marker.name = ss.str();
	int_marker.description = s.str();

	InteractiveMarkerControl control, control3;
	control3.always_visible = true;

	// control2.interaction_mode = InteractiveMarkerControl::BUTTON;
	// control2.name = "box";
	// control2.markers.push_back( makeBox(int_marker, station) );
	// int_marker.controls.push_back(control2);

	control3.interaction_mode = InteractiveMarkerControl::MENU;
	control3.name = "menu_only_control";
	control3.markers.push_back( makeBox(int_marker, station) );
	control3.markers.push_back( makeText(int_marker, int_marker.name) );
	int_marker.controls.push_back(control3);

	// control3.interaction_mode = InteractiveMarkerControl::NONE;
	// control3.name = "text";
	// control3.markers.push_back( makeText(int_marker, int_marker.name) );
	// int_marker.controls.push_back(control3);

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "move_x";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "move_y";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
	menu_handler.apply( *server, int_marker.name );
}

void displaypoints(string filename){
	server->clear();
	server->applyChanges();
	server2->clear();
	server2->applyChanges();
	waypoint_count = 0;

	std::vector< std::vector<std::string> > dataList;
	ifstream file(filename.c_str());
	string line = "";
	while (getline(file, line)){
		vector<string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
		dataList.push_back(vec);
	}
	file.close();
	vector<string> vec;
	geometry_msgs::Pose pose;
	for(int i =0; i<dataList.size(); i++){	
		pose.position.x = atof(dataList[i][0].c_str());
		pose.position.y = atof(dataList[i][1].c_str());
		pose.position.z = atof(dataList[i][2].c_str());
		pose.orientation.x = atof(dataList[i][3].c_str());
		pose.orientation.y = atof(dataList[i][4].c_str());
		pose.orientation.z = atof(dataList[i][5].c_str());
		pose.orientation.w = atof(dataList[i][6].c_str());

		if(atoi(dataList[i][8].c_str()))
			make6DofMarker(++waypoint_count, pose, false );
		else
			make6DofMarker(++waypoint_count, pose, true );
	}
	server->applyChanges();
}


void displayadjacency(string filename){
	std::vector< std::vector<std::string> > dataList;
	ifstream file(filename.c_str());
	string line = "";
	while (getline(file, line)){
		vector<string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
		dataList.push_back(vec);
	}
	file.close();
	vector<string> vec;
	geometry_msgs::Pose pose;
	for(int i =0; i<dataList.size(); i++){	
		for(int j=1; j<dataList[i].size() - 1; j++){
			stringstream s1,s2;
			s1 << dataList[i][0];
			s2 << dataList[i][j];
			makelineMarker(s1.str(), s2.str(), 0);
			server2->applyChanges();
		}
	}
}
void goalstatusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
	if(!msg->status_list.empty()){
		char buffer[200];
		std::size_t length = msg->status_list[msg->status_list.size() - 1].text.copy(buffer,17,0);
		buffer[length]='\0';
		//std::cout << msg->status_list[msg->status_list.size() - 1].text << std::endl;
		if(strcmp(buffer, "Goal reached.") == 0 || mission_complete){   

			reached_goal_ = true;

		}
		else{
			reached_goal_ = false;

		}
	}
}

void goalstatusCallback1(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
	if(!msg->status_list.empty()){
		char buffer[200];
		std::size_t length = msg->status_list[msg->status_list.size() - 1].text.copy(buffer,17,0);
		buffer[length]='\0';
		//std::cout << msg->status_list[msg->status_list.size() - 1].text << std::endl;
		if(strcmp(buffer, "Goal reached.") == 0 || mission_complete){
			reached_goal_ = true;
		}
		else{
			reached_goal_ = false;
		}
	}
}


void gCallback(const std_msgs::Int32ConstPtr& msg){
	ROS_INFO("Received done");
	if(msg->data == 1 || mission_complete){
		ros::Duration(1).sleep();
		reached_goal_ = true;
	}
	else
		reached_goal_ = false;
}

void missionCallback(const std_msgs::StringConstPtr& msg){
	ROS_INFO("Recieved mission: %s", msg->data.c_str());
	std::string mission;
	mission = (char *)msg->data.c_str();
	missionPlanner(mission);
}

void pointsCallback(const std_msgs::StringConstPtr& msg){
	ROS_INFO("Recieved files");
	std::string files;
	files = (char *)msg->data.c_str();
	std::vector<std::string> vec;
	boost::algorithm::split(vec, files, boost::is_any_of("+"));
	std::string points_file = vec[0].c_str();
	std::string adjacency_file = vec[1].c_str();

	ifstream src_points(points_file.c_str(), ios::binary);
	if(!src_points.is_open()){
		ROS_INFO("Invalid Points File");
		return;
	}
	stringstream filename;
	std::string workspace;
	ros::param::get("/workspace", workspace);
	filename << "/home/" << getenv("USER") <<"/"<<workspace<<"/src/navgraph_solver/navgraphs/points_rviz.csv";
	if(points_file.compare(filename.str()) != 0){
		ofstream dst_points(filename.str().c_str(), ios::binary);
		istreambuf_iterator<char> begin_source(src_points);
		istreambuf_iterator<char> end_source;
		ostreambuf_iterator<char> begin_dest(dst_points); 
		copy(begin_source, end_source, begin_dest);
		src_points.close();
		dst_points.close();
		displaypoints(filename.str());
	}
	else
		displaypoints(points_file);

	if(!adjacency_file.empty()){
		ifstream src_adjacency(adjacency_file.c_str(), std::ios::binary);
		if(!src_adjacency.is_open()){
			ROS_INFO("Invalid Adjacency File");
			return;
		}

		stringstream adjacency;
		adjacency << "/home/" << getenv("USER") <<"/"<< workspace <<"/src/navgraph_solver/navgraphs/adjacency_rviz.csv";
		if(adjacency_file.compare(adjacency.str()) != 0){
			ofstream dst_adjacency(adjacency.str().c_str(), ios::binary);
			istreambuf_iterator<char> begin_source(src_adjacency);
			istreambuf_iterator<char> end_source;
			ostreambuf_iterator<char> begin_dest(dst_adjacency); 
			copy(begin_source, end_source, begin_dest);
			src_adjacency.close();
			dst_adjacency.close();
			displayadjacency(adjacency.str());
		}
		else{
			displayadjacency(adjacency_file);
		}
	}
}

void markerCallback(const geometry_msgs::PointStampedConstPtr& msg){
	int name = 0 ; 
	geometry_msgs::Pose pose;
	pose.position = msg->point;
	pose.position.z = 0;
	pose.orientation.w = 1;

	if(activate_proh==0) {
		make6DofMarker(++waypoint_count, pose, false );
		server->applyChanges();
	}
	else {
		ROS_INFO("publish point tool is controlled by PROHIBITEDZONEPANEL apply modifications or save to enable it in WAYPOINTPANEL!") ; 
	}
}

void add_station(){

	make6DofMarker(++waypoint_count, robotPose, true );
	server->applyChanges();
}

void add_waypoint(){

	make6DofMarker(++waypoint_count, robotPose, false );
	server->applyChanges();
}


void launcherCallback(const std_msgs::Int32ConstPtr& msg){
	if(msg->data == 3){
		//initialize_move_base = true;
	}

	//ABORT MISSION 
	else if(msg->data == 4){
		if(!mission_complete)
			//update abort mission global variable
			abort_mission = true;
	}

	//SAVE POINTS AND LINKS IN FILE 
	else if(msg->data == 1){
		ROS_INFO("Saving OnScreen Points");
		InteractiveMarker marker;
		stringstream filename, adjacency;
		std::string workspace;
		ros::param::get("/workspace", workspace);
		filename << "/home/" << getenv("USER") <<"/"<< workspace << "/src/navgraph_solver/navgraphs/points_rviz.csv";
		adjacency << "/home/" << getenv("USER") <<"/"<< workspace << "/src/navgraph_solver/navgraphs/adjacency_rviz.csv";

		//save points
		int k = 1;
		if(!server->empty()){
			ofstream points_file(filename.str().c_str());
			for(int i = 0; i<server->size(); i++){
				std::stringstream s;
				s << i+k;
				if(server->get(s.str(), marker)){
					if (points_file.is_open())
						points_file << marker.pose.position.x << "," << marker.pose.position.y << "," <<
					marker.pose.position.z << "," << marker.pose.orientation.x << "," <<
					marker.pose.orientation.y << "," << marker.pose.orientation.z << "," <<
					marker.pose.orientation.w << "," << 0 << "," <<  
					int(marker.controls[0].markers[0].color.r) << "\n";
					else
						ROS_INFO("Could not open file");
				}
				else{
					i -= 1;
					k += 1;
				}
			}
			points_file.close();
		}
		//save adjacency 
		int n = 1, m = 1;
		if(!server2->empty()){
			ofstream adjacency_file (adjacency.str().c_str());
			for(int i = 0; i<server->size(); i++){
				stringstream sss;
				sss << i+m;
				if(server->get(sss.str(), marker)){
					if (adjacency_file.is_open())
						adjacency_file << i+m << ",";
					for(int j = 0; j<server->size(); j++){
						std::stringstream s, ss;
						s << i+m << "," << j+n;
						ss << j+n;
						if(server2->get(s.str(), marker)){
							if (adjacency_file.is_open())
								adjacency_file << j+n << ",";
							else
								ROS_INFO("Could not open file");
						}
						else{
							if(!server->get(ss.str(), marker)){
								j -= 1;
								n += 1;
							}
						}
					}
					if (adjacency_file.is_open())
						adjacency_file << "\n";
				}
				else{
					i -= 1;
					m += 1;
				}
			}
		}
	}
	
	
	//Publish point tool control 
	else if(msg->data == 10){
        //activate_proh = 1 : publish point tool is controlled by 
        //PROHIBITEDZONEPANEL apply modifications or save to enable it in WAYPOINTPANEL!
		activate_proh = 1;
	}
	else if(msg->data == 11){
        //activate_proh = 0 : publish point tool is controlled by
        //WAYPOINTPANEL press "add" button in PROHIBITEDZONEPANEL to enable it!
		activate_proh = 0;
	}
    
    //DELETE POINTS  
	else if(msg->data == 5){
		server->clear();
		server->applyChanges();
		server2->clear();
		server2->applyChanges();
		waypoint_count = 0;
	}
	//DELETE ADJACECNCY 
	else if(msg->data == 6){
		server2->clear();
		server2->applyChanges();
	}
	//ADD WAYPOINT 
	else if (msg->data == 17){
		add_waypoint() ; 
	}
	//ADD STATION 
	else if (msg->data == 18){
		add_station() ; 
	}
}

void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
	robotPose.position = msg->pose.pose.position  ; 
	robotPose.orientation = msg->pose.pose.orientation ; 
}

int main(int argc, char **argv){
	ros::init(argc, argv, "station_navigator");
	ros::NodeHandle nh;

	ros::Rate r(10);

	

	// tf::TransformListener tf(ros::Duration(10));
	// move_base::MoveBase move_base(tf);

	menu_handler.insert( "Waypoint", &processFeedback );
	menu_handler.insert( "Station", &processFeedback );
	menu_handler.insert( "Delete", &processFeedback );
	menu_handler.insert( "Mark for Linking", &processFeedback );
	menu_handler2.insert( "Query", &processFeedback2 );
	menu_handler2.insert( "Delete", &processFeedback2 );
	menu_handler2.insert( "Make Bidirectional", &processFeedback2 );
	menu_handler2.insert( "Toggle TEB/DWA", &processFeedback2 );

	server.reset( new interactive_markers::InteractiveMarkerServer("markers","",false) );
	server2.reset( new interactive_markers::InteractiveMarkerServer("connections","",false) );

	// wp_client = nh.serviceClient<navgraph_solver::waypoint>("/waypoint_service");


	next_station = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // next_station = nh.advertise<geometry_msgs::PoseStamped>("/flex_nav_global/goal", 10);


    //ros::Subscriber goal_status_sub = nh.subscribe("/goal_status", 10000, goalstatusCallback2); 

	localplanner = nh.advertise<std_msgs::Bool>("/localplanner", 10);
	 ros::Subscriber goal_status = nh.subscribe("/move_base/status", 1000, goalstatusCallback);   
	//ros::Subscriber goal_status1 = nh.subscribe("/low_level_planner1/status", 1000, goalstatusCallback1);
	//ros::Subscriber goal_status = nh.subscribe("/low_level_planner/status", 1000, goalstatusCallback);

	ros::Subscriber launcher_sub = nh.subscribe("/panel_msgs", 10, launcherCallback);
	ros::Subscriber mission_sub = nh.subscribe("/mission", 10, missionCallback);
	ros::Subscriber files_sub = nh.subscribe("/points_files", 10, pointsCallback);
	ros::Subscriber marker = nh.subscribe("/clicked_point", 10, markerCallback);
	ros::Subscriber go = nh.subscribe("/listen", 1000, gCallback);
	ros::Subscriber  pose_sub2 = nh.subscribe("/amcl_pose", 10, robotPoseCallback);


	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();
	
	return 0;
}
