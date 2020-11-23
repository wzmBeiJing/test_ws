#include <navgraph_solver/my_astar.h>
//#include <iostream>
//#include <fstream>
#include <iterator>
#include <algorithm>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
using namespace std; 

namespace navgraph_global_planner{

	AStar::AStar(){
		NODES = 0;
	}

	AStar::~AStar(){
		NODES = 0;
	}

	// Creating a shortcut for int, int pair type 
	typedef pair<int, int> Pair; 

	// A Utility Function to check whether given cell (row, col) 
	// is a valid cell or not. 
	bool AStar::isValid(int node) 
	{ 
		// Returns true if node number
		// is in range 
		return (node >= 0) && (node < NODES);
	} 

	// A Utility Function to check whether destination cell has 
	// been reached or not 
	bool AStar::isDestination(int src, int dest) 
	{ 
		if (src==dest) 
			return (true); 
		else
			return (false); 
	} 

	// A Utility Function to calculate the distance. 
	double AStar::calculateDistance(int a, int b) 
	{ 
		// Return using the distance formula
		float ax = atof(dataList[a][0].c_str());
		float ay = atof(dataList[a][1].c_str());
		float bx = atof(dataList[b][0].c_str());
		float by = atof(dataList[b][1].c_str());
		return ((double)sqrt ((ax-bx)*(ax-bx) 
							+ (ay-by)*(ay-by))); 
	} 

	// A Utility Function to trace the path from the source 
	// to destination 
	void AStar::tracePath(cell *cellDetails, int dest, std::vector<geometry_msgs::Pose> &totalpath) 
	{ 
		printf ("The Path is "); 
		int node = dest; 
		totalpath.clear();
		stack<int> Path;
		pathcost = cellDetails[node].f; 

		while (!(cellDetails[node].parent == node))
		{ 
			Path.push (node); 
			int temp_node = cellDetails[node].parent; 
			node = temp_node; 
		}

		Path.push (node);

		// ofstream output;
		// output.open ("output.csv");
		while (!Path.empty()) 
		{ 
			int p = Path.top(); 
			Path.pop(); 
			geometry_msgs::Pose pose_;
			printf("-> (%d) ",p);
			if(Path.empty()){
			// 	for(int i = 0; i<8; i++)
			// 		output << dataList[p][i] << ",";
			// 	output << dataList[p][8] << "\n";

				pose_.position.x = atof(dataList[p][0].c_str());
				pose_.position.y = atof(dataList[p][1].c_str());
				pose_.position.z = atof(dataList[p][2].c_str());
				pose_.orientation.x = atof(dataList[p][3].c_str());
				pose_.orientation.y = atof(dataList[p][4].c_str());
				pose_.orientation.z = atof(dataList[p][5].c_str());
				pose_.orientation.w = atof(dataList[p][6].c_str());
				totalpath.push_back(pose_);

			}
			else{
				// for(int i = 0; i<3; i++)
				// 	output << dataList[p][i] << ",";
				int p_next = Path.top();
				double yaw = atan2(atof(dataList[p_next][1].c_str()) - atof(dataList[p][1].c_str()),
							 atof(dataList[p_next][0].c_str()) - atof(dataList[p][0].c_str()));
				geometry_msgs::Quaternion angles;
				angles = tf::createQuaternionMsgFromYaw(yaw);
				// output << angles.x << ",";
				// output << angles.y << ",";
				// output << angles.z << ",";
				// output << angles.w << ",";
				// output << dataList[p][7] << ",";
				// output << dataList[p][8] << "\n";

				pose_.position.x = atof(dataList[p][0].c_str());
				pose_.position.y = atof(dataList[p][1].c_str());
				pose_.position.z = atof(dataList[p][2].c_str());
				pose_.orientation.x = angles.x;
				pose_.orientation.y = angles.y;
				pose_.orientation.z = angles.z;
				pose_.orientation.w = angles.w;
				totalpath.push_back(pose_);
			}
		}
		printf ("\n"); 
		// output.close();

		return; 
	} 

	// A Function to find the shortest path between 
	// a given source cell to a destination cell according 
	// to A* Search Algorithm 
	void AStar::aStarSearch(int src, int dest, std::vector<geometry_msgs::Pose> &totalpath) 
	{ 
		pathcost = 0;
		// If the source is out of range 
		if (isValid (src) == false) 
		{ 
			printf ("Source is invalid\n"); 
			return; 
		} 

		// If the destination is out of range 
		if (isValid (dest) == false) 
		{ 
			printf ("Destination is invalid\n"); 
			return; 
		} 

		// If the destination cell is the same as source cell 
		if (isDestination(src, dest) == true) 
		{ 
			printf ("We are already at the destination\n"); 
			return; 
		} 

		// Create a closed list and initialise it to false which means 
		// that no cell has been included yet 
		// This closed list is implemented as a boolean array 
		bool closedList[NODES]; 
		memset(closedList, false, sizeof (closedList)); 

		// Declare an array of structure to hold the details 
		//of that cell 
		cell *cellDetails;
		cellDetails = new cell[NODES];

		int i;

		for (i=0; i<NODES; i++) { 
			cellDetails[i].f = FLT_MAX; 
			cellDetails[i].g = FLT_MAX; 
			cellDetails[i].h = FLT_MAX; 
			cellDetails[i].parent = -1; 
		} 

		// Initialising the parameters of the starting node 
		i = src; 
		cellDetails[i].f = 0.0; 
		cellDetails[i].g = 0.0; 
		cellDetails[i].h = 0.0; 
		cellDetails[i].parent = i; 

		/* 
		Create an open list having information as- 
		<f, i> 
		where f = g + h, 
		and i is the node index of that cell 
		Note that 0 <= i <= NODES-1
		This open list is implenented as a set of pair.*/
		set<Pair> openList; 
		// Put the starting cell on the open list and set its 
		// 'f' as 0 
		openList.insert(make_pair (0.0, i));

		// We set this boolean value as false as initially 
		// the destination is not reached. 
		bool foundDest = false; 

		while (!openList.empty()) { 
			Pair p = *openList.begin(); 

			// Remove this vertex from the open list 
			openList.erase(openList.begin()); 

			// Add this vertex to the closed list 
			i = p.second; 
			closedList[i] = true; 
		
			//Generating all the successors of this cell 

			// To store the 'g', 'h' and 'f' of the 8 successors 
			double gNew, hNew, fNew; 

			for(int k = 0; k<NODES; k++){
				if(adM[i][k]){
					// Only process this cell if this is a valid one 
					if (isValid(k) == true){ 
						// If the destination cell is the same as the 
						// current successor 
						if (isDestination(k, dest) == true) { 
							// Set the Parent of the destination cell 
							cellDetails[k].parent = i; 
							printf ("The destination cell is found\n"); 
							tracePath (cellDetails, dest, totalpath); 
							foundDest = true; 
							return; 
						} 
						// If the successor is already on the closed 
						// list or if it is blocked, then ignore it. 
						// Else do the following 
						else if (closedList[k] == false){ 
							gNew = cellDetails[i].g + calculateDistance (k, i);; 
							hNew = calculateDistance (k, dest); 
							fNew = gNew + hNew; 


							// If it isn’t on the open list, add it to 
							// the open list. Make the current square 
							// the parent of this square. Record the 
							// f, g, and h costs of the square cell 
							//			 OR 
							// If it is on the open list already, check 
							// to see if this path to that square is better, 
							// using 'f' cost as the measure. 
							if (cellDetails[k].f == FLT_MAX || 
									cellDetails[k].f > fNew) 
							{ 
								openList.insert( make_pair(fNew, k)); 

								// Update the details of this cell 
								cellDetails[k].f = fNew; 
								cellDetails[k].g = gNew; 
								cellDetails[k].h = hNew; 
								cellDetails[k].parent = i; 
							} 
						} 
					} 
				}
			}
		}

		// When the destination cell is not found and the open 
		// list is empty, then we conclude that we failed to 
		// reach the destiantion cell. This may happen when the 
		// there is no way to destination cell (due to blockages) 
		if (foundDest == false) 
			printf("Failed to find the Destination Cell\n"); 
		return; 
	}


	// Driver program to test above function 
	// int main(int argc, char **argv) { 
	void AStar::initialize(const std::string& points, const std::string& adjacency){
		initialized = false;
		NODES = 0;
		pathcost = 0;
		dataList.clear();

		//read CSV file for points
		ifstream file(points.c_str());
		string line = "";
		while (getline(file, line)){
			vector<string> vec;
			boost::algorithm::split(vec, line, boost::is_any_of(","));
			dataList.push_back(vec);
		}
		file.close();
		vector<string> vec;
		float data;
		cout << "Read points" << endl;
		for(int i =0; i<dataList.size(); i++){	
			vec = dataList[i];
			for(int j=0; j<vec.size();j++)
			{
				data = atof(dataList[i][j].c_str());
				cout<<data << " , ";
			}
			cout<<endl;
		}
		NODES = dataList.size();

		//Create adjacency matrix
		/* Description of the Grid- 
		1--> The cell is not blocked 
		0--> The cell is blocked */
		ifstream file2(adjacency.c_str());
		vector<vector<string> > adj;
		adM = new int *[NODES];
		for(int i = 0; i <NODES; i++)
	    	adM[i] = new int[NODES];
		for(int i=0; i<NODES; i++){
			for(int j=0; j<NODES; j++){
				adM[i][j] = 0;
			}
		}
		line = "";
		while (getline(file2, line)){
			vector<string> vec;
			boost::algorithm::split(vec, line, boost::is_any_of(","));
			adj.push_back(vec);
		}
		file2.close();
		cout << "Adjacency Matrix" << endl;

		for(int i =0; i<adj.size(); i++){	
			vec = adj[i];
			for(int j=0; j<vec.size()-1;j++)
			{
				int data = atoi(adj[i][j].c_str());
				adM[i][data-1] = 1;
				// adM[data][i] = 1;
			}
		}

		for(int i=0; i<NODES; i++){
			for(int j=0; j<NODES; j++){
				cout << adM[i][j] << " ";
			}
			cout << endl;
		}
		initialized = true; 

		// int src = 18;
		// int dest = 27;

		// aStarSearch(dataList, adM, src, dest); 

		// return(0); 
	}

	int AStar::nearestnode(geometry_msgs::PoseStamped pose){
		int max_node = 0;
		double max_distance = FLT_MAX;
		float ax,ay,bx,by;
		double distance;
		for(int node = 0; node < dataList.size(); node++){
			ax = atof(dataList[node][0].c_str());
			ay = atof(dataList[node][1].c_str());
			bx = pose.pose.position.x;
			by = pose.pose.position.y;
			distance =  (double)sqrt ((ax-bx)*(ax-bx) 
							+ (ay-by)*(ay-by));
			if(distance<max_distance){
				max_distance = distance;
				max_node = node;
			}
		}
		ROS_INFO("Nearest node index: %d\n", max_node);
		return max_node;

	}

}
