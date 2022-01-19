#include <ros/ros.h>
#include <global_planner/rrtOctomap.h>
#include <global_planner/utils.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
	ros::init(argc, argv, "RRT_test_node");
	ros::NodeHandle nh;
	cout << "test for rrt base class~" << endl;
	const int N = 3;
	KDTree::Point<N> start_point; start_point[0] = 0; start_point[1] = 0; start_point[2] = 1;
	KDTree::Point<N> goal_point; goal_point[0] = 2.1; goal_point[1] = 1.123; goal_point[2] = 1000.05;
	std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_;
	parent_[goal_point] = start_point;

	cout << start_point + goal_point << endl;
	cout << start_point - goal_point << endl;
	cout << 5 * start_point << endl;
	cout << parent_[goal_point] << endl;

	// Test:

	std::vector<double> start {0.0, 0.0, 1.0};
	std::vector<double> goal {-2.938, 12.50, 1.0};
	std::vector<double> collisionBox, envBox;

	double delQ, dR, connectGoalRatio, mapRes, timeout;
	nh.getParam("/collision_box", collisionBox);
	nh.getParam("/env_box", envBox);
	nh.getParam("/rrt_incremental_distance", delQ);
	nh.getParam("/rrt_incremental_distance", connectGoalRatio);
	nh.getParam("/goal_reach_distance", dR);
	nh.getParam("/map_resolution", mapRes);
	nh.getParam("/timeout", timeout);
	
	rrt::rrtOctomap<N> rrtplanner (nh, start, goal, collisionBox, envBox, mapRes, delQ, dR, connectGoalRatio, timeout);
	cout << rrtplanner << endl;

	cout << "plan: " << endl;
	std::vector<KDTree::Point<N>> plan;
	rrtplanner.makePlan(plan);

	// visualize sample points:
	std::vector<KDTree::Point<N>> sp;
	rrtplanner.getSamplePoints(sp);
	std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> edge_dict = rrtplanner.getParentDict();


	visualization_msgs::MarkerArray sp_vis_array = wrapRRTVisMsg(sp, edge_dict, 1, 0, 0);
	visualization_msgs::MarkerArray path_vis_array = wrapPathMsg(plan);
	ros::Publisher sp_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/rrt_sample_points", 1);
	ros::Publisher path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/rrt_planned_path", 10);
	ros::Rate (10);
	while (ros::ok()){
		sp_vis_pub.publish(sp_vis_array);
		path_vis_pub.publish(path_vis_array);
	}

	return 0;
}