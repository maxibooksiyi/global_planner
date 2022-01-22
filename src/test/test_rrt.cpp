#include <ros/ros.h>
#include <global_planner/rrtOctomap.h>
#include <global_planner/utils.h>
#include <geometry_msgs/PointStamped.h>

using std::cout;
using std::endl;


bool newMsg = false;
std::vector<double> newPoint {0, 0, 1.0};
void clickedPointCB(const geometry_msgs::PointStamped::ConstPtr& cp){
	newPoint[0] = cp->point.x;
	newPoint[1] = cp->point.y;
	newPoint[2] = 1.0; // set height to be 1.0 m
	newMsg = true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "RRT_test_node");
	ros::NodeHandle nh;

	// subscriber for clicked start and goal:
	ros::Subscriber clickedPointSub = nh.subscribe("/clicked_point", 1000, clickedPointCB);

	cout << "test for rrt~" << endl;
	const int N = 3;

	// Test start goal:
	// std::vector<double> start {0.0, 0.0, 1.0};
	// std::vector<double> goal {-2.938, 12.50, 1.0};
	std::vector<double> start {0.0, 0.0, 1.0};
	std::vector<double> goal {0.0, 0.0, 1.0};
	std::vector<double> collisionBox, envBox;

	double delQ, dR, connectGoalRatio, mapRes, timeout;
	bool visRRT, visPath;
	nh.getParam("/collision_box", collisionBox);
	nh.getParam("/env_box", envBox);
	nh.getParam("/rrt_incremental_distance", delQ);
	nh.getParam("/rrt_incremental_distance", connectGoalRatio);
	nh.getParam("/goal_reach_distance", dR);
	nh.getParam("/map_resolution", mapRes);
	nh.getParam("/timeout", timeout);
	nh.getParam("/vis_RRT", visRRT);
	nh.getParam("/vis_path", visPath);
	
	rrt::rrtOctomap<N> rrtplanner (nh, start, goal, collisionBox, envBox, mapRes, delQ, dR, connectGoalRatio, timeout, visRRT, visPath);
	cout << rrtplanner << endl;

	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		if (countLoop > 0){
			cout << "[Planner Node]: Request No. " << countLoop << endl;
			std::vector<KDTree::Point<N>> plan;
			rrtplanner.makePlan(plan);
		}

		cout << "[Planner Node]: Wait for start point..." << endl;
		while (ros::ok()){
			if (newMsg){
				start = newPoint;
				rrtplanner.updateStart(start);
				newMsg = false;
				cout << "[Planner Node]: start point OK. (" << start[0] << " " << start[1] << " " << start[2] << ")" << endl;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}

		cout << "[Planner Node]: Wait for goal point..." << endl;
		while (ros::ok()){
			if (newMsg){
				goal = newPoint;
				rrtplanner.updateGoal(goal);
				newMsg = false;
				cout << "[Planner Node]: goal point OK. (" << goal[0] << " " << goal[1] << " " << goal[2] << ")" << endl;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}

		++countLoop;
	}
	// visualize sample points:
	// std::vector<KDTree::Point<N>> sp;
	// rrtplanner.getSamplePoints(sp);
	// std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> edge_dict = rrtplanner.getParentDict();


	// visualization_msgs::MarkerArray sp_vis_array = wrapRRTVisMsg(sp, edge_dict, 1, 0, 0);
	// visualization_msgs::MarkerArray path_vis_array = wrapPathMsg(plan);
	// ros::Publisher sp_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/rrt_sample_points", 1);
	// ros::Publisher path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/rrt_planned_path", 10);
	;
	// ros::Rate r(1);
	// while (ros::ok()){
	// 	// sp_vis_pub.publish(sp_vis_array);
	// 	// path_vis_pub.publish(path_vis_array);
	// 	r.sleep();
	// }



	return 0;
}