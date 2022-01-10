#include <ros/ros.h>
#include <global_planner/rrtBase.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
	cout << "test for rrt base class~" << endl;
	const int N = 3;
	KDTree::Point<N> start_point; start_point[0] = 1.1; start_point[1] = 0; start_point[2] = -1;
	KDTree::Point<N> goal_point; goal_point[0] = 2.1; goal_point[1] = 1.123; goal_point[2] = 1000.05;
	std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_;
	parent_[goal_point] = start_point;

	cout << parent_[goal_point] << endl;
	rrt::rrtBase<N> r ();
	return 0;
}