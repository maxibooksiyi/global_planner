#include <ros/ros.h>
#include <global_planner/rrtBase.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
	cout << "test for rrt base class~" << endl;
	rrt::rrtBase<3> r ();
	return 0;
}