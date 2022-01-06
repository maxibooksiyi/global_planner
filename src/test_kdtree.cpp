#include <ros/ros.h>
#include <global_planner/KDTree.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
	cout << "Test for KDtree!" << endl;
	
	Point<3> p1;
	p1[0] = 0; p1[1] = 0; p1[2] = 0;

	Point<3> p2;
	p2[0] = 1; p2[1] = 1; p1[2] = 1;

	Point<3> p3; 
	p3[0] = -1; p3[1] = 1; p3[2] = 0;


	KDTree<3, int> tree;
	tree.insert(p1, 1);
	tree.insert(p2, 2);
	tree.insert(p3, 3);

	cout << tree.kNNValue(p1, 2) << endl;

	return 0;
}