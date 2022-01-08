#include <ros/ros.h>
#include <global_planner/KDTree.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
	cout << "Test for KDtree!" << endl;
	
	KDTree::Point<3> p1;
	p1[0] = 0; p1[1] = 0; p1[2] = 0;

	KDTree::Point<3> p2;
	p2[0] = 1; p2[1] = 1; p2[2] = 1;

	KDTree::Point<3> p3; 
	p3[0] = -1; p3[1] = 1; p3[2] = 0;

	KDTree::Point<3> p4;
	p4[0] = -0.5; p4[1] = -0.5; p4[2] = 0.5;

	KDTree::KDTree<3, int> tree;
	tree.insert(p1, 1);
	tree.insert(p2, 2);
	tree.insert(p3, 3);

	cout << tree.kNNValue(p1, 2) << endl;
	cout << "p1: " << p1 << endl;
	cout << "p2: " << p2 << endl;
	cout << "p3: " << p3 << endl; 

	KDTree::Point<3> pnn;
	tree.nearestNeighbor(p4, pnn);
	cout << "nearest neighbor for p4 " << p4  << " is: " << pnn << endl;

	std::vector<KDTree::Point<3>> knnVec;
	tree.knn(p4,5,knnVec);
	int i = 0;
	for (KDTree::Point<3> p: knnVec){
		cout << i << "th nearest: " << p << ", with distance: " << Distance(p, p4) << endl;
		++i;
	}

	double neighborhood_dis = 100; int max_neighor = 2;
	std::vector<KDTree::Point<3>> neighborhood;
	tree.boundedRangeSearch(p4, neighborhood_dis, max_neighor, neighborhood);

	for (KDTree::Point<3> p: neighborhood){
		cout << p << ", Distance: " << Distance(p, p4) << endl;
	}

	return 0;
}