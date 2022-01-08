/*
*	File: rrtBase.h
*	---------------
*   Base class for RRT planner.
*/
#ifndef RRTBASE_H
#define RRTBASE_H
#include <global_planner/KDTree.h>

using std::cout; using std::endl;

namespace rrt{
	template <std::size_t N>
	struct Node{
		KDTree::Point<N> pos;
		Node* parent;
		Node(){
			this->parent = NULL;
		}
		Node(KDTree::Point<N> p){
			this->pos = p;
			this->parent = NULL;
		}
	};

	template <std::size_t N>
	class rrtBase{
	private:
		KDTree::Point<N> start_;
		KDTree::Point<N> goal_;
		KDTree::KDTree<N, int> ktree_; // KDTree

		std::vector<double> collisionBox_;

		double delQ_; // incremental distance
		Node<N> root_;

	public:
		// Default constructor
		rrtBase(); 

		// Constructor
		rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, double delQ);

		// collision checking function based on map and collision box:
		bool checkCollision(KDTree::Point<N>& q);

		// random sample in valid space (based on current map)
		void randomConfig(KDTree::Point<N>& qRand);

		// Find the nearest vertex (node) in the tree
		void nearestVertex(KDTree::Point<N>& qNear);

		// Steer function: basd on delta
		void newConfig(KDTree::Point<N>& qNear, KDTree::Point<N>& qRand);

	};
}

#endif