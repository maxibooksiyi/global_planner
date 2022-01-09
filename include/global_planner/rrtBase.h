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
		std::vector<double> collisionBox_; // half of (lx, ly, lz)
		double delQ_; // incremental distance

	protected:
		KDTree::KDTree<N, int> ktree_; // KDTree
		Node<N>* root_;

	public:
		// Default constructor
		rrtBase(); 

		// Constructor
		rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, double delQ);

		virtual ~rrtBase();

		// collision checking function based on map and collision box:
		virtual bool checkCollision(KDTree::Point<N>& q) = 0;

		// random sample in valid space (based on current map)
		virtual void randomConfig(KDTree::Point<N>& qRand) = 0;

		// Find the nearest vertex (node) in the tree
		virtual void nearestVertex(KDTree::Point<N>& qNear) = 0;

		// Steer function: basd on delta
		virtual void newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew) = 0;

		// add the new vertex to the RRT: add to KDTree
		void addVertex(const KDTree::Point<N>& qNew);

		// add the new edge to RRT: add to rrt (????????????????)
		void addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew);

		// return start point:
		KDTree::Point<N> getStart();

		// return goal point:
		KDTree::Point<N> getGoal();

		// return collision box:
		std::vector<double> getCollisionBox();
	};
}

#endif