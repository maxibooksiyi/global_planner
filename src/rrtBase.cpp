#include <global_planner/rrtBase.h>

namespace rrt{
	template <std::size_t N>
	rrtBase<N>::rrtBase(){};

	// Constructor:
	template <std::size_t N>
	rrtBase<N>::rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, double delQ) 
	: collisionBox_(collisionBox), delQ_(delQ){
		this->start_ = start;
		this->goal_ = goal;
		this->root_.pos = start;
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getStart(){
		return this->start_;
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getGoal(){
		return this->goal_;
	}

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getCollisionBox(){
		return this->collisionBox_;
	}
}