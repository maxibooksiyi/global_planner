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
}