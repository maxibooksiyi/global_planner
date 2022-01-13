#include <global_planner/rrtOctomap.h>

namespace rrt{
	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(){}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR)
	: rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR){
		// TODO
		double test = 1;
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR)
	: rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR){
		// TODO
		double test = 1;
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateMap(){
		// TODO
		double test = 1;
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollision(const KDTree::Point<N>& q){
		// TODO
		return false;
	}

	template <std::size_t N>
	void rrtOctomap<N>::randomConfig(KDTree::Point<N>& qRand){
		// TODO
		double test = 1;
	}

	template <std::size_t N>
	void rrtOctomap<N>::makePlan(std::vector<KDTree::Point<N>>& plan){
		// TOOD
		double test = 1;
	}

}

