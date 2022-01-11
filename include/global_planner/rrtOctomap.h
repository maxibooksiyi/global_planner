/*
*	File: rrtOctomap.h
*	---------------
*   RRT planner class based on Octomap.
*/
#ifndef RRTOCTOMAP_H
#define RRTOCTOMAP_H
#include <global_planner/rrtBase.h>

namespace rrt{
	template <std::size_t N>
	class rrtOctomap : public rrtBase<N>{
	private:

	public:
		// default constructor
		rrtOctomap();

		// constructor using point format
		rrtOctomap(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR);

		// constructor using vector format
		rrtOctomap(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR);

		// update octomap
		void updateMap();	
		
		// collision checking function based on map and collision box:
		bool checkCollision(const KDTree::Point<N>& q);

		// random sample in valid space (based on current map)
		void randomConfig(KDTree::Point<N>& qRand);

		// *** Core function: make plan based on all input ***
		void makePlan(std::vector<KDTree::Point<N>>& plan);
	};
}

#endif