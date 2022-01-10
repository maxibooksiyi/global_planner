#include <global_planner/rrtBase.h>

namespace rrt{
	template <std::size_t N>
	rrtBase<N>::rrtBase(){};

	// Constructor:
	template <std::size_t N>
	rrtBase<N>::rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR) 
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR){
		this->start_ = start;
		this->goal_ = goal;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	void rrtBase<N>::nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear){
		this->ktree_.nearestNeighbor(qKey, qNear);
	}

	template <std::size_t N>
	void rrtBase<N>::newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew){
		// TODO: implement steer function
	}

	template <std::size_t N>
	void rrtBase<N>::backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan){
		KDTree::Point<N> ptr = qGoal;
		while (ptr != this->emptyToken_){
			plan.push_back(this->parent_[ptr]);
		}
	}

	template <std::size_t N>
	bool rrtBase<N>::isReach(const KDTree::Point<N>& q){
		return KDTree::Distance(q, this->goal_) <= this->dR_;
	}

	template <std::size_t N>
	void rrtBase<N>::addVertex(const KDTree::Point<N>& qNew){
		this->ktree_.insert(qNew);
	}

	template <std::size_t N>
	void rrtBase<N>::addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew){
		this->parent_[qNew] = qNear;
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

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getEnvBox(){
		return this->envBox_;
	}

	template <std::size_t N>
	double rrtBase<N>::getReachRadius(){
		return this->dR_;
	}
}