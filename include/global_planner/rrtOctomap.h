/*
*	File: rrtOctomap.h
*	---------------
*   RRT planner class based on Octomap.
*/
#ifndef RRTOCTOMAP_H
#define RRTOCTOMAP_H
#include<ros/ros.h>
#include <global_planner/rrtBase.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

namespace rrt{
	template <std::size_t N>
	class rrtOctomap : public rrtBase<N>{
	private:
		ros::NodeHandle nh_;

	protected:
		ros::ServiceClient mapClient_;

		double mapRes_;
		octomap::OcTree* map_;
		

	public:
		// default constructor
		rrtOctomap();

		// constructor using point format
		rrtOctomap(const ros::NodeHandle& nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double mapRes);

		// constructor using vector format
		rrtOctomap(const ros::NodeHandle& nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double mapRes);

		// update octomap
		virtual void updateMap();	
		
		// collision checking function based on map and collision box:
		virtual bool checkCollision(const KDTree::Point<N>& q);

		// random sample in valid space (based on current map)
		virtual void randomConfig(KDTree::Point<N>& qRand);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan);

		double getMapRes();
	};


	// ===========================Function Definition=======================================
	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(){}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const ros::NodeHandle& nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double mapRes)
	: nh_(nh), rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR), mapRes_(mapRes){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const ros::NodeHandle& nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double mapRes)
	: nh_(nh), rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR), mapRes_(mapRes){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateMap(){
		octomap_msgs::GetOctomap mapSrv;
		bool service_success = this->mapClient_.call(mapSrv);
		ros::Rate rate(10);
		while (not service_success and ros::ok()){
			service_success = this->mapClient_.call(mapSrv);
			ROS_INFO("Wait for Octomap Service...");
			rate.sleep();
		}
		octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(mapSrv.response.map);
		this->map_ = dynamic_cast<octomap::OcTree*>(abtree);
		this->map_->setResolution(this->mapRes_);
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

	template <std::size_t N>
	double rrtOctomap<N>::getMapRes(){
		return this->mapRes_;
	}


	// =================Operator overload==============================
	template <std::size_t N>
	std::ostream &operator<<(std::ostream &os, rrtOctomap<N> &rrtplanner){
        os << "============INFO============\n";
        os << "[Planner INFO: RRT planner with octomap]\n";
        os << "[Start from " <<  rrtplanner.getStart() << " to " <<  rrtplanner.getGoal() << " \n";
        std::vector<double> collisionBox = rrtplanner.getCollisionBox();
        os << "[Collision Box: " << collisionBox[0] << " " << collisionBox[1] << " " <<  collisionBox[2] << "]\n";
        double res = rrtplanner.getMapRes();
        os << "[Map Res: " << res << "] \n";
        os << "============================";
        return os;
    }
}

#endif