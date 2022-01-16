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
		std::vector<KDTree::Point<N>> samplePoints_; // for debug and visualization

	protected:
		ros::ServiceClient mapClient_;

		double mapRes_;
		double envLimit_[6];
		double sampleRegion_[6];
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
		void updateSampleRegion();// helper function for update sample region
		
		// collision checking function based on map and collision box: TRUE => Collision
		virtual bool checkCollision(const KDTree::Point<N>& q);
		bool checkCollision(const octomap::point3d& p);
		bool checkCollisionPoint(const octomap::point3d &p);
		bool checkCollisionLine(const KDTree::Point<N>& q1, const KDTree::Point<N>& q2);
		bool checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2);

		// random sample in valid space (based on current map)
		virtual void randomConfig(KDTree::Point<N>& qRand);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan);

		// Helper function for collision checking:
		void point2Octomap(const KDTree::Point<N>& q, octomap::point3d& p);


		double getMapRes();
		void getSamplePoints(std::vector<KDTree::Point<N>>& sp);
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
		// this->map_->setResolution(this->mapRes_);
		double min_x, max_x, min_y, max_y, min_z, max_z;
		this->map_->getMetricMax(max_x, max_y, max_z);
		this->map_->getMetricMin(min_x, min_y, min_z);
		this->envLimit_[0] = min_x; this->envLimit_[1] = max_x; this->envLimit_[2] = min_y; this->envLimit_[3] = max_y; this->envLimit_[4] = min_z; this->envLimit_[5] = max_z;
		this->updateSampleRegion();
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateSampleRegion(){
		double xmin = std::max(this->envBox_[0], this->envLimit_[0]); this->sampleRegion_[0] = xmin;
		double xmax = std::min(this->envBox_[1], this->envLimit_[1]); this->sampleRegion_[1] = xmax;
		double ymin = std::max(this->envBox_[2], this->envLimit_[2]); this->sampleRegion_[2] = ymin;
		double ymax = std::min(this->envBox_[3], this->envLimit_[3]); this->sampleRegion_[3] = ymax;
		double zmin = std::max(this->envBox_[4], this->envLimit_[4]); this->sampleRegion_[4] = zmin;
		double zmax = std::min(this->envBox_[5], this->envLimit_[5]); this->sampleRegion_[5] = zmax;
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollision(const KDTree::Point<N>& q){
		octomap::point3d p;
		this->point2Octomap(q, p);
		return this->checkCollision(p);
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollision(const octomap::point3d& p){
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - this->collisionBox_[0]/2; xmax = p.x() + this->collisionBox_[0]/2;
		ymin = p.y() - this->collisionBox_[1]/2; ymax = p.y() + this->collisionBox_[1]/2;
		zmin = p.z() - this->collisionBox_[2]/2; zmax = p.z() + this->collisionBox_[2]/2;

		for (double x=xmin; x<xmax; x+=this->mapRes_){
			for (double y=ymin; y<ymax; y+=this->mapRes_){
				for (double z=zmin; z<zmax; z+=this->mapRes_){
					if (!this->checkCollisionPoint(octomap::point3d (x, y, z))){
						// do nothing
					}
					else{
						return true;
					}
				}
			}
		}
		return false;
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionPoint(const octomap::point3d &p){
		octomap::OcTreeNode* nptr = this->map_->search(p);
		if (nptr == NULL){
			return true;
		}
		return this->map_->isNodeOccupied(nptr);
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionLine(const KDTree::Point<N>& q1, const KDTree::Point<N>& q2){
		octomap::point3d p1, p2;
		this->point2Octomap(q1, p1);
		this->point2Octomap(q2, p2);
		return this->checkCollisionLine(p1, p2);
	}
	
	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2){
		std::vector<octomap::point3d> ray;
		this->map_->computeRay(p1, p2, ray);
		for (octomap::point3d p: ray){
			if (this->checkCollision(p)){
				return true;
			}
		}
		return false;
	}

	template <std::size_t N>
	void rrtOctomap<N>::randomConfig(KDTree::Point<N>& qRand){
		bool valid = false;
		double x, y, z;
		octomap::point3d p;
		while (not valid){
			p.x() = randomNumber(this->sampleRegion_[0], this->sampleRegion_[1]);
			p.y() = randomNumber(this->sampleRegion_[2], this->sampleRegion_[3]);
			p.z() = randomNumber(this->sampleRegion_[4], this->sampleRegion_[5]);
			valid = not this->checkCollision(p);
		}

		qRand[0] = p.x(); qRand[1] = p.y(); qRand[2] = p.z();
	}

	template <std::size_t N>
	void rrtOctomap<N>::makePlan(std::vector<KDTree::Point<N>>& plan){
		bool findPath = false;
		bool timeout = false;
		ros::Time startTime = ros::Time::now();
		double dT;
		int sampleNum = 0;
		KDTree::Point<N> qBack;

		cout << "[Global Planner INFO]: Start planning!" << endl;
		this->samplePoints_.clear();
		this->addVertex(this->start_);
		while (ros::ok() and not findPath and not timeout){	
			ros::Time currentTime = ros::Time::now();
			dT = (currentTime - startTime).toSec();
			if (dT >= 5){
				timeout = true;
			}

			// 1. sample:
			KDTree::Point<N> qRand;
			double randomValue = randomNumber(0, 10);
			if (randomValue >= 1.0){ // random sample trick
				this->randomConfig(qRand);
			}
			else{
				qRand = this->goal_;
			}

			// 2. find nearest neighbor:
			KDTree::Point<N> qNear;
			this->nearestVertex(qRand, qNear);

			// 3. new config by steering function:
			KDTree::Point<N> qNew;
			this->newConfig(qNear, qRand, qNew);

			// 4. Add new config to vertex and edge:
			if (not this->checkCollisionLine(qNew, qNear)){
				this->addVertex(qNew);
				this->addEdge(qNear, qNew);
				++sampleNum;
				this->samplePoints_.push_back(qNew);

				// 5. check whether goal has been reached
				findPath = this->isReach(qNew);
				if (findPath){
					qBack = qNew;
				}
			}
		}
		cout << "[Global Planner INFO]: Finish planning. with sample number: " << sampleNum << endl;

		// final step: back trace using the last one
		if (findPath){
			this->backTrace(qBack, plan);
			cout << "[Global Planner INFO]: path found!" << endl;
		}

		if (timeout){
			cout << "[Global Planner INFO]: TIMEOUT!" << endl;
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::point2Octomap(const KDTree::Point<N>& q, octomap::point3d& p){
		p.x() = q[0];
		p.y() = q[1];
		p.z() = q[2];
	}

	template <std::size_t N>
	double rrtOctomap<N>::getMapRes(){
		return this->mapRes_;
	}

	template <std::size_t N>
	void rrtOctomap<N>::getSamplePoints(std::vector<KDTree::Point<N>>& sp){
		sp = this->samplePoints_;
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