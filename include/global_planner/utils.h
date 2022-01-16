#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <global_planner/KDTree.h>

template <std::size_t N>
visualization_msgs::MarkerArray wrapVisMsg(const std::vector<KDTree::Point<N>>& sp, double r=0, double g=0, double b=0){
	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> sp_vis_array;
	visualization_msgs::Marker sp_marker;
	for (int i=0; i < sp.size(); ++i){
			// sp_marker
			KDTree::Point<N> currentPoint = sp[i];
			sp_marker.header.frame_id = "map";
			sp_marker.id = 8888+i;
			sp_marker.type = visualization_msgs::Marker::SPHERE;
			sp_marker.pose.position.x = currentPoint[0];
			sp_marker.pose.position.y = currentPoint[1];
			sp_marker.pose.position.z = currentPoint[2];
			// sp_marker.lifetime = ros::Duration(3);
			sp_marker.scale.x = 0.5;
			sp_marker.scale.y = 0.5;
			sp_marker.scale.z = 0.5;
			sp_marker.color.a = 0.8;
			sp_marker.color.r = r;
			sp_marker.color.g = g;
			sp_marker.color.b = b;

			
			sp_vis_array.push_back(sp_marker);
		}
		msg.markers = sp_vis_array;
	return msg;
}

#endif