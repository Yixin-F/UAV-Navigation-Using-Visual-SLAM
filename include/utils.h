#ifndef GLOBALPLANNERUTILS_H
#define GLOBALPLANNERUTILS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <KDTree.h>
#include <geometry_msgs/Point.h>

template <std::size_t N>
visualization_msgs::MarkerArray wrapRRTVisMsg(const std::vector<KDTree::Point<N>>& sp, std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher>& edge_dict, double r=0, double g=0, double b=0){
	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> sp_vis_array;
	visualization_msgs::Marker sp_marker;
	visualization_msgs::Marker line_marker;
	std::vector<geometry_msgs::Point> line_vis;
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
			sp_marker.scale.x = 0.2;
			sp_marker.scale.y = 0.2;
			sp_marker.scale.z = 0.2;
			sp_marker.color.a = 0.8;
			sp_marker.color.r = r;
			sp_marker.color.g = g;
			sp_marker.color.b = b;
			sp_vis_array.push_back(sp_marker);
		}

	// line marker
	for (const auto& p: edge_dict){
		// cout << "current: " << p.first << " parent: " << p.second << endl;
		KDTree::Point<N> currentPoint = p.first;
		KDTree::Point<N> prevPoint = p.second;
		if (prevPoint[0] != -11311){
			geometry_msgs::Point p1, p2;
			p1.x = currentPoint[0];
			p1.y = currentPoint[1];
			p1.z = currentPoint[2];
			p2.x = prevPoint[0];
			p2.y = prevPoint[1];
			p2.z = prevPoint[2]; 
			
			line_vis.push_back(p1);
			line_vis.push_back(p2);
		}
	}
	line_marker.header.frame_id = "map";
	line_marker.points = line_vis;
	line_marker.id = 1000000;
	line_marker.type = visualization_msgs::Marker::LINE_LIST;
	line_marker.scale.x = 0.05;
	line_marker.scale.y = 0.05;
	line_marker.scale.z = 0.05;
	line_marker.color.a = 1.0;
	line_marker.color.r = r*0.5;
	line_marker.color.g = g;
	line_marker.color.b = 1;
	sp_vis_array.push_back(line_marker);
	msg.markers = sp_vis_array;
	return msg;
}

template <std::size_t N>
visualization_msgs::MarkerArray wrapPathMsg(const std::vector<KDTree::Point<N>> &plan){
	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> path_vis_array;
	visualization_msgs::Marker waypoint;
	visualization_msgs::Marker line_marker;
	std::vector<geometry_msgs::Point> line_vis;
	for (int i=0; i < plan.size(); ++i){
			KDTree::Point<N> currentPoint = plan[i];
			if (i != plan.size() - 1){
				KDTree::Point<N> nextPoint = plan[i+1];
				geometry_msgs::Point p1, p2;
				p1.x = currentPoint[0];
				p1.y = currentPoint[1];
				p1.z = currentPoint[2];
				p2.x = nextPoint[0];
				p2.y = nextPoint[1];
				p2.z = nextPoint[2]; 
				line_vis.push_back(p1);
				line_vis.push_back(p2);
			}
			// waypoint
			waypoint.header.frame_id = "map";
			waypoint.id = i;
			waypoint.ns = "waypoint";
			waypoint.type = visualization_msgs::Marker::SPHERE;
			waypoint.pose.position.x = currentPoint[0];
			waypoint.pose.position.y = currentPoint[1];
			waypoint.pose.position.z = currentPoint[2];
			// waypoint.lifetime = ros::Duration(0.5);
			waypoint.scale.x = 0.2;
			waypoint.scale.y = 0.2;
			waypoint.scale.z = 0.2;
			waypoint.color.a = 0.8;
			waypoint.color.r = 0.3;
			waypoint.color.g = 1;
			waypoint.color.b = 0.5;
	
			path_vis_array.push_back(waypoint);
		}
	line_marker.header.frame_id = "map";
	line_marker.points = line_vis;
	line_marker.ns = "path_line";
	line_marker.id = 0;
	line_marker.type = visualization_msgs::Marker::LINE_LIST;
	line_marker.scale.x = 0.05;
	line_marker.scale.y = 0.05;
	line_marker.scale.z = 0.05;
	line_marker.color.a = 1.0;
	line_marker.color.r = 0.5;
	line_marker.color.g = 0.1;
	line_marker.color.b = 1;
	path_vis_array.push_back(line_marker);
	msg.markers = path_vis_array;
	return msg;
}



#endif