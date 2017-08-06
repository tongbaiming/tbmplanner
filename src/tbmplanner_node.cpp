/*
 * tbmplanner_node.cpp
 *
 *  Created on: 2017年7月31日
 *      Author: tbm
 */

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <tbmplanner/tbmplanner.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tbmplanner");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	tbm::tbmPlanner planner(nh, nh_private);
	ros::spin();
	return 0;
}


