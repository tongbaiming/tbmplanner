/*
 * tbmplanner.h
 *
 *  Created on: 2017年7月31日
 *      Author: tbm
 */

#ifndef INCLUDE_TBMPLANNER_TBMPLANNER_H_
#define INCLUDE_TBMPLANNER_TBMPLANNER_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>
#include <tbmplanner/tbmplanner_srv.h>
#include <tbmplanner/matlabsolver_srv.h>
#include <tbmplanner/gain_srv.h>
#include <tbmplanner/gain_full_srv.h>
#include "std_msgs/String.h"
#include <cmath>
//#include "engine.h"

#define SQRT2 0.70711

namespace tbm
{
	struct Params
	{
	  std::vector<double> camPitch_;
	  std::vector<double> camHorizontal_;
	  std::vector<double> camVertical_;
	  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;

	  //double igProbabilistic_;
	  double igFree_;
	  double igOccupied_;
	  double igUnmapped_;
	  //double igArea_;
	  double gainRange_;
	  //double degressiveCoeff_;
	  //double zero_gain_;

	  //double v_max_;
	  //double dyaw_max_;
	  //double dOvershoot_;
	  //double extensionRange_;
	  //bool exact_root_;
	  //int initIterations_;
	  //int cuttoffIterations_;
	  //double dt_;

	  double minX_;
	  double minY_;
	  double minZ_;
	  double maxX_;
	  double maxY_;
	  double maxZ_;
	  //bool softBounds_;
	  //Eigen::Vector3d boundingBox_;

	  //double meshResolution_;

	  //ros::Publisher inspectionPath_;
	  //std::string navigationFrame_;

	  //bool log_;
	  //double log_throttle_;
	  double pcl_throttle_;
	  //double inspection_throttle_;
	};
	class tbmPlanner
	{
	public:
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		ros::ServiceServer plannerService_;
		ros::ServiceServer matlabGainService_;
		ros::ServiceServer matlabGainFullService_;
		ros::ServiceClient matlabSolver_;
		ros::Subscriber pointcloud_sub_;
		Params params_;
		volumetric_mapping::OctomapManager * manager_;
		bool ready_;
		ros::Subscriber posClient_;
		ros::Subscriber odomClient_;
		ros::Publisher matlabOdome_pub_;

	public:
		tbmPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
		~tbmPlanner();
		bool setParams();
		 void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
		 void odomCallback(const nav_msgs::Odometry& pose);
		 bool plannerCallback(tbmplanner::tbmplanner_srv::Request& req, tbmplanner::tbmplanner_srv::Response& res);
		 void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
		 bool gainCallback(tbmplanner::gain_srv::Request& req, tbmplanner::gain_srv::Response& res);
		 bool gainFullCallback(tbmplanner::gain_full_srv::Request& req, tbmplanner::gain_full_srv::Response& res);
		 void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
		 double  coculateGain(tbmplanner::point_msg pointMsg);
	};
}

#endif /* INCLUDE_TBMPLANNER_TBMPLANNER_H_ */
