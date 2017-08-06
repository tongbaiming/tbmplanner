/*
 * tbmplanner.cpp
 *
 *  Created on: 2017年7月31日
 *      Author: tbm
 */

#include "tbmplanner/tbmplanner.h"

tbm::tbmPlanner::tbmPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
	nh_(nh),
	nh_private_(nh_private),
	ready_(false),
	manager_(NULL)
{
	manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);
	//std::cout << "manager_->getMapSize().norm() = " << manager_->getMapSize().norm() << std::endl;
	plannerService_ = nh_.advertiseService("tbmplanner", &tbm::tbmPlanner::plannerCallback, this);
	matlabGainService_ = nh_private_.advertiseService("matlabGainService", &tbm::tbmPlanner::gainCallback, this);
	posClient_ = nh_.subscribe("pose", 10, &tbm::tbmPlanner::posCallback, this);
	odomClient_ = nh_.subscribe("odometry", 10, &tbm::tbmPlanner::odomCallback, this);
	pointcloud_sub_ = nh_.subscribe("pointcloud_throttled", 1, &tbm::tbmPlanner::insertPointcloudWithTf, this);
	matlabSolver_ = nh_.serviceClient<tbmplanner::matlabsolver_srv>("/matlab_solver");
	matlabOdome_pub_ = nh_.advertise<nav_msgs::Odometry>("/matlabOdome",100);
	if(!setParams())
	{
		ROS_ERROR("Could not start the planner. Parameters missing!");
	}
}

tbm::tbmPlanner::~tbmPlanner()
{
	if(manager_)
	{
		delete manager_;
	}
}

bool tbm::tbmPlanner::setParams()
{
	std::string ns=ros::this_node::getName();
	params_.pcl_throttle_ = 0.333;
	params_.camPitch_ = {15.0};
	params_.camHorizontal_ = {90.0};
	params_.camVertical_ = {60.0};
	params_.gainRange_ = 1.0;
	params_.igFree_ = 0.0;
	params_.igOccupied_ = 0.0;
	params_.igUnmapped_ = 1.0;
	params_.minX_ = -8.0;
	params_.minY_ = -8.0;
	params_.minZ_ = -0.0;
	params_.maxX_ = 8.0;
	params_.maxY_ = 8.0;
	params_.maxZ_ = 3.0;
	for (int i = 0; i < params_.camPitch_.size(); i++) {
	    double pitch = M_PI * params_.camPitch_[i] / 180.0;
	    double camTop = (pitch - M_PI * params_.camVertical_[i] / 360.0) + M_PI / 2.0;
	    double camBottom = (pitch + M_PI * params_.camVertical_[i] / 360.0) - M_PI / 2.0;
	    double side = M_PI * (params_.camHorizontal_[i]) / 360.0 - M_PI / 2.0;
	    Eigen::Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
	    Eigen::Vector3d top(cos(camTop), 0.0, -sin(camTop));
	    Eigen::Vector3d right(cos(side), sin(side), 0.0);
	    Eigen::Vector3d left(cos(side), -sin(side), 0.0);
	    Eigen::AngleAxisd m = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
	    Eigen::Vector3d rightR = m * right;
	    Eigen::Vector3d leftR = m * left;
	    rightR.normalize();
	    leftR.normalize();
	    std::vector<Eigen::Vector3d> camBoundNormals;
	    camBoundNormals.push_back(bottom);
	    // ROS_INFO("bottom: (%2.2f, %2.2f, %2.2f)", bottom[0], bottom[1], bottom[2]);
	    camBoundNormals.push_back(top);
	    // ROS_INFO("top: (%2.2f, %2.2f, %2.2f)", top[0], top[1], top[2]);
	    camBoundNormals.push_back(rightR);
	    // ROS_INFO("rightR: (%2.2f, %2.2f, %2.2f)", rightR[0], rightR[1], rightR[2]);
	    camBoundNormals.push_back(leftR);
	    // ROS_INFO("leftR: (%2.2f, %2.2f, %2.2f)", leftR[0], leftR[1], leftR[2]);
	    params_.camBoundNormals_.push_back(camBoundNormals);
	  }
	return true;
}

void tbm::tbmPlanner::posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
	//manager_->
	ready_=true;
}

void tbm::tbmPlanner::odomCallback(const nav_msgs::Odometry& odome)
{
	static tf::TransformListener listener;
	tf::StampedTransform transform;
	try{
		listener.lookupTransform("/navigation", odome.header.frame_id, odome.header.stamp, transform);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		return;
	}
	tf::Pose poseTF;
	tf::poseMsgToTF(odome.pose.pose, poseTF);
	tf::Vector3 position = poseTF.getOrigin();
	position = transform * position;
	tf::Quaternion quat = poseTF.getRotation();
	quat = transform * quat;
	tf::Pose poseTF_trans = transform * poseTF;
	nav_msgs::Odometry odome_trans;
	tf::poseTFToMsg(poseTF_trans, odome_trans.pose.pose);
	matlabOdome_pub_.publish(odome_trans);
	ready_=true;
}

void tbm::tbmPlanner::insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
	static double last = ros::Time::now().toSec();
	if (last + params_.pcl_throttle_ < ros::Time::now().toSec())
	{
	    manager_->insertPointcloudWithTf(pointcloud);
	    last += params_.pcl_throttle_;
	  }
}

bool tbm::tbmPlanner::plannerCallback(tbmplanner::tbmplanner_srv::Request& req, tbmplanner::tbmplanner_srv::Response& res)
{
	static long iteration = 0;
	tbmplanner::matlabsolver_srv matlabsolverSrv;
	matlabsolverSrv.request.header.stamp = ros::Time::now();
	matlabsolverSrv.request.header.seq = iteration;
	if(matlabSolver_.call(matlabsolverSrv))
	{
		ROS_INFO("call matlabsolverSrv %ld", iteration);
		iteration++;
		return true;
	}
	else
	{
		iteration++;
		return false;
	}

}

void tbm::tbmPlanner::setStateFromOdometryMsg(const nav_msgs::Odometry& pose)
{

}

bool tbm::tbmPlanner::gainCallback(tbmplanner::gain_srv::Request& req, tbmplanner::gain_srv::Response& res)
{
	std::cout << "I am in tbmPlanner::gainCallback, flag 1" << std::endl;

	auto x = req.x;
	std::cout << x.front() << std::endl;
	std::cout << req.x[0] << std::endl;

	double gain = 0.0;
	const double disc = manager_->getResolution();
	Eigen::Vector3d origin(req.x[0], req.x[1], req.x[2]);
	Eigen::Vector3d vec;
	double rangeSq =pow(params_.gainRange_, 2.0);
	for (vec[0] = std::max(req.x[0] - params_.gainRange_, params_.minX_);
	      vec[0] < std::min(req.x[0] + params_.gainRange_, params_.maxX_); vec[0] += disc) {
	    for (vec[1] = std::max(req.x[1] - params_.gainRange_, params_.minY_);
	        vec[1] < std::min(req.x[1] + params_.gainRange_, params_.maxY_); vec[1] += disc) {
	      for (vec[2] = std::max(req.x[2] - params_.gainRange_, params_.minZ_);
	          vec[2] < std::min(req.x[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc) {
	        Eigen::Vector3d dir = vec - origin;
	        // Skip if distance is too large
	        if (dir.transpose().dot(dir) > rangeSq) {
	          continue;
	        }
	        bool insideAFieldOfView = false;
	        // Check that voxel center is inside one of the fields of view.
	        //ROS_ERROR
	        for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = params_
	            .camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++) {
	          bool inThisFieldOfView = true;
	          for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
	              itSingleCBN != itCBN->end(); itSingleCBN++) {
	            Eigen::Vector3d normal = Eigen::AngleAxisd(req.x[3], Eigen::Vector3d::UnitZ())
	                * (*itSingleCBN);
	            double val = dir.dot(normal.normalized());
	            if (val < SQRT2 * disc) {
	              inThisFieldOfView = false;
	              break;
	            }
	          }
	          if (inThisFieldOfView) {
	            insideAFieldOfView = true;
	            break;
	          }
	        }
	        if (!insideAFieldOfView) {
	          continue;
	        }
	        // Check cell status and add to the gain considering the corresponding factor.
	        double probability;
	        volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
	            vec, &probability);
	        //tbm:如果这个点本身kUnknown并且从origin到这个点vec的射线上没有kOcuupied,那么这个点就是kUnknown的点
	        if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
	          // Rayshooting to evaluate inspectability of cell
	          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
	              != this->manager_->getVisibility(origin, vec, false)) {
	            gain += params_.igUnmapped_;
	            // TODO: Add probabilistic gain
	            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
	          }
	        } else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
	          // Rayshooting to evaluate inspectability of cell
	          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
	              != this->manager_->getVisibility(origin, vec, false)) {
	            gain += params_.igOccupied_;
	            // TODO: Add probabilistic gain
	            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
	          }
	        } else {
	          // Rayshooting to evaluate inspectability of cell
	          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
	              != this->manager_->getVisibility(origin, vec, false)) {
	            gain += params_.igFree_;
	            // TODO: Add probabilistic gain
	            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
	          }
	        }
	      }
	    }
	  }
	// Scale with volume
	  gain *= pow(disc, 3.0);
	// Check the gain added by inspectable surface
	  res.gain = gain;

	//res.gain = 666;
	return true;
}
