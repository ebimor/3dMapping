/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <point_cloud_localization/PointCloudLocalization.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::transformPointCloud;

PointCloudLocalization::PointCloudLocalization() {}
PointCloudLocalization::~PointCloudLocalization() {}

bool PointCloudLocalization::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudLocalization");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudLocalization::LoadParameters(const ros::NodeHandle& n) {
  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
  if (!pu::Get("frame_id/base", base_frame_id_)) return false;

  ROS_INFO("am I stuck?!");

  try
  {
    listener.waitForTransform(fixed_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform(fixed_frame_id_, base_frame_id_, ros::Time(0), newTransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_INFO("ERRORORRRRRRR");
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Transform geTransform;
  geTransform.translation.x = newTransform.getOrigin().x();
  geTransform.translation.y = newTransform.getOrigin().y();
  geTransform.translation.z = newTransform.getOrigin().z();

  geTransform.rotation.x = newTransform.getRotation().x();
  geTransform.rotation.y = newTransform.getRotation().y();
  geTransform.rotation.z = newTransform.getRotation().z();
  geTransform.rotation.w = newTransform.getRotation().w();



  initial_loc_ = gr::FromROS(geTransform);
  Eigen::Matrix<double, 3, 1> T = initial_loc_.translation.Eigen();
  Eigen::Matrix<double, 3, 3> R = initial_loc_.rotation.Eigen();

  ROS_INFO_STREAM("Initial translation is: "<<T);
  ROS_INFO_STREAM("Initial rotation is: "<<R);

  integrated_estimate_ = gu::PoseDelta(initial_loc_, initial_loc_);
  //prev_integrated_estimate_ = integrated_estimate_;


/*
  // Load initial position.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  if (!pu::Get("init/position/x", init_x)) return false;
  if (!pu::Get("init/position/y", init_y)) return false;
  if (!pu::Get("init/position/z", init_z)) return false;
  if (!pu::Get("init/orientation/roll", init_roll)) return false;
  if (!pu::Get("init/orientation/pitch", init_pitch)) return false;
  if (!pu::Get("init/orientation/yaw", init_yaw)) return false;

  integrated_estimate_.translation = gu::Vec3(init_x, init_y, init_z);
  integrated_estimate_.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);
*/
  // Load algorithm parameters.
  if (!pu::Get("localization/tf_epsilon", params_.tf_epsilon)) return false;
  if (!pu::Get("localization/corr_dist", params_.corr_dist)) return false;
  if (!pu::Get("localization/iterations", params_.iterations)) return false;


  if (!pu::Get("localization/transform_thresholding", transform_thresholding_))
    return false;
  if (!pu::Get("localization/max_translation", max_translation_)) return false;
  if (!pu::Get("localization/max_rotation", max_rotation_)) return false;
  if (!pu::Get("localization/icp_threshold", icp_threshold_)) return false;


  return true;
}

bool PointCloudLocalization::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  query_pub_ = nl.advertise<PointCloud>("localization_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloud>("localization_reference_points", 10, false);
  aligned_pub_ =
      nl.advertise<PointCloud>("localization_aligned_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "localization_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "localization_integrated_estimate", 10, false);

  return true;
}

const gu::Transform3& PointCloudLocalization::GetIncrementalEstimate() const {

  //double curr_x = incremental_estimate_.translation(0);
  //double curr_y = incremental_estimate_.translation(1);

  //std::cout<<"current location is: "<<curr_x<<" , "<<curr_y<<std::endl;

  return incremental_estimate_;
}

const gu::Transform3& PointCloudLocalization::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

void PointCloudLocalization::SetIntegratedEstimate(
    const gu::Transform3& integrated_estimate) {
  integrated_estimate_ = integrated_estimate;

/*
  // Publish transform between fixed frame and localization frame.
  geometry_msgs::TransformStamped tf;
  tf.transform = gr::ToRosTransform(integrated_estimate_);
  tf.header.stamp = stamp_;
  tf.header.frame_id = fixed_frame_id_;
  tf.child_frame_id = base_frame_id_;
  tfbr_.sendTransform(tf);
  */
}

bool PointCloudLocalization::MotionUpdate(
    const PointCloud::Ptr& query) {
  // Store the incremental transform from odometry.
  //incremental_estimate_ = incremental_odom;  //gu::Transform3::Identity(); // incremental_odom;
  stamp_.fromNSec(query->header.stamp*1e3);

  bool transformation_exists = true;
  try
  {
    listener.waitForTransform(fixed_frame_id_, base_frame_id_, stamp_, ros::Duration(0.1));
    listener.lookupTransform(fixed_frame_id_, base_frame_id_, stamp_, newTransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s. Motion Update Failed.", ex.what());
    transformation_exists = false;
  }

  gu::Transform3 absolute_estimate_;

  if(transformation_exists){
    geometry_msgs::Transform geTransform;
    geTransform.translation.x = newTransform.getOrigin().x();
    geTransform.translation.y = newTransform.getOrigin().y();
    geTransform.translation.z = newTransform.getOrigin().z();

    geTransform.rotation.x = newTransform.getRotation().x();
    geTransform.rotation.y = newTransform.getRotation().y();
    geTransform.rotation.z = newTransform.getRotation().z();
    geTransform.rotation.w = newTransform.getRotation().w();

    absolute_estimate_= gr::FromROS(geTransform);
    rough_integrated_estimate_ = gu::PoseDelta(initial_loc_, absolute_estimate_);
    incremental_estimate_ = gu::PoseDelta(integrated_estimate_, rough_integrated_estimate_);
  }else{
    rough_integrated_estimate_ = integrated_estimate_;
    incremental_estimate_ = gu::Transform3::Identity();
  }

  refined_estimate = false;

  return true;
}

bool PointCloudLocalization::TransformPointsToFixedFrame(
    const PointCloud& points, PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Compose the current incremental estimate (from odometry) with the
  // integrated estimate, and transform the incoming point cloud.
  const gu::Transform3 estimate = refined_estimate? integrated_estimate_: rough_integrated_estimate_;
  //gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::TransformPointsToSensorFrame(
    const PointCloud& points, PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Compose the current incremental estimate (from odometry) with the
  // integrated estimate, then invert to go from world to sensor frame.
  const gu::Transform3 estimate = refined_estimate? gu::PoseInverse(integrated_estimate_): gu::PoseInverse(rough_integrated_estimate_);
    //gu::PoseUpdate(integrated_estimate_, incremental_estimate_));
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::MeasurementUpdate(const PointCloud::Ptr& query,
                                               const PointCloud::Ptr& reference,
                                               PointCloud* aligned_query,
                                               bool debug) {
  if (aligned_query == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

/*
  //use the above pose update to roughly align the two pcls
  const Eigen::Matrix<double, 3, 3> Rot = incremental_estimate_.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> Trans = incremental_estimate_.translation.Eigen();

  Eigen::Matrix4d tff;
  tff.block(0, 0, 3, 3) = Rot;
  tff.block(0, 3, 3, 1) = Trans;
  //cast the tff as float for final transformation
  Eigen::Matrix4f tff_float = tff.cast<float>();

  //transform querry pcl to align it with the reference
  PointCloud::Ptr aligned_pcl(new PointCloud);
  pcl::transformPointCloud(*query, *aligned_pcl, tff);
*/
  //now perform icp between algined querry and reference pcl
  // ICP-based alignment. Generalized ICP does (roughly) plane-to-plane
  // matching, and is much more robust than standard ICP.
  GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(params_.tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.corr_dist);
  icp.setMaximumIterations(params_.iterations);
  icp.setRANSACIterations(0);
  icp.setMaximumOptimizerIterations(100); // default 20

  icp.setInputSource(query);
  icp.setInputTarget(reference);


  /*
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(params_.tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.corr_dist);
  //icp.setMaximumIterations(params_.iterations);
  icp.setRANSACOutlierRejectionThreshold(params_.corr_dist);
  icp.setMaximumIterations(50); // default 20

  icp.setInputCloud(query);
  icp.setInputTarget(reference);
  */

  //PointCloud unused;
  icp.align(*aligned_query);

 if(debug)
    ROS_INFO_STREAM("has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore());

 if(icp.getFitnessScore() < icp_threshold_ && icp.hasConverged()){
    // Retrieve transformation and estimate and update.
    const Eigen::Matrix4f T = icp.getFinalTransformation(); //*tff_float;
    //pcl::transformPointCloud(*query, *aligned_query, T);


    gu::Transform3 pose_update;
    pose_update.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    pose_update.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                    T(1, 0), T(1, 1), T(1, 2),
                                    T(2, 0), T(2, 1), T(2, 2));


    // Only update if the transform is small enough.
    if (!transform_thresholding_ ||
        (pose_update.translation.Norm() <= max_translation_ &&
         pose_update.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
      incremental_estimate_ = gu::PoseUpdate(incremental_estimate_, pose_update);
    } else {
      ROS_WARN(
          " %s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
          name_.c_str(), pose_update.translation.Norm(),
          pose_update.rotation.ToEulerZYX().Norm());
    }

    //prev_integrated_estimate_ = integrated_estimate_;
    integrated_estimate_ = gu::PoseUpdate(rough_integrated_estimate_, incremental_estimate_);
  }else{
    if(debug)
      ROS_WARN("Motion update ICP did not converge");
    integrated_estimate_ = rough_integrated_estimate_;

    return false;
  }

  refined_estimate = true;
  
  return true;
}

void PointCloudLocalization::PublishPoints(const PointCloud& points,
                                           const ros::Publisher& pub) const {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() > 0) {
    PointCloud out;
    out = points;
    out.header.frame_id = base_frame_id_;
    pub.publish(out);
  }
}

void PointCloudLocalization::PublishPose(const gu::Transform3& pose,
                                         const ros::Publisher& pub) const {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() == 0)
   return;

  // Convert from gu::Transform3 to ROS's PoseStamped type and publish.
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.pose = gr::ToRosPose(pose);
  ros_pose.header.frame_id = fixed_frame_id_;
  ros_pose.header.stamp = stamp_;
  pub.publish(ros_pose);
}
