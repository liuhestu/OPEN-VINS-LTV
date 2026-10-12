/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#if ROS_AVAILABLE == 2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
using Path = nav_msgs::msg::Path;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PathPtr = Path::ConstSharedPtr;
rclcpp::Publisher<Path>::SharedPtr pub_path;
#else
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
using Path = nav_msgs::Path;
using PoseStamped = geometry_msgs::PoseStamped;
using PathPtr = Path::ConstPtr;
ros::Publisher pub_path;
#endif

#include <algorithm>

#include "alignment/AlignTrajectory.h"
#include "alignment/AlignUtils.h"
#include "utils/Loader.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

void align_and_publish(const PathPtr &msg);
std::vector<double> times_gt;
std::vector<Eigen::Matrix<double, 7, 1>> poses_gt;
std::string alignment_type;

int main(int argc, char **argv) {

  std::string verbosity, path_to_gt;
#if ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("live_align_trajectory");
  verbosity = node->declare_parameter<std::string>("verbosity", "INFO");
  alignment_type = node->declare_parameter<std::string>("alignment_type", "posyaw");
  path_to_gt = node->declare_parameter<std::string>("path_gt", "");
#else
  ros::init(argc, argv, "live_align_trajectory");
  ros::NodeHandle nh("~");
  nh.param<std::string>("verbosity", verbosity, "INFO");
  nh.param<std::string>("alignment_type", alignment_type, "posyaw");
  nh.param<std::string>("path_gt", path_to_gt, "");
#endif
  ov_core::Printer::setPrintLevel(verbosity);
  if (path_to_gt.empty()) {
    PRINT_ERROR("[LOAD]: Please provide a groundtruth file path!\n");
    return EXIT_FAILURE;
  }
  const std::vector<std::string> methods = {"posyaw", "posyawsingle", "se3", "se3single", "sim3", "none"};
  if (std::find(methods.begin(), methods.end(), alignment_type) == methods.end()) {
    PRINT_ERROR("[ALIGN]: Invalid alignment_type: %s\n", alignment_type.c_str());
    return EXIT_FAILURE;
  }

  boost::filesystem::path infolder(path_to_gt);
  if (infolder.extension() == ".csv") {
    std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
    ov_eval::Loader::load_data_csv(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
  } else {
    std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
    ov_eval::Loader::load_data(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
  }

  // Keep the original topics; launch remaps them for custom estimator namespaces.
#if ROS_AVAILABLE == 2
  pub_path = node->create_publisher<Path>("/ov_msckf/pathgt", 2);
  auto sub = node->create_subscription<Path>("/ov_msckf/pathimu", 1, align_and_publish);
  RCLCPP_INFO(node->get_logger(), "Subscribing: %s; publishing: %s", sub->get_topic_name(), pub_path->get_topic_name());
  rclcpp::spin(node);
  sub.reset();
  pub_path.reset();
  node.reset();
  rclcpp::shutdown();
#else
  ros::Subscriber sub = nh.subscribe("/ov_msckf/pathimu", 1, align_and_publish);
  pub_path = nh.advertise<Path>("/ov_msckf/pathgt", 2);
  ROS_INFO("Subscribing: %s", sub.getTopic().c_str());
  ROS_INFO("Publishing: %s", pub_path.getTopic().c_str());
  ros::spin();
#endif
  return EXIT_SUCCESS;
}

void align_and_publish(const PathPtr &msg) {

  // Convert the message into correct vector format
  // tx ty tz qx qy qz qw
  std::vector<double> times_temp;
  std::vector<Eigen::Matrix<double, 7, 1>> poses_temp;
  for (auto const &pose : msg->poses) {
#if ROS_AVAILABLE == 2
    times_temp.push_back(rclcpp::Time(pose.header.stamp).seconds());
#else
    times_temp.push_back(pose.header.stamp.toSec());
#endif
    Eigen::Matrix<double, 7, 1> pose_tmp;
    pose_tmp << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y,
        pose.pose.orientation.z, pose.pose.orientation.w;
    poses_temp.push_back(pose_tmp);
  }

  if (poses_temp.size() < 3)
    return;

  // Intersect timestamps
  std::vector<double> gt_times_temp = times_gt;
  std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_temp = poses_gt;
  ov_eval::AlignUtils::perform_association(0, 0.02, times_temp, gt_times_temp, poses_temp, gt_poses_temp);

  // Return failure if we didn't have any common timestamps
  if (poses_temp.size() < 3) {
    PRINT_ERROR(RED "[TRAJ]: unable to get enough common timestamps between trajectories.\n" RESET);
    PRINT_ERROR(RED "[TRAJ]: does the estimated trajectory publish the rosbag timestamps??\n" RESET);
    return;
  }

  // Perform alignment of the trajectories
  Eigen::Matrix3d R_ESTtoGT;
  Eigen::Vector3d t_ESTinGT;
  double s_ESTtoGT;
  ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_type);
  Eigen::Vector4d q_ESTtoGT = ov_core::rot_2_quat(R_ESTtoGT);
  PRINT_DEBUG("[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n", q_ESTtoGT(0), q_ESTtoGT(1),
              q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0), t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

  // Finally lets calculate the aligned trajectories
  // TODO: maybe in the future we could live publish trajectory errors...
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  Path arr_groundtruth;
  arr_groundtruth.header = msg->header;
  for (size_t i = 0; i < gt_times_temp.size(); i += std::floor(gt_times_temp.size() / 16384.0) + 1) {

    // Convert into the correct frame
    double timestamp = gt_times_temp.at(i);
    Eigen::Matrix<double, 7, 1> pose_inGT = gt_poses_temp.at(i);
    Eigen::Vector3d pos_IinEST = R_ESTtoGT.transpose() * (pose_inGT.block(0, 0, 3, 1) - t_ESTinGT) / s_ESTtoGT;
    Eigen::Vector4d quat_ESTtoI = ov_core::quat_multiply(pose_inGT.block(3, 0, 4, 1), q_ESTtoGT);
    // Finally push back
    PoseStamped posetemp;
    posetemp.header = msg->header;
#if ROS_AVAILABLE == 2
    posetemp.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(timestamp * 1e9)));
#else
    posetemp.header.stamp = ros::Time(timestamp);
#endif
    posetemp.pose.orientation.x = quat_ESTtoI(0);
    posetemp.pose.orientation.y = quat_ESTtoI(1);
    posetemp.pose.orientation.z = quat_ESTtoI(2);
    posetemp.pose.orientation.w = quat_ESTtoI(3);
    posetemp.pose.position.x = pos_IinEST(0);
    posetemp.pose.position.y = pos_IinEST(1);
    posetemp.pose.position.z = pos_IinEST(2);
    arr_groundtruth.poses.push_back(posetemp);
  }
#if ROS_AVAILABLE == 2
  pub_path->publish(arr_groundtruth);
#else
  pub_path.publish(arr_groundtruth);
#endif
}
