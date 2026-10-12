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

#include <functional>

#include "utils/Recorder.h"
#include "utils/print.h"

int main(int argc, char **argv) {

  std::string verbosity, topic, topic_type, fileoutput;
#if ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pose_to_file");
  verbosity = node->declare_parameter<std::string>("verbosity", "INFO");
  topic = node->declare_parameter<std::string>("topic", "/ov_msckf/poseimu");
  topic_type = node->declare_parameter<std::string>("topic_type", "PoseWithCovarianceStamped");
  fileoutput = node->declare_parameter<std::string>("output", "/tmp/traj_estimate.txt");
#else
  ros::init(argc, argv, "pose_to_file");
  ros::NodeHandle nh("~");
  nh.param<std::string>("verbosity", verbosity, "INFO");
  nh.getParam("topic", topic);
  nh.getParam("topic_type", topic_type);
  nh.getParam("output", fileoutput);
#endif
  ov_core::Printer::setPrintLevel(verbosity);

  // Debug
  PRINT_DEBUG("Done reading config values");
  PRINT_DEBUG(" - topic = %s", topic.c_str());
  PRINT_DEBUG(" - topic_type = %s", topic_type.c_str());
  PRINT_DEBUG(" - file = %s", fileoutput.c_str());

  // Create the recorder object
  ov_eval::Recorder recorder(fileoutput);

  // Subscribe to topic
#if ROS_AVAILABLE == 2
  rclcpp::SubscriptionBase::SharedPtr sub;
  using std::placeholders::_1;
  if (topic_type == "PoseWithCovarianceStamped") {
    sub = node->create_subscription<ov_eval::PoseWithCovarianceStamped>(
        topic, 9999, std::bind(&ov_eval::Recorder::callback_posecovariance, &recorder, _1));
  } else if (topic_type == "PoseStamped") {
    sub = node->create_subscription<ov_eval::PoseStamped>(topic, 9999, std::bind(&ov_eval::Recorder::callback_pose, &recorder, _1));
  } else if (topic_type == "TransformStamped") {
    sub =
        node->create_subscription<ov_eval::TransformStamped>(topic, 9999, std::bind(&ov_eval::Recorder::callback_transform, &recorder, _1));
  } else if (topic_type == "Odometry") {
    sub = node->create_subscription<ov_eval::Odometry>(topic, 9999, std::bind(&ov_eval::Recorder::callback_odometry, &recorder, _1));
#else
  ros::Subscriber sub;
  if (topic_type == std::string("PoseWithCovarianceStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_posecovariance, &recorder);
  } else if (topic_type == std::string("PoseStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_pose, &recorder);
  } else if (topic_type == std::string("TransformStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_transform, &recorder);
  } else if (topic_type == std::string("Odometry")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_odometry, &recorder);

#endif
  } else {
    PRINT_ERROR("The specified topic type is not supported");
    PRINT_ERROR("topic_type = %s", topic_type.c_str());
    PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
    std::exit(EXIT_FAILURE);
  }

  // Done!
#if ROS_AVAILABLE == 2
  rclcpp::spin(node);
  sub.reset();
  node.reset();
  rclcpp::shutdown();
#else
  ros::spin();
#endif
  return EXIT_SUCCESS;
}
