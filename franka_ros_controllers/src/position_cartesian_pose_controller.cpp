/***************************************************************************

*
* @package: franka_ros_controllers
* @metapackage: franka_ros_interface
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2021, Saif Sidhik.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/
#include <franka_ros_controllers/position_cartesian_pose_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_ros_controllers {

bool PositionCartesianPoseController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_pose_subscriber_ = node_handle.subscribe(
      "/franka_ros_interface/motion_controller/arm/pose_commands", 20, &PositionCartesianPoseController::cartesianPoseCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "PositionCartesianPoseController: Error getting position joint interface from hardware!");
    return false;
  }
  
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }
  
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  return true;
}

void PositionCartesianPoseController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  pos_d_target_ = initial_pose_;
  elapsed_time_ = ros::Duration(0.0);
}

void PositionCartesianPoseController::update(const ros::Time& time,
                                            const ros::Duration& period) {
  //cartesian_pose_handle_->setCommand(pos_d_target_);
  
  
  elapsed_time_ += period;

  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] -= delta_x;
  new_pose[14] -= delta_z;
  cartesian_pose_handle_->setCommand(new_pose);
}

void PositionCartesianPoseController::cartesianPoseCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {
    /*if (msg->mode == franka_core_msgs::JointCommand::POSITION_MODE){
      if (msg->position.size() != 7) {
        ROS_ERROR_STREAM(
            "PositionCartesianPoseController: Published Commands are not of size 7");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;
      }
      else if (checkPositionLimits(msg->position)) {
         ROS_ERROR_STREAM(
            "PositionCartesianPoseController: Commanded positions are beyond allowed position limits.");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;

      }
      else
      {
        std::copy_n(msg->position.begin(), 7, pos_d_target_.begin());
      }
      
    }*/
    // else ROS_ERROR_STREAM("PositionCartesianPoseController: Published Command msg are not of JointCommand::POSITION_MODE! Dropping message");
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::PositionCartesianPoseController,
                       controller_interface::ControllerBase)
