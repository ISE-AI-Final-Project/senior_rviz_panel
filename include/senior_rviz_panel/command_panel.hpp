/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#ifndef SENIOR_RVIZ_PANEL__COMMAND_PANEL_HPP_
#define SENIOR_RVIZ_PANEL__COMMAND_PANEL_HPP_

#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

namespace senior_rviz_panel
{
class CommandPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit CommandPanel(QWidget * parent = 0);
  ~CommandPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_command;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_rviz_text;

  void topicCallback(const std_msgs::msg::String& msg);

  QLabel * label_;
  QLabel * label_target_object;
  QPushButton * button_capture;
  QPushButton * button_req_ism;
  QPushButton * button_req_pem;
  QPushButton * button_make_collision;

  QPushButton * button_generate_all_grasp;
  QPushButton * button_generate_best_grasp;

  QPushButton * button_plan_aim_grip;

  QPushButton * button_trigger_aim;
  QPushButton * button_trigger_grip;

  QPushButton * button_plan_home;
  QPushButton * button_trigger_home;

  QPushButton * button_attach_obj;
  QPushButton * button_detach_obj;

  QPushButton * button_gripper_open;
  QPushButton * button_gripper_close;

  QPushButton * button_fake_point_cloud;
  QPushButton * button_fake_object_pose;

  QComboBox * dropdown_target_obj_;
  void sendTargetObjParam(const QString &target);

private Q_SLOTS:
  void buttonActivated(const std::string& command, const QString& label_text);

  void onTargetObjChanged(const QString &text);
};

}  // namespace senior_rviz_panel

#endif  // SENIOR_RVIZ_PANEL__COMMAND_PANEL_HPP_
