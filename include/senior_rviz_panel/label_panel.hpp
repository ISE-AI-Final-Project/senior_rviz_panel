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

#ifndef SENIOR_RVIZ_PANEL__LABEL_PANEL_HPP_
#define SENIOR_RVIZ_PANEL__LABEL_PANEL_HPP_

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

namespace senior_rviz_panel
{
class LabelPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit LabelPanel(QWidget * parent = 0);
  ~LabelPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_command;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_rviz_text;

  void topicCallback(const std_msgs::msg::String& msg);

  QLabel * label_;
  QLabel * label_target_object;

  QPushButton * button_x_plus;
  QPushButton * button_x_minus;

  QPushButton * button_y_plus;
  QPushButton * button_y_minus;

  QPushButton * button_z_plus;
  QPushButton * button_z_minus;

  QPushButton * button_rx_plus;
  QPushButton * button_rx_minus;

  QPushButton * button_ry_plus;
  QPushButton * button_ry_minus;

  QPushButton * button_rz_plus;
  QPushButton * button_rz_minus;

  QLineEdit * step_input_;

  std::string current_obj;
  QComboBox * dropdown_target_obj_;

  std::string inc_x;
  QComboBox * dropdown_x;
  std::string inc_y;
  QComboBox * dropdown_y;
  std::string inc_z;
  QComboBox * dropdown_z;
  std::string inc_rx;
  QComboBox * dropdown_rx;
  std::string inc_ry;
  QComboBox * dropdown_ry;
  std::string inc_rz;
  QComboBox * dropdown_rz;

private Q_SLOTS:
  void buttonActivated(const std::string& command, const std::string& label_text);

  void onComboBoxChanged(std::string& var, const QString& text);
};

}  // namespace senior_rviz_panel

#endif  // SENIOR_RVIZ_PANEL__LABEL_PANEL_HPP_
