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

#include <QVBoxLayout>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <senior_rviz_panel/command_panel.hpp>

namespace senior_rviz_panel
{
CommandPanel::CommandPanel(QWidget * parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("Please start Main Node first.");
  layout->addWidget(label_);

  button_capture = new QPushButton("Capture");
  layout->addWidget(button_capture);
  QObject::connect(button_capture, &QPushButton::released, this,
                   [this]() { buttonActivated("capture", "Capturing RGB and Depth..."); });

  // Same line
  auto * hLayout_target_object = new QHBoxLayout();
  dropdown_target_obj_ = new QComboBox(this);
  dropdown_target_obj_->addItems({"sunscreen", "acnewash", "cereal2"});
  label_target_object = new QLabel("Target Object:");
  hLayout_target_object->addWidget(label_target_object);
  hLayout_target_object->addWidget(dropdown_target_obj_);
  layout->addLayout(hLayout_target_object);

  QObject::connect(dropdown_target_obj_, &QComboBox::currentTextChanged, this,
                   &CommandPanel::onTargetObjChanged);

  // Same line
  auto * hLayout = new QHBoxLayout();
  button_req_ism = new QPushButton("Request ISM");
  button_req_pem = new QPushButton("Request PEM");
  hLayout->addWidget(button_req_ism);
  hLayout->addWidget(button_req_pem);
  layout->addLayout(hLayout);

  QObject::connect(button_req_ism, &QPushButton::released, this,
                   [this]() { buttonActivated("req_ism", "Requesting ISM..."); });
  QObject::connect(button_req_pem, &QPushButton::released, this,
                   [this]() { buttonActivated("req_pem", "Requesting PEM..."); });

  // Same line
  auto * hLayout2 = new QHBoxLayout();
  button_generate_all_grasp = new QPushButton("Gen All Grasp");
  button_generate_best_grasp = new QPushButton("Gen Best Grasp");
  hLayout2->addWidget(button_generate_all_grasp);
  hLayout2->addWidget(button_generate_best_grasp);
  layout->addLayout(hLayout2);

  QObject::connect(button_generate_all_grasp, &QPushButton::released, this, [this]() {
    buttonActivated("generate_all_grasp", "Generating All Grasp Pose...");
  });
  QObject::connect(button_generate_best_grasp, &QPushButton::released, this, [this]() {
    buttonActivated("generate_best_grasp", "Generating Best Grasp Pose...");
  });

  button_make_collision = new QPushButton("Make Collision");
  layout->addWidget(button_make_collision);
  QObject::connect(button_make_collision, &QPushButton::released, this,
                   [this]() { buttonActivated("make_collision", "Making Collision..."); });

  // Same Line
  auto * hLayout3 = new QHBoxLayout();
  button_plan_aim = new QPushButton("Plan Aim");
  button_trigger_aim = new QPushButton("Trigger Aim");
  hLayout3->addWidget(button_plan_aim);
  hLayout3->addWidget(button_trigger_aim);
  layout->addLayout(hLayout3);

  QObject::connect(button_plan_aim, &QPushButton::released, this,
                   [this]() { buttonActivated("plan_aim", "Planning Aim..."); });
  QObject::connect(button_trigger_aim, &QPushButton::released, this,
                   [this]() { buttonActivated("trigger_aim", "Triggering Aim..."); });

  // Same Line
  auto * hLayout5 = new QHBoxLayout();
  button_plan_grip = new QPushButton("Plan Grip");
  button_trigger_grip = new QPushButton("Trigger Grip");
  hLayout5->addWidget(button_plan_grip);
  hLayout5->addWidget(button_trigger_grip);
  layout->addLayout(hLayout5);

  QObject::connect(button_plan_grip, &QPushButton::released, this,
                   [this]() { buttonActivated("plan_grip", "Planning Grip..."); });
  QObject::connect(button_trigger_grip, &QPushButton::released, this,
                   [this]() { buttonActivated("trigger_grip", "Triggering Grip..."); });

  // Same Line
  auto * hLayout6 = new QHBoxLayout();
  button_plan_home = new QPushButton("Plan Home");
  button_trigger_home = new QPushButton("Trigger Home");
  hLayout6->addWidget(button_plan_home);
  hLayout6->addWidget(button_trigger_home);
  layout->addLayout(hLayout6);

  QObject::connect(button_plan_home, &QPushButton::released, this,
                   [this]() { buttonActivated("plan_home", "Planning Home..."); });
  QObject::connect(button_trigger_home, &QPushButton::released, this,
                   [this]() { buttonActivated("trigger_home", "Triggering Home..."); });

  // Same line
  auto * hLayout4 = new QHBoxLayout();
  button_gripper_open = new QPushButton("Gripper Open");
  button_gripper_close = new QPushButton("Gripper Close");
  hLayout4->addWidget(button_gripper_open);
  hLayout4->addWidget(button_gripper_close);
  layout->addLayout(hLayout4);

  QObject::connect(button_gripper_open, &QPushButton::released, this,
                   [this]() { buttonActivated("gripper_open", "Opening Gripper..."); });
  QObject::connect(button_gripper_close, &QPushButton::released, this,
                   [this]() { buttonActivated("gripper_close", "Closing Gripper..."); });

  // Same line
  auto * hLayout7 = new QHBoxLayout();
  button_fake_point_cloud = new QPushButton("Fake Point Cloud");
  button_fake_object_pose = new QPushButton("Fake Object Pose");
  hLayout7->addWidget(button_fake_point_cloud);
  hLayout7->addWidget(button_fake_object_pose);
  layout->addLayout(hLayout7);

  QObject::connect(button_fake_point_cloud, &QPushButton::released, this,
                   [this]() { buttonActivated("fake_point_cloud", "Fake Point Cloud"); });
  QObject::connect(button_fake_object_pose, &QPushButton::released, this,
                   [this]() { buttonActivated("fake_object_pose", "Fake Object Pose"); });
}

CommandPanel::~CommandPanel() = default;

void CommandPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_command = node->create_publisher<std_msgs::msg::String>("/main/main_command", 10);
  subscription_rviz_text = node->create_subscription<std_msgs::msg::String>(
      "/main/rviz_text", 10, std::bind(&CommandPanel::topicCallback, this, std::placeholders::_1));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void CommandPanel::topicCallback(const std_msgs::msg::String& msg)
{
  label_->setText(QString(msg.data.c_str()));
}

void CommandPanel::buttonActivated(const std::string& command, const QString& label_text)
{
  auto message = std_msgs::msg::String();
  message.data = command;
  publisher_command->publish(message);
  label_->setText(label_text);
}

void CommandPanel::onTargetObjChanged(const QString& text) { sendTargetObjParam(text); }

void CommandPanel::sendTargetObjParam(const QString& target)
{
  auto node = rclcpp::Node::make_shared("rviz_set_param_client");
  auto client =
      node->create_client<rcl_interfaces::srv::SetParameters>("/main/set_parameters");

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node->get_logger(), "Service not available.");
    label_->setText(QString("Main Node not available. Failed to set param."));
    return;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter param;
  param.name = "target_obj";
  param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  param.value.string_value = target.toStdString();
  request->parameters.push_back(param);

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Param set to: %s", target.toStdString().c_str());
    label_->setText(QString("Target Object set to: %1").arg(target));
  }
}

}  // namespace senior_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(senior_rviz_panel::CommandPanel, rviz_common::Panel)
