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

  QObject::connect(button_generate_all_grasp, &QPushButton::released, this,
                   [this]() { buttonActivated("generate_all_grasp", "Generating All Grasp Pose..."); });
  QObject::connect(button_generate_best_grasp, &QPushButton::released, this,
                   [this]() { buttonActivated("generate_best_grasp", "Generating Best Grasp Pose..."); });

  button_make_collision = new QPushButton("Make Collision");
  layout->addWidget(button_make_collision);
  QObject::connect(button_make_collision, &QPushButton::released, this,
                   [this]() { buttonActivated("make_collision", "Making Collision..."); });


  button_publish_goal = new QPushButton("Publish Goal");
  layout->addWidget(button_publish_goal);
  QObject::connect(button_publish_goal, &QPushButton::released, this,
                   [this]() { buttonActivated("publish_goal", "Publishing Goal..."); });

  // Same line
  auto * hLayout3 = new QHBoxLayout();
  button_gripper_open = new QPushButton("Gripper Open");
  button_gripper_close = new QPushButton("Gripper Close");
  hLayout3->addWidget(button_gripper_open);
  hLayout3->addWidget(button_gripper_close);
  layout->addLayout(hLayout3);

  QObject::connect(button_gripper_open, &QPushButton::released, this,
                   [this]() { buttonActivated("gripper_open", "Opening Gripper..."); });
  QObject::connect(button_gripper_close, &QPushButton::released, this,
                   [this]() { buttonActivated("gripper_close", "Closing Gripper..."); });
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

}  // namespace senior_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(senior_rviz_panel::CommandPanel, rviz_common::Panel)
