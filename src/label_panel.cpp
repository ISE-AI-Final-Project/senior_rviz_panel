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
#include <senior_rviz_panel/label_panel.hpp>

namespace senior_rviz_panel
{
LabelPanel::LabelPanel(QWidget * parent) : Panel(parent)
{
  current_obj = std::string("sunscreen");
  inc_x = std::string("5");
  inc_y = std::string("5");
  inc_z = std::string("5");
  inc_rx = std::string("5");
  inc_ry = std::string("5");
  inc_rz = std::string("5");

  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("Let's Label.");
  label_->setMaximumWidth(300);
  layout->addWidget(label_);

  auto * hLayout_target_object = new QHBoxLayout();
  dropdown_target_obj_ = new QComboBox(this);
  dropdown_target_obj_->addItems(
      {"acnewash", "cereal", "contactcleaning", "jbl", "lactose", "orange", "purple", "sunscreen"});
  dropdown_target_obj_->setCurrentText("sunscreen");

  label_target_object = new QLabel("Target Object:");
  hLayout_target_object->addWidget(label_target_object);
  hLayout_target_object->addWidget(dropdown_target_obj_);
  layout->addLayout(hLayout_target_object);

  QObject::connect(dropdown_target_obj_, &QComboBox::currentTextChanged, this,
                   [this](const QString& text) { onComboBoxChanged(current_obj, text); });

  auto * hLayout_x = new QHBoxLayout();
  button_x_plus = new QPushButton("+X");
  button_x_minus = new QPushButton("-X");
  dropdown_x = new QComboBox(this);
  dropdown_x->addItems({"5", "2", "1"});
  hLayout_x->addWidget(button_x_plus);
  hLayout_x->addWidget(button_x_minus);
  hLayout_x->addWidget(dropdown_x);
  layout->addLayout(hLayout_x);
  QObject::connect(button_x_plus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":+x:" + inc_x, current_obj + ":+x:" + inc_x);
  });
  QObject::connect(button_x_minus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":-x:" + inc_x, current_obj + ":-x:" + inc_x);
  });
  QObject::connect(dropdown_x, &QComboBox::currentTextChanged, this,
                   [this](const QString& text) { onComboBoxChanged(inc_x, text); });

  auto * hLayout_y = new QHBoxLayout();
  button_y_plus = new QPushButton("+Y");
  button_y_minus = new QPushButton("-Y");
  dropdown_y = new QComboBox(this);
  dropdown_y->addItems({"5", "2", "1"});
  hLayout_y->addWidget(button_y_plus);
  hLayout_y->addWidget(button_y_minus);
  hLayout_y->addWidget(dropdown_y);
  layout->addLayout(hLayout_y);
  QObject::connect(button_y_plus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":+y:" + inc_y, current_obj + ":+y:" + inc_y);
  });
  QObject::connect(button_y_minus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":-y:" + inc_y, current_obj + ":-y:" + inc_y);
  });
  QObject::connect(dropdown_y, &QComboBox::currentTextChanged, this,
                   [this](const QString& text) { onComboBoxChanged(inc_y, text); });

  auto * hLayout_z = new QHBoxLayout();
  button_z_plus = new QPushButton("+Z");
  button_z_minus = new QPushButton("-Z");
  dropdown_z = new QComboBox(this);
  dropdown_z->addItems({"5", "2", "1"});
  hLayout_z->addWidget(button_z_plus);
  hLayout_z->addWidget(button_z_minus);
  hLayout_z->addWidget(dropdown_z);
  layout->addLayout(hLayout_z);
  QObject::connect(button_z_plus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":+z:" + inc_z, current_obj + ":+z:" + inc_z);
  });
  QObject::connect(button_z_minus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":-z:" + inc_z, current_obj + ":-z:" + inc_z);
  });
  QObject::connect(dropdown_z, &QComboBox::currentTextChanged, this,
                   [this](const QString& text) { onComboBoxChanged(inc_z, text); });

  auto * hLayout_rx = new QHBoxLayout();
  button_rx_plus = new QPushButton("+RX");
  button_rx_minus = new QPushButton("-RX");
  dropdown_rx = new QComboBox(this);
  dropdown_rx->addItems({"5", "2", "1"});
  hLayout_rx->addWidget(button_rx_plus);
  hLayout_rx->addWidget(button_rx_minus);
  hLayout_rx->addWidget(dropdown_rx);
  layout->addLayout(hLayout_rx);
  QObject::connect(button_rx_plus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":+rx:" + inc_rx, current_obj + ":+rx:" + inc_rx);
  });
  QObject::connect(button_rx_minus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":-rx:" + inc_rx, current_obj + ":-rx:" + inc_rx);
  });
  QObject::connect(dropdown_rx, &QComboBox::currentTextChanged, this,
                   [this](const QString& text) { onComboBoxChanged(inc_rx, text); });

  auto * hLayout_ry = new QHBoxLayout();
  button_ry_plus = new QPushButton("+RY");
  button_ry_minus = new QPushButton("-RY");
  dropdown_ry = new QComboBox(this);
  dropdown_ry->addItems({"5", "2", "1"});
  hLayout_ry->addWidget(button_ry_plus);
  hLayout_ry->addWidget(button_ry_minus);
  hLayout_ry->addWidget(dropdown_ry);
  layout->addLayout(hLayout_ry);
  QObject::connect(button_ry_plus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":+ry:" + inc_ry, current_obj + ":+ry:" + inc_ry);
  });
  QObject::connect(button_ry_minus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":-ry:" + inc_ry, current_obj + ":-ry:" + inc_ry);
  });
  QObject::connect(dropdown_ry, &QComboBox::currentTextChanged, this,
                   [this](const QString& text) { onComboBoxChanged(inc_ry, text); });

  auto * hLayout_rz = new QHBoxLayout();
  button_rz_plus = new QPushButton("+RZ");
  button_rz_minus = new QPushButton("-RZ");
  dropdown_rz = new QComboBox(this);
  dropdown_rz->addItems({"5", "2", "1"});
  hLayout_rz->addWidget(button_rz_plus);
  hLayout_rz->addWidget(button_rz_minus);
  hLayout_rz->addWidget(dropdown_rz);
  layout->addLayout(hLayout_rz);
  QObject::connect(button_rz_plus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":+rz:" + inc_y, current_obj + ":+rz:" + inc_rz);
  });
  QObject::connect(button_rz_minus, &QPushButton::released, this, [this]() {
    buttonActivated(current_obj + ":-rz:" + inc_y, current_obj + ":-rz:" + inc_rz);
  });
  QObject::connect(dropdown_rz, &QComboBox::currentTextChanged, this,
                   [this](const QString& text) { onComboBoxChanged(inc_rz, text); });
}

LabelPanel::~LabelPanel() = default;

void LabelPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_command = node->create_publisher<std_msgs::msg::String>("/main/label_command", 10);
  subscription_rviz_text = node->create_subscription<std_msgs::msg::String>(
      "/main/rviz_text", 10, std::bind(&LabelPanel::topicCallback, this, std::placeholders::_1));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void LabelPanel::topicCallback(const std_msgs::msg::String& msg)
{
  label_->setText(QString(msg.data.c_str()));
}

void LabelPanel::buttonActivated(const std::string& label, const std::string& label_text)
{
  auto message = std_msgs::msg::String();
  message.data = label;
  publisher_command->publish(message);
  label_->setText(QString::fromStdString(label_text));
}

void LabelPanel::onComboBoxChanged(std::string& var, const QString& text)
{
  var = text.toStdString();
}

}  // namespace senior_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(senior_rviz_panel::LabelPanel, rviz_common::Panel)
