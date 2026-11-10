#pragma once

#include <memory>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QTextEdit;
class QTimer;

namespace pinky_mission_panel {
class MissionPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit MissionPanel(QWidget *parent = nullptr);
  ~MissionPanel() override;
private:
  void command(const QString &op);
  void configure();
  void updateStatus(const std::string &json);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  QTimer *timer_;
  QLabel *count_;
  QLabel *status_;
  QLabel *pose_;
  QLabel *progress_;
  QLabel *nav_;
  QLabel *error_;
  QTextEdit *points_;
  QComboBox *kind_;
  QSpinBox *cycles_;
  QDoubleSpinBox *wait_;
  QCheckBox *continuous_;
  QSpinBox *edit_index_;
  QDoubleSpinBox *edit_x_;
  QDoubleSpinBox *edit_y_;
  QDoubleSpinBox *edit_yaw_;
  QCheckBox *use_yaw_;
};
}
