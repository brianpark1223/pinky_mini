#include "pinky_mission_panel/mission_panel.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QStringList>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace pinky_mission_panel {
MissionPanel::MissionPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *root = new QVBoxLayout(this);
  auto *scroll = new QScrollArea(this);
  auto *body = new QWidget(scroll);
  auto *layout = new QVBoxLayout(body);
  scroll->setWidget(body);
  scroll->setWidgetResizable(true);
  root->addWidget(scroll);
  auto button = [this](QVBoxLayout *box, const QString &label, const QString &op) {
    auto *b = new QPushButton(label, this);
    box->addWidget(b);
    connect(b, &QPushButton::clicked, this, [this, op]() { command(op); });
  };
  auto *waypoints = new QGroupBox("四个位置 · RViz Publish Point 点击地图", body);
  auto *wl = new QVBoxLayout(waypoints);
  count_ = new QLabel("已选择: 0", waypoints);
  wl->addWidget(count_);
  points_ = new QTextEdit(waypoints);
  points_->setReadOnly(true);
  points_->setMinimumHeight(90);
  wl->addWidget(points_);
  button(wl, "删除最后一个点", "undo");
  button(wl, "清空全部点", "clear_points");
  auto *edit = new QFormLayout();
  edit_index_ = new QSpinBox(waypoints); edit_index_->setRange(1, 99);
  edit_x_ = new QDoubleSpinBox(waypoints); edit_x_->setRange(-10000, 10000); edit_x_->setDecimals(3);
  edit_y_ = new QDoubleSpinBox(waypoints); edit_y_->setRange(-10000, 10000); edit_y_->setDecimals(3);
  edit_yaw_ = new QDoubleSpinBox(waypoints); edit_yaw_->setRange(-180, 180); edit_yaw_->setDecimals(1);
  use_yaw_ = new QCheckBox("使用指定朝向", waypoints);
  edit->addRow("修改编号", edit_index_); edit->addRow("X (m)", edit_x_);
  edit->addRow("Y (m)", edit_y_); edit->addRow("Yaw (°)", edit_yaw_);
  edit->addRow(use_yaw_);
  wl->addLayout(edit);
  auto *apply = new QPushButton("修改目标点", waypoints);
  wl->addWidget(apply);
  connect(apply, &QPushButton::clicked, this, [this]() {
    QJsonObject obj{{"op", "set_waypoint"}, {"index", edit_index_->value() - 1},
                    {"x", edit_x_->value()}, {"y", edit_y_->value()}};
    obj.insert("yaw", use_yaw_->isChecked() ? QJsonValue(edit_yaw_->value() * 3.141592653589793 / 180.0) : QJsonValue::Null);
    std_msgs::msg::String msg; msg.data = QJsonDocument(obj).toJson(QJsonDocument::Compact).toStdString();
    command_pub_->publish(msg);
  });
  layout->addWidget(waypoints);

  auto *settings = new QGroupBox("任务设置", body);
  auto *sf = new QFormLayout(settings);
  kind_ = new QComboBox(settings);
  kind_->addItem("1 只有送货一：1→2→3→1", "delivery_1");
  kind_->addItem("2 只有送货二：1→2→4→1", "delivery_2");
  kind_->addItem("3 同时下单，先送一：1→2→3→4→1", "delivery_3");
  kind_->addItem("4 同时下单，先送二：1→2→4→3→1", "delivery_4");
  kind_->addItem("5 先一后二：1→2→3→2→4→1", "delivery_5");
  kind_->addItem("6 先二后一：1→2→4→2→3→1", "delivery_6");
  kind_->insertSeparator(kind_->count());
  kind_->addItem("通用：顺序导航", "sequence"); kind_->addItem("通用：循环巡逻", "patrol");
  kind_->addItem("通用：往返导航", "roundtrip"); kind_->addItem("通用：定点等待", "wait");
  kind_->addItem("通用：拍照巡检", "photo");
  cycles_ = new QSpinBox(settings); cycles_->setRange(1, 100000);
  wait_ = new QDoubleSpinBox(settings); wait_->setRange(0, 3600); wait_->setSuffix(" s");
  continuous_ = new QCheckBox("持续巡逻，直到取消", settings);
  sf->addRow("任务类型", kind_); sf->addRow("巡逻次数", cycles_);
  sf->addRow("每点等待", wait_); sf->addRow(continuous_);
  auto *save_settings = new QPushButton("应用设置", settings);
  sf->addRow(save_settings);
  connect(save_settings, &QPushButton::clicked, this, [this]() { configure(); });
  layout->addWidget(settings);

  auto *control = new QGroupBox("控制", body);
  auto *cl = new QVBoxLayout(control);
  button(cl, "开始任务", "start"); button(cl, "暂停（取消当前 Nav2 Goal）", "pause");
  button(cl, "继续", "resume"); button(cl, "取消任务", "cancel");
  button(cl, "清空任务", "clear_task");
  button(cl, "软件紧急停止入口（取消 Goal）", "emergency_stop");
  button(cl, "保存任务", "save"); button(cl, "重新加载任务", "load");
  cl->addWidget(new QLabel("实体急停必须在现场可用；软件取消不等于实体急停。", control));
  layout->addWidget(control);

  auto *state = new QGroupBox("状态", body);
  auto *sl = new QVBoxLayout(state);
  status_ = new QLabel("等待任务管理器", state); pose_ = new QLabel("位置: --", state);
  progress_ = new QLabel("进度: --", state); nav_ = new QLabel("Nav2: --", state);
  error_ = new QLabel("", state); error_->setWordWrap(true);
  error_->setStyleSheet("QLabel { color: #ff5555; font-weight: bold; }");
  for (auto *l : {status_, pose_, progress_, nav_, error_}) sl->addWidget(l);
  layout->addWidget(state);
  layout->addStretch();

  static int instance = 0;
  node_ = std::make_shared<rclcpp::Node>("pinky_mission_panel_" + std::to_string(++instance));
  command_pub_ = node_->create_publisher<std_msgs::msg::String>("/pinky_mission/command", 10);
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  status_sub_ = node_->create_subscription<std_msgs::msg::String>("/pinky_mission/status", qos,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { updateStatus(msg->data); });
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {
    if (rclcpp::ok() && node_) rclcpp::spin_some(node_);
  });
  timer_->start(100);
}

MissionPanel::~MissionPanel() { timer_->stop(); status_sub_.reset(); command_pub_.reset(); node_.reset(); }

void MissionPanel::command(const QString &op) {
  QJsonObject obj{{"op", op}};
  std_msgs::msg::String msg;
  msg.data = QJsonDocument(obj).toJson(QJsonDocument::Compact).toStdString();
  command_pub_->publish(msg);
}

void MissionPanel::configure() {
  QJsonObject obj{{"op", "configure"}, {"kind", kind_->currentData().toString()},
                  {"cycles", cycles_->value()}, {"wait_seconds", wait_->value()},
                  {"continuous", continuous_->isChecked()}};
  std_msgs::msg::String msg;
  msg.data = QJsonDocument(obj).toJson(QJsonDocument::Compact).toStdString();
  command_pub_->publish(msg);
}

void MissionPanel::updateStatus(const std::string &json) {
  const auto data = QJsonDocument::fromJson(QByteArray::fromStdString(json)).object();
  const auto points = data.value("points").toArray();
  count_->setText(QString("已选择: %1").arg(points.size()));
  QString lines;
  const QStringList point_names{"1 待机", "2 取货", "3 送货一", "4 送货二"};
  for (int i = 0; i < points.size(); ++i) {
    auto p = points[i].toObject();
    lines += QString("%1: X=%2  Y=%3  Yaw=%4\n")
      .arg(i < point_names.size() ? point_names[i] : QString::number(i + 1)).arg(p.value("x").toDouble(), 0, 'f', 3)
      .arg(p.value("y").toDouble(), 0, 'f', 3)
      .arg(p.value("yaw").isNull() ? "自动" : QString::number(p.value("yaw").toDouble() * 180.0 / 3.141592653589793, 'f', 1));
  }
  points_->setPlainText(lines);
  const int kind_index = kind_->findData(data.value("kind").toString());
  if (kind_index >= 0) kind_->setCurrentIndex(kind_index);
  status_->setText(QString("任务状态: %1 · 运动闸门: %2")
      .arg(data.value("state").toString(), data.value("motion_enabled").toBool() ? "开" : "关"));
  auto pose = data.value("pose").toArray();
  pose_->setText(pose.size() >= 3 ? QString("位置: X=%1 Y=%2 Yaw=%3°")
      .arg(pose[0].toDouble(), 0, 'f', 2).arg(pose[1].toDouble(), 0, 'f', 2)
      .arg(pose[2].toDouble() * 180.0 / 3.141592653589793, 0, 'f', 1) : "位置: TF 不可用");
  progress_->setText(QString("目标: %1 · 进度: %2/%3 · %4")
      .arg(data.value("target").toString()).arg(data.value("step").toInt())
      .arg(data.value("total").toInt()).arg(data.value("feedback").toString()));
  nav_->setText(QString("Nav2: %1 · Map: %2 · Scan: %3 · Costmap: %4")
      .arg(data.value("nav2").toBool() ? "在线" : "离线")
      .arg(data.value("map").toBool() ? "有" : "无")
      .arg(data.value("scan").toBool() ? "有" : "无")
      .arg(data.value("costmap").toBool() ? "有" : "无"));
  error_->setText(QString("错误: %1").arg(data.value("error").toString()));
}
}

PLUGINLIB_EXPORT_CLASS(pinky_mission_panel::MissionPanel, rviz_common::Panel)
