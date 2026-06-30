#include <cmath>
#include <memory>
#include <string>

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/common/MouseEvent.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/rendering/UserCamera.hh>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class GazeboNavGoalGuiPlugin : public GUIPlugin
{
  public: GazeboNavGoalGuiPlugin()
  {
    this->setObjectName("gazeboNavGoalGuiPlugin");
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setStyleSheet(
      "QFrame#gazeboNavGoalFrame {"
      " background-color: rgba(32, 36, 40, 220);"
      " border: 1px solid rgba(255,255,255,80);"
      " border-radius: 8px;"
      "}"
      " QLabel { color: white; }"
      " QPushButton { padding: 6px 10px; }"
    );

    auto *outerLayout = new QVBoxLayout();
    outerLayout->setContentsMargins(12, 12, 12, 12);

    auto *frame = new QFrame(this);
    frame->setObjectName("gazeboNavGoalFrame");
    auto *frameLayout = new QVBoxLayout(frame);
    frameLayout->setContentsMargins(12, 10, 12, 10);
    frameLayout->setSpacing(8);

    this->statusLabel = new QLabel("Goal mode OFF", frame);
    this->statusLabel->setWordWrap(true);
    frameLayout->addWidget(this->statusLabel);

    auto *buttonRow = new QHBoxLayout();
    this->toggleButton = new QPushButton("Set Nav Goal", frame);
    this->toggleButton->setCheckable(true);
    QObject::connect(
      this->toggleButton,
      &QPushButton::toggled,
      [this](bool _checked)
      {
        this->armed = _checked;
        this->UpdateStatus();
      });
    buttonRow->addWidget(this->toggleButton);

    auto *hintLabel = new QLabel("Click floor in Gazebo to send goal", frame);
    hintLabel->setWordWrap(true);
    buttonRow->addWidget(hintLabel, 1);
    frameLayout->addLayout(buttonRow);

    outerLayout->addWidget(frame);
    outerLayout->addStretch();
    this->setLayout(outerLayout);
    this->move(18, 80);
    this->resize(320, 110);

    if (!rclcpp::ok())
    {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    this->rosNode = std::make_shared<rclcpp::Node>("gazebo_nav_goal_gui");
    this->goalPublisher =
      this->rosNode->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/gazebo/nav_goal_pose", 10);

    this->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    this->executor->add_node(this->rosNode);
    this->spinner = std::thread([this]()
    {
      this->executor->spin();
    });

    gui::MouseEventHandler::Instance()->AddReleaseFilter(
      "gazebo_nav_goal_release_filter",
      [this](const common::MouseEvent &_event)
      {
        return this->OnMouseRelease(_event);
      });
  }

  public: ~GazeboNavGoalGuiPlugin() override
  {
    gui::MouseEventHandler::Instance()->RemoveReleaseFilter(
      "gazebo_nav_goal_release_filter");

    if (this->executor)
    {
      this->executor->cancel();
    }

    if (this->spinner.joinable())
    {
      this->spinner.join();
    }
  }

  public: void Load(sdf::ElementPtr) override
  {
    this->UpdateStatus();
    this->show();
  }

  private: bool OnMouseRelease(const common::MouseEvent &_event)
  {
    if (!this->armed || _event.Button() != common::MouseEvent::LEFT)
    {
      return false;
    }

    auto camera = gui::get_active_camera();
    if (!camera)
    {
      this->statusLabel->setText("No active camera");
      return false;
    }

    ignition::math::Vector3d origin;
    ignition::math::Vector3d direction;
    const auto mousePos = _event.Pos();
    camera->CameraToViewportRay(mousePos.X(), mousePos.Y(), origin, direction);

    const double dz = direction.Z();
    if (std::abs(dz) < 1e-6)
    {
      this->statusLabel->setText("Click failed: ray parallel to floor");
      this->toggleButton->setChecked(false);
      return true;
    }

    const double floorZ = 0.0;
    const double t = (floorZ - origin.Z()) / dz;
    if (t <= 0.0)
    {
      this->statusLabel->setText("Click failed: floor not visible");
      this->toggleButton->setChecked(false);
      return true;
    }

    const auto hit = origin + direction * t;
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->rosNode->now();
    goal.pose.position.x = hit.X();
    goal.pose.position.y = hit.Y();
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;

    this->goalPublisher->publish(goal);

    this->lastGoalText =
      "Sent goal x=" + std::to_string(hit.X()).substr(0, 5) +
      " y=" + std::to_string(hit.Y()).substr(0, 5);
    this->toggleButton->setChecked(false);
    return true;
  }

  private: void UpdateStatus()
  {
    if (this->armed)
    {
      this->statusLabel->setText("Goal mode ON: click floor in Gazebo");
    }
    else if (!this->lastGoalText.empty())
    {
      this->statusLabel->setText(QString::fromStdString(this->lastGoalText));
    }
    else
    {
      this->statusLabel->setText("Goal mode OFF");
    }
  }

  private: QLabel *statusLabel{nullptr};
  private: QPushButton *toggleButton{nullptr};
  private: bool armed{false};
  private: std::string lastGoalText;
  private: rclcpp::Node::SharedPtr rosNode;
  private: rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPublisher;
  private: std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  private: std::thread spinner;
};

GZ_REGISTER_GUI_PLUGIN(GazeboNavGoalGuiPlugin)
}  // namespace gazebo
