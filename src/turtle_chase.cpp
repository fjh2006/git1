#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"//生成新海龟
#include "turtlesim/srv/kill.hpp"
#include "geometry_msgs/msg/twist.hpp"//速度和旋转
#include <random>

class TurtleChase : public rclcpp::Node {
public:
  TurtleChase() : Node("turtle_chase") {
    spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
    pub_turtleA_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    pub_turtleB_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);//创建发布者

    pose_sub_A_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&TurtleChase::poseCallbackA, this, std::placeholders::_1));
    pose_sub_B_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle2/pose", 10, std::bind(&TurtleChase::poseCallbackB, this, std::placeholders::_1));
//订阅位置，回调函数为poseCallback
    if (!spawn_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Spawn service not available.");
    } else if (!kill_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Kill service not available.");
    } else {
      spawnTurtles();
    }//排除错误

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TurtleChase::chase, this));
    
    random_engine_ = std::make_unique<std::mt19937>(std::random_device{}());
    random_distribution_x_ = std::make_unique<std::uniform_real_distribution<>>(0.0, 10.0);
    random_distribution_y_ = std::make_unique<std::uniform_real_distribution<>>(0.0, 10.0);
  }//创建两个在0到10之间均匀分布的随机数

  void move_turtleA(double linear, double angular) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    pub_turtleA_->publish(msg);
  }

  void move_turtleB(double linear, double angular) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    pub_turtleB_->publish(msg);
  }
void killAndRespawnTurtleB() {
  killTurtleB();
  // 等待一段时间或某个条件满足
  rclcpp::sleep_for(std::chrono::seconds(1)); // 等待1秒
  spawnTurtleB();
}
  void chase() {
  if (turtleA_pose_ && turtleB_pose_) {
    double dx = turtleB_pose_->x - turtleA_pose_->x;
    double dy = turtleB_pose_->y - turtleA_pose_->y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double angle_towards_B = std::atan2(dy, dx);

    double angular_speed = 1.0;
    double linear_speed = 0.5;
    double angular_velocity = angular_speed * (angle_towards_B - turtleA_pose_->theta);
    double linear_velocity = linear_speed * (1.0 - std::exp(-distance / 10.0));

    move_turtleA(std::max(0.0, linear_velocity), angular_velocity);

    
    double random_linear_velocity = (*random_distribution_x_)(*random_engine_) - 5.0;
    double random_angular_velocity = (*random_distribution_y_)(*random_engine_) - 5.0;

    
    random_linear_velocity = std::max(-1.0, std::min(1.0, random_linear_velocity));
    random_angular_velocity = std::max(-1.0, std::min(1.0, random_angular_velocity));

    move_turtleB(random_linear_velocity, random_angular_velocity);

    if (distance < 0.8) { 
      killAndRespawnTurtleB();
    }
  }
}

  void poseCallbackA(const turtlesim::msg::Pose::SharedPtr msg) {
    turtleA_pose_ = msg;
  }//当收到turtle1的位置信息时，更新turtleA_pose_成员变量
  void poseCallbackB(const turtlesim::msg::Pose::SharedPtr msg) {
    turtleB_pose_ = msg;
  }

  void spawnTurtles() {
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = 3.0;
    request->y = 3.0;
    request->theta = 0.0;
    spawn_client_->async_send_request(request, std::bind(&TurtleChase::spawnCallback, this, std::placeholders::_1));
//创建一个Spawn服务请求，设置初始位置和方向，然后异步发送请求，并设置回调函数为spawnCallback
    request->x = 5.0;
    request->y = 5.0;
    spawn_client_->async_send_request(request, std::bind(&TurtleChase::spawnCallback, this, std::placeholders::_1));
  //同上
  }

  void killTurtleB() {
    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = "turtle2";
    kill_client_->async_send_request(request, std::bind(&TurtleChase::killCallback, this, std::placeholders::_1));
  }

  void spawnTurtleB() {
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = "turtle2";
    request->x = (*random_distribution_x_)(*random_engine_);
    request->y = (*random_distribution_y_)(*random_engine_);
    request->theta = 0.0;
    spawn_client_->async_send_request(request, std::bind(&TurtleChase::spawnCallback, this, std::placeholders::_1));
  }//创建一个Spawn服务请求，设置turtle2的名称和随机位置，然后异步发送请求，并设置回调函数为spawnCallback

  void killAllTurtles() {
    std::vector<std::string> turtle_names = {"turtle1", "turtle2"};
    for (const auto& name : turtle_names) {
      auto request = std::make_shared<turtlesim::srv::Kill::Request>();
      request->name = name;
      kill_client_->async_send_request(request, std::bind(&TurtleChase::killCallback, this, std::placeholders::_1));
    }
  }

private:
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtleA_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtleB_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_A_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_B_;
  rclcpp::TimerBase::SharedPtr timer_;
  turtlesim::msg::Pose::SharedPtr turtleA_pose_;
  turtlesim::msg::Pose::SharedPtr turtleB_pose_;
  std::unique_ptr<std::mt19937> random_engine_;
  std::unique_ptr<std::uniform_real_distribution<>> random_distribution_x_;
  std::unique_ptr<std::uniform_real_distribution<>> random_distribution_y_;

  void spawnCallback(std::shared_future<turtlesim::srv::Spawn::Response::SharedPtr> future) {
    auto result = future.get();
    if (result) {
      RCLCPP_INFO(this->get_logger(), "Spawned turtle with name: '%s'", result->name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle.");
    }
  }

  void killCallback(std::shared_future<turtlesim::srv::Kill::Response::SharedPtr> future) {
    auto result = future.get();
    if (result) {
      RCLCPP_INFO(this->get_logger(), "Killed turtle.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to kill turtle.");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}