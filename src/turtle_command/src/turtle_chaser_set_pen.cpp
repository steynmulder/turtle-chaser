#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/set_pen.hpp"


using namespace std::chrono_literals;

/*
Service client to change the trail color of the chaser turtle
*/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 5) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: turtle set_pen client r g b width");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("turtle_chaser_set_pen");
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client =
    node->create_client<turtlesim::srv::SetPen>("turtle2/set_pen");

  auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
  request->r = atoll(argv[1]);
  request->g = atoll(argv[2]);
  request->b = atoll(argv[3]);
  request->width = atoll(argv[4]);
  

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully set pen for turtle chaser");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}