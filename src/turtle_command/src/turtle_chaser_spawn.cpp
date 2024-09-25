#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"


using namespace std::chrono_literals;

/*
Service client to spawn the chaser turtle upon the start of the simulation
*/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: spawn client X Y name");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("turtle_chaser_spawn");
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client =
    node->create_client<turtlesim::srv::Spawn>("spawn");

  auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
  request->x = atoll(argv[1]);
  request->y = atoll(argv[2]);
  request->name = argv[3];

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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully spawned turtle chaser");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}