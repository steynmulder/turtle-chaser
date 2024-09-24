#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/*
The TurtleRunner will be one of two turtles in the simulation. This one will be moving randomly around the map
with a constant linear velocity and a random angular velocity. The TurtleChaser will follow its position and try to
'tag' it, upon which event, the TurtleRunner will be teleported to a new random position and the chase starts again.
*/ 

class TurtleRunner : public rclcpp::Node 
{
    public:
        TurtleRunner()
        : Node("turtle_runner"), count_(0)
        {
            // publisher of random movement to the turtle that will run away
            goal_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&TurtleRunner::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            srand(time(NULL));
            auto goal_msg = geometry_msgs::msg::Twist();
            // the turtle will always move forward at a constant velocity
            goal_msg.linear.x = 1.0;
            // the turtle will randomly turn -90 to 90 degrees on every callback
            goal_msg.angular.z = ((double) rand() / (RAND_MAX)) * 6.26 - 3.14;
            RCLCPP_INFO(this->get_logger(), "Publishing random movement for runner: angle: [%f]", goal_msg.angular.z);
            goal_pub_->publish(goal_msg);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goal_pub_;
        size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleRunner>());
    rclcpp::shutdown();
    return 0;
}