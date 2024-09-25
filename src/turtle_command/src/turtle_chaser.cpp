#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;
/*
The TurtleChaser will be moving after the TurtleRunner by subscribing to the Pose that the turtlesim_node emits.
*/
class TurtleChaser : public rclcpp::Node
{
    public:
        TurtleChaser()
        : Node("turtle_chaser"), count_(0)
        {
            // subscriber to the turtle Pose. only interested in the x,y coords
            target_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtleChaser::topic_callback, this, std::placeholders::_1));

            // TODO: spawn a new turtle and make its trail red

        }

    private:
    void topic_callback(const turtlesim::msg::Pose &msg) const
    { 
        // TODO: placeholder for logic for movement towards runner
        RCLCPP_INFO(this->get_logger(), "I heard: [%f], [%f]'", msg.x, msg.y);
    }
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr target_sub_;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleChaser>());
    rclcpp::shutdown();
    return 0;
}