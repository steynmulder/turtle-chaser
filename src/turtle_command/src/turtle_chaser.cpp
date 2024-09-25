#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

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
            // subscriber to the runner turtle Pose. only interested in the x,y coords
            target_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtleChaser::target_pose_callback, this, std::placeholders::_1));

            // subscriber to the runner turtle Pose. only interested in the x,y coords
            self_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,
            std::bind(&TurtleChaser::self_pose_callback, this, std::placeholders::_1));

            // publisher to chaser turtle cmd_vel
            chase_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&TurtleChaser::chase_callback, this));

            // client for teleporting runner turtle to a new position once it is tagged
            teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");

            // client for spawning chaser turtle upon simulation start
            spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");

            // client for spawning chaser turtle upon simulation start
            set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("turtle2/set_pen");

            while (!teleport_client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for teleport service to become available");
            }

            while (!spawn_client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for spawn service to become available");
            }

        }

        // service client to spawn the chaser turtle
        void spawn_turtle(float x, float y, const std::string &name) {
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = x;
            request->y = y;
            request->name = name;

            auto result_future = spawn_client_->async_send_request(request);

            // only call the set_pen service once the chaser turtle has been created and the service is available
            while (!set_pen_client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for set_pen service to become available");
            }

            this->set_pen_turtle(255, 0, 0, 5);
        }

        // service client to set the color of the chaser turtle's trail
        void set_pen_turtle(float r, float g, float b, float width)
        {
            auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
            request->r = r;
            request->g = g;
            request->b = b;
            request->width = width;

            auto result_future = set_pen_client_->async_send_request(request);

        }

        // service client to teleport the runner turtle to a new location once it is tagged by the chaser turtle
        void teleport_turtle(float x, float y, float theta)
        {
            auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            request->x = x;
            request->y = y;
            request->theta = theta;

            auto result_future = teleport_client_->async_send_request(request);
            
        }

    private:

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr target_sub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr self_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chase_pub_;
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;

        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        turtlesim::msg::Pose target_pose_;
        turtlesim::msg::Pose self_pose_;
        
        void target_pose_callback(const turtlesim::msg::Pose &msg)
        { 
            // get pose of runner turtle
            target_pose_= msg;

        }

        void self_pose_callback(const turtlesim::msg::Pose &msg)
        { 
            // get pose of chaser turtle
            self_pose_ = msg;
        }

        void chase_callback()
        {
            try 
            {
                geometry_msgs::msg::Twist chase_msg;

                // linear velocity should be constant
                chase_msg.linear.x = 2.0;

                // calculate difference between current angle of chaser turtle and angle needed to get to runner turtle
                double y_dist = target_pose_.y - self_pose_.y;
                double x_dist = target_pose_.x - self_pose_.x;

                double angular_error = atan2(y_dist, x_dist) - self_pose_.theta;

                while (angular_error > M_PI) angular_error -= 2 * M_PI;
                while (angular_error < -M_PI) angular_error += 2 * M_PI;

                // multiply angular velocity by 2 for faster turning
                chase_msg.angular.z = 2 * angular_error;

                chase_pub_->publish(chase_msg);

                // if the chaser turtle has come into the vicinity of the runner, the latter is 'tagged' and teleported to a new position
                if (sqrt((x_dist * x_dist) + (y_dist * y_dist)) < 1.0) {
                    srand(time(NULL));
                    float x = ((double) rand() / (RAND_MAX)) * 9 + 1;
                    float y = ((double) rand() / (RAND_MAX)) * 9 + 1;

                    this->teleport_turtle(x, y, 0.0);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_INFO(this->get_logger(), "Target not found");
            }
            
        }

    
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleChaser>();

    node->spawn_turtle(1.0, 1.0, "turtle2");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}