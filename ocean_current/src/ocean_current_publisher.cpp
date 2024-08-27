#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <random>

/*
    This class publishes a current as a random walk process.
*/
class OceanCurrentPublisher : public rclcpp::Node
{
public:
    OceanCurrentPublisher() : Node("ocean_current_publisher")
    {
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/ocean_current", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&OceanCurrentPublisher::publish_current, this));

        // Initialize the random number generator
        rng_ = std::default_random_engine(std::random_device{}());
        distribution_ = std::uniform_real_distribution<double>(-0.005, 0.005);

        current_.x = 0.0;
        current_.y = 0.0;
        current_.z = 0.0;
    }

private:
    void publish_current()
    {
        current_.x += distribution_(rng_);
        current_.y += distribution_(rng_);

        double max_speed = 0.1;
        current_.x = std::clamp(current_.x, -max_speed, max_speed);
        current_.y = std::clamp(current_.y, -max_speed, max_speed);

        RCLCPP_DEBUG(this->get_logger(), "Publishing Ocean Current: x=%f, y=%f, z=%f", current_.x, current_.y, current_.z);

        publisher_->publish(current_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::default_random_engine rng_;
    std::uniform_real_distribution<double> distribution_;
    geometry_msgs::msg::Vector3 current_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OceanCurrentPublisher>());
    rclcpp::shutdown();
    return 0;
}