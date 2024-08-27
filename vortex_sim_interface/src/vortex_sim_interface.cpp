#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

class VortexSimInterface : public rclcpp::Node
{
public:
    VortexSimInterface() : Node("vortex_sim_interface")
    {
        subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            "thrust/thruster_forces", 10, std::bind(&VortexSimInterface::thruster_callback, this, std::placeholders::_1));

        pub1_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster1", 10);
        pub2_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster2", 10);
        pub3_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster3", 10);
        pub4_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster4", 10);
        pub5_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster5", 10);
        pub6_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster6", 10);
        pub7_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster7", 10);
        pub8_ = this->create_publisher<std_msgs::msg::Float64>("/orca/cmd_thruster8", 10);

        publishers_ = {pub1_, pub2_, pub3_, pub4_, pub5_, pub6_, pub7_, pub8_};

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/orca/pose_gt", 10, std::bind(&VortexSimInterface::pose_gt_callback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orca/path_gt", 10);

        path_msg_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "VortexSimInterface has been started.");
    }

private:
    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub1_ ;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub3_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub4_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub5_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub6_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub7_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub8_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    void thruster_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg)
    {
        if (msg->thrust.size() != 8) {
            RCLCPP_ERROR(this->get_logger(), "Received ThrusterForces with incorrect size. Expected 8, got %zu", msg->thrust.size());
            return;
        }

        for (size_t i = 0; i < 8; ++i) {
            auto thrust_msg = std::make_unique<std_msgs::msg::Float64>();
            thrust_msg->data = static_cast<double>(msg->thrust[i]);
            publishers_[i]->publish(std::move(thrust_msg));
        }

        RCLCPP_DEBUG(this->get_logger(), "Published thrust values: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
            msg->thrust[0], msg->thrust[1], msg->thrust[2], msg->thrust[3], msg->thrust[4], msg->thrust[5], msg->thrust[6], msg->thrust[7]);
    }

    void pose_gt_callback(const geometry_msgs::msg::Pose::SharedPtr pose_msg)
    {   
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->get_clock()->now();
        pose_stamped.header.frame_id = "map";  // Ensure this matches your frame ID
        pose_stamped.pose = *pose_msg;  // Copy the Pose data into PoseStamped

        // Append the PoseStamped to the path
        path_msg_.poses.push_back(pose_stamped);

        if (path_msg_.poses.size() > 500)
        {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }

        // Publish the path
        path_pub_->publish(path_msg_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}