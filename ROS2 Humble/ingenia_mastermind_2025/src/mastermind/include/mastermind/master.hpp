/*
Archivo: master.hpp
Versión: V0.0
Autores: Miguel Ángel García de Vicente
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class MasterNode : public rclcpp::Node {
    public:

        MasterNode() : Node("master"), count_(0) {
            publisher_ = this->create_publisher<std_msgs::msg::String>("prueba_topic", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&MasterNode::timer_callback, this));
        }

    private:

        void timer_callback() {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};
