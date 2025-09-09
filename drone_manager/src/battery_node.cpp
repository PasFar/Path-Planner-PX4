#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BatteryEmulator : public rclcpp::Node {
public:
    BatteryEmulator() : Node("battery_emulator"), battery_low_(false) {
        pub_ = this->create_publisher<std_msgs::msg::String>("/battery", 10);
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/battery",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
                if (msg->data == "charged_") {
                    battery_low_ = false;
                    RCLCPP_INFO(this->get_logger(), "Battery replenished. Restarting timer.");
                    restart_timer();
                }
            }
        );
        start_discharge_timer();
    }

private:
    void start_discharge_timer() {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(365),
            std::bind(&BatteryEmulator::publish_battery_low, this)
        );
    }

    void restart_timer() {
        if (timer_) {
            timer_->cancel();
        }
        battery_low_ = false;
        start_discharge_timer();
    }

    void publish_battery_low() {
        if (!battery_low_) {
            std_msgs::msg::String msg;
            msg.data = "battery_low";
            pub_->publish(msg);
            battery_low_ = true;
            RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool battery_low_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryEmulator>());
    rclcpp::shutdown();
    return 0;
}
