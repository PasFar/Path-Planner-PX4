#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

enum class State {
    Takeoff,
    Coverage,
    Land,
    Completed
};

class FSMNode : public rclcpp::Node {
public:
    FSMNode() : Node("fsm_node"), state_(State::Takeoff), waypoint_idx_(0), takeoff_waited_(false) {

        // --- Publishers ---
        cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/seed_pdt_drone/command", 10);
        battery_pub_ = this->create_publisher<std_msgs::msg::String>("/battery", 10); 

        // --- Subscribers ---
        auto qos_odom = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::BestEffort);
        odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>("/model/x500_depth_0/odometry", qos_odom, std::bind(&FSMNode::odom_callback, this, std::placeholders::_1));
        battery_sub_ = this->create_subscription<std_msgs::msg::String>("/battery", 10, std::bind(&FSMNode::battery_callback, this, std::placeholders::_1));
        
        // --- TF2 Listener ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        waypoints_ = {"goal1", "goal2", "goal3", "goal4", "goal5", "goal6", "goal7", "goal8", "goal9"};

        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&FSMNode::run, this));
     
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    State state_;
    std::vector<std::string> waypoints_;

    size_t waypoint_idx_ = 0;
    size_t last_visited_waypoint_idx_ = 0;

    bool battery_low_ = false;
    bool takeoff_waited_ = false;
    bool land_waiting_ = false;
    bool mission_completed_ = false;
    bool coverage_completed_ = false;
    bool coverage_restarted_ = false;
    
    nav_msgs::msg::Odometry odom_;


    void run() {
        switch (state_) {
            case State::Takeoff:
                if (!takeoff_waited_) {
                    send_takeoff();
                    takeoff_waited_ = true;
                    RCLCPP_INFO(this->get_logger(), "State transition: Takeoff started. Waiting to reach target altitude.");
                } else {
                    const double takeoff_altitude = 1.5; 
                    if (std::abs(odom_.pose.pose.position.z - takeoff_altitude) < 0.3) {
                        takeoff_waited_ = false;
                        state_ = State::Coverage;
                        RCLCPP_INFO(this->get_logger(), "State transition: Takeoff complete. Altitude reached. Switching to Coverage.");
                    }
                }
                break;

            case State::Coverage:
                if (battery_low_) {
                    constexpr size_t landing_goal_idx = 4; // goal5 
                    if (waypoint_idx_ != landing_goal_idx) {
                        last_visited_waypoint_idx_ = (waypoint_idx_ > 0) ? waypoint_idx_ - 1 : 0;
                        waypoint_idx_ = landing_goal_idx;
                        send_fly_to(waypoints_[waypoint_idx_]);
                        RCLCPP_INFO(this->get_logger(), "Battery low. Returning to charging station.");
                    } else {
                        if (check_odom_(waypoints_[landing_goal_idx])) {
                            state_ = State::Land;
                            RCLCPP_INFO(this->get_logger(), "Battery low. Switching to Land state at charging station.");
                        }
                    }
                } else if (mission_completed_ == false) {
                    if (waypoint_idx_ < waypoints_.size() && !coverage_completed_) {
                        if (waypoint_idx_ == 0) {
                            send_fly_to(waypoints_[waypoint_idx_]);
                            waypoint_idx_++;
                            RCLCPP_INFO(this->get_logger(), "Flying to first waypoint: %s", waypoints_[waypoint_idx_-1].c_str());
                        } else {
                            if (check_odom_(waypoints_[waypoint_idx_-1]) || coverage_restarted_) {
                                coverage_restarted_ = false;
                                if (waypoint_idx_ < waypoints_.size()) {
                                    send_fly_to(waypoints_[waypoint_idx_]);
                                    waypoint_idx_++;
                                    RCLCPP_INFO(this->get_logger(), "Waypoint reached. Flying to next waypoint: %s", waypoints_[waypoint_idx_-1].c_str());
                                }
                            }
                        }
                    } else {
                        if (!coverage_completed_) {
                            if (check_odom_(waypoints_.back())) {
                                if (waypoint_idx_ != 0) {
                                    waypoint_idx_ = 4;
                                    send_fly_to(waypoints_[waypoint_idx_]);
                                    coverage_completed_ = true;
                                    RCLCPP_INFO(this->get_logger(), "Coverage complete. Returning to charging station.");
                                }
                            }
                        } else {
                            if (check_odom_(waypoints_[waypoint_idx_])) {
                                state_ = State::Land;
                                RCLCPP_INFO(this->get_logger(), "Coverage complete. Switching to Land state.");
                            }
                        }
                    }
                    }
                break;

            case State::Land:
                if (!land_waiting_) {
                    send_land();
                    if (battery_low_) {
                        land_waiting_ = true;
                    } else {
                        state_ = State::Completed;
                    }
                } else {
                    constexpr double landed_altitude = 0.02;
                    if (std::abs(odom_.pose.pose.position.z) < landed_altitude) {
                        send_battery_replenished();
                        battery_low_ = false;
                        land_waiting_ = false;
                        if (!mission_completed_ && !coverage_completed_) {
                            waypoint_idx_ = last_visited_waypoint_idx_;
                            state_ = State::Takeoff;
                            coverage_restarted_ = true;
                            RCLCPP_INFO(this->get_logger(), "Battery replenished. Resuming coverage from waypoint %zu.", waypoint_idx_);
                        } else {
                            state_ = State::Completed;
                        }
                    }
                }
                break;

            case State::Completed:
                mission_completed_ = true;
                RCLCPP_INFO(this->get_logger(), "Mission completed.");
                break;
        }
    }

    void send_takeoff() {
        std_msgs::msg::String msg;
        msg.data = "takeoff";
        cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Takeoff command sent.");
    }

    void send_fly_to(const std::string& waypoint) {
        std_msgs::msg::String msg;
        msg.data = "flyto(" + waypoint + ")";
        cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Fly-to command sent to waypoint: %s", waypoint.c_str());
    }

    void send_land() {
        std_msgs::msg::String msg;
        msg.data = "land";
        cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Land command sent.");
    }

    void send_battery_replenished() {
        std_msgs::msg::String msg;
        msg.data = "charged_";
        battery_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Battery replenished message sent.");
    }

    void battery_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "battery_low") {
            battery_low_ = true;
            RCLCPP_INFO(this->get_logger(), "Battery low received.");
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry msg){
        odom_ = msg;
    }

    bool check_odom_(const std::string wp_name){

        geometry_msgs::msg::Pose wp_pose;
        if(checkTransform(wp_name, wp_pose)){
            if(std::abs(odom_.pose.pose.position.x - wp_pose.position.x) < 0.3 &&
               std::abs(odom_.pose.pose.position.y - wp_pose.position.y) < 0.3 &&
               std::abs(odom_.pose.pose.position.z - wp_pose.position.z) < 0.3){
                return true;
            } else {
                return false;
            }
        }else{
            return false;
        }
        
    }

    bool checkTransform(const std::string &frame_name, geometry_msgs::msg::Pose &pose)
    {
        geometry_msgs::msg::TransformStamped tf_appr;
        try
        {   
            tf_appr = tf_buffer_->lookupTransform("map", frame_name, tf2::TimePointZero);
            pose.position.x = tf_appr.transform.translation.x;
            pose.position.y = tf_appr.transform.translation.y;
            pose.position.z = tf_appr.transform.translation.z;
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find a transform from map to %s: %s", frame_name.c_str(), ex.what());
            return false;
        }
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FSMNode>());
    rclcpp::shutdown();
    return 0;
}
