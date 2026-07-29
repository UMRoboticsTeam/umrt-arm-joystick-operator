//
// Created by Noah on 2024-08-18.
//

#ifndef ARM_FIRMWARE_JOYSTICK_TELEOP_NODE_HPP
#define ARM_FIRMWARE_JOYSTICK_TELEOP_NODE_HPP

#include <string>
#include <unordered_map>
#include <tuple>
#include <chrono>

#include <boost/variant.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class JoystickTeleopNode : public rclcpp::Node {
public:
    static inline const std::unordered_map<std::string, std::tuple<boost::variant<int, double, std::string>, std::string>> DEFAULT_PARAMETERS = {
        {"deadman_button", {0, "Joystick button to enable movement (int)"}},
        {"slow_button", {1, "Joystick button to move at a slower speed (int)"}},
        {"gripper.open_button", {2, "Joystick button to open the gripper (int)"}},
        {"gripper.close_button", {3, "Joystick button to close the gripper (int)"}},
        {"gripper.speed", {50.0, "Speed to move the gripper at when a button is held, in (% of range)/s (double)"}},
        {"gripper.min", {0.0, "Minimum value to allow gripper to be set to, also used in conjunction with gripper_max to determine the range for gripper_speed (double)"}},
        {"gripper.max", {180.0, "Maximum value to allow gripper to be set to, also used in conjunction with gripper_min to determine the range for gripper_speed (double)"}},
        {"wrist_buttons.pitch_up", {11, "Joystick button to pitch the wrist up (int)"}},
        {"wrist_buttons.pitch_down", {12, "Joystick button to pitch the wrist down (int)"}},
        {"wrist_buttons.roll_left", {13, "Joystick button to roll the wrist left (int)"}},
        {"wrist_buttons.roll_right", {14, "Joystick button to roll the wrist right (int)"}},
        {"axis_x.joystick_axis", {0, "Joystick axis corresponding the X axis (int)"}},
        {"axis_y.joystick_axis", {1, "Joystick axis corresponding to the Y axis (int)"}},
        {"axis_z.joystick_axis", {2, "Joystick axis corresponding to the Z axis (int)"}},
        {"axis_x.invert", {0, "Invert the joystick values corresponding to the X axis (bool)"}},
        {"axis_y.invert", {0, "Invert the joystick values corresponding to the Y axis (bool)"}},
        {"axis_z.invert", {0, "Invert the joystick values corresponding to the Z axis (bool)"}},
        {"slow_modifier", {0.1, "Multiplier to apply to speeds when the slow button is held (double)"}},
        {"servo_twist_topic", {"/servo_node/delta_twist_cmds", "Topic to publish cartesian movements to (string)"}},
        {"gripper_topic", {"/gripper_pos", "Topic to publish gripper positions to (string)"}},
        {"joy_topic", {"/joy", "Topic to read Joy messages from (string)"}}
    };

    static constexpr int PUBLISHER_QUEUE_DEPTH = 10;

    JoystickTeleopNode();

    void sendValues(const geometry_msgs::msg::TwistStamped& twist1, const geometry_msgs::msg::TwistStamped& twist2, const std_msgs::msg::Float64MultiArray& gripper_values);

protected:
    void initializeParameters();

    void handleJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    // Should be treated as const, not const because initializing with initializer list was inconvenient and ugly
    size_t deadman_button;
    size_t slow_button;
    size_t gripper_open_button;
    size_t gripper_close_button;
    size_t wrist_pitch_up_button;
    size_t wrist_pitch_down_button;
    size_t wrist_roll_left_button;
    size_t wrist_roll_right_button;
    size_t axis_x_joystick_axis;
    size_t axis_y_joystick_axis;
    size_t axis_z_joystick_axis;
    bool axis_x_invert;
    bool axis_y_invert;
    bool axis_z_invert;
    double gripper_speed;
    double slow_modifier;
    double gripper_min;
    double gripper_max;
    std::string servo_twist_topic;
    std::string gripper_topic;
    std::string joy_topic;

    std_msgs::msg::Float64MultiArray last_gripper;
    std::chrono::steady_clock::time_point last_time;
    bool gripper_moving;

    // gripper_speed converted from (% of range)/s to (servo units)/s
    // E.g. a speed of 0.2 for a servo going from 0 to 180 would convert to 36 (servo units)/s
    double gripper_speed_converted;

    bool movement_enabled;
};


#endif //ARM_FIRMWARE_JOYSTICK_TELEOP_NODE_HPP
