//
// Created by Noah on 2024-08-18.
//

#include "umrt-arm-joystick-operator/joystick_teleop_node.hpp"
#include <boost/math/special_functions/sign.hpp>

// Helper functions
double getAxisValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t axis);
int getButtonValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t button);
double getButtonsAsAxisValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t positive_button, const size_t negative_button);

#define SGN(x)

JoystickTeleopNode::JoystickTeleopNode() : Node("joystick_teleop") {
    this->initializeParameters();
    this->last_gripper.data = { 0.0 };
    this->last_time = std::chrono::steady_clock::now();
    this->gripper_moving = false;

    this->servo_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(this->servo_twist_topic, JoystickTeleopNode::PUBLISHER_QUEUE_DEPTH);
    this->gripper_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(this->gripper_topic, JoystickTeleopNode::PUBLISHER_QUEUE_DEPTH);
    this->joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            this->joy_topic,
            10,
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { this->handleJoy(msg); }
    );

    // Convert gripper speed to (servo units)/s
    this->gripper_speed_converted = (this->gripper_max - this->gripper_min) * this->gripper_speed / 100.0;

    this->movement_enabled = false;
}


void JoystickTeleopNode::handleJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
    // Check that the deadman switch is engaged
    // Since buttons is an array, we need to check that it is longer than the deadman button index first
    if (msg->buttons.size() > this->deadman_button && msg->buttons[this->deadman_button]) {
        // Check if slow-mode enabled
        double multiplier = getButtonValue(msg, this->slow_button) ? this->slow_modifier : 1.0;

        // Construct the message (maybe refactor into a function)
        auto twist = geometry_msgs::msg::TwistStamped();
        auto twist_wrs = geometry_msgs::msg::TwistStamped();
        twist.twist.linear.x = getAxisValue(msg, this->axis_x_joystick_axis);// * this->axis_speed * multiplier;
        twist.twist.linear.y = getAxisValue(msg, this->axis_y_joystick_axis);// * this->axis_speed * multiplier;
        twist.twist.linear.z = getAxisValue(msg, this->axis_z_joystick_axis);// * this->axis_speed * multiplier;
        twist_wrs.twist.angular.x = getButtonsAsAxisValue(msg, this->wrist_pitch_up_button, this->wrist_pitch_down_button);
        twist_wrs.twist.angular.y = getButtonsAsAxisValue(msg, this->wrist_pitch_up_button, this->wrist_pitch_down_button);
        twist_wrs.twist.angular.z = getButtonsAsAxisValue(msg, this->wrist_pitch_up_button, this->wrist_pitch_down_button);
        twist.header.frame_id = "base_link";
        twist_wrs.header.frame_id = "wrist_link";
        twist.header.stamp = this->get_clock()->now();
        twist_wrs.header.stamp = twist.header.stamp;

        // invert axes
        if (this->axis_x_invert) {
            twist.twist.linear.x *= -1;
        }

        if (this->axis_y_invert) {
            twist.twist.linear.y *= -1;
        }

        if (this->axis_z_invert) {
            twist.twist.linear.z *= -1;
        }

        std_msgs::msg::Float64MultiArray gripper;

        // Calculate new gripper position
        // <0: closing, 0: stopped, >0: opening
        gripper.data = this->last_gripper.data;
        int gripper_direction = getButtonValue(msg, this->gripper_open_button) - getButtonValue(msg, this->gripper_close_button);
        if (gripper_direction) {
            // Direction is non-zero, therefore we need to move

            // Find current time
            std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();

            // If the gripper was already moving, then we can change the position this time step
            // If not, we need to note down the time and wait until the next time step to calculate how far to move
            if (this->gripper_moving) {
                // Calculate time since last gripper update
                std::chrono::duration<double> delta = t - this->last_time;

                // Apply the gripper velocity over this time
                // I don't want to blindly trust that the button value is normalized, so we extract the sign
                // An argument could be made that I shouldn't trust the joystick axes values either, but I don't think that is as big of a risk
                // As well, unlike the joystick axes the buttons are very easy to normalize
                gripper.data[0] = std::clamp(
                        gripper.data[0] += delta.count() * gripper_speed_converted * multiplier * boost::math::sign(gripper_direction),
                        this->gripper_min,
                        this->gripper_max
                );
            }
            else {
                // Gripper is supposed to be moving, but we are not guaranteed to have a correct last_time we can use to calculate the distance
                // Therefore we need to skip this step, but we can flag that we're supposed to be moving
                // Note that we do not have an extra step once the button is released, so this step is truly skipped
                this->gripper_moving = true;
            }

            // Save the current time
            this->last_time = t;
        }
        else {
            // The gripper is not being moved, change the flag to false
            this->gripper_moving = false;
        }

        // Publish the new values
        this->sendValues(twist, twist_wrs, gripper);
    } else if (this->movement_enabled) {
        // Deadman switch no longer engaged, stop movement
        this->gripper_moving = false;
        geometry_msgs::msg::TwistStamped twist1;
        geometry_msgs::msg::TwistStamped twist2;
        twist1.header.frame_id = "base_link";
        twist1.header.stamp = this->get_clock()->now();
        
        twist2.header.frame_id = "wrist_link";
        twist2.header.stamp = twist1.header.stamp;
        
        this->sendValues(twist1, twist2, this->last_gripper);
        this->movement_enabled = false; // Needs to be after sendValues since that sets movement_enabled = true
    }
}

void JoystickTeleopNode::sendValues(const geometry_msgs::msg::TwistStamped& twist1, const geometry_msgs::msg::TwistStamped& twist2, const std_msgs::msg::Float64MultiArray& gripper) {

    this->servo_twist_publisher->publish(twist1);
    this->servo_twist_publisher->publish(twist2);
    this->gripper_publisher->publish(gripper);

    this->movement_enabled = true;
    this->last_gripper = gripper;
}

void JoystickTeleopNode::initializeParameters() {
    RCLCPP_INFO(this->get_logger(), "Parameter initialization starting...");
    /*  Regex to apply to parameter list:
            YAML_NAME                   CPP_NAME                PARAM_TYPE  CAST_TYPE   AS_TYPE
            deadman_button              deadman_button          INTEGER     int         int
            slow_button                 slow_button             INTEGER     int         int
            gripper.open_button         gripper_open_button     INTEGER     int         int
            gripper.close_button        gripper_close_button    INTEGER     int         int
            wrist_buttons.pitch_up      wrist_pitch_up_button   INTEGER     int         int
            wrist_buttons.pitch_down    wrist_pitch_down_button INTEGER     int         int
            wrist_buttons.roll_left     wrist_roll_left_button  INTEGER     int         int
            wrist_buttons.roll_right    wrist_roll_right_button INTEGER     int         int
            axis_x.joystick_axis        axis_x_joystick_axis    INTEGER     int         int
            axis_y.joystick_axis        axis_y_joystick_axis    INTEGER     int         int
            axis_z.joystick_axis        axis_z_joystick_axis    INTEGER     int         int
            axis_x.invert               axis_x_invert           INTEGER     int         int
            axis_y.invert               axis_y_invert           INTEGER     int         int
            axis_z.invert               axis_z_invert           INTEGER     int         int
            gripper.speed               gripper_speed           DOUBLE      double      double
            slow_modifier               slow_modifier           DOUBLE      double      double
            gripper.min                 gripper_min             DOUBLE      double      double
            gripper.max                 gripper_max             DOUBLE      double      double
            servo_twist_topic           servo_twist_topic       STRING      std::string string
            joy_topic                   joy_topic               STRING      std::string string
            gripper_topic               gripper_topic           STRING      std::string string
        Find:
            ^\s*([\.\w]+)\s+(\w+)\s+(\w+)\s+([\w:]+)\s+(\w+)$
        Replace:
            rcl_interfaces::msg::ParameterDescriptor $2_d;
            $2_d.name = "$1";
            const auto& [$2_default, $2_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at($2_d.name);
            $2_d.description = $2_description;
            $2_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_$3;
            $2_d.read_only = true;
            $2_d.dynamic_typing = false;
            this->declare_parameter($2_d.name, boost::get<$4>($2_default), $2_d);
            this->$2 = this->get_parameter("$1").as_$5();\n
     */

    rcl_interfaces::msg::ParameterDescriptor deadman_button_d;
    deadman_button_d.name = "deadman_button";
    const auto& [deadman_button_default, deadman_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(deadman_button_d.name);
    deadman_button_d.description = deadman_button_description;
    deadman_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    deadman_button_d.read_only = true;
    deadman_button_d.dynamic_typing = false;
    this->declare_parameter(deadman_button_d.name, boost::get<int>(deadman_button_default), deadman_button_d);
    this->deadman_button = this->get_parameter("deadman_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor slow_button_d;
    slow_button_d.name = "slow_button";
    const auto& [slow_button_default, slow_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(slow_button_d.name);
    slow_button_d.description = slow_button_description;
    slow_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    slow_button_d.read_only = true;
    slow_button_d.dynamic_typing = false;
    this->declare_parameter(slow_button_d.name, boost::get<int>(slow_button_default), slow_button_d);
    this->slow_button = this->get_parameter("slow_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor gripper_open_button_d;
    gripper_open_button_d.name = "gripper.open_button";
    const auto& [gripper_open_button_default, gripper_open_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_open_button_d.name);
    gripper_open_button_d.description = gripper_open_button_description;
    gripper_open_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    gripper_open_button_d.read_only = true;
    gripper_open_button_d.dynamic_typing = false;
    this->declare_parameter(gripper_open_button_d.name, boost::get<int>(gripper_open_button_default), gripper_open_button_d);
    this->gripper_open_button = this->get_parameter("gripper.open_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor gripper_close_button_d;
    gripper_close_button_d.name = "gripper.close_button";
    const auto& [gripper_close_button_default, gripper_close_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_close_button_d.name);
    gripper_close_button_d.description = gripper_close_button_description;
    gripper_close_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    gripper_close_button_d.read_only = true;
    gripper_close_button_d.dynamic_typing = false;
    this->declare_parameter(gripper_close_button_d.name, boost::get<int>(gripper_close_button_default), gripper_close_button_d);
    this->gripper_close_button = this->get_parameter("gripper.close_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor wrist_pitch_up_button_d;
    wrist_pitch_up_button_d.name = "wrist_buttons.pitch_up";
    const auto& [wrist_pitch_up_button_default, wrist_pitch_up_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(wrist_pitch_up_button_d.name);
    wrist_pitch_up_button_d.description = wrist_pitch_up_button_description;
    wrist_pitch_up_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    wrist_pitch_up_button_d.read_only = true;
    wrist_pitch_up_button_d.dynamic_typing = false;
    this->declare_parameter(wrist_pitch_up_button_d.name, boost::get<int>(wrist_pitch_up_button_default), wrist_pitch_up_button_d);
    this->wrist_pitch_up_button = this->get_parameter("wrist_buttons.pitch_up").as_int();

    rcl_interfaces::msg::ParameterDescriptor wrist_pitch_down_button_d;
    wrist_pitch_down_button_d.name = "wrist_buttons.pitch_down";
    const auto& [wrist_pitch_down_button_default, wrist_pitch_down_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(wrist_pitch_down_button_d.name);
    wrist_pitch_down_button_d.description = wrist_pitch_down_button_description;
    wrist_pitch_down_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    wrist_pitch_down_button_d.read_only = true;
    wrist_pitch_down_button_d.dynamic_typing = false;
    this->declare_parameter(wrist_pitch_down_button_d.name, boost::get<int>(wrist_pitch_down_button_default), wrist_pitch_down_button_d);
    this->wrist_pitch_down_button = this->get_parameter("wrist_buttons.pitch_down").as_int();

    rcl_interfaces::msg::ParameterDescriptor wrist_roll_left_button_d;
    wrist_roll_left_button_d.name = "wrist_buttons.roll_left";
    const auto& [wrist_roll_left_button_default, wrist_roll_left_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(wrist_roll_left_button_d.name);
    wrist_roll_left_button_d.description = wrist_roll_left_button_description;
    wrist_roll_left_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    wrist_roll_left_button_d.read_only = true;
    wrist_roll_left_button_d.dynamic_typing = false;
    this->declare_parameter(wrist_roll_left_button_d.name, boost::get<int>(wrist_roll_left_button_default), wrist_roll_left_button_d);
    this->wrist_roll_left_button = this->get_parameter("wrist_buttons.roll_left").as_int();

    rcl_interfaces::msg::ParameterDescriptor wrist_roll_right_button_d;
    wrist_roll_right_button_d.name = "wrist_buttons.roll_right";
    const auto& [wrist_roll_right_button_default, wrist_roll_right_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(wrist_roll_right_button_d.name);
    wrist_roll_right_button_d.description = wrist_roll_right_button_description;
    wrist_roll_right_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    wrist_roll_right_button_d.read_only = true;
    wrist_roll_right_button_d.dynamic_typing = false;
    this->declare_parameter(wrist_roll_right_button_d.name, boost::get<int>(wrist_roll_right_button_default), wrist_roll_right_button_d);
    this->wrist_roll_right_button = this->get_parameter("wrist_buttons.roll_right").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_x_joystick_axis_d;
    axis_x_joystick_axis_d.name = "axis_x.joystick_axis";
    const auto& [axis_x_joystick_axis_default, axis_x_joystick_axis_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_x_joystick_axis_d.name);
    axis_x_joystick_axis_d.description = axis_x_joystick_axis_description;
    axis_x_joystick_axis_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_x_joystick_axis_d.read_only = true;
    axis_x_joystick_axis_d.dynamic_typing = false;
    this->declare_parameter(axis_x_joystick_axis_d.name, boost::get<int>(axis_x_joystick_axis_default), axis_x_joystick_axis_d);
    this->axis_x_joystick_axis = this->get_parameter("axis_x.joystick_axis").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_y_joystick_axis_d;
    axis_y_joystick_axis_d.name = "axis_y.joystick_axis";
    const auto& [axis_y_joystick_axis_default, axis_y_joystick_axis_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_y_joystick_axis_d.name);
    axis_y_joystick_axis_d.description = axis_y_joystick_axis_description;
    axis_y_joystick_axis_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_y_joystick_axis_d.read_only = true;
    axis_y_joystick_axis_d.dynamic_typing = false;
    this->declare_parameter(axis_y_joystick_axis_d.name, boost::get<int>(axis_y_joystick_axis_default), axis_y_joystick_axis_d);
    this->axis_y_joystick_axis = this->get_parameter("axis_y.joystick_axis").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_z_joystick_axis_d;
    axis_z_joystick_axis_d.name = "axis_z.joystick_axis";
    const auto& [axis_z_joystick_axis_default, axis_z_joystick_axis_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_z_joystick_axis_d.name);
    axis_z_joystick_axis_d.description = axis_z_joystick_axis_description;
    axis_z_joystick_axis_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_z_joystick_axis_d.read_only = true;
    axis_z_joystick_axis_d.dynamic_typing = false;
    this->declare_parameter(axis_z_joystick_axis_d.name, boost::get<int>(axis_z_joystick_axis_default), axis_z_joystick_axis_d);
    this->axis_z_joystick_axis = this->get_parameter("axis_z.joystick_axis").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_x_invert_d;
    axis_x_invert_d.name = "axis_x.invert";
    const auto& [axis_x_invert_default, axis_x_invert_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_x_invert_d.name);
    axis_x_invert_d.description = axis_x_invert_description;
    axis_x_invert_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_x_invert_d.read_only = true;
    axis_x_invert_d.dynamic_typing = false;
    this->declare_parameter(axis_x_invert_d.name, boost::get<int>(axis_x_invert_default), axis_x_invert_d);
    this->axis_x_invert = this->get_parameter("axis_x.invert").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_y_invert_d;
    axis_y_invert_d.name = "axis_y.invert";
    const auto& [axis_y_invert_default, axis_y_invert_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_y_invert_d.name);
    axis_y_invert_d.description = axis_y_invert_description;
    axis_y_invert_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_y_invert_d.read_only = true;
    axis_y_invert_d.dynamic_typing = false;
    this->declare_parameter(axis_y_invert_d.name, boost::get<int>(axis_y_invert_default), axis_y_invert_d);
    this->axis_y_invert = this->get_parameter("axis_y.invert").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_z_invert_d;
    axis_z_invert_d.name = "axis_z.invert";
    const auto& [axis_z_invert_default, axis_z_invert_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_z_invert_d.name);
    axis_z_invert_d.description = axis_z_invert_description;
    axis_z_invert_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_z_invert_d.read_only = true;
    axis_z_invert_d.dynamic_typing = false;
    this->declare_parameter(axis_z_invert_d.name, boost::get<int>(axis_z_invert_default), axis_z_invert_d);
    this->axis_z_invert = this->get_parameter("axis_z.invert").as_int();

    rcl_interfaces::msg::ParameterDescriptor gripper_speed_d;
    gripper_speed_d.name = "gripper.speed";
    const auto& [gripper_speed_default, gripper_speed_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_speed_d.name);
    gripper_speed_d.description = gripper_speed_description;
    gripper_speed_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gripper_speed_d.read_only = true;
    gripper_speed_d.dynamic_typing = false;
    this->declare_parameter(gripper_speed_d.name, boost::get<double>(gripper_speed_default), gripper_speed_d);
    this->gripper_speed = this->get_parameter("gripper.speed").as_double();

    rcl_interfaces::msg::ParameterDescriptor slow_modifier_d;
    slow_modifier_d.name = "slow_modifier";
    const auto& [slow_modifier_default, slow_modifier_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(slow_modifier_d.name);
    slow_modifier_d.description = slow_modifier_description;
    slow_modifier_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    slow_modifier_d.read_only = true;
    slow_modifier_d.dynamic_typing = false;
    this->declare_parameter(slow_modifier_d.name, boost::get<double>(slow_modifier_default), slow_modifier_d);
    this->slow_modifier = this->get_parameter("slow_modifier").as_double();

    rcl_interfaces::msg::ParameterDescriptor gripper_min_d;
    gripper_min_d.name = "gripper.min";
    const auto& [gripper_min_default, gripper_min_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_min_d.name);
    gripper_min_d.description = gripper_min_description;
    gripper_min_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gripper_min_d.read_only = true;
    gripper_min_d.dynamic_typing = false;
    this->declare_parameter(gripper_min_d.name, boost::get<double>(gripper_min_default), gripper_min_d);
    this->gripper_min = this->get_parameter("gripper.min").as_double();

    rcl_interfaces::msg::ParameterDescriptor gripper_max_d;
    gripper_max_d.name = "gripper.max";
    const auto& [gripper_max_default, gripper_max_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_max_d.name);
    gripper_max_d.description = gripper_max_description;
    gripper_max_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gripper_max_d.read_only = true;
    gripper_max_d.dynamic_typing = false;
    this->declare_parameter(gripper_max_d.name, boost::get<double>(gripper_max_default), gripper_max_d);
    this->gripper_max = this->get_parameter("gripper.max").as_double();

    rcl_interfaces::msg::ParameterDescriptor servo_twist_topic_d;
    servo_twist_topic_d.name = "servo_twist_topic";
    const auto& [servo_twist_topic_default, servo_twist_topic_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(servo_twist_topic_d.name);
    servo_twist_topic_d.description = servo_twist_topic_description;
    servo_twist_topic_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    servo_twist_topic_d.read_only = true;
    servo_twist_topic_d.dynamic_typing = false;
    this->declare_parameter(servo_twist_topic_d.name, boost::get<std::string>(servo_twist_topic_default), servo_twist_topic_d);
    this->servo_twist_topic = this->get_parameter("servo_twist_topic").as_string();

    rcl_interfaces::msg::ParameterDescriptor joy_topic_d;
    joy_topic_d.name = "joy_topic";
    const auto& [joy_topic_default, joy_topic_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(joy_topic_d.name);
    joy_topic_d.description = joy_topic_description;
    joy_topic_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    joy_topic_d.read_only = true;
    joy_topic_d.dynamic_typing = false;
    this->declare_parameter(joy_topic_d.name, boost::get<std::string>(joy_topic_default), joy_topic_d);
    this->joy_topic = this->get_parameter("joy_topic").as_string();

    rcl_interfaces::msg::ParameterDescriptor gripper_topic_d;
    gripper_topic_d.name = "gripper_topic";
    const auto& [gripper_topic_default, gripper_topic_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_topic_d.name);
    gripper_topic_d.description = gripper_topic_description;
    gripper_topic_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    gripper_topic_d.read_only = true;
    gripper_topic_d.dynamic_typing = false;
    this->declare_parameter(gripper_topic_d.name, boost::get<std::string>(gripper_topic_default), gripper_topic_d);
    this->gripper_topic = this->get_parameter("gripper_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Parameters initialized!");
}

double getAxisValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t axis) {
    // Ensure axis exists, return 0.0 if not. Note axis is size_t so always >= 0
    return (msg->axes.size() > axis) ? msg->axes[axis] : 0.0;
}

int getButtonValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t button) {
    // Ensure button exists, return 0 if not. Note button is size_t so always >= 0
    return (msg->buttons.size() > button) ? msg->buttons[button] : 0;
}

double getButtonsAsAxisValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t positive_button, const size_t negative_button) {
    // Ensure buttons exists, return 0 if not. Note button is size_t so always >= 0
    return (msg->buttons.size() > positive_button && msg->buttons.size() > negative_button) ? (double)(msg->buttons[positive_button]-msg->buttons[negative_button]) : 0.0;
}