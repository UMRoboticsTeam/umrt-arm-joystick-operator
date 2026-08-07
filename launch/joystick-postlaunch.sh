set -e

# Enable the servo nodes
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
ros2 service call /servo_node_gripper/start_servo std_srvs/srv/Trigger {}

# Change the drift directions, because I can't figure out how to set the 
ros2 service call /servo_node/change_drift_dimensions moveit_msgs/srv/ChangeDriftDimensions "drift_x_translation: false
drift_y_translation: false
drift_z_translation: false
drift_x_rotation: false
drift_y_rotation: false
drift_z_rotation: true
transform_jog_frame_to_drift_frame:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 
