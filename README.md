# ros_ultrasonic_bumper_speed_filter
Node to be used to filter the speed of the robot according to the range measures acquired by Ultrasonic Bumper board through ros_ultrasonic_bumper node

## Topics
### Topics subscribed
* **cmd_vel**: the speed "Twist" message to be filtered
* **ranges**: the "ultrasnd_bump_ranges" message emitted by Ultrasonic Bumper node

### Topics emitted
* **cmd_vel_filtered**: the nel "Twist" message filtered according to ranges measured by Ultrasonic Bumper node
