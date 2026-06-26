# hiwonder_ros2
ROS2 package to control Hiwonder servos

The write-only driver exposes service calls to release and resume servo torque:

```bash
ros2 service call /motors/disable std_srvs/srv/Trigger "{}"
ros2 service call /motors/enable std_srvs/srv/Trigger "{}"
```
