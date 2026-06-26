# hiwonder_ros2

ROS 2 package for talking to the Hiwonder serial-bus servos. This repo currently ships two nodes:

- `write_only`: the node used by the real-robot launch
- `hiwonder_ros_node`: an older alternating read/write node that is still built but not used by the main launch path

## `write_only`

This is the node started by `antsy_control/real_robot.launch.py`.

Topics and services:

- subscribes: `/actuators`
- services: `/motors/disable`, `/motors/enable`

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `device` | `"/dev/ttyUSB0"` | Serial device used to communicate with the servo bus. |
| `baud_rate` | `9600` | Serial baud rate for the servo bus. |
| `motor_read_rate` | `4` | Used to derive the write timer period. The node writes at `1000 / motor_read_rate / 2` milliseconds. |

Behavior notes:

- `/motors/disable` unloads the servos and pauses outgoing writes
- `/motors/enable` resumes writes and immediately resends the latest actuator command if one exists

## `hiwonder_ros_node`

This older node alternates between command writes and optional position reads.

Topics:

- subscribes: `/joint_states`
- publishes: `/joint_states_read`

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `device` | `"/dev/ttyUSB0"` | Serial device used to communicate with the servo bus. |
| `baud_rate` | `9600` | Serial baud rate for the servo bus. |
| `motor_read_rate` | `4` | Timer frequency used for the alternating read/write loop. |

Important detail: in the current source, the actual read call in `alternatedReadWrite()` is commented out, so this node mostly exists as a reference implementation unless you re-enable position reads.
