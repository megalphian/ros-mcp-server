# MCP Functions

This is the list of tools exposed by the ROS MCP Server, with parameters and return values.

## get_topics
- **purpose**: Retrieve available topics and their types via rosapi.
- **returns**: `{ topics: list[str], types: list[str] }` or string when unavailable

## pub_twist
- **purpose**: Publish `geometry_msgs/Twist` to `/cmd_vel`.
- **params**:
  - **linear**: `[x, y, z]` linear velocities in m/s
  - **angular**: `[roll, pitch, yaw]` angular velocities in rad/s
- **returns**: status string

## pub_twist_seq
- **purpose**: Publish a timed sequence of Twist commands.
- **params**:
  - **linear**: list of `[x, y, z]` triples
  - **angular**: list of `[roll, pitch, yaw]` triples
  - **duration**: list of durations (seconds) per step
- **returns**: none (status via logs)

## sub_image
- **purpose**: Capture one image from the camera topic and save it to `screenshots/`.
- **returns**: status string

## get_image_base64
- **purpose**: Capture one image and return a base64-encoded PNG for UI display.
- **returns**: `{ image_data: str, format: "data:image/png;base64,<...>", message: str }` or string

## pub_jointstate
- **purpose**: Publish `sensor_msgs/JointState` to `/joint_states`.
- **params**:
  - **name**: list[str]
  - **position**: list[float]
  - **velocity**: list[float]
  - **effort**: list[float]
- **returns**: status string

## sub_jointstate
- **purpose**: Subscribe once to `/joint_states`.
- **returns**: JSON string of the latest JointState or string when unavailable

## get_scan_data
- **purpose**: Subscribe once to `/scan` for `sensor_msgs/LaserScan`.
- **returns**: dict of LaserScan or string when unavailable

## play_audio
- **purpose**: Publish `irobot_create_msgs/AudioNoteVector` to `/cmd_audio`.
- **params**:
  - **notes**: list of objects, either
    - `{ frequency: float, max_runtime: float }` seconds, or
    - `{ frequency: float, max_runtime: { sec: int, nanosec: int } }`
  - **append**: bool, append to queue if supported
- **returns**: status string

## set_lightring
- **purpose**: Control Create 3 light ring via `/cmd_lightring`.
- **params**:
  - **leds**: list of 6 RGB dicts `{ red: 0-255, green: 0-255, blue: 0-255 }`
    - Publisher sets `override_system=true` by default
- **returns**: status string

## get_dock_status
- **purpose**: Subscribe once to `/dock_status`.
- **returns**: dict of `irobot_create_msgs/DockStatus` or string when unavailable

## get_button_state
- **purpose**: Subscribe once to `/hmi/buttons`.
- **returns**: dict of `turtlebot4_msgs/UserButton` or string when unavailable

## set_user_led
- **purpose**: Set TurtleBot4 HMI user LED via `/hmi/led`.
- **params**:
  - **led**: int (typically 0 or 1)
  - **color**: int (implementation-defined enum; often 0–7)
  - **blink_period**: float seconds; 0 for solid
  - **duty_cycle**: float 0.0–1.0
- **returns**: status string
