from mcp.server.fastmcp import FastMCP
from typing import List, Any
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState, LaserScan
from msgs.irobot_create_msgs import AudioNoteVector, LightringLeds, DockStatus
from msgs.turtlebot4_msgs import UserButton, UserLed
from msgs.nav_msgs import Odometry
from slam.slam_mapping import OccupancyGridSLAM

# Configure target robot and rosbridge URL
ROBOT_ID = "11"  # Replace with your robot ID
URL = "wss://robohub.eng.uwaterloo.ca/uwbot-" + ROBOT_ID + "-rosbridge/"  # Replace with your rosbridge server IP address

# Initialize MCP server and shared WebSocket transport
mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(URL)

# Instantiate message helpers bound to topics used by the tools
twist = Twist(ws_manager, topic="/cmd_vel")
image = Image(ws_manager, topic="/oakd/rgb/image_raw")
jointstate = JointState(ws_manager, topic="/joint_states")
laserscan = LaserScan(ws_manager, topic="/scan")
audio = AudioNoteVector(ws_manager, topic="/cmd_audio")
lightring = LightringLeds(ws_manager, topic="/cmd_lightring")
dockstatus = DockStatus(ws_manager, topic="/dock_status")
userbutton = UserButton(ws_manager, topic="/hmi/buttons")
userled = UserLed(ws_manager, topic="/hmi/led")
odometry = Odometry(ws_manager, topic="/odom")

# SLAM instance (separate websocket connection) created lazily
_slam_instance: OccupancyGridSLAM | None = None

@mcp.tool()
def get_topics():
    """List available ROS topics and types via rosapi.

    Returns:
        dict | str: {"topics": list[str], "types": list[str]} or a message if none found.
    """
    topic_info = ws_manager.get_topics()
    ws_manager.close()

    if topic_info:
        topics, types = zip(*topic_info)
        return {
            "topics": list(topics),
            "types": list(types)
        }
    else:
        return "No topics found"

@mcp.tool()
def pub_twist(linear: List[Any], angular: List[Any]):
    """Publish a geometry_msgs/Twist to `/cmd_vel`.

    Parameters:
        linear: [x, y, z] linear velocities in m/s.
        angular: [roll, pitch, yaw] angular velocities in rad/s.

    Returns:
        str: Result message.
    """
    msg = twist.publish(linear, angular)
    ws_manager.close()
    
    if msg is not None:
        return "Twist message published successfully"
    else:
        return "No message published"

@mcp.tool()
def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any]):
    """Publish a timed sequence of Twist commands.

    Parameters:
        linear: list of [x, y, z] triples for each step.
        angular: list of [roll, pitch, yaw] triples for each step.
        duration: list of durations (s) for each step.
    """
    twist.publish_sequence(linear, angular, duration)


@mcp.tool()
def sub_image():
    """Capture one image from the camera topic and save it to `screenshots/`.

    Returns:
        str: Status string indicating save result.
    """
    msg = image.subscribe()
    ws_manager.close()
    
    if msg is not None:
        return "Image data received and downloaded successfully"
    else:
        return "No image data received"

@mcp.tool()
def get_image_base64():
    """Capture one image and return it as a base64 PNG string for display.

    Returns:
        dict | str: {"image_data": base64, "format": "data:image/png;base64,<...>", "message": str}
        or error string when unavailable.
    """
    img_base64 = image.get_base64_image()
    ws_manager.close()
    
    if img_base64 is not None:
        return {
            "image_data": img_base64,
            "format": "data:image/png;base64," + img_base64,
            "message": "Image captured successfully"
        }
    else:
        return "No image data received"

@mcp.tool()
def pub_jointstate(name: list[str], position: list[float], velocity: list[float], effort: list[float]):
    """Publish a sensor_msgs/JointState to `/joint_states`.

    Parameters:
        name: joint names.
        position: joint positions (rad or meters depending on joint type).
        velocity: joint velocities.
        effort: joint efforts.

    Returns:
        str: Result message.
    """
    msg = jointstate.publish(name, position, velocity, effort)
    ws_manager.close()
    if msg is not None:
        return "JointState message published successfully"
    else:
        return "No message published"

@mcp.tool()
def sub_jointstate():
    """Subscribe once to `/joint_states` and return the latest message as JSON string."""
    msg = jointstate.subscribe()
    ws_manager.close()
    if msg is not None:
        return msg
    else:
        return "No JointState data received"

@mcp.tool()
def get_scan_data():
    """Subscribe once to `/scan` and return the latest sensor_msgs/LaserScan as dict."""
    scan_data = laserscan.subscribe()
    ws_manager.close()
    
    if scan_data is not None:
        return scan_data
    else:
        return "No laser scan data received"

@mcp.tool()
def play_audio(notes: List[Any], append: bool = False):
    """Publish an AudioNoteVector to `/cmd_audio`.

    Parameters:
        notes: list of note objects. Supported forms:
            - {"frequency": float, "max_runtime": float seconds}
            - {"frequency": float, "max_runtime": {"sec": int, "nanosec": int}}
        append: when True, appends to current queue if supported.

    Returns:
        str: Result message.
    """
    msg = audio.publish(notes, append)
    ws_manager.close()
    
    if msg is not None:
        return "Audio command published successfully"
    else:
        return "No message published"

@mcp.tool()
def set_lightring(leds: List[Any]):
    """Control the Create 3 light ring via `/cmd_lightring`.

    Parameters:
        leds: list of 6 RGB dicts: [{"red": 0-255, "green": 0-255, "blue": 0-255}, ...]
              `override_system` is enabled by default in the publisher.

    Returns:
        str: Result message.
    """
    msg = lightring.publish(leds)
    ws_manager.close()
    
    if msg is not None:
        return "Lightring command published successfully"
    else:
        return "No message published"

@mcp.tool()
def get_dock_status():
    """Subscribe once to `/dock_status` and return irobot_create_msgs/DockStatus as dict."""
    status = dockstatus.subscribe()
    ws_manager.close()
    
    if status is not None:
        return status
    else:
        return "No dock status data received"

@mcp.tool()
def get_button_state():
    """Subscribe once to `/hmi/buttons` and return turtlebot4_msgs/UserButton as dict."""
    state = userbutton.subscribe()
    ws_manager.close()
    
    if state is not None:
        return state
    else:
        return "No button state data received"

@mcp.tool()
def set_user_led(led: int, color: int, blink_period: float, duty_cycle: float):
    """Set TurtleBot4 HMI user LED via `/hmi/led`.

    Parameters:
        led: LED index (typically 0 or 1).
        color: implementation-defined color enum (commonly 0-7).
        blink_period: seconds per blink cycle; 0 for solid.
        duty_cycle: 0.0-1.0 fraction of cycle that LED is on.

    Returns:
        str: Result message.
    """
    msg = userled.publish(led, color, blink_period, duty_cycle)
    ws_manager.close()
    
    if msg is not None:
        return "User LED command published successfully"
    else:
        return "No message published"

@mcp.tool()
def get_odometry_data():
    """Subscribe once to `/odom` and return nav_msgs/Odometry as dict."""
    odom_data = odometry.subscribe()
    ws_manager.close()
    
    if odom_data is not None:
        return odom_data
    else:
        return "No odometry data received"


@mcp.tool()
def slam_start(scan_topic: str = "/scan", odom_topic: str = "/odom"):
    """Start in-memory occupancy grid SLAM using LaserScan and Odometry.

    Parameters:
        scan_topic: topic name for sensor_msgs/LaserScan
        odom_topic: topic name for nav_msgs/Odometry
    """
    global _slam_instance
    if _slam_instance is None:
        _slam_instance = OccupancyGridSLAM(URL, scan_topic=scan_topic, odom_topic=odom_topic)
    else:
        _slam_instance.stop()
        _slam_instance = OccupancyGridSLAM(URL, scan_topic=scan_topic, odom_topic=odom_topic)
    _slam_instance.start()
    return "SLAM started"


@mcp.tool()
def slam_stop():
    """Stop the SLAM background loop and close its connection."""
    global _slam_instance
    if _slam_instance is None:
        return "SLAM not running"
    _slam_instance.stop()
    _slam_instance = None
    return "SLAM stopped"


@mcp.tool()
def slam_reset_map():
    """Clear the occupancy grid to unknown state."""
    if _slam_instance is None:
        return "SLAM not running"
    _slam_instance.reset_map()
    return "Map reset"


@mcp.tool()
def slam_get_map():
    """Get the current occupancy grid as metadata and flattened data array (ROS-like)."""
    if _slam_instance is None:
        return "SLAM not running"
    return _slam_instance.get_map_data()


@mcp.tool()
def slam_get_map_image():
    """Get the current occupancy grid as a base64 PNG image string."""
    if _slam_instance is None:
        return "SLAM not running"
    img_b64 = _slam_instance.get_map_image_base64()
    if img_b64:
        return {"image_data": img_b64, "format": "data:image/png;base64," + img_b64}
    return "No map image available"


@mcp.tool()
def slam_get_stats():
    """Get SLAM statistics including scan count, pose info, and grid occupancy stats."""
    if _slam_instance is None:
        return "SLAM not running"
    return _slam_instance.get_slam_stats()

if __name__ == "__main__":
    mcp.run(transport="streamable-http")
