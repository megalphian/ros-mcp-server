from mcp.server.fastmcp import FastMCP
from typing import List, Any
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState, LaserScan
from msgs.irobot_create_msgs import AudioNoteVector, LightringLeds, DockStatus
from msgs.turtlebot4_msgs import UserButton, UserLed
from msgs.nav_msgs import Odometry

# Function to load configuration
def load_config(config_path="config.json"):
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        return config
    except FileNotFoundError:
        print(f"Configuration file not found at {config_path}. Using default values.")
        return {} # Return empty dict to use defaults
    except json.JSONDecodeError:
        print(f"Error decoding JSON from {config_path}. Using default values.")
        return {}

# Load configuration
config = load_config()

# Use configuration values or defaults
LOCAL_IP = config.get("LOCAL_IP", "127.0.0.1")
ROBOT_ID = config.get("ROBOT_ID", "12")
ROSBRIDGE_URL_BASE = config.get("ROSBRIDGE_URL_BASE", "wss://robohub.eng.uwaterloo.ca/uwbot-")
ROSBRIDGE_URL_SUFFIX = config.get("ROSBRIDGE_URL_SUFFIX", "-rosbridge/")

URL = ROSBRIDGE_URL_BASE + ROBOT_ID + ROSBRIDGE_URL_SUFFIX

mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(URL)
twist = Twist(ws_manager, topic="/cmd_vel")
image = Image(ws_manager, topic="/oakd/rgb/image_raw")
depth = Image(ws_manager, topic="/stereo/depth")
jointstate = JointState(ws_manager, topic="/joint_states")
laserscan = LaserScan(ws_manager, topic="/scan")
audio = AudioNoteVector(ws_manager, topic="/cmd_audio")
lightring = LightringLeds(ws_manager, topic="/cmd_lightring")
dockstatus = DockStatus(ws_manager, topic="/dock_status")
userbutton = UserButton(ws_manager, topic="/hmi/buttons")
userled = UserLed(ws_manager, topic="/hmi/led")
odometry = Odometry(ws_manager, topic="/odom")

@mcp.tool()
def get_topics():
    topic_info = ws_manager.get_topics()
    ws_manager.close()

    if topic_info:
        topics, types = zip(*topic_info)
        return {"topics": list(topics), "types": list(types)}
    else:
        return "No topics found"

@mcp.tool()
def pub_twist(linear: List[Any], angular: List[Any]):
    if len(linear) != 3 or len(angular) != 3:
        return "Linear and angular velocities must be lists of 3 elements each."

    msg = twist.publish(linear, angular)
    ws_manager.close()

    if msg is not None:
        return "Twist message published successfully"
    else:
        return "No message published"

@mcp.tool()
def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any]):
    twist.publish_sequence(linear, angular, duration)

@mcp.tool()
def save_image():
    msg = image.subscribe(save=True)
    ws_manager.close()

    if msg is not None:
        return "Image data received and downloaded successfully"
    else:
        return "No image data received"

@mcp.tool()
def get_image_base64():
    """Get image as base64 string for direct display in chat"""
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
    msg = jointstate.publish(name, position, velocity, effort)
    ws_manager.close()
    if msg is not None:
        return "JointState message published successfully"
    else:
        return "No message published"

@mcp.tool()
def sub_jointstate():
    msg = jointstate.subscribe()
    ws_manager.close()
    if msg is not None:
        return msg
    else:
        return "No JointState data received"
      
@mcp.tool()
def get_scan_data():
    scan_data = laserscan.subscribe()
    ws_manager.close()
    
    if scan_data is not None:
        return scan_data
    else:
        return "No laser scan data received"

@mcp.tool()
def play_audio(notes: List[Any], append: bool = False):
    msg = audio.publish(notes, append)
    ws_manager.close()
    
    if msg is not None:
        return "Audio command published successfully"
    else:
        return "No message published"

@mcp.tool()
def set_lightring(leds: List[Any]):
    msg = lightring.publish(leds)
    ws_manager.close()
    
    if msg is not None:
        return "Lightring command published successfully"
    else:
        return "No message published"

@mcp.tool()
def get_dock_status():
    status = dockstatus.subscribe()
    ws_manager.close()
    
    if status is not None:
        return status
    else:
        return "No dock status data received"

@mcp.tool()
def get_button_state():
    state = userbutton.subscribe()
    ws_manager.close()
    
    if state is not None:
        return state
    else:
        return "No button state data received"

@mcp.tool()
def set_user_led(led: int, color: int, blink_period: float, duty_cycle: float):
    msg = userled.publish(led, color, blink_period, duty_cycle)
    ws_manager.close()
    
    if msg is not None:
        return "User LED command published successfully"
    else:
        return "No message published"

@mcp.tool()
def get_odometry_data():
    odom_data = odometry.subscribe()
    ws_manager.close()
    
    if odom_data is not None:
        return odom_data
    else:
        return "No odometry data received"

if __name__ == "__main__":
    mcp.run(transport="streamable-http")