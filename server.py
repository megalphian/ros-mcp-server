from mcp.server.fastmcp import FastMCP
from typing import List, Any, Optional
from pathlib import Path
import json
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState
from contextlib import asynccontextmanager

LOCAL_IP = "127.0.0.1"  # Replace with your local IP address
ROBOT_ID = "08"  # Replace with your robot ID
URL = "wss://robohub.eng.uwaterloo.ca/uwbot-" + ROBOT_ID + "-rosbridge/"  # Replace with your rosbridge server IP address

mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(URL)
twist = Twist(ws_manager, topic="/cmd_vel")
image = Image(ws_manager, topic="/oakd/rgb/image_raw")
jointstate = JointState(ws_manager, topic="/joint_states")


@asynccontextmanager
async def lifespan(app: FastMCP):
    # Startup logic
    await ws_manager.connect()
    try:
        yield
    finally:
        # Shutdown logic
        ws_manager.close()

@mcp.tool()
def get_topics():
    topic_info = ws_manager.get_topics()

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
    msg = twist.publish(linear, angular)
    
    if msg is not None:
        return "Twist message published successfully"
    else:
        return "No message published"

@mcp.tool()
def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any]):
    twist.publish_sequence(linear, angular, duration)


@mcp.tool()
def sub_image():
    msg = image.subscribe()
    
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
    if msg is not None:
        return "JointState message published successfully"
    else:
        return "No message published"

@mcp.tool()
def sub_jointstate():
    msg = jointstate.subscribe()
    if msg is not None:
        return msg
    else:
        return "No JointState data received"

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--transport',
        default="streamable-http",
        choices=["streamable-http", "stdio"],
    )
    args = parser.parse_args()
    mcp.run(transport=args.transport)
    
    
