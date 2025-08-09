from typing import List, Any, Protocol, Optional
import json


class Transport(Protocol):
    def send(self, message: dict) -> None:
        ...

    def receive_binary(self) -> bytes:
        ...


class JointState:
    def __init__(self, transport: Transport, topic: str = "/joint_states"):
        self.transport = transport
        self.topic = topic

    def publish(self, name: List[str], position: List[float], velocity: List[float], effort: List[float]):
        msg = {
            "op": "publish",
            "topic": self.topic,
            "type": "sensor_msgs/JointState",
            "msg": {
                "header": {},
                "name": name,
                "position": position,
                "velocity": velocity,
                "effort": effort,
            },
        }
        self.transport.send(msg)
        return msg

    def subscribe(self, timeout: float = 2.0) -> Optional[str]:
        subscribe_msg = {
            "op": "subscribe",
            "topic": self.topic,
            "type": "sensor_msgs/JointState",
        }
        self.transport.send(subscribe_msg)
        raw = self.transport.receive_binary()
        if not raw:
            return None
        try:
            if isinstance(raw, bytes):
                raw = raw.decode("utf-8")
            data = json.loads(raw)
            if "msg" in data:
                return json.dumps(data["msg"], indent=2, ensure_ascii=False)
            return json.dumps(data, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[JointState] Failed to parse: {e}")
            return None