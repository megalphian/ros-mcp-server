import json
from typing import Optional, Protocol


class Transport(Protocol):
    def send(self, message: dict) -> None:
        ...

    def receive_binary(self) -> bytes:
        ...


class DockStatus:
    def __init__(self, transport: Transport, topic: str):
        self.transport = transport
        self.topic = topic

    def subscribe(self) -> Optional[dict]:
        try:
            subscribe_msg = {
                "op": "subscribe",
                "topic": self.topic,
                "type": "irobot_create_msgs/DockStatus",
            }
            self.transport.send(subscribe_msg)

            raw = self.transport.receive_binary()
            if not raw:
                return None

            if isinstance(raw, bytes):
                raw = raw.decode("utf-8")

            data = json.loads(raw)
            if "msg" in data:
                return data["msg"]
            return data
        except Exception as e:
            print(f"[DockStatus] Failed to receive or parse: {e}")
            return None
