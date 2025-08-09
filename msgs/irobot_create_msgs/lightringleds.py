from typing import Any, List, Protocol


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...


class LightringLeds:
    def __init__(self, publisher: Publisher, topic: str):
        self.publisher = publisher
        self.topic = topic

    def publish(self, leds: List[Any], override_system: bool = True):
        msg = {
            "op": "publish",
            "topic": self.topic,
            "type": "irobot_create_msgs/msg/LightringLeds",
            "msg": {
                "header": {},
                "override_system": override_system,
                "leds": leds,
            },
        }
        self.publisher.send(msg)
        return msg
