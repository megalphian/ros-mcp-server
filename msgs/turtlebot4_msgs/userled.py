from typing import Protocol


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...


class UserLed:
    def __init__(self, publisher: Publisher, topic: str):
        self.publisher = publisher
        self.topic = topic

    def publish(self, led: int, color: int, blink_period: float, duty_cycle: float):
        msg = {
            "op": "publish",
            "topic": self.topic,
            "type": "turtlebot4_msgs/msg/UserLed",
            "msg": {
                "led": led,
                "color": color,
                "blink_period": blink_period,
                "duty_cycle": duty_cycle,
            },
        }
        self.publisher.send(msg)
        return msg
