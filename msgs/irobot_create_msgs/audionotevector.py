from typing import Any, List, Protocol


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...


class AudioNoteVector:
    def __init__(self, publisher: Publisher, topic: str):
        self.publisher = publisher
        self.topic = topic

    def publish(self, notes: List[Any], append: bool = False):
        msg = {
            "op": "publish",
            "topic": self.topic,
            "type": "irobot_create_msgs/AudioNoteVector",
            "msg": {
                "header": {},
                "notes": notes,
                "append": append,
            },
        }
        self.publisher.send(msg)
        return msg
