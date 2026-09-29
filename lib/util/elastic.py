# Copyright (c) 2023-2025 Gold87 and other Elastic contributors
# This software can be modified and/or shared under the terms
# defined by the Elastic license:
# https://github.com/Gold872/elastic-dashboard/blob/main/LICENSE

import json
from enum import Enum

from ntcore import NetworkTableInstance, PubSubOptions


class NotificationLevel(Enum):
    """
    Represents the possible levels of notifications for the Elastic dashboard. These levels are
    used to indicate the severity or type of notification.
    """

    INFO = "INFO"
    """Informational Message"""
    WARNING = "WARNING"
    """Warning message"""
    ERROR = "ERROR"
    """Error message"""


class Notification:

    def __init__(
        self,
        level: NotificationLevel = NotificationLevel.INFO,
        title: str = "",
        description: str = "",
        display_time_millis: int = 3000,
        width: float = 350,
        height: float = -1,
    ) -> None:
        self._level = level
        self._title = title
        self._description = description
        self._display_time_millis = display_time_millis
        self._width = width
        self._height = height

    def set_level(self, level: NotificationLevel) -> None:
        self._level = level

    def get_level(self) -> NotificationLevel:
        return self._level

    def set_title(self, title: str) -> None:
        self._title = title

    def get_title(self) -> str:
        return self._title

    def set_description(self, description: str) -> None:
        self._description = description

    def get_description(self) -> str:
        return self._description

    def set_display_time_seconds(self, seconds: float) -> None:
        self.set_display_time_millis(int(round(seconds * 1000)))

    def set_display_time_millis(self, display_time_millis: int) -> None:
        self._display_time_millis = display_time_millis

    def get_display_time_millis(self) -> int:
        return self._display_time_millis

    def set_width(self, width: float) -> None:
        self._width = width

    def get_width(self) -> float:
        return self._width

    def set_height(self, height: float) -> None:
        self._height = height

    def get_height(self) -> float:
        return self._height

    def with_level(self, level: NotificationLevel) -> "Notification":
        self._level = level
        return self

    def with_title(self, title: str) -> "Notification":
        self.set_title(title)
        return self

    def with_description(self, description: str) -> "Notification":
        self.set_description(description)
        return self

    def with_display_seconds(self, seconds: float) -> "Notification":
        return self.with_display_milliseconds(int(round(seconds * 1000)))

    def with_display_milliseconds(self, display_time_millis: int) -> "Notification":
        self.set_display_time_millis(display_time_millis)
        return self

    def with_width(self, width: float) -> "Notification":
        self.set_width(width)
        return self

    def with_height(self, height: float) -> "Notification":
        self.set_height(height)
        return self

    def with_automatic_height(self) -> "Notification":
        self._height = -1
        return self

    def with_no_auto_dismiss(self) -> "Notification":
        self.set_display_time_millis(0)
        return self

    def to_dict(self) -> dict:
        return {
            "level": self._level.value,
            "title": self._title,
            "description": self._description,
            "displayTime": self._display_time_millis,
            "width": self._width,
            "height": self._height,
        }


class Elastic:
    NotificationLevel = NotificationLevel
    Notification = Notification

    _notification_topic = NetworkTableInstance.getDefault().getStringTopic(
        "/Elastic/RobotNotifications"
    )
    _notification_publisher = _notification_topic.publish(
        PubSubOptions(sendAll=True, keepDuplicates=True)
    )
    _selected_tab_topic = NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab")
    _selected_tab_publisher = _selected_tab_topic.publish(PubSubOptions(keepDuplicates=True))

    @staticmethod
    def send_notification(notification: Notification) -> None:
        """
        Sends an notification to the Elastic dashboard. The notification is serialized as a JSON string
        before being published.

        :param notification: the `Notification` object containing notification details
        """
        try:
            Elastic._notification_publisher.set(json.dumps(notification.to_dict()))
        except (TypeError, ValueError) as e:
            print(e)

    @staticmethod
    def select_tab(tab_name: str) -> None:
        """
        Selects the tab of the dashboard with the given name. If no tab matches the name, this will
        have no effect on the widgets or tabs in view.

        If the given name is a number, Elastic will select the tab whose index equals the number
        provided.

        :param tab_name: the name of the tab to select
        """
        Elastic._selected_tab_publisher.set(tab_name)
