#!/usr/bin/env python3
"""Telegram <-> ROS bridge.

Subscribes: /reasoning/response  (std_msgs/String) — Claude's text reply
Publishes:  /reasoning/input     (std_msgs/String) — user message

Config (env):
  TELEGRAM_BOT_TOKEN   required
  TELEGRAM_ALLOWED_IDS optional, comma-separated Telegram user IDs to allow.
                       If unset, the bot accepts messages from any user.

Uses Telegram Bot HTTP API directly via `requests` (long-polling). This
avoids pulling in python-telegram-bot's asyncio runtime alongside ROS.
"""

import os
import threading
import time

import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String


class TelegramBridge(Node):
    def __init__(self):
        super().__init__('telegram_bridge')

        token = os.environ.get('TELEGRAM_BOT_TOKEN')
        if not token:
            raise RuntimeError('TELEGRAM_BOT_TOKEN is not set')
        self._base_url = f'https://api.telegram.org/bot{token}'

        allowed = os.environ.get('TELEGRAM_ALLOWED_IDS', '').strip()
        self._allowed_ids: set[int] | None = (
            {int(x) for x in allowed.split(',') if x.strip()}
            if allowed else None
        )

        self._last_update_id = 0
        self._last_chat_id: int | None = None

        self.input_pub = self.create_publisher(String, '/reasoning/input', 10)
        self.response_sub = self.create_subscription(
            String, '/reasoning/response', self._on_response, 10
        )
        self.photo_sub = self.create_subscription(
            String, '/reasoning/photo', self._on_photo, 10
        )

        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

        if self._allowed_ids:
            self.get_logger().info(
                f'telegram_bridge ready (allowlist of {len(self._allowed_ids)})'
            )
        else:
            self.get_logger().info('telegram_bridge ready (allowlist OFF — any user)')

    # ---------- Telegram -> ROS ----------

    def _poll_loop(self) -> None:
        while rclpy.ok():
            try:
                r = requests.get(
                    f'{self._base_url}/getUpdates',
                    params={
                        'offset': self._last_update_id + 1,
                        'timeout': 30,
                    },
                    timeout=35,
                )
                if r.status_code != 200:
                    self.get_logger().warn(f'telegram getUpdates {r.status_code}')
                    time.sleep(5)
                    continue
                data = r.json()
            except requests.RequestException as e:
                self.get_logger().warn(f'telegram poll error: {e}')
                time.sleep(5)
                continue
            for upd in data.get('result', []):
                self._last_update_id = upd['update_id']
                self._handle_update(upd)

    def _handle_update(self, upd: dict) -> None:
        msg = upd.get('message') or {}
        text = msg.get('text')
        chat = msg.get('chat') or {}
        user = msg.get('from') or {}
        chat_id = chat.get('id')
        user_id = user.get('id')
        if not text or chat_id is None:
            return
        if self._allowed_ids is not None and user_id not in self._allowed_ids:
            self._send(chat_id, 'Unauthorized.')
            self.get_logger().warn(f'rejected msg from user_id={user_id}')
            return

        self._last_chat_id = chat_id
        ros_msg = String()
        ros_msg.data = text
        self.input_pub.publish(ros_msg)

    # ---------- ROS -> Telegram ----------

    def _on_response(self, msg: String) -> None:
        if self._last_chat_id is None:
            return
        self._send(self._last_chat_id, msg.data)

    def _on_photo(self, msg: String) -> None:
        if self._last_chat_id is None:
            return
        path = msg.data
        if not path:
            return
        try:
            with open(path, 'rb') as f:
                r = requests.post(
                    f'{self._base_url}/sendPhoto',
                    data={'chat_id': self._last_chat_id},
                    files={'photo': f},
                    timeout=30,
                )
            if r.status_code != 200:
                self.get_logger().warn(
                    f'telegram sendPhoto {r.status_code}: {r.text[:200]}'
                )
        except (OSError, requests.RequestException) as e:
            self.get_logger().warn(f'telegram sendPhoto error: {e}')

    def _send(self, chat_id: int, text: str) -> None:
        try:
            requests.post(
                f'{self._base_url}/sendMessage',
                json={'chat_id': chat_id, 'text': text[:4000]},
                timeout=10,
            )
        except requests.RequestException as e:
            self.get_logger().warn(f'telegram send error: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TelegramBridge()
    except RuntimeError as e:
        print(f'telegram_bridge startup failed: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
