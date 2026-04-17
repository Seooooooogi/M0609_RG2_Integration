#!/usr/bin/env python3
"""
Standalone virtual gripper node for RViz visualization in DRCF emulator mode.

Mirrors the /onrobot/sendCommand service interface of the real OnRobot driver
so grip_test.py can call the same service regardless of mode.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from onrobot_rg_msgs.srv import SetCommand

GRIPPER_OPEN   = -0.558505  # rg2_finger_joint open  (URDF lower limit)
GRIPPER_CLOSED =  0.785398  # rg2_finger_joint closed (URDF upper limit)
PUBLISH_RATE   = 1.0 / 50.0  # 50 Hz — matches real OnRobot driver
SPEED          = 1.0          # rad/s animation speed
DONE_TOL       = 0.01         # rad — "reached target" tolerance


class GripperVirtualNode(Node):

    def __init__(self):
        super().__init__('gripper_virtual_node')
        self._cb_group = ReentrantCallbackGroup()

        self._pub = self.create_publisher(JointState, '/gripper_joint_states', 10)
        self._position = 0.0
        self._target   = 0.0
        self._lock     = threading.Lock()

        self.create_timer(PUBLISH_RATE, self._publish_cb, callback_group=self._cb_group)
        self.create_service(
            SetCommand, '/onrobot/sendCommand',
            self._send_command_cb, callback_group=self._cb_group,
        )
        self.get_logger().info('GripperVirtualNode ready — /onrobot/sendCommand')

    def _send_command_cb(self, req, res):
        if req.command == 'c':
            target = GRIPPER_CLOSED
        elif req.command == 'o':
            target = GRIPPER_OPEN
        else:
            res.success = False
            res.message = f'Unknown command: {req.command!r}'
            return res

        with self._lock:
            self._target = target

        # Block until animation reaches target
        while True:
            with self._lock:
                done = abs(self._position - target) < DONE_TOL
            if done:
                break
            time.sleep(PUBLISH_RATE)

        res.success = True
        res.message = ''
        return res

    def _publish_cb(self):
        with self._lock:
            diff = self._target - self._position
            step = SPEED * PUBLISH_RATE
            if abs(diff) <= step:
                self._position = self._target
            else:
                self._position += step * (1.0 if diff > 0 else -1.0)
            pos = self._position

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['rg2_finger_joint']
        msg.position = [pos]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperVirtualNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
