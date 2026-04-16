import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

import DR_init

# Robot configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# Motion parameters
VELOCITY = 60
ACC = 60

# Digital output states
ON, OFF = 1, 0

# Gripper joint limits (from onrobot_rg2_model_macro.xacro)
GRIPPER_OPEN   = -0.558505   # upper limit — fingers open
GRIPPER_CLOSED = 0.785398   # lower limit — fingers closed

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# RViz 시각화를 위한 가짜 /gripper_joint_states 노드 퍼블리셔
class GripperVirtualPublisher(Node):

    JOINT_NAME = "rg2_finger_joint"
    PUBLISH_RATE = 0.05        # 20 Hz
    SPEED = 1.0                # rad/s — adjust for faster/slower animation

    def __init__(self):
        super().__init__("gripper_virtual_publisher", namespace=ROBOT_ID)
        self._pub = self.create_publisher(JointState, "/gripper_joint_states", 10)
        self._position = 0.0
        self._target   = 0.0
        self._timer = self.create_timer(self.PUBLISH_RATE, self._publish)

    def set_grip(self):
        self._target = GRIPPER_CLOSED

    def set_release(self):
        self._target = GRIPPER_OPEN

    def _publish(self):
        # Step toward target by SPEED * dt each tick
        dt = self.PUBLISH_RATE
        diff = self._target - self._position
        step = self.SPEED * dt
        if abs(diff) <= step:
            self._position = self._target
        else:
            self._position += step * (1.0 if diff > 0 else -1.0)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.JOINT_NAME]
        msg.position = [self._position]
        self._pub.publish(msg)


def initialize_robot():
    """Set robot mode to autonomous. Skip Tool/TCP (not registered in virtual DRCF)."""
    from DSR_ROBOT2 import ROBOT_MODE_AUTONOMOUS, set_robot_mode, get_robot_mode

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

    print("#" * 50)
    print("Virtual mode — Tool/TCP preset skipped (not available in emulator)")
    print(f"ROBOT_ID:    {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_MODE (0:manual, 1:auto): {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}, ACC: {ACC}")
    print("#" * 50)


def perform_task(gripper_pub: GripperVirtualPublisher):
    """Grip/release loop with RViz-visible gripper state."""
    from DSR_ROBOT2 import set_digital_output, movej, wait

    def wait_digital_input_virtual(timeout=1.0):
        wait(timeout)

    def grip():
        print("Gripping...")
        # set_digital_output(1, ON) # DRCF 환경에서 Onrobot이 동작하지 않기 때문에 의미 없음
        # set_digital_output(2, OFF)
        gripper_pub.set_grip()
        wait_digital_input_virtual()

    def release():
        print("Releasing...")
        # set_digital_output(2, ON) # DRCF 환경에서 Onrobot이 동작하지 않기 때문에 의미 없음
        # set_digital_output(1, OFF)
        gripper_pub.set_release()
        wait_digital_input_virtual()

    JReady = [0, 0, 90, 0, 90, 0]
    print("Moving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    while rclpy.ok():
        grip()
        wait(0.5)
        release()
        wait(0.5)


def main(args=None):
    rclpy.init(args=args)

    # DSR motion node (for DR_init / DSR_ROBOT2 service calls)
    motion_node = rclpy.create_node("grip_test_virtual", namespace=ROBOT_ID)
    DR_init.__dsr__node = motion_node

    # Gripper publisher node (separate so spin thread can drive the timer)
    gripper_pub = GripperVirtualPublisher()

    # Dedicated executor for gripper_pub so it doesn't conflict with
    # DSR_ROBOT2's spin_until_future_complete (which uses the global executor)
    gripper_executor = SingleThreadedExecutor()
    gripper_executor.add_node(gripper_pub)
    spin_thread = threading.Thread(
        target=gripper_executor.spin, daemon=True
    )
    spin_thread.start()

    try:
        initialize_robot()
        perform_task(gripper_pub)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        gripper_pub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
