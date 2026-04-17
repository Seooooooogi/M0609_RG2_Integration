import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
import DR_init
from onrobot_rg_msgs.srv import SetCommand

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"
VELOCITY    = 60
ACC         = 60
ON, OFF     = 1, 0

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot(mode: str):
    from DSR_ROBOT2 import ROBOT_MODE_AUTONOMOUS, set_robot_mode, get_robot_mode

    if mode == 'real':
        from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(2)
        print("#" * 50)
        print(f"ROBOT_TCP:  {get_tcp()}")
        print(f"ROBOT_TOOL: {get_tool()}")
    else:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1)
        print("#" * 50)
        print("Virtual mode — Tool/TCP preset skipped (not registered in emulator)")

    print(f"ROBOT_ID:    {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_MODE (0:manual, 1:auto): {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}, ACC: {ACC}")
    print("#" * 50)


def make_gripper_caller():
    """
    /onrobot/sendCommand 서비스 클라이언트를 전용 executor에서 실행.
    DSR_ROBOT2 내부 executor와 충돌 방지를 위해 SingleThreadedExecutor 분리.
    """
    client_node = rclpy.create_node("gripper_client", namespace=ROBOT_ID)
    cli = client_node.create_client(SetCommand, '/onrobot/sendCommand')

    executor = SingleThreadedExecutor()
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def call(command: str) -> bool:
        req = SetCommand.Request()
        req.command = command
        future = cli.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=10.0)
        return future.done() and future.result().success

    return call


def perform_task(mode: str, gripper_call):
    from DSR_ROBOT2 import movej, wait

    if mode == 'real':
        from DSR_ROBOT2 import set_digital_output, get_digital_input

        def wait_digital_input(sig_num):
            while not get_digital_input(sig_num):
                wait(0.5)

        def grip():
            print("Gripping...")
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
            wait_digital_input(1)

        def release():
            print("Releasing...")
            set_digital_output(2, ON)
            set_digital_output(1, OFF)
            wait_digital_input(2)

    else:
        # virtual: /onrobot/sendCommand 서비스 — 애니메이션 완료까지 blocking
        def grip():
            print("Gripping...")
            gripper_call('c')

        def release():
            print("Releasing...")
            gripper_call('o')

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

    motion_node = rclpy.create_node("grip_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = motion_node

    motion_node.declare_parameter("mode", "virtual")
    mode = motion_node.get_parameter("mode").get_parameter_value().string_value

    gripper_call = make_gripper_caller() if mode == 'virtual' else None

    try:
        initialize_robot(mode)
        perform_task(mode, gripper_call)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
