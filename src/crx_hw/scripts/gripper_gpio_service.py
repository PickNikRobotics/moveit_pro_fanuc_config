#!/usr/bin/env python3
import time
import threading
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool  # True=open, False=close

from fanuc_msgs.srv import (
    SetBoolIO,
    SetAnalogIO,
    SetGroupIO,
    SetNumReg,
    SetGenOverride,
    GetBoolIO,
    GetAnalogIO,
    GetGroupIO,
    GetNumReg,
    SetPayloadID,
)
from fanuc_msgs.msg import (
    IOType,
    IOCmd,
    IOState,
    NumRegCmd,
    NumRegState,
    NumReg,
    BoolIO,
)


OPEN_TIMEOUT_SEC = 5.0
CLOSE_TIMEOUT_SEC = 5.0
AFTER_MOVE_SETTLE_SEC = 1.0
PUBLISH_PERIOD_SEC = 0.01


class GPIOGripperNode(Node):
    def __init__(self):
        super().__init__("gpio_gripper_node")

        # --- Shared state (guarded) -------------------------------------------------------
        self._io = defaultdict(dict)   # maps: io_type(int) -> {index(int): value(bool)}
        self._regs = defaultdict(float)  # maps: index(int) -> value(float)
        self._io_lock = threading.Lock()
        self._io_updated = threading.Event()  # notify waiters that IO changed

        # Precompute constants so we don't construct IOType repeatedly
        self._F = IOType().F  # Fanuc "Flag" type (matches your usage)

        # Pre-build a reusable IOCmd with 4 BoolIO lines (indices 1..4)
        self._bool_cmd = IOCmd()
        self._bool_cmd.values = [
            BoolIO(io_type=IOType(type=self._F), index=i + 1, value=False) for i in range(4)
        ]

        # Callback group so service + subs can run concurrently
        self._cb_group = ReentrantCallbackGroup()

        # --- Pubs/Subs --------------------------------------------------------------------
        self.num_reg_pub = self.create_publisher(NumRegCmd, "/fanuc_gpio_controller/num_reg_cmd", 10)
        self.bool_io_pub = self.create_publisher(IOCmd, "/fanuc_gpio_controller/io_cmd", 10)

        self.io_state_sub = self.create_subscription(
            IOState,
            "/fanuc_gpio_controller/io_state",
            self.io_state_callback,
            10,
            callback_group=self._cb_group,
        )
        self.num_reg_state_sub = self.create_subscription(
            NumRegState,
            "/fanuc_gpio_controller/num_reg_state",
            self.num_reg_state_callback,
            10,
            callback_group=self._cb_group,
        )

        # --- Service: std_srvs/SetBool(data=True->open, False->close) ---------------------
        self.gripper_srv = self.create_service(
            SetBool, "/actuate_fanuc_gripper", self.handle_gripper_set, callback_group=self._cb_group
        )

        self.get_logger().info("GPIOGripperNode ready. Service: /gripper/set (SetBool)")

    # -------------------------- Subscriptions ---------------------------------------------

    def io_state_callback(self, msg: IOState):
        with self._io_lock:
            for io_val in msg.values:
                self._io[io_val.io_type.type][io_val.index] = bool(io_val.value)
        self._io_updated.set()

    def num_reg_state_callback(self, msg: NumRegState):
        with self._io_lock:
            for reg in msg.values:
                self._regs[reg.index] = float(reg.value)

    # -------------------------- Initialization --------------------------------------------

    def initialize_gpio(self):
        """Set initial num-reg values as in your original code."""
        num_reg_msg = NumRegCmd()
        num_reg_msg.values = [NumReg() for _ in range(4)]
        num_reg_msg.values[0].index = 1
        num_reg_msg.values[0].value = 255.0
        num_reg_msg.values[1].index = 2
        num_reg_msg.values[1].value = 255.0
        num_reg_msg.values[2].index = 3
        num_reg_msg.values[2].value = 0.0
        num_reg_msg.values[3].index = 4
        num_reg_msg.values[3].value = 255.0
        self.num_reg_pub.publish(num_reg_msg)
        self.get_logger().info("Initialized GPIO num registers.")

    def activate(self):
        """Drive output 1 high until input F[1] is true, then release."""
        # NOTE: Your original code read self.io["F"][1], but the key is an int IO type.
        self.get_logger().info("Attempting to set bool register 1 to true...")
        while not self._get_io(self._F, 1, default=False) and rclpy.ok():
            self._set_bool_line(1, False)
            time.sleep(PUBLISH_PERIOD_SEC)
        self._set_bool_line(1, False)
        self.get_logger().info("Activation successful: able to set bool register.")

    # -------------------------- Service Handler -------------------------------------------

    def handle_gripper_set(self, req: SetBool.Request, res: SetBool.Response):
        """
        req.data == True  -> OPEN
        req.data == False -> CLOSE
        Returns success True/False based on timeout.
        """
        try:
            if req.data:
                ok = self.open()
                res.message = "Opened" if ok else "Open timed out"
            else:
                ok = self.close()
                res.message = "Closed" if ok else "Close timed out"
            res.success = bool(ok)
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            res.success = False
            res.message = f"Exception: {e}"
        return res

    # -------------------------- Gripper Ops -----------------------------------------------

    def open(self) -> bool:
        """
        Pulse output at index 3 until input F[3] becomes True (opened), then release.
        Uses timeout and returns True/False.
        """
        t0 = time.time()
        # Drive "open" line (index 3) high while not open
        while not self._get_io(self._F, 3, default=False) and rclpy.ok():
            self._set_bool_line(3, True)
            if self._wait_for(lambda: self._get_io(self._F, 3, default=False), timeout=PUBLISH_PERIOD_SEC):
                break  # opened
            if time.time() - t0 > OPEN_TIMEOUT_SEC:
                self._set_bool_line(3, False)
                self.get_logger().warn("Open timed out.")
                return False

        # Deassert the command and wait for coil to release / edges to settle
        self._set_bool_line(3, False)
        time.sleep(AFTER_MOVE_SETTLE_SEC)

        # Ensure the command is truly low and the state remains open
        while self._get_io(self._F, 3, default=False) and rclpy.ok():
            # Keep publishing the low state just in case the controller expects it
            self.bool_io_pub.publish(self._bool_cmd)
            if self._wait_for(lambda: not self._get_io(self._F, 3, default=False),
                              timeout=PUBLISH_PERIOD_SEC):
                break

        self.get_logger().info(f"Open completed in {time.time() - t0:.3f}s")
        return True

    def close(self) -> bool:
        """
        Pulse output at index 2 until input F[2] becomes True (closed), then release.
        Uses timeout and returns True/False.
        """
        t0 = time.time()
        # Drive "close" line (index 2) high while not closed
        while not self._get_io(self._F, 2, default=False) and rclpy.ok():
            self._set_bool_line(2, True)
            if self._wait_for(lambda: self._get_io(self._F, 2, default=False), timeout=PUBLISH_PERIOD_SEC):
                break  # closed
            if time.time() - t0 > CLOSE_TIMEOUT_SEC:
                self._set_bool_line(2, False)
                self.get_logger().warn("Close timed out.")
                return False

        # Deassert the command and wait for settle
        self._set_bool_line(2, False)
        time.sleep(AFTER_MOVE_SETTLE_SEC)

        # Ensure command low and input stable
        while self._get_io(self._F, 2, default=False) and rclpy.ok():
            self.bool_io_pub.publish(self._bool_cmd)
            if self._wait_for(lambda: not self._get_io(self._F, 2, default=False),
                              timeout=PUBLISH_PERIOD_SEC):
                break

        self.get_logger().info(f"Close completed in {time.time() - t0:.3f}s")
        return True

    # -------------------------- Helpers ----------------------------------------------------

    def _set_bool_line(self, index: int, value: bool):
        """Set the Nth BoolIO line value and publish."""
        # indices are 1-based in your original code (1..4)
        assert 1 <= index <= 4
        self._bool_cmd.values[index - 1].value = bool(value)
        self.bool_io_pub.publish(self._bool_cmd)

    def _get_io(self, io_type_int: int, index: int, default=False) -> bool:
        with self._io_lock:
            return bool(self._io.get(io_type_int, {}).get(index, default))

    def _wait_for(self, predicate, timeout: float) -> bool:
        """
        Wait up to 'timeout' seconds for predicate() to become True.
        Uses an Event that is signaled by io_state_callback to avoid busy-wait.
        """
        if predicate():
            return True
        self._io_updated.clear()
        # Wait (interruptible) – if event fires, re-check predicate; if not, time elapsed
        self._io_updated.wait(timeout=timeout)
        return predicate()


def main():
    rclpy.init()
    node = GPIOGripperNode()
    node.initialize_gpio()

    # Multi-threaded executor + reentrant callbacks allow
    # subscriber callbacks to run while the service is blocking/waiting.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

