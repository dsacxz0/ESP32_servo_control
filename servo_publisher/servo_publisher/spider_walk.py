#!/usr/bin/env python3
import sys, time, select, termios, tty, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NUM_SERVOS     = 12
TOPIC_NAME     = '/servo_angle'
THROTTLE_SEC   = 0.5
TIMER_PERIOD   = 0.02
HIP_STEP = 20
KNEE_RAISE = 30

INITIALPOSE = [105.0, 60.0, 110.0, 70.0, 85.0, 125.0, 115.0, 60.0, 140.0, 60.0, 125.0, 55.0]

FORWARDPOSE = [
  [INITIALPOSE[0] - HIP_STEP, INITIALPOSE[1] + KNEE_RAISE, INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] - HIP_STEP, INITIALPOSE[5] + KNEE_RAISE, INITIALPOSE[6] - HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9] + KNEE_RAISE, INITIALPOSE[10] - HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1] + KNEE_RAISE, INITIALPOSE[2] - HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5] + KNEE_RAISE, INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] - HIP_STEP, INITIALPOSE[9] + KNEE_RAISE, INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] - HIP_STEP, INITIALPOSE[3] + KNEE_RAISE, INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7] + KNEE_RAISE, INITIALPOSE[8] - HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11] + KNEE_RAISE],
  [INITIALPOSE[0] - HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3] + KNEE_RAISE, INITIALPOSE[4] - HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] - HIP_STEP, INITIALPOSE[7] + KNEE_RAISE, INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] - HIP_STEP, INITIALPOSE[11] + KNEE_RAISE]
]
BACKWARDPOSE = [
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1] + KNEE_RAISE, INITIALPOSE[2] - HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5] + KNEE_RAISE, INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] - HIP_STEP, INITIALPOSE[9] + KNEE_RAISE, INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0] - HIP_STEP, INITIALPOSE[1] + KNEE_RAISE, INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] - HIP_STEP, INITIALPOSE[5] + KNEE_RAISE, INITIALPOSE[6] - HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9] + KNEE_RAISE, INITIALPOSE[10] - HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0] - HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3] + KNEE_RAISE, INITIALPOSE[4] - HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] - HIP_STEP, INITIALPOSE[7] + KNEE_RAISE, INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] - HIP_STEP, INITIALPOSE[11] + KNEE_RAISE],
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] - HIP_STEP, INITIALPOSE[3] + KNEE_RAISE, INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7] + KNEE_RAISE, INITIALPOSE[8] - HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11] + KNEE_RAISE]
]

CLOCKWISEPOSE = [
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1] + KNEE_RAISE, INITIALPOSE[2] - HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5] + KNEE_RAISE, INITIALPOSE[6] - HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9] + KNEE_RAISE, INITIALPOSE[10] - HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1], INITIALPOSE[2], INITIALPOSE[3], INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5], INITIALPOSE[6], INITIALPOSE[7], INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9], INITIALPOSE[10]- HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0]- HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3] + KNEE_RAISE, INITIALPOSE[4]- HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7] + KNEE_RAISE, INITIALPOSE[8]- HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11] + KNEE_RAISE],
  [INITIALPOSE[0]- HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3], INITIALPOSE[4]- HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] - HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11]]
]

COUNTERCLOCKWISEPOSE = [
  [INITIALPOSE[0] - HIP_STEP, INITIALPOSE[1] + KNEE_RAISE, INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] - HIP_STEP, INITIALPOSE[5] + KNEE_RAISE, INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] - HIP_STEP, INITIALPOSE[9] + KNEE_RAISE, INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0] - HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] + HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] - HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] + HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] - HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] + HIP_STEP, INITIALPOSE[11]],
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] - HIP_STEP, INITIALPOSE[3] + KNEE_RAISE, INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] - HIP_STEP, INITIALPOSE[7] + KNEE_RAISE, INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] - HIP_STEP, INITIALPOSE[11] + KNEE_RAISE],
  [INITIALPOSE[0] + HIP_STEP, INITIALPOSE[1], INITIALPOSE[2] - HIP_STEP, INITIALPOSE[3], INITIALPOSE[4] + HIP_STEP, INITIALPOSE[5], INITIALPOSE[6] - HIP_STEP, INITIALPOSE[7], INITIALPOSE[8] + HIP_STEP, INITIALPOSE[9], INITIALPOSE[10] - HIP_STEP, INITIALPOSE[11]]
]

class ServoSpeedNode(Node):
    def __init__(self):
        super().__init__('servo_speed_keyboard')
        self.pub = self.create_publisher(JointTrajectory, TOPIC_NAME, 10)
        self.names = [f'servo_{i}' for i in range(1, NUM_SERVOS + 1)]
        self._lock = threading.Lock()

        self.dir_state = 'stop'
        self.last_sent_dir = 'stop'
        self.next_allowed_t = 0.0
        self.pending_reset = False
        self.step = 0

        self._timer = self.create_timer(TIMER_PERIOD, self._on_timer)

    def _pose_for(self, direction: str, step: int):
        if direction == 'w':
            return FORWARDPOSE[step].copy()
        if direction == 's':
            return BACKWARDPOSE[step].copy()
        if direction == 'd':
            return CLOCKWISEPOSE[step].copy()
        if direction == 'a':
            return COUNTERCLOCKWISEPOSE[step].copy()
        return INITIALPOSE.copy()

    def _publish_positions(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 0
        msg.points = [pt]
        self.pub.publish(msg)
        print(msg)

    def _on_timer(self):
        now = time.monotonic()
        with self._lock:
            
            if self.pending_reset:
                self.step = 0
                pos = INITIALPOSE.copy()
                self._publish_positions(pos)
                self.last_sent_dir = 'stop'
                self.next_allowed_t = now + THROTTLE_SEC
                self.pending_reset = False
                return

            if self.dir_state != self.last_sent_dir:
                self.step = 0
                pos = self._pose_for(self.dir_state, self.step)
                self._publish_positions(pos)
                self.last_sent_dir = self.dir_state
                self.next_allowed_t = now + THROTTLE_SEC
                return

            if self.dir_state != 'stop' and now >= self.next_allowed_t:
                self.step = (self.step + 1) % 4
                pos = self._pose_for(self.dir_state, self.step)
                self._publish_positions(pos)
                self.next_allowed_t = now + THROTTLE_SEC
                return
            
def read_keys_forever(node: ServoSpeedNode, stop_evt: threading.Event):
    if not sys.stdin.isatty():
        node.get_logger().error("No TTY detected. Start with -it (or tty:true + stdin_open:true).")
        stop_evt.set()
        return

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        keymap = {
            'w':'w','W':'w','a':'a','A':'a','s':'s','S':'s','d':'d','D':'d',
            ' ':'space',
            'q':'quit','Q':'quit','\x1b':'quit'
        }
        while rclpy.ok() and not stop_evt.is_set():
            r,_,_ = select.select([sys.stdin], [], [], 0.01)
            if not r:
                continue
            ch = sys.stdin.read(1)
            act = keymap.get(ch)
            if act in ('w','a','s','d'):
                with node._lock:
                    node.dir_state = act
            elif act == 'space':
                with node._lock:
                    node.dir_state = 'stop'
                    node.pending_reset = True
            elif act == 'quit':
                stop_evt.set()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = ServoSpeedNode()
    exec = SingleThreadedExecutor()
    exec.add_node(node)
    stop_evt = threading.Event()
    spin_thread = threading.Thread(target=exec.spin, daemon=True)
    spin_thread.start()
    try:
        read_keys_forever(node, stop_evt)
    finally:
        rclpy.shutdown()
        spin_thread.join()
        node.destroy_node()

if __name__ == '__main__':
    main()
