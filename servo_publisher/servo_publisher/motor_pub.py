#!/usr/bin/env python3
import sys, time, select, termios, tty, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NUM_SERVOS     = 2
TOPIC_NAME     = '/motor_speed'
SERVO_A_IDX    = 0
SERVO_B_IDX    = 1
SPEED    = 1000.0      
PUBLISH_HZ     = 100.0
THROTTLE_SEC   = 0.5
TIMER_PERIOD   = 0.02

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
    
    def _calc_velocity(self, direction: str):
        va = vb = 0.0
        if direction == 'w':
            va = +SPEED; vb = -SPEED
        elif direction == 's':
            va = -SPEED; vb = +SPEED
        elif direction == 'd':
            va = +SPEED; vb = +SPEED
        elif direction == 'a':
            va = -SPEED; vb = -SPEED
        velocity = [0.0] * NUM_SERVOS
        velocity[SERVO_A_IDX] = va
        velocity[SERVO_B_IDX] = vb
        return velocity

    def _publish_speed(self, velocity):
        msg = JointTrajectory()
        msg.joint_names = self.names
        pt = JointTrajectoryPoint()
        pt.velocities = velocity
        msg.points = [pt]
        self.pub.publish(msg)
        print(msg)

    def _on_timer(self):
        now = time.monotonic()
        with self._lock:
            
            if self.pending_reset:
                vel = [0.0] * NUM_SERVOS
                self._publish_speed(vel)
                self.last_sent_dir = 'stop'
                self.next_allowed_t = now + THROTTLE_SEC
                self.pending_reset = False
                return

            if self.dir_state != self.last_sent_dir:
                vel = self._calc_velocity(self.dir_state)
                self._publish_speed(vel)
                self.last_sent_dir = self.dir_state
                self.next_allowed_t = now + THROTTLE_SEC
                return

            if self.dir_state != 'stop' and now >= self.next_allowed_t:
                vel = self._calc_velocity(self.dir_state)
                self._publish_speed(vel)
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
