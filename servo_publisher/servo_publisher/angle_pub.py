#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NUM_SERVOS = 12

class ServoTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('servo_trajectory_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/servo_angle', 10)
        self.positions = [90.0] * NUM_SERVOS
        self.get_logger().info(f'輸入 q 離開；可控制 1–{NUM_SERVOS} 號舵機')

    def publish_joint_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [f'servo_{i}' for i in range(1, NUM_SERVOS+1)]
        pt = JointTrajectoryPoint()
        pt.positions = self.positions.copy()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 0
        msg.points = [pt]
        self.pub.publish(msg)
        print(msg)
        self.get_logger().info(f'Published → {self.positions}')

    def run_menu(self):
        try:
            while rclpy.ok():
                choice = input(f'\n選擇要控制哪顆舵機 (1–{NUM_SERVOS}, q = 離開)：').strip()
                if choice.lower() == 'q':
                    break
                if not choice.isdigit() or not (1 <= int(choice) <= NUM_SERVOS):
                    print(f'輸入錯誤，請輸入 1 到 {NUM_SERVOS} 或 q。')
                    continue
                idx = int(choice) - 1
                self.angle_menu(idx)
        except (KeyboardInterrupt, EOFError):
            pass

    def angle_menu(self, idx: int):
        prompt = f'Servo {idx+1} 角度 (0–240) 或 b 返回：'
        while rclpy.ok():
            inp = input(prompt).strip()
            if inp.lower() == 'b':
                return
            try:
                ang = float(inp)
                if not 0.0 <= ang <= 240.0:
                    raise ValueError
            except ValueError:
                print('角度範圍錯誤，請輸入 0 到 240 之間，或 b 返回。')
                continue

            self.positions[idx] = ang
            self.publish_joint_trajectory()

def main(args=None):
    rclpy.init(args=args)
    node = ServoTrajectoryPublisher()
    node.run_menu()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
