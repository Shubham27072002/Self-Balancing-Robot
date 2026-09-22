#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import EntityWrench, Entity

class BalanceTest(Node):
    def __init__(self):
        super().__init__('balance_test')
        self.pitch = 0.0
        self.torques = [0.25, -0.25, 0.25, -0.25]
        self.test_index = 0
        self.torque_active = False
        self.torque_start_time = 0.0
        self.stable_start_time = None
        self.finished = False
        self.waiting_for_recovery = False
        self.create_subscription(Imu,'/imu/data',self.imu_callback,10)
        self.wrench_pub = self.create_publisher(EntityWrench,'/wrench_persistent',10)
        self.clear_pub = self.create_publisher(Entity,'/wrench_clear',10)
        self.create_timer(0.01,self.test_loop)
        self.get_logger().info('Balance test started')

    def imu_callback(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.asin(sinp)

    def apply_torque(self, torque):
        msg = EntityWrench()
        msg.entity.name = 'balancing_robot'
        msg.entity.type = Entity.MODEL
        msg.wrench.torque.y = torque
        self.wrench_pub.publish(msg)
        self.get_logger().info(f'Applied torque: {torque:.3f} N.m')

    def clear_torque(self):
        msg = Entity()
        msg.name = 'balancing_robot'
        msg.type = Entity.MODEL
        for _ in range(3):
            self.clear_pub.publish(msg)
        self.get_logger().info('Torque cleared. Waiting for robot to stabilize.')

    def test_loop(self):
        now = (self.get_clock().now().nanoseconds* 1e-9)
        if self.torque_active:
            if (now - self.torque_start_time >= 0.25):
                self.clear_torque()
                self.torque_active = False
                self.waiting_for_recovery = True
                self.stable_start_time = None
            return

        if self.test_index >= len(self.torques):
            self.get_logger().info('Balance test completed')
            self.finished = True
            return

        if abs(self.pitch) < 0.05:
            if self.stable_start_time is None:
                self.stable_start_time = now
            elif (now - self.stable_start_time >= 3.0):
                if self.waiting_for_recovery:
                    self.waiting_for_recovery = False
                if self.test_index >= len(self.torques):
                    self.get_logger().info('Balance test completed')
                    self.finished = True
                    return
                torque = self.torques[self.test_index]
                self.apply_torque(torque)
                self.test_index += 1
                self.torque_active = True
                self.torque_start_time = now
                self.stable_start_time = None
        else:
            self.stable_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = BalanceTest()
    try:
        while (rclpy.ok() and not node.finished):
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clear_torque()
        rclpy.spin_once(node,timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()