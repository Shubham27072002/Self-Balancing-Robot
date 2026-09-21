#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # DEFINING PARAMETERS
        # PITCH CONTOLLER PARAMETERS 
        self.declare_parameter('pitch_kp', 0.0)
        self.declare_parameter('pitch_ki', 0.0)
        self.declare_parameter('pitch_kd', 0.0)
        self.declare_parameter('pitch_integral_limit', 1.0)

        # VELOCITY CONTROLLER PARAMETERS 
        self.declare_parameter('velocity_kp', 0.0)
        self.declare_parameter('velocity_ki', 0.0)
        self.declare_parameter('velocity_kd', 0.0)
        self.declare_parameter('velocity_integral_limit', 0.15)

        # POSITION CONTROLLER PARAMETER 
        self.declare_parameter('position_kp', 0.0)
        self.declare_parameter('position_ki', 0.0)
        self.declare_parameter('position_kd', 0.0)
        self.declare_parameter('position_integral_limit', 0.3)
        self.declare_parameter('max_hold_velocity', 0.10)

        # YAW CONTROLLER PARAMETERS
        self.declare_parameter('max_yaw_wheel_velocity', 8.0)
        self.declare_parameter('max_angular_acceleration', 1.0)
        self.declare_parameter('max_yaw_rate', 0.8)

        # TELEOP PARAMETERS
        self.declare_parameter('max_linear_acceleration', 0.6)
        self.declare_parameter('stop_velocity_threshold', 0.05)

        # SAFETY LIMITS
        self.declare_parameter('max_velocity', 30.0)
        self.declare_parameter('fall_angle', 1.2)
        self.declare_parameter('max_pitch_offset', 0.10)         

        # LOAD PARAMETERS 
        self.pitch_kp = self.get_parameter('pitch_kp').value
        self.pitch_ki = self.get_parameter('pitch_ki').value
        self.pitch_kd = self.get_parameter('pitch_kd').value
        self.pitch_integral_limit = self.get_parameter('pitch_integral_limit').value
        self.velocity_kp = self.get_parameter('velocity_kp').value
        self.velocity_ki = self.get_parameter('velocity_ki').value
        self.velocity_kd = self.get_parameter('velocity_kd').value
        self.velocity_integral_limit = self.get_parameter('velocity_integral_limit').value
        self.position_kp = self.get_parameter('position_kp').value
        self.position_ki = self.get_parameter('position_ki').value
        self.position_kd = self.get_parameter('position_kd').value
        self.position_integral_limit = self.get_parameter('position_integral_limit').value
        self.max_hold_velocity = self.get_parameter('max_hold_velocity').value
        self.max_yaw_wheel_velocity = self.get_parameter('max_yaw_wheel_velocity').value
        self.max_angular_acceleration = self.get_parameter('max_angular_acceleration').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_linear_acceleration = self.get_parameter('max_linear_acceleration').value
        self.stop_velocity_threshold = self.get_parameter('stop_velocity_threshold').value

        # ROBOT PARAMETER 
        self.wheel_radius = 0.08
        self.wheel_separation = 0.225        

        self.max_velocity = self.get_parameter('max_velocity').value
        self.fall_angle = self.get_parameter('fall_angle').value
        self.max_pitch_offset = self.get_parameter('max_pitch_offset').value       

        # CONTROLLER STATES
        self.pitch = 0.0
        self.target_pitch = 0.0
        self.previous_pitch_error = 0.0
        self.pitch_integral_error = 0.0

        self.robot_velocity = 0.0
        self.filtered_robot_velocity = 0.0
        self.previous_filtered_velocity = 0.0
        self.commanded_velocity = 0.0
        self.target_velocity = 0.0
        self.velocity_integral_error = 0.0

        self.robot_position = 0.0
        self.hold_position = 0.0
        self.position_integral_error = 0.0
        self.position_initialized = False

        self.commanded_yaw_rate = 0.0
        self.target_yaw_rate = 0.0

        self.teleop_linear_active = False
        self.hold_pending = False

        # FLAGS 
        self.imu_received = False
        self.joint_state_received = False 

        # SUBSCRIBERS      
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # PUBLISHER 
        self.cmd_vel_pub = self.create_publisher(Float64MultiArray,'/wheel_velocity_controller/commands',10) 

        # CONTROL LOOP INITIALIZATION
        self.pitch_last_time = self.get_clock().now()
        self.vel_last_time = self.get_clock().now()
        self.create_timer(0.01, self.pitch_control_loop)
        self.create_timer(0.10, self.velocity_control_loop)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.asin(sinp)
        self.imu_received = True   

    def joint_state_callback(self, msg: JointState):
        left_index = msg.name.index('base_left_wheel_joint')
        right_index = msg.name.index('base_right_wheel_joint')
        left_wheel_position = msg.position[left_index]
        right_wheel_position = msg.position[right_index]
        left_wheel_velocity = msg.velocity[left_index]
        right_wheel_velocity = msg.velocity[right_index]
        self.robot_velocity = (self.wheel_radius * 
                               (left_wheel_velocity + right_wheel_velocity)/ 2.0)
        self.joint_state_received = True
        if not self.imu_received:
            return

        if not self.position_initialized:
            self.initial_left_wheel_position = left_wheel_position
            self.initial_right_wheel_position = right_wheel_position
            self.initial_pitch = self.pitch
            self.robot_position = 0.0
            self.hold_position = 0.0
            self.position_initialized = True
            return

        delta_left = (left_wheel_position - self.initial_left_wheel_position)
        delta_right = (right_wheel_position - self.initial_right_wheel_position)
        delta_pitch = (self.pitch - self.initial_pitch)
        average_wheel_angle = (delta_left + delta_right) / 2.0
        self.robot_position = (self.wheel_radius * (average_wheel_angle + delta_pitch))

    def cmd_vel_callback(self, msg: Twist):
        linear_command = msg.linear.x
        self.commanded_yaw_rate = msg.angular.z
        if linear_command != 0.0:
            if not self.teleop_linear_active:
                self.position_integral_error = 0.0
                self.velocity_integral_error = 0.0
            self.teleop_linear_active = True
            self.hold_pending = False
            self.commanded_velocity = linear_command
        else:
            self.commanded_velocity = 0.0
            if self.teleop_linear_active:
                self.hold_pending = True

    def velocity_control_loop(self):
        if (not self.joint_state_received or not self.position_initialized):
            return

        now = self.get_clock().now()
        dt = (now - self.vel_last_time).nanoseconds * 1e-9
        self.vel_last_time = now

        if dt <= 0.0:
            return

        if abs(self.pitch) > self.fall_angle:
            return

        self.filtered_robot_velocity = (0.7 * self.filtered_robot_velocity
                                        + 0.3 * self.robot_velocity)
        desired_yaw_rate = max(-self.max_yaw_rate,
                               min(self.max_yaw_rate,self.commanded_yaw_rate))
        max_yaw_change = (self.max_angular_acceleration * dt)
        yaw_difference = (desired_yaw_rate - self.target_yaw_rate)
        yaw_difference = max(-max_yaw_change,min(max_yaw_change,yaw_difference))
        self.target_yaw_rate += yaw_difference

        if self.teleop_linear_active:
            max_velocity_change = (self.max_linear_acceleration * dt)
            velocity_difference = (self.commanded_velocity - self.target_velocity)
            velocity_difference = max(-max_velocity_change,
                                      min(max_velocity_change, velocity_difference))
            self.target_velocity += velocity_difference

            if (self.hold_pending
                and self.target_velocity == 0.0
                and abs(self.filtered_robot_velocity) <= self.stop_velocity_threshold):
                self.target_velocity = 0.0
                self.commanded_velocity = 0.0
                self.hold_position = self.robot_position
                self.position_integral_error = 0.0
                self.velocity_integral_error = 0.0
                self.teleop_linear_active = False
                self.hold_pending = False

        else:
            position_error = (self.hold_position - self.robot_position)
            self.position_integral_error += (position_error * dt)
            self.position_integral_error = max(-self.position_integral_limit, 
                                               min(self.position_integral_limit,self.position_integral_error))
            position_derivative = (-self.filtered_robot_velocity)
            position_output = (self.position_kp * position_error
                               + self.position_ki * self.position_integral_error
                               + self.position_kd * position_derivative)
            self.target_velocity = max(-self.max_hold_velocity,
                                       min(self.max_hold_velocity,position_output))

        velocity_error = (self.target_velocity - self.filtered_robot_velocity)
        velocity_derivative = (self.filtered_robot_velocity - self.previous_filtered_velocity) / dt
        self.previous_filtered_velocity = self.filtered_robot_velocity
        self.velocity_integral_error += (velocity_error * dt)
        self.velocity_integral_error = max(-self.velocity_integral_limit,
                                           min(self.velocity_integral_limit,self.velocity_integral_error))
        velocity_p = (self.velocity_kp * velocity_error)
        velocity_i = (self.velocity_ki * self.velocity_integral_error)
        velocity_d = (-self.velocity_kd * velocity_derivative)
        velocity_output = (velocity_p + velocity_i + velocity_d)
        pitch_offset = max(-self.max_pitch_offset, 
                           min(self.max_pitch_offset,velocity_output))
        self.target_pitch = pitch_offset

    def pitch_control_loop(self):
        if (not self.imu_received or not self.joint_state_received):
            return

        now = self.get_clock().now()
        dt = (now - self.pitch_last_time).nanoseconds * 1e-9
        self.pitch_last_time = now

        if dt <= 0.0:
            return

        if abs(self.pitch) > self.fall_angle:
            self.pitch_integral_error = 0.0
            self.velocity_integral_error = 0.0
            self.position_integral_error = 0.0
            self.commanded_velocity = 0.0
            self.target_velocity = 0.0
            self.target_pitch = 0.0
            self.commanded_yaw_rate = 0.0
            self.target_yaw_rate = 0.0
            self.previous_pitch_error = 0.0
            self.previous_filtered_velocity = self.filtered_robot_velocity
            self.teleop_linear_active = False
            self.hold_pending = False
            self.cmd_vel_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
            return

        pitch_error = (self.pitch - self.target_pitch)
        self.pitch_integral_error += (pitch_error * dt)
        self.pitch_integral_error = max(-self.pitch_integral_limit,
                                        min(self.pitch_integral_limit,self.pitch_integral_error))
        pitch_derivative = (pitch_error - self.previous_pitch_error) / dt
        self.previous_pitch_error = pitch_error
        pitch_output = (self.pitch_kp * pitch_error + 
                        self.pitch_ki * self.pitch_integral_error + 
                        self.pitch_kd * pitch_derivative)

        wheel_velocity = (self.target_velocity / self.wheel_radius)
        common_wheel_velocity = (wheel_velocity + pitch_output)
        common_wheel_velocity = max(-self.max_velocity,
                                    min(self.max_velocity,common_wheel_velocity))

        yaw_wheel_velocity = (self.wheel_separation / (2.0 * self.wheel_radius) * self.target_yaw_rate)
        yaw_wheel_velocity = max(-self.max_yaw_wheel_velocity,
                                 min(self.max_yaw_wheel_velocity,yaw_wheel_velocity))
        available_yaw = max(0.0,self.max_velocity - abs(common_wheel_velocity))
        yaw_wheel_velocity = max(-available_yaw,
                                 min(available_yaw,yaw_wheel_velocity))

        left_command = (common_wheel_velocity - yaw_wheel_velocity)
        right_command = (common_wheel_velocity + yaw_wheel_velocity)
        self.cmd_vel_pub.publish(Float64MultiArray(data=[left_command, right_command]))     

def main(args=None):
    rclpy.init(args=args)
    Controller_node = None
    try:
        Controller_node = Controller()
        rclpy.spin(Controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        if Controller_node is not None:
            Controller_node.cmd_vel_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
            Controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()