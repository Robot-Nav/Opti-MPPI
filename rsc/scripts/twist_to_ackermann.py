#!/usr/bin/env python3
"""
twist_to_ackermann.py
将任意跟踪控制器输出的 cmd_vel (geometry_msgs/Twist) 统一约束为
vehicle_simulator Ackermann 模式所需的 /ackermann_cmd。当前 J15 仿真与实物流程
不启用此工具；正式流程保持 Twist 接口并由差速执行层施加三轮可行域约束。

运动学关系:
  wz = vx * tan(steering_angle) / wheelbase
  => steering_angle = atan(wz * wheelbase / vx)

参数:
  ~wheelbase       轴距 (m), 默认 0.500
  ~max_steer       最大转向角 (rad), 默认 atan(0.5/0.5)
  ~max_speed       最大前进速度 (m/s), 默认 1.0
  ~max_acceleration 最大纵向加速度 (m/s^2), 默认 1.2
  ~max_deceleration 最大纵向减速度 (m/s^2), 默认 1.2
  ~max_steering_rate 最大转角变化率 (rad/s), 默认 1.2
  ~max_lateral_acceleration 最大横向加速度 (m/s^2), 默认 0.6
  ~input_topic     输入话题, 默认 "/cmd_vel"
  ~output_topic    输出话题, 默认 "/ackermann_cmd"
"""

import math
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class TwistToAckermann:
    def __init__(self):
        self.wheelbase = rospy.get_param('~wheelbase', 0.500)
        self.max_steer = rospy.get_param('~max_steer', math.atan(0.5 / 0.5))
        self.max_speed = rospy.get_param('~max_speed', 1.0)
        self.max_acceleration = rospy.get_param('~max_acceleration', 1.2)
        self.max_deceleration = rospy.get_param('~max_deceleration', 1.2)
        self.max_steering_rate = rospy.get_param('~max_steering_rate', 1.2)
        self.max_lateral_acceleration = rospy.get_param(
            '~max_lateral_acceleration', 0.6)
        input_topic = rospy.get_param('~input_topic', '/cmd_vel')
        output_topic = rospy.get_param('~output_topic', '/ackermann_cmd')
        executed_topic = rospy.get_param(
            '~executed_cmd_topic', '/constrained_cmd_vel')

        self.pub = rospy.Publisher(output_topic, AckermannDriveStamped, queue_size=1)
        self.executed_pub = rospy.Publisher(executed_topic, Twist, queue_size=1)
        self.sub = rospy.Subscriber(input_topic, Twist, self.callback, queue_size=1)
        self.last_speed = 0.0
        self.last_steer = 0.0
        self.last_stamp = None

        rospy.loginfo(
            "Ackermann command governor: %s -> %s "
            "(wheelbase=%.2f, steer<=%.2f, Rmin=%.3f)",
            input_topic, output_topic, self.wheelbase, self.max_steer,
            self.wheelbase / math.tan(self.max_steer))

    def callback(self, twist):
        msg = AckermannDriveStamped()
        now = rospy.Time.now()
        msg.header.stamp = now
        msg.header.frame_id = 'base_link'

        if self.last_stamp is None:
            dt = 0.05
        else:
            dt = max(1e-3, min(0.5, (now - self.last_stamp).to_sec()))

        requested_vx = max(0.0, min(self.max_speed, twist.linear.x))
        wz = twist.angular.z

        delta_v = requested_vx - self.last_speed
        if delta_v >= 0.0:
            delta_v = min(delta_v, self.max_acceleration * dt)
        else:
            delta_v = max(delta_v, -self.max_deceleration * dt)
        vx = self.last_speed + delta_v

        msg.drive.speed = vx

        if abs(vx) < 1e-6:
            requested_steer = 0.0
        else:
            requested_steer = math.atan(wz * self.wheelbase / vx)
            # 同时满足机械转角和 v^2*kappa 横向加速度约束。
            lateral_steer = math.atan(
                self.max_lateral_acceleration * self.wheelbase /
                max(vx * vx, 1e-6))
            steer_limit = min(self.max_steer, lateral_steer)
            requested_steer = max(
                -steer_limit, min(steer_limit, requested_steer))

        max_steer_delta = self.max_steering_rate * dt
        steer_delta = max(
            -max_steer_delta,
            min(max_steer_delta, requested_steer - self.last_steer))
        steer = self.last_steer + steer_delta
        msg.drive.steering_angle = steer

        self.pub.publish(msg)

        executed = Twist()
        executed.linear.x = vx
        if abs(vx) >= 1e-6:
            executed.angular.z = vx * math.tan(steer) / self.wheelbase
        self.executed_pub.publish(executed)

        self.last_speed = vx
        self.last_steer = steer
        self.last_stamp = now


if __name__ == '__main__':
    rospy.init_node('twist_to_ackermann')
    node = TwistToAckermann()
    rospy.spin()
