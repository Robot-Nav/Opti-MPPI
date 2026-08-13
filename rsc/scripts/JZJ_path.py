#!/usr/bin/env python3
# scripts/fixed_path_publisher.py (ROS1 version)

import rospy
import math
import sys
import select
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

MINIMUM_TURNING_RADIUS = 1.0
MAXIMUM_PATH_LENGTH = 30.0
PATH_SAMPLE_SPACING = 0.05
MAXIMUM_TANGENT_MISMATCH = 0.10

# 手动实现四元数与欧拉角转换
def euler_from_quaternion(quat):
    x, y, z, w = quat
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class FixedPathPublisher:
    def __init__(self):
        # 参数声明
        self.path_type = rospy.get_param('~path_type', 'straight')
        self.path_length = rospy.get_param('~path_length', 8.0)   # 仅用于直线/矩形
        self.path_points = rospy.get_param('~path_points', 50)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)
        self.output_topic = rospy.get_param('~output_topic', '/plan')
        self.waypoints_only = rospy.get_param('~waypoints_only', False)
        self.start_x = rospy.get_param('~start_x', 0.0)
        self.start_y = rospy.get_param('~start_y', 0.0)
        self.start_yaw = rospy.get_param('~start_yaw', 0.0)

        # 发布器 (latch=True 使后连接节点也能收到最新路径)
        self.path_pub = rospy.Publisher(
            self.output_topic, Path, queue_size=1, latch=True)

        # 订阅initialpose以手动设置起点（可选）
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)

        # 定时发布
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_path)

        # 生成初始路径
        self.current_path = self.generate_path()

        rospy.loginfo('Fixed Path Publisher initialized')
        rospy.loginfo('Path type: %s', self.path_type)
        rospy.loginfo('Fixed start position: (%.2f, %.2f, %.2f)',
                      self.start_x, self.start_y, self.start_yaw)

        # 立即发布一次
        self.publish_path(None)

    def initial_pose_callback(self, msg):
        """通过initialpose话题设置新的固定起点"""
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.start_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        rospy.loginfo('Set new fixed start pose: (%.2f, %.2f, %.2f)',
                      self.start_x, self.start_y, self.start_yaw)
        self.current_path = self.generate_path()

    def generate_path(self):
        """根据类型生成路径"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id

        if self.waypoints_only:
            poses = self.generate_lattice_waypoints()
        elif self.path_type == 'straight':
            poses = self.generate_straight_path()
        elif self.path_type == 'mixed_turns':
            poses = self.generate_mixed_turns_path()
        elif self.path_type == 'hairpin':
            poses = self.generate_hairpin_path()
        elif self.path_type == 's_curve':
            poses = self.generate_benchmark_s_curve_path()
        elif self.path_type == 'wuyuce_11_5':
            poses = self.generate_benchmark_wuyuce_path()
        elif self.path_type == 'ellipse':
            poses = self.generate_ellipse_path()
        elif self.path_type == 'double_eight':
            poses = self.generate_double_eight_path()
        else:
            rospy.logwarn('Unknown path type: %s, using straight', self.path_type)
            poses = self.generate_straight_path()

        if not self.waypoints_only:
            self.validate_forward_kinematic_path(poses)
        path_msg.poses = poses
        return path_msg

    @staticmethod
    def pose_yaw(pose_stamped):
        orientation = pose_stamped.pose.orientation
        return euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])[2]

    def validate_forward_kinematic_path(self, poses):
        """Reject sampled paths that cannot be followed by the R=1m chassis."""
        if len(poses) < 2:
            raise ValueError('a tracking path must contain at least two poses')

        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in poses
        ]
        yaws = [self.pose_yaw(pose) for pose in poses]
        maximum_curvature = 0.0
        maximum_heading_mismatch = 0.0
        sampled_length = 0.0

        for index in range(len(points) - 1):
            dx = points[index + 1][0] - points[index][0]
            dy = points[index + 1][1] - points[index][1]
            distance = math.hypot(dx, dy)
            if distance <= 1e-8:
                continue
            sampled_length += distance
            segment_yaw = math.atan2(dy, dx)
            mismatch = abs(math.atan2(
                math.sin(segment_yaw - yaws[index]),
                math.cos(segment_yaw - yaws[index])))
            maximum_heading_mismatch = max(maximum_heading_mismatch, mismatch)
            if math.cos(mismatch) < -1e-6:
                raise ValueError('path contains a reverse-directed segment')

        # Three-point circumcircle curvature is independent of the sampling
        # interval and detects geometric corners that pose headings can hide.
        for index in range(1, len(points) - 1):
            ax = points[index][0] - points[index - 1][0]
            ay = points[index][1] - points[index - 1][1]
            bx = points[index + 1][0] - points[index][0]
            by = points[index + 1][1] - points[index][1]
            a = math.hypot(ax, ay)
            b = math.hypot(bx, by)
            c = math.hypot(
                points[index + 1][0] - points[index - 1][0],
                points[index + 1][1] - points[index - 1][1])
            denominator = a * b * c
            if denominator <= 1e-12:
                continue
            twice_area = abs(ax * by - ay * bx)
            curvature = 2.0 * twice_area / denominator
            maximum_curvature = max(maximum_curvature, curvature)

        minimum_radius = (
            float('inf') if maximum_curvature <= 1e-9
            else 1.0 / maximum_curvature
        )
        # A centimetre tolerance only absorbs floating point and chord
        # discretisation error; the constructed analytical radii remain 1m.
        if sampled_length > MAXIMUM_PATH_LENGTH + 1e-6:
            raise ValueError(
                'path length %.3fm exceeds %.3fm' %
                (sampled_length, MAXIMUM_PATH_LENGTH))
        if minimum_radius < MINIMUM_TURNING_RADIUS - 0.01:
            raise ValueError(
                'path violates minimum turning radius: %.3fm < %.3fm' %
                (minimum_radius, MINIMUM_TURNING_RADIUS))
        if maximum_heading_mismatch > MAXIMUM_TANGENT_MISMATCH:
            raise ValueError(
                'path heading is not tangent to its forward geometry: '
                '%.3frad > %.3frad' %
                (maximum_heading_mismatch, MAXIMUM_TANGENT_MISMATCH))
        rospy.loginfo(
            'Path kinematic check passed: length=%.3fm, sampled R_min=%s, '
            'max tangent mismatch=%.3frad, forward-only=yes',
            sampled_length,
            'inf' if not math.isfinite(minimum_radius)
            else '%.3fm' % minimum_radius,
            maximum_heading_mismatch)

    def generate_lattice_waypoints(self):
        """Sparse pose constraints planned segment-by-segment by lattice.

        The headings are selected from the 16-bin lattice and the corner
        anchors describe forward-only R=1m turns.  These are constraints for
        the planner, not an already sampled reference trajectory.
        """
        if self.path_type == 'straight':
            relative = [(self.path_length, 0.0, 0.0)]
        elif self.path_type == 'mixed_turns':
            _, relative, _ = self.sample_motion_primitives(
                self.mixed_turns_primitives())
        elif self.path_type == 'hairpin':
            _, relative, _ = self.sample_motion_primitives(
                self.hairpin_primitives())
        else:
            raise ValueError(
                'lattice benchmark supports straight, mixed_turns, hairpin')
        rospy.loginfo('Publishing %d sparse lattice constraints for %s',
                      len(relative), self.path_type)
        return self.transform_relative_path(relative)

    def generate_straight_path(self):
        """直线路径（保留原功能）"""
        poses = []
        end_x = self.start_x + self.path_length * math.cos(self.start_yaw)
        end_y = self.start_y + self.path_length * math.sin(self.start_yaw)

        for i in range(self.path_points):
            t = i / (self.path_points - 1)
            x = self.start_x * (1 - t) + end_x * t
            y = self.start_y * (1 - t) + end_y * t
            yaw = self.start_yaw
            poses.append(self.create_pose_stamped(x, y, yaw))

        rospy.loginfo('Generated straight path from (%.2f, %.2f) to (%.2f, %.2f)',
                      self.start_x, self.start_y, end_x, end_y)
        return poses

    def transform_relative_path(self, relative_poses):
        """将基准路径的相对坐标统一变换到配置的固定起点。"""
        poses = []
        cos_start = math.cos(self.start_yaw)
        sin_start = math.sin(self.start_yaw)
        for x_rel, y_rel, yaw_rel in relative_poses:
            x = self.start_x + x_rel * cos_start - y_rel * sin_start
            y = self.start_y + x_rel * sin_start + y_rel * cos_start
            poses.append(self.create_pose_stamped(x, y, yaw_rel + self.start_yaw))
        return poses

    @staticmethod
    def mixed_turns_primitives():
        """直线和不同半径左右弯组成的约19.26m复合路径。"""
        return [
            ('line', 3.0),
            ('arc', 2.0, math.pi / 3.0),
            ('line', 3.0),
            ('arc', 1.0, -2.0 * math.pi / 3.0),
            ('line', 3.5),
            ('arc', 1.5, math.pi / 3.0),
            ('line', 4.0),
        ]

    @staticmethod
    def hairpin_primitives():
        """两次90度弯和一次R=1m 180度发卡弯组成的约20.85m路径。"""
        return [
            ('line', 3.0),
            ('arc', 1.5, math.pi / 2.0),
            ('line', 3.0),
            ('arc', 1.0, -math.pi),
            ('line', 3.0),
            ('arc', 1.5, math.pi / 2.0),
            ('line', 4.0),
        ]

    def sample_motion_primitives(self, primitives):
        """按弧长采样前向直线/圆弧基元，并返回段末稀疏约束。"""
        relative_poses = [(0.0, 0.0, 0.0)]
        segment_endpoints = []
        x_rel = 0.0
        y_rel = 0.0
        yaw_rel = 0.0
        analytical_length = 0.0

        for primitive in primitives:
            if primitive[0] == 'line':
                length = float(primitive[1])
                if length <= 0.0:
                    raise ValueError('line primitive length must be positive')
                count = max(
                    1, int(math.ceil(length / PATH_SAMPLE_SPACING)))
                start_x, start_y = x_rel, y_rel
                for sample in range(1, count + 1):
                    distance = length * float(sample) / count
                    relative_poses.append((
                        start_x + distance * math.cos(yaw_rel),
                        start_y + distance * math.sin(yaw_rel),
                        yaw_rel))
                x_rel, y_rel, yaw_rel = relative_poses[-1]
                analytical_length += length
            elif primitive[0] == 'arc':
                radius = float(primitive[1])
                signed_turn = float(primitive[2])
                if radius < MINIMUM_TURNING_RADIUS:
                    raise ValueError(
                        'arc primitive radius %.3fm is below %.3fm' %
                        (radius, MINIMUM_TURNING_RADIUS))
                if abs(signed_turn) <= 1e-9:
                    raise ValueError('arc primitive turn must be non-zero')
                turn_sign = 1.0 if signed_turn > 0.0 else -1.0
                center_x = (
                    x_rel
                    - turn_sign * radius * math.sin(yaw_rel))
                center_y = (
                    y_rel
                    + turn_sign * radius * math.cos(yaw_rel))
                radial_start = math.atan2(
                    y_rel - center_y, x_rel - center_x)
                arc_length = radius * abs(signed_turn)
                count = max(
                    2, int(math.ceil(
                        arc_length / PATH_SAMPLE_SPACING)))
                start_yaw = yaw_rel
                for sample in range(1, count + 1):
                    ratio = float(sample) / count
                    radial_angle = radial_start + ratio * signed_turn
                    relative_poses.append((
                        center_x + radius * math.cos(radial_angle),
                        center_y + radius * math.sin(radial_angle),
                        start_yaw + ratio * signed_turn))
                x_rel, y_rel, yaw_rel = relative_poses[-1]
                analytical_length += arc_length
            else:
                raise ValueError(
                    'unsupported motion primitive: %s' %
                    (primitive[0],))
            segment_endpoints.append((x_rel, y_rel, yaw_rel))

        if analytical_length > MAXIMUM_PATH_LENGTH + 1e-9:
            raise ValueError(
                'benchmark path length %.3fm exceeds %.3fm' %
                (analytical_length, MAXIMUM_PATH_LENGTH))
        return relative_poses, segment_endpoints, analytical_length

    def generate_mixed_turns_path(self):
        """多半径左右复合弯：测试曲率切换、直线恢复与连续纠偏。"""
        relative_poses, _, length = self.sample_motion_primitives(
            self.mixed_turns_primitives())
        poses = self.transform_relative_path(relative_poses)
        rospy.loginfo(
            'Generated mixed-turns benchmark: %d points, length %.3fm, '
            'analytical R_min=1.000m',
            len(poses), length)
        return poses

    def generate_hairpin_path(self):
        """含R=1m 180度发卡弯的复合路径：测试紧弯和出弯稳定性。"""
        relative_poses, _, length = self.sample_motion_primitives(
            self.hairpin_primitives())
        poses = self.transform_relative_path(relative_poses)
        rospy.loginfo(
            'Generated hairpin benchmark: %d points, length %.3fm, '
            'analytical R_min=1.000m',
            len(poses), length)
        return poses

    def generate_ellipse_path(self):
        """闭合椭圆路径：长度小于30m，理论最小转弯半径大于1m。"""
        semi_major = 3.0
        semi_minor = 2.0
        theoretical_minimum_radius = semi_minor ** 2 / semi_major
        if theoretical_minimum_radius < MINIMUM_TURNING_RADIUS:
            raise ValueError(
                'ellipse minimum radius %.3fm is below %.3fm' %
                (theoretical_minimum_radius, MINIMUM_TURNING_RADIUS))

        # x=a*sin(t), y=b*(1-cos(t))：从原点出发且初始切向为+x。
        segment_count = max(
            16, int(math.ceil(
                2.0 * math.pi * max(semi_major, semi_minor)
                / PATH_SAMPLE_SPACING)))
        relative_poses = []
        for index in range(segment_count + 1):
            parameter = 2.0 * math.pi * index / segment_count
            x_rel = semi_major * math.sin(parameter)
            y_rel = semi_minor * (1.0 - math.cos(parameter))
            dx = semi_major * math.cos(parameter)
            dy = semi_minor * math.sin(parameter)
            yaw_rel = math.atan2(dy, dx)
            relative_poses.append((x_rel, y_rel, yaw_rel))

        sampled_length = sum(
            math.hypot(
                relative_poses[index + 1][0] - relative_poses[index][0],
                relative_poses[index + 1][1] - relative_poses[index][1])
            for index in range(len(relative_poses) - 1))
        if sampled_length > MAXIMUM_PATH_LENGTH + 1e-6:
            raise ValueError(
                'ellipse path length %.3fm exceeds %.3fm' %
                (sampled_length, MAXIMUM_PATH_LENGTH))

        poses = self.transform_relative_path(relative_poses)
        rospy.loginfo(
            'Generated ellipse path: %d points, length %.3fm, '
            'analytical R_min=%.3fm',
            len(poses), sampled_length, theoretical_minimum_radius)
        return poses

    def generate_double_eight_path(self):
        """纵向双八字：三个环、两个真实交叉点，形状对应手绘示意图。

        使用平滑傅里叶参数曲线：
          x(t) = sin(3t) + 0.75 sin(t) - 0.22 sin(5t)
          y(t) = 5.3 (cos(t) - 1)

        t从0到2*pi。路径从原点出发，初始切向沿+x方向；整条
        曲线位置、切向和曲率连续，并形成上、中、下三个环。
        """
        vertical_scale = 5.3
        first_harmonic = 0.75
        fifth_harmonic = -0.22
        segment_count = max(
            600, int(math.ceil(MAXIMUM_PATH_LENGTH / PATH_SAMPLE_SPACING)))

        relative_poses = []
        for index in range(segment_count + 1):
            parameter = 2.0 * math.pi * index / segment_count

            x_rel = (
                math.sin(3.0 * parameter)
                + first_harmonic * math.sin(parameter)
                + fifth_harmonic * math.sin(5.0 * parameter))
            y_rel = vertical_scale * (math.cos(parameter) - 1.0)

            dx = (
                3.0 * math.cos(3.0 * parameter)
                + first_harmonic * math.cos(parameter)
                + 5.0 * fifth_harmonic * math.cos(5.0 * parameter))
            dy = -vertical_scale * math.sin(parameter)
            yaw_rel = math.atan2(dy, dx)

            relative_poses.append((x_rel, y_rel, yaw_rel))

        sampled_length = sum(
            math.hypot(
                relative_poses[index + 1][0] - relative_poses[index][0],
                relative_poses[index + 1][1] - relative_poses[index][1])
            for index in range(len(relative_poses) - 1))
        if sampled_length > MAXIMUM_PATH_LENGTH + 1e-6:
            raise ValueError(
                'double-eight path length %.3fm exceeds %.3fm' %
                (sampled_length, MAXIMUM_PATH_LENGTH))

        poses = self.transform_relative_path(relative_poses)
        rospy.loginfo(
            'Generated crossed vertical double-eight path: %d points, '
            'length %.3fm',
            len(poses), sampled_length)
        return poses

    def generate_benchmark_s_curve_path(self):
        """与 controller_benchmark 完全一致的正弦 S 弯路径。"""
        segment_count = 120
        relative_poses = []
        for i in range(segment_count + 1):
            x = 10.0 * i / segment_count
            phase = 2.0 * math.pi * x / 10.0
            y = 0.8 * math.sin(phase)
            slope = 0.8 * 2.0 * math.pi / 10.0 * math.cos(phase)
            relative_poses.append((x, y, math.atan(slope)))
        poses = self.transform_relative_path(relative_poses)
        rospy.loginfo('Generated benchmark S-curve path with %d points', len(poses))
        return poses

    def generate_benchmark_wuyuce_path(self):
        """11.5 三顶点路线，用1m切线圆弧替代不可执行的几何尖角。"""
        vertices = [(0.0, 0.0), (5.5, 3.5), (16.8, 3.7)]
        radius = MINIMUM_TURNING_RADIUS
        spacing = PATH_SAMPLE_SPACING
        p0, corner, p2 = vertices

        incoming_length = math.hypot(
            corner[0] - p0[0], corner[1] - p0[1])
        outgoing_length = math.hypot(
            p2[0] - corner[0], p2[1] - corner[1])
        incoming = (
            (corner[0] - p0[0]) / incoming_length,
            (corner[1] - p0[1]) / incoming_length)
        outgoing = (
            (p2[0] - corner[0]) / outgoing_length,
            (p2[1] - corner[1]) / outgoing_length)
        incoming_yaw = math.atan2(incoming[1], incoming[0])
        outgoing_yaw = math.atan2(outgoing[1], outgoing[0])
        signed_turn = math.atan2(
            incoming[0] * outgoing[1] - incoming[1] * outgoing[0],
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1])
        tangent_offset = radius * math.tan(abs(signed_turn) / 2.0)
        if tangent_offset >= min(incoming_length, outgoing_length):
            raise ValueError('11.5 fillet does not fit adjacent line segments')

        tangent_in = (
            corner[0] - tangent_offset * incoming[0],
            corner[1] - tangent_offset * incoming[1])
        tangent_out = (
            corner[0] + tangent_offset * outgoing[0],
            corner[1] + tangent_offset * outgoing[1])
        turn_sign = 1.0 if signed_turn >= 0.0 else -1.0
        left_normal = (-incoming[1], incoming[0])
        center = (
            tangent_in[0] + turn_sign * radius * left_normal[0],
            tangent_in[1] + turn_sign * radius * left_normal[1])

        relative_poses = []

        def append_line(start, end, yaw, include_start):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            count = max(1, int(math.ceil(length / spacing)))
            first = 0 if include_start else 1
            for sample in range(first, count + 1):
                ratio = float(sample) / count
                relative_poses.append((
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                    yaw))

        append_line(p0, tangent_in, incoming_yaw, True)

        start_angle = math.atan2(
            tangent_in[1] - center[1], tangent_in[0] - center[0])
        arc_length = radius * abs(signed_turn)
        arc_count = max(2, int(math.ceil(arc_length / spacing)))
        for sample in range(1, arc_count + 1):
            ratio = float(sample) / arc_count
            radial_angle = start_angle + ratio * signed_turn
            relative_poses.append((
                center[0] + radius * math.cos(radial_angle),
                center[1] + radius * math.sin(radial_angle),
                incoming_yaw + ratio * signed_turn))

        append_line(tangent_out, p2, outgoing_yaw, False)

        poses = self.transform_relative_path(relative_poses)
        rospy.loginfo(
            'Generated curvature-feasible 11.5 route with %d points '
            '(fillet R=%.2fm, turn=%+.3frad)',
            len(poses), radius, signed_turn)
        return poses

    def create_pose_stamped(self, x, y, yaw):
        """辅助函数：创建带姿态的PoseStamped"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # 四元数 from yaw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def publish_path(self, event):
        """定时发布路径"""
        if self.current_path and len(self.current_path.poses) > 0:
            self.current_path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.current_path)
            rospy.logdebug('Published path with %d points', len(self.current_path.poses))

    def update_path_type(self, path_type):
        """动态切换路径类型"""
        self.path_type = path_type
        self.current_path = self.generate_path()
        rospy.loginfo('Path type updated to: %s', path_type)


def main():
    rospy.init_node('fixed_path_publisher')
    node = FixedPathPublisher()

    # 命令行交互（非阻塞输入）
    print("\n" + "=" * 50)
    print("Fixed Path Publisher Running (ROS1)")
    print("=" * 50)
    print("Commands:")
    print("  s - switch to straight path")
    print("  m - switch to mixed-turns benchmark path")
    print("  h - switch to hairpin benchmark path")
    print("  2 - switch to benchmark S-curve path")
    print("  3 - switch to benchmark 11.5 polyline path")
    print("  4 - switch to ellipse path")
    print("  5 - switch to double-eight path")
    print("  p - print path info")
    print("  Ctrl+C - exit")
    print("=" * 50)
    print("Fixed start: (%.2f, %.2f, %.2f)" % (node.start_x, node.start_y, node.start_yaw))
    print("=" * 50)

    rate = rospy.Rate(10)  # 10 Hz 检查输入
    try:
        while not rospy.is_shutdown():
            # 检查标准输入（非阻塞）
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if not cmd:
                    rate.sleep()
                    continue
                if cmd == 's':
                    node.update_path_type('straight')
                elif cmd == 'm':
                    node.update_path_type('mixed_turns')
                elif cmd == 'h':
                    node.update_path_type('hairpin')
                elif cmd == '2':
                    node.update_path_type('s_curve')
                elif cmd == '3':
                    node.update_path_type('wuyuce_11_5')
                elif cmd == '4':
                    node.update_path_type('ellipse')
                elif cmd == '5':
                    node.update_path_type('double_eight')
                elif cmd == 'p':
                    path = node.current_path
                    print("\nPath Info:")
                    print("  Type: %s" % node.path_type)
                    print("  Fixed start: (%.2f, %.2f, %.2f)" % (node.start_x, node.start_y, node.start_yaw))
                    print("  Points: %d" % len(path.poses))
                    if len(path.poses) > 0:
                        first = path.poses[0].pose.position
                        last = path.poses[-1].pose.position
                        print("  Start: (%.2f, %.2f)" % (first.x, first.y))
                        print("  End:   (%.2f, %.2f)" % (last.x, last.y))
                else:
                    print("Unknown command")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
