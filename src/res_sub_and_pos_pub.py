import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

def spherical_to_unit_vector(theta):
    """角度轉成單位向量 (2D 平面)"""
    return np.array([np.cos(theta), np.sin(theta)])

class MultiListener(Node):
    def __init__(self):
        super().__init__('multi_listener')

        self.left_angle = None
        self.right_angle = None

        # 訂閱兩個 topic
        self.sub_left_angle = self.create_subscription(
            Float32, '/sound_angle_left', self.left_angle_callback, 10
        )
        self.sub_right_angle = self.create_subscription(
            Float32, '/sound_angle_right', self.right_angle_callback, 10
        )

        # 發布聲源位置
        self.pub_position = self.create_publisher(Point, '/sound_source/position', 10)
        self.pub_distance = self.create_publisher(Float32, '/sound_source/distance', 10)
        self.pub_angle = self.create_publisher(Float32, '/sound_source/angle', 10)

        # 固定兩個 Respeaker 的位置 (單位: 公尺)
        self.mic1_pos = np.array([0.0, 0.0])
        self.mic2_pos = np.array([0.15, 0.0])

    def left_angle_callback(self, msg):
        self.left_angle = msg.data
        self.get_logger().info(f'[left_angle] {self.left_angle:.3f} rad')
        self.try_calculate_position()

    def right_angle_callback(self, msg):
        self.right_angle = msg.data
        self.get_logger().info(f'[right_angle] {self.right_angle:.3f} rad')
        self.try_calculate_position()

    def try_calculate_position(self):
        """當兩個角度都有時就計算聲源位置"""
        if self.left_angle is None or self.right_angle is None:
            return

        u1 = spherical_to_unit_vector(self.left_angle)
        u2 = spherical_to_unit_vector(self.right_angle)

        AA = np.array([
            [u1[0], 0, -1, 0],
            [u1[1], 0, 0, -1],
            [0, u2[0], -1, 0],
            [0, u2[1], 0, -1]
        ], dtype=float)

        bb = np.array([
            self.mic1_pos[0],
            self.mic1_pos[1],
            self.mic2_pos[0],
            self.mic2_pos[1]
        ], dtype=float)

        xx = np.linalg.pinv(AA) @ bb
        x, y = xx[0], xx[1]

        distance = np.sqrt(x**2 + y**2)
        angle = np.arctan2(y, x)

        # Debug log
        self.get_logger().info(
            f"聲源位置: x={x:.3f}, y={y:.3f}, 距離={distance:.3f} m, 角度={np.degrees(angle):.2f}°"
        )

        pos_msg = Point()
        pos_msg.x = float(x)
        pos_msg.y = float(y)
        pos_msg.z = 0.0
        self.pub_position.publish(pos_msg)

        self.pub_distance.publish(Float32(data=float(distance)))
        self.pub_angle.publish(Float32(data=float(angle)))  # 注意：這裡是 rad

def main(args=None):
    rclpy.init(args=args)
    node = MultiListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()