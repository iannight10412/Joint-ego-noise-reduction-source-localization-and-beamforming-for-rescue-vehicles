import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        self.get_logger().info("✅ AngleSubscriber node started")

        # subscribe to ROS2 topic
        self.subscription = self.create_subscription(
            String,
            '/sound_source/string',
            self.listener_callback,
            10
        )
        self.subscription  # 防止垃圾回收

        # inital Serial
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
            self.get_logger().info("✅ Serial connected to Arduino (/dev/ttyACM0)")
        except Exception as e:
            self.get_logger().error(f"❌ Serial connection failed: {e}")
            self.ser = None

        self.waiting_done = False
        self.first_time = True

        #  Arduino 是否傳回 DONE 或 RESET
        self.timer = self.create_timer(0.1, self.check_arduino_done)

    def listener_callback(self, msg):
        """處理 ROS2 topic 訊息"""
        data = msg.data.strip()
        self.get_logger().info(f"接收到原始字串: {data}")

        # 如果不是第一次，而且還在等 Arduino 回覆 → 忽略
        if not self.first_time and self.waiting_done:
            self.get_logger().warn("⚠️ 等待 Arduino DONE，忽略新資料")
            return

        self.send_to_arduino(data)

    def send_to_arduino(self, data):
        """將 distance,angle 傳給 Arduino"""
        try:
            distance_str, angle_str = data.split(",")
            distance = float(distance_str)
            angle = float(angle_str)

            if self.ser is not None and self.ser.is_open:
                data_to_send = f"{distance:.3f},{angle:.2f}\n"
                self.ser.write(data_to_send.encode('utf-8'))
                self.get_logger().info(f"已傳送到 Arduino: {data_to_send.strip()}")

                if self.first_time:
                    self.get_logger().info("✅ 第一次傳送，略過 DONE 檢查")
                    self.first_time = False
                else:
                    self.waiting_done = True

        except Exception as e:
            self.get_logger().error(f"❌ 解析失敗: {data}, 錯誤: {e}")

    def check_arduino_done(self):
        """定時檢查 Arduino 是否傳回 DONE 或 RESET"""
        if self.ser is not None and self.ser.in_waiting > 0:
            responses = []
            while self.ser.in_waiting > 0:
                responses.append(self.ser.readline().decode('utf-8').strip())

            for response in responses:
                if response == "DONE":
                    self.get_logger().info("✅ 收到 Arduino 回覆 DONE")
                    self.waiting_done = False
                elif response == "RESET":
                    self.get_logger().warn("⚠️ 收到 Arduino RESET → 無條件重新執行")
                    self.waiting_done = False
                    self.first_time = True  # 可視需求重置第一次旗標
                elif response:
                    self.get_logger().warn(f"⚠️ Arduino 回覆: {response}")


def main(args=None):
    rclpy.init(args=args)
    node = AngleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
