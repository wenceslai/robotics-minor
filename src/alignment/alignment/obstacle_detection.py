import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor


class ObstacleDetection(Node):
    def __init__(self, threshold):
        super().__init__('sonar_proximity_node')
        self.threshold = threshold

        # Publishing sensor data and the True/False proximity status
        self.pub_sensor_1 = self.create_publisher(Range, 'sonar/sensor_1', 10)
        self.pub_sensor_2 = self.create_publisher(Range, 'sonar/sensor_2', 10)
        self.pub_proximity = self.create_publisher(Bool, 'proximity_status', 10)

        self.sensor_1 = DistanceSensor(echo=22, trigger=17)
        self.sensor_2 = DistanceSensor(echo=5, trigger=27)

        self.timer = self.create_timer(0.1, self.process_sensors)
        self.distance_1 = None
        self.distance_2 = None

    def process_sensors(self):
        msg1 = Range()
        msg1.header.stamp = self.get_clock().now().to_msg()
        msg1.range = self.sensor_1.distance * 100  # Convert to cm
        msg1.max_range = 400
        msg1.min_range = 2
        self.pub_sensor_1.publish(msg1)
        self.distance_1 = msg1.range

        msg2 = Range()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.range = self.sensor_2.distance * 100  # Convert to cm
        msg2.max_range = 400
        msg2.min_range = 2
        self.pub_sensor_2.publish(msg2)
        self.distance_2 = msg2.range

        self.get_logger().info(f"Sensor 1: {msg1.range:.2f} cm, Sensor 2: {msg2.range:.2f} cm")

        if self.distance_1 is not None and self.distance_2 is not None:
            close = self.distance_1 < self.threshold or self.distance_2 < self.threshold
            proximity_msg = Bool()
            proximity_msg.data = close
            self.pub_proximity.publish(proximity_msg)

            self.get_logger().info(f"Proximity: {'Close' if close else 'Safe'}")


def main(args=None):
    rclpy.init(args=args)
    threshold = 80  
    node = ObstacleDetection(threshold)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()