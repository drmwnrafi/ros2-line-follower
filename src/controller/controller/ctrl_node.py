import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rclpy.time import Duration

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Controller node started.')
        
        # Parameters
        self.declare_parameter('route', 'A')
        route = self.get_parameter('route').get_parameter_value().string_value
        if route == 'A':
            self.turn_at = {1, 2}
            self.reset_at = 2
        elif route == 'B':
            self.turn_at = {2, 3}
            self.reset_at = 4
        elif route == 'C':
            self.turn_at = set()
            self.reset_at = 4
        else:
            self.get_logger().warn(f"Unknown route {route}, defaulting to A")
            self.turn_at = {1, 2}
            self.reset_at = 2
        self.get_logger().info(f"Route set to {route}: turn at {self.turn_at}, reset at {self.reset_at}")
        
        self.kp = 0.4
        self.linear_speed = 0.3
        self.line_threshold = 200
        self.intersection_sensor_count = 3
        self.intersection_time_threshold = 0.5
        
        self.intersection_count = 0
        self.in_intersection = False
        self.last_intersection_time = self.get_clock().now()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        display_image = cv_image.copy()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        
        chunk_width = width // 8
        sensor_readings = []
        active_sensor_readings = []
        
        for i in range(8):
            start = i * chunk_width
            end = (i + 1) * chunk_width if i < 7 else width
            chunk = gray[:, start:end]
            avg_intensity = np.mean(chunk)
            sensor_value = 255 - avg_intensity
            sensor_readings.append(sensor_value)
            active_value = sensor_value if sensor_value > self.line_threshold else 0
            active_sensor_readings.append(active_value)
            
            cv2.line(display_image, (start, 0), (start, height), (0, 255, 0), 2)
            cv2.putText(display_image, f'{sensor_value:.0f}', (start + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        sensor_weights = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]
        weighted_sum = sum(w * r for w, r in zip(sensor_weights, active_sensor_readings))
        total = sum(active_sensor_readings)
        error = weighted_sum / total if total != 0 else 0.0
        
        active_count = sum(1 for r in sensor_readings if r > self.line_threshold)
        current_time = self.get_clock().now()
        time_since_last = current_time - self.last_intersection_time

        if active_count >= self.intersection_sensor_count:
            if not self.in_intersection:
                if time_since_last.nanoseconds / 1e9 > self.intersection_time_threshold:
                    self.intersection_count += 1
                    self.last_intersection_time = current_time
                    self.get_logger().info(f"Intersection detected! Total count: {self.intersection_count}")
                    cv2.putText(display_image, 'INTERSECTION', (width // 3, height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
                self.in_intersection = True

            if self.intersection_count in self.turn_at:
                error = 3.5
            else:
                error = 0.0
        else:
            self.in_intersection = False
        
        if (self.get_clock().now() - self.last_intersection_time).nanoseconds / 1e9 > 1 and self.intersection_count == self.reset_at:  
            cv2.putText(display_image, 'RESET!', (width // 3, height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)
            self.intersection_count = 0

        angular_z = -self.kp * error
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = angular_z
        self.publisher.publish(twist)

        cv2.putText(display_image, f'Error: {error:.2f}', (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(display_image, f'Intersections: {self.intersection_count}', (width - 210, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        cv2.imshow('Line Following Display', display_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    controller_node = Controller()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()