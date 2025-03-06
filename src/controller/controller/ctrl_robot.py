import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SetEntityState
import cv2
import time
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.limit_switches = ['/limit_switch_a/bumper_states', '/limit_switch_agv/bumper_states',
                              '/limit_switch_b/bumper_states', '/limit_switch_c/bumper_states',
                              '/limit_switch_out/bumper_states']
        for switch_topic in self.limit_switches:
            self.create_subscription(ContactsState, 
                                     switch_topic, 
                                     lambda msg, topic=switch_topic: self.on_ls(msg, topic), 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.tf_subscription = self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
        self.t = np.array([0.0, 0.0, 0.0])

        self.declare_parameter('route', 'cycle')
        self.route_param = self.get_parameter('route').get_parameter_value().string_value
        self.cycling = self.route_param == 'cycle'
        self.curr_route_idx = 0
        self.routes = ['A', 'B', 'C']
        self.curr_route = self.routes[self.curr_route_idx] if self.cycling else self.route_param
        self.init_route_config()
        
        self.kp = 0.3
        self.linear_speed = 0.3
        
        self.line_tol = 200
        self.intersect_sensor_count = 3
        self.intersect_time_tol = 0.5
        self.intersect_count = 0
        self.in_intersection = False
        self.last_intersect_time = self.get_clock().now()
        
        self.agv_ls = False
        self.out_ls = False
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.pending_reset = False

    def init_route_config(self):
        self.move_objs = []
        if self.curr_route == 'A':
            self.turn_at = {1, 2}
            self.reset_at = 2
            self.rfid = np.array([[6.30, -3.25, 0.0], [1.55, -5.03, 0]])
            self.move_objs = [
                {'cond': 'rfid0_and_agv_ls', 'name': 'box1', 'pose': [6.2, -2.753, 0.255], 'triggered': False},
                {'cond': 'rfid1_and_agv_ls', 'name': 'box1', 'pose':[2.013100, -5.397891, 0.26], 'triggered': False},
                {'cond': 'out_ls', 'name': 'box1', 'pose': [7.32, -2.8, 0.33], 'triggered': False}
            ]
        elif self.curr_route == 'B':
            self.turn_at = {2, 3}
            self.reset_at = 4
            self.rfid = np.array([[9.8, -3.25, 0], [1.55, -5.03, 0]])
            self.move_objs = [
                {'cond': 'rfid0_and_agv_ls', 'name': 'box2', 'pose': [9.7, -2.800000,0.255011], 'triggered': False},
                {'cond': 'rfid1_and_agv_ls', 'name': 'box2', 'pose':[2.013100, -5.397891, 0.26], 'triggered': False},
                {'cond': 'out_ls', 'name': 'box2', 'pose': [10.82, -2.8, 0.33], 'triggered': False}
            ]
        elif self.curr_route == 'C':
            self.turn_at = set()
            self.reset_at = 4
            self.rfid = np.array([[11.6, -3.25, 0.0], [1.55, -5.03, 0]])
            self.move_objs = [
                {'cond': 'rfid0_and_agv_ls', 'name': 'box3', 'pose': [11.55, -2.800000,0.255011], 'triggered': False},
                {'cond': 'rfid1_and_agv_ls', 'name': 'box3', 'pose':[2.013100, -5.397891, 0.26], 'triggered': False},
                {'cond': 'out_ls', 'name': 'box3', 'pose': [12.7, -2.8, 0.33], 'triggered': False}
            ]
        self.get_logger().info(f'Initialized route {self.curr_route}')
        
    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'base_link':
                self.t = np.array([transform.transform.translation.x, 
                                   transform.transform.translation.y, 
                                   transform.transform.translation.z])

    def on_ls(self, msg, switch_name):
        if switch_name == '/limit_switch_agv/bumper_states':
            self.agv_ls = bool(msg.states)
        elif switch_name == '/limit_switch_out/bumper_states':
            self.out_ls = bool(msg.states)

    def check_rfid(self, t, rfid_list, tol=0.1):
        for i, rfid in enumerate(rfid_list):
            if np.linalg.norm(t - rfid) < tol:
                return i
        return -1

    def find_ones(self, sensor_readings):
        return [index for index, value in enumerate(sensor_readings) if value == 1]

    def move_object(self, name, pose):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = pose[0]
        req.state.pose.position.y = pose[1]
        req.state.pose.position.z = pose[2]
        req.state.reference_frame = 'world'
        future = self.client.call_async(req)
        future.add_done_callback(self.move_object_done)

    def move_object_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Object moved successfully')
            else:
                self.get_logger().error('Failed to move object')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            return

        display_image = cv_image.copy()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        chunk_width = width // 8
        sensor_readings = []

        for i in range(8):
            start = i * chunk_width
            end = (i + 1) * chunk_width if i < 7 else width
            chunk = gray[:, start:end]
            avg_intensity = np.mean(chunk)
            sensor_value = 1 if (255 - avg_intensity) > self.line_tol else 0
            sensor_readings.append(sensor_value)
            cv2.line(display_image, (start, 0), (start, height), (0, 255, 0), 2)
            cv2.putText(display_image, str(sensor_value), (start + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        sensor_weights = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]
        weighted_sum = sum(w * r for w, r in zip(sensor_weights, sensor_readings))
        total = sum(sensor_readings)
        error = weighted_sum / total if total != 0 else 0.0
        active_count = sum(sensor_readings)
        current_time = self.get_clock().now()
        time_since_last = current_time - self.last_intersect_time

        if active_count >= self.intersect_sensor_count:
            if not self.in_intersection:
                if time_since_last.nanoseconds / 1e9 > self.intersect_time_tol:
                    self.intersect_count += 1
                    self.last_intersect_time = current_time
                self.in_intersection = True
                cv2.putText(display_image, 'INTERSECTION', (width // 3, height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
            if self.intersect_count in self.turn_at:
                max_idx_one = max(self.find_ones(sensor_readings))
                error = sensor_weights[max_idx_one]
            else:
                error = 0.0
        else:
            self.in_intersection = False

        if (self.get_clock().now() - self.last_intersect_time).nanoseconds / 1e9 > 1 and self.intersect_count == self.reset_at:  
            self.intersect_count = 0
            cv2.putText(display_image, 'RESET!', (width // 3, height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)
            
        detected_rfid = self.check_rfid(self.t, self.rfid)

        for move_obj in self.move_objs:
            if move_obj['triggered']:
                continue
            cond = False
            if move_obj['cond'] == 'rfid0_and_agv_ls':
                cond = (detected_rfid == 0) and self.agv_ls
            elif move_obj['cond'] == 'rfid1_and_agv_ls':
                cond = (detected_rfid == 1) and self.agv_ls
            elif move_obj['cond'] == 'out_ls':
                cond = self.out_ls
            if cond:
                self.move_object(move_obj['name'], move_obj['pose'])
                move_obj['triggered'] = True
                if move_obj['cond'] == 'out_ls':
                    self.pending_reset = True  
        
        if self.pending_reset and not self.out_ls:
            for obj in self.move_objs:
                obj['triggered'] = False
            self.pending_reset = False
            self.get_logger().info('All triggers reset for next loop.')
            if self.cycling:
                self.curr_route_idx = (self.curr_route_idx + 1) % 3
                self.curr_route = self.routes[self.curr_route_idx]
                self.init_route_config()
                self.get_logger().info(f'Switched to route {self.curr_route}')

        if (detected_rfid == 1 and self.agv_ls) or (detected_rfid == 0 and not self.agv_ls):
            if detected_rfid == 1 and self.agv_ls:
                self.get_logger().info("Detected RFID 1 and AGV limit switch active. Delaying for 1 second.")
                time.sleep(1) 
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
        else:
            angular_z = -self.kp * error
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = angular_z
            self.publisher.publish(twist)
            
        cv2.putText(display_image, f'Error: {error:.2f}', (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(display_image, f'Intersections: {self.intersect_count}', (width - 210, height - 10),
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