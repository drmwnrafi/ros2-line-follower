import rclpy
from rclpy.node import Node
from conveyorbelt_msgs.msg import ConveyorBeltState, ConveyorPower
from tf2_msgs.msg import TFMessage
import numpy as np

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')
        
        # Publisher setup
        self.pub = self.create_publisher(ConveyorPower, '/CONVEYORPOWER', 10)
        self.last_powers = None
        
        # Subscribers
        self.create_subscription(ConveyorBeltState, '/CONVEYORSTATE', self.on_state, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 10)

        self.declare_parameter('route', 'cycle')
        
        self.routes = ['A', 'B', 'C']
        self.current_route_index = 0
        self.cycling = False
        
        self.setup_params()
        
        self.activated_rfids = set()
        self.pending_switch = False
        self.switch_distance = 1.0
        
    def on_state(self, msg):
        self.curr_powers = msg.powers
    
    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == 'base_link':
                self.position = np.array([
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z
                ])
                self.check_rfid()

    def setup_params(self):
        route_param = self.get_parameter('route').get_parameter_value().string_value
        self.cycling = route_param == 'cycle'

        if self.cycling:
            self.current_route = self.routes[self.current_route_index]
        else:
            self.current_route = route_param

        route_configs = {
            'A': {'rfid': [[6.30, -3.25, 0.0], [1.55, -5.03, 0]],
                  'powers': {0: (1, 20.0), 1: (4, -20.0)}},
            'B': {'rfid': [[9.8, -3.25, 0], [1.55, -5.03, 0]],
                  'powers': {0: (2, 20.0), 1: (4, -20.0)}},
            'C': {'rfid': [[11.6, -3.25, 0.0], [1.55, -5.03, 0]],
                  'powers': {0: (3, 20.0), 1: (4, -20.0)}}
        }
        
        cfg = route_configs.get(self.current_route, route_configs['A'])
        self.conveyor_powers = cfg['powers']
        self.rfid_positions = np.array(cfg['rfid'])
        self.activated_rfids = set()
        self.get_logger().info(f'Initialized route: {self.current_route}')

    def check_rfid(self, tol=0.1):
        for idx, pos in enumerate(self.rfid_positions):
            if np.linalg.norm(self.position - pos) < tol:
                self.handle_rfid(idx)
                return

        if self.pending_switch and self.cycling:
            if all(np.linalg.norm(self.position - pos) > self.switch_distance 
                   for pos in self.rfid_positions):
                self.switch_route()

    def handle_rfid(self, rfid_idx):
        if rfid_idx not in self.conveyor_powers: return
        
        belt_id, power = self.conveyor_powers[rfid_idx]
        if 0 <= (belt_idx := belt_id - 1) < len(self.curr_powers):
            new_powers = [0.0] * len(self.curr_powers)
            new_powers[belt_idx] = power
            self.update_powers(new_powers)

        if self.cycling:
            self.activated_rfids.add(rfid_idx)
            if self.activated_rfids >= {0, 1}:
                self.pending_switch = True
                self.get_logger().info('Ready for route switch')
                
    
    def switch_route(self):
        self.current_route_index = (self.current_route_index + 1) % 3
        self.setup_params()
        self.pending_switch = False
        self.get_logger().info(f'Switched to route {self.current_route}')

    def update_powers(self, powers):
        if powers != self.last_powers:
            msg = ConveyorPower()
            msg.powers = powers
            self.pub.publish(msg)
            self.last_powers = list(powers)
            self.get_logger().info(f"Updated powers: {powers}")

def main(args=None):
    rclpy.init(args=args)
    controller = ConveyorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()