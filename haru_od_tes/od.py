import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import odrive
import json
import threading

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        self.declare_parameter("config_file", "config.json")
        config_path = self.get_parameter("config_file").value
        self.get_logger().info(f'Loading config from {config_path}')
        
        with open(config_path, 'r') as file:
            self.config = json.load(file)
        
        self.odrives = {}
        self.load_odrives()
        
        self.subscribers = []
        for motor in self.config["motors"]:
            topic_name = motor["topic_name"]
            sub = self.create_subscription(Float32, topic_name, self.create_motor_callback(motor), 10)
            self.subscribers.append(sub)
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
    
    def load_odrives(self):
        for motor in self.config["motors"]:
            serial = motor["serial_number"]
            if serial not in self.odrives:
                self.get_logger().info(f'Connecting to ODrive {serial}...')
                self.odrives[serial] = odrive.find_any(serial_number=serial)
                self.get_logger().info(f'Connected to ODrive {serial}')
    
    def create_motor_callback(self, motor):
        def callback(msg):
            odrv = self.odrives.get(motor["serial_number"])
            if odrv:
                axis = odrv.axis0 if motor["axis"] == 0 else odrv.axis1
                if axis.controller.config.control_mode != 2:
                    self.get_logger().warn(f'Switching {motor["serial_number"]} axis {motor["axis"]} to velocity control mode')
                    axis.controller.config.control_mode = 2  # CONTROL_MODE_VELOCITY_CONTROL
                axis.controller.input_vel = msg.data
            else:
                self.get_logger().error(f'ODrive {motor["serial_number"]} not found!')
        return callback
    
    def joy_callback(self, msg):
        if msg.buttons[1] == 1:
            self.get_logger().info('Calibrating all motors...')
            for odrv in self.odrives.values():
                odrv.axis0.requested_state = 3  # AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                odrv.axis1.requested_state = 3
        elif msg.buttons[3] == 1:
            self.get_logger().info('Starting closed-loop control...')
            for odrv in self.odrives.values():
                odrv.axis0.controller.config.control_mode = 2  # Set velocity control mode
                odrv.axis1.controller.config.control_mode = 2  # Set velocity control mode
                odrv.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL
                odrv.axis1.requested_state = 8

def main(args=None):
    rclpy.init(args=args)
    node = ODriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
