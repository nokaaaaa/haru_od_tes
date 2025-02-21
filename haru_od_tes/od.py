import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
import odrive
import json
import math

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        self.declare_parameter("config_file", "config.json")
        config_path = self.get_parameter("config_file").value
        self.get_logger().info(f'Loading config from {config_path}')
        
        with open(config_path, 'r') as file:
            self.config = json.load(file)
        
        self.R = self.config.get("R", 0.2)
        self.wheel_radius = self.config.get("wheel_radius", 0.05)
        
        self.odrives = {}
        self.load_odrives()
        
        self.create_subscription(Twist, '/pid_cmd_vel', self.twist_callback, 10)
        self.create_subscription(Bool, '/calib', self.calib_callback, 10)
        self.create_subscription(Bool, '/closed', self.closed_loop_callback, 10)
        
        for motor in self.config["motors"]:
            self.create_subscription(Float32, motor["topic_name"], lambda msg, m=motor: self.motor_callback(msg, m), 10)
    
    def load_odrives(self):
        for motor in self.config["motors"]:
            serial = motor["serial_number"]
            if serial not in self.odrives:
                self.get_logger().info(f'Connecting to ODrive {serial}...')
                self.odrives[serial] = odrive.find_any(serial_number=serial)
                self.get_logger().info(f'Connected to ODrive {serial}')
    
    def twist_callback(self, msg):
        Vx = msg.linear.x
        Vy = msg.linear.y
        omega = msg.angular.z
        
        coeff = 1 / (2 * math.pi * self.wheel_radius)
        r2 = math.sqrt(2)
        v1 = coeff * ((-Vx + Vy) / r2 + self.R * omega)
        v2 = coeff * ((-Vx - Vy) / r2 + self.R * omega)
        v3 = coeff * ((Vx - Vy) / r2 + self.R * omega)
        v4 = coeff * ((Vx + Vy) / r2 + self.R * omega)
        
        wheel_speeds = [v1, v2, v3, v4]
        
        for i, motor in enumerate(self.config["motors"]):
            odrv = self.odrives.get(motor["serial_number"])
            if odrv:
                axis = odrv.axis0 if motor["axis"] == 0 else odrv.axis1
                if axis.controller.config.control_mode != 2:
                    self.get_logger().warn(f'Switching {motor["serial_number"]} axis {motor["axis"]} to velocity control mode')
                    axis.controller.config.control_mode = 2  
                axis.controller.input_vel = wheel_speeds[i]
            else:
                self.get_logger().error(f'ODrive {motor["serial_number"]} not found!')
    
    def motor_callback(self, msg, motor):
        odrv = self.odrives.get(motor["serial_number"])
        if odrv:
            axis = odrv.axis0 if motor["axis"] == 0 else odrv.axis1
            axis.controller.input_vel = msg.data
        else:
            self.get_logger().error(f'ODrive {motor["serial_number"]} not found!')
    
    def calib_callback(self, msg):
        if msg.data:
            self.get_logger().info('Calibrating all motors...')
            for odrv in self.odrives.values():
                odrv.axis0.requested_state = 3  # AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                odrv.axis1.requested_state = 3
    
    def closed_loop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Starting closed-loop control...')
            for odrv in self.odrives.values():
                odrv.axis0.controller.config.control_mode = 2
                odrv.axis1.controller.config.control_mode = 2
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
