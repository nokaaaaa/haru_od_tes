import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE
import json
import time

# JSON設定の読み込み
CONFIG_FILE = "config.json"
with open(CONFIG_FILE, "r") as f:
    CONFIG = json.load(f)

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        self.get_logger().info("ODrive Controller Node Started")
        
        # ODriveデバイスの接続
        self.odrives = {}
        for motor in CONFIG["motors"]:
            serial = motor["serial_number"]
            if serial not in self.odrives:
                self.get_logger().info(f"Connecting to ODrive {serial}...")
                self.odrives[serial] = odrive.find_any(serial_number=serial)
                self.get_logger().info(f"Connected to ODrive {serial}")
        
        # 各モーター用の購読者を作成
        self.subscribers = []
        for motor in CONFIG["motors"]:
            sub = self.create_subscription(Float32, motor["topic_name"],
                                           lambda msg, m=motor: self.set_motor_speed(m, msg),
                                           10)
            self.subscribers.append(sub)
        
        # Joyトピック購読
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
    
    def set_motor_speed(self, motor, msg):
        """ 指定されたモーターに速度指令を送る """
        serial = motor["serial_number"]
        axis = motor["axis"]
        odrive_obj = self.odrives.get(serial)
        if odrive_obj:
            odrive_axis = odrive_obj.axis0 if axis == 0 else odrive_obj.axis1
            odrive_axis.controller.input_vel = msg.data
        
    def joy_callback(self, msg):
        """ /joyメッセージを受け取って処理 """
        if msg.buttons[1] == 1:
            self.calibrate_all_motors()
        elif msg.buttons[3] == 1:
            self.start_closed_loop()
        
    def calibrate_all_motors(self):
        """ すべてのモーターをキャリブレーション """
        self.get_logger().info("Starting motor calibration...")
        for serial, odrive_obj in self.odrives.items():
            for axis in [odrive_obj.axis0, odrive_obj.axis1]:
                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(0.1)
        self.get_logger().info("Calibration started")
    
    def start_closed_loop(self):
        """ すべてのモーターをクローズドループ制御に切り替える """
        self.get_logger().info("Switching motors to closed-loop control...")
        for serial, odrive_obj in self.odrives.items():
            for axis in [odrive_obj.axis0, odrive_obj.axis1]:
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                time.sleep(0.1)
        self.get_logger().info("Motors in closed-loop control")

def main(args=None):
    rclpy.init(args=args)
    node = ODriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
