import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGraphicsScene, QGraphicsView, QGraphicsRectItem, QLabel
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor
import threading


class BoolPublisherNode(Node):
    def __init__(self):
        super().__init__("bool_publisher")

        self.publishers_dict = {
            "START": self.create_publisher(Bool, "start", 10),
            "RESET": self.create_publisher(Bool, "reset", 10),
            "CALIB": self.create_publisher(Bool, "calib", 10),
            "CLOSED": self.create_publisher(Bool, "closed", 10),
        }

        self.pose_subscription = self.create_subscription(
            Pose2D, "/pose", self.pose_callback, 10
        )

        self.pose = Pose2D()
        self.get_logger().info("BoolPublisherNode has started.")

    def publish_true(self, topic_name):
        if topic_name in self.publishers_dict:
            msg = Bool()
            msg.data = True
            self.publishers_dict[topic_name].publish(msg)
            self.get_logger().info(f"Published True to {topic_name}")

    def pose_callback(self, msg):
        self.pose = msg


class PoseVisualizer(QGraphicsView):
    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setFixedSize(560, 1120)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.scene.setSceneRect(0, 0, 560, 1120)

        self.pose_rect = QGraphicsRectItem(-40, -40, 80, 80)
        self.pose_rect.setBrush(QColor('blue'))
        self.scene.addItem(self.pose_rect)

        self.pose_label = QLabel(self)
        self.pose_label.setStyleSheet("font-size: 16px; color: black;")
        self.pose_label.setGeometry(10, 10, 200, 50)
        self.scene.addWidget(self.pose_label)

    def update_pose(self):
        pose = self.ros_node.pose
        x_pixel = (pose.x + 3.5) * 50
        y_pixel = (7 - pose.y) * 100
        theta_deg = math.degrees(-pose.theta)

        self.pose_rect.setPos(x_pixel, y_pixel)
        self.pose_rect.setRotation(theta_deg)

        self.pose_label.setText(f"x: {pose.x:.2f}, y: {pose.y:.2f}, theta: {pose.theta:.2f}")


class BoolPublisherGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Bool Publisher & Pose Viewer")
        self.showFullScreen()

        main_layout = QHBoxLayout()

        left_layout = QVBoxLayout()

        exit_button = QPushButton("Exit", self)
        exit_button.setStyleSheet("min-width: 80px; min-height: 40px;")
        exit_button.clicked.connect(self.close_application)
        left_layout.addWidget(exit_button, alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)

        button_layout = QGridLayout()
        self.buttons = {
            "START": "START",
            "RESET": "RESET",
            "CALIB": "CALIB",
            "CLOSED": "CLOSED",
        }

        button_style = """
        QPushButton {
            border: 2px solid #8f8f91;
            border-radius: 100px;
            background-color: #f0f0f0;
            min-width: 200px;
            min-height: 200px;
            max-width: 200px;
            max-height: 200px;
        }
        QPushButton:pressed {
            background-color: #d0d0d0;
        }
        """

        positions = [(0, 0), (0, 2), (2, 0), (2, 2)]
        for position, (button_text, topic) in zip(positions, self.buttons.items()):
            button = QPushButton(button_text, self)
            button.setStyleSheet(button_style)
            button.clicked.connect(lambda checked, t=topic: self.ros_node.publish_true(t))
            button_layout.addWidget(button, *position)

        button_container = QWidget()
        button_container.setLayout(button_layout)
        button_container.setFixedSize(1000, 1000)  # ボタンの高さを減らす

        left_layout.addWidget(button_container, alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)

        self.pose_viewer = PoseVisualizer(ros_node)

        main_layout.addLayout(left_layout)
        main_layout.addWidget(self.pose_viewer)

        self.setLayout(main_layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.pose_viewer.update_pose)
        self.timer.start(10)

    def close_application(self):
        self.close()


def ros_spin(node):
    rclpy.spin(node)


def main():
    app = QApplication(sys.argv)
    rclpy.init()
    node = BoolPublisherNode()
    gui = BoolPublisherGUI(node)
    gui.show()

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
