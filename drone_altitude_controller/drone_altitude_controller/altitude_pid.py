import sys
import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QPushButton, QLabel, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
import numpy as np
import time

class PIDAltitudeNode(Node):
    def __init__(self):
        super().__init__('pid_altitude_controller')
        self.pub = self.create_publisher(Actuators, '/command/motor_speed', 10)
        self.sub = self.create_subscription(Odometry, '/x500/odom', self.odom_callback, 10)

        self.Kp = 100.0
        self.Ki = 40.0
        self.Kd = 100.0
        self.setpoint = 3.0

        self.current_altitude = 0.0
        self.motor_cmd = 0.0
        self.active = False
        self.last_update_time = self.get_clock().now()

        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.02

    def odom_callback(self, msg: Odometry):
        self.current_altitude = msg.pose.pose.position.z
        self.get_logger().info(f"Altitude update: {self.current_altitude}") # Squelch spam


    def pid_update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = self.dt
        if not self.active:
            self.motor_cmd = 0.0
            act = Actuators()
            act.velocity = [0.0] * 4
            self.pub.publish(act)
            self.integral = 0.0
            self.prev_error = 0.0
            return

        error = self.setpoint - self.current_altitude
        self.integral += error * dt
        self.integral = max(-100.0, min(self.integral, 100.0))
        
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        base_hover_thrust = 500.0 #
        thrust = base_hover_thrust + output
        
        thrust = max(0.0, min(thrust, 1000.0))
        self.motor_cmd = thrust

        act = Actuators()
        act.velocity = [thrust] * 4
        self.pub.publish(act)

class PIDAltitudeGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.kp_slider, self.kp_label = self._create_slider_label("Kp", node.Kp, 0, 300, 100)
        self.ki_slider, self.ki_label = self._create_slider_label("Ki", node.Ki, 0, 100, 100)
        self.kd_slider, self.kd_label = self._create_slider_label("Kd", node.Kd, 0, 200, 100)
        self.sp_slider, self.sp_label = self._create_slider_label("Setpoint Height", node.setpoint, 0, 10, 100)

        self.altitude_label = QLabel("Current Altitude: 0.00 m")
        self.setpoint_label = QLabel(f"Setpoint: {node.setpoint:.2f} m")
        self.motor_label = QLabel("Current Motor Cmd: 0.00 rad/s")
        self.status_label = QLabel("Status: STOPPED")
        self.status_label.setStyleSheet("font-weight: bold")


        self.start_btn = QPushButton("Start Motors")
        self.stop_btn = QPushButton("Stop Motors")
        self.start_btn.clicked.connect(self.start_motors)
        self.stop_btn.clicked.connect(self.stop_motors)

        layout = QVBoxLayout()
        for slider, label in [(self.kp_slider, self.kp_label), (self.ki_slider, self.ki_label),
                              (self.kd_slider, self.kd_label), (self.sp_slider, self.sp_label)]:
            layout.addWidget(label)
            layout.addWidget(slider)

        layout.addWidget(self.altitude_label)
        layout.addWidget(self.setpoint_label)
        layout.addWidget(self.motor_label)
        layout.addWidget(self.status_label)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.stop_btn)
        layout.addLayout(button_layout)

        self.setLayout(layout)
        self.setWindowTitle("X500 Altitude PID Control")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(int(self.node.dt * 1000)) # 50Hz

    def _create_slider_label(self, name, initial, min_val, max_val, scale):
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val * scale))
        slider.setMaximum(int(max_val * scale))
        slider.setValue(int(initial * scale))
        label = QLabel(f"{name}: {initial:.2f}")

        def on_value_change(val):
            real_val = val / scale
            label.setText(f"{name}: {real_val:.2f}")
            if name == "Kp":
                self.node.Kp = real_val
            elif name == "Ki":
                self.node.Ki = real_val
            elif name == "Kd":
                self.node.Kd = real_val
            elif name == "Setpoint Height":
                self.node.setpoint = real_val
                self.setpoint_label.setText(f"Setpoint: {real_val:.2f} m")

        slider.valueChanged.connect(on_value_change)
        return slider, label

    def start_motors(self):
        self.node.active = True
        self.status_label.setText("Status: RUNNING")
        self.status_label.setStyleSheet("font-weight: bold; color: green")


    def stop_motors(self):
        self.node.active = False
        self.status_label.setText("Status: STOPPED")
        self.status_label.setStyleSheet("font-weight: bold; color: red")

    def update_ui(self):
        if "AUTO-TUNING" not in self.status_label.text():
            rclpy.spin_once(self.node, timeout_sec=0)
            self.node.pid_update()
        
        self.altitude_label.setText(f"Current Altitude: {self.node.current_altitude:.2f} m")
        self.motor_label.setText(f"Current Motor Cmd: {self.node.motor_cmd:.2f} rad/s")
        
        self.kp_label.setText(f"Kp: {self.node.Kp:.2f}")
        self.kp_slider.setValue(int(self.node.Kp * 100))
        self.ki_label.setText(f"Ki: {self.node.Ki:.2f}")
        self.ki_slider.setValue(int(self.node.Ki * 100))
        self.kd_label.setText(f"Kd: {self.node.Kd:.2f}")
        self.kd_slider.setValue(int(self.node.Kd * 100))


def main():
    rclpy.init()
    pid_node = PIDAltitudeNode()
    
    app = QApplication(sys.argv)
    gui = PIDAltitudeGUI(pid_node)
    gui.show()
    
    exit_code = app.exec_()
    
    pid_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()