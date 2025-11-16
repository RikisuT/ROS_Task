import sys
import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
# QApplication is needed for processEvents
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt, QTimer
import numpy as np
import time

class PIDPositionNode(Node):
    def __init__(self):
        super().__init__('pid_position_controller')
        self.pub = self.create_publisher(Actuators, '/command/motor_speed', 10)
        self.sub = self.create_subscription(Odometry, '/x500/odom', self.odom_callback, 10)

        # === Z-Axis (Altitude) PID ===
        self.Kp_z = 100.0
        self.Ki_z = 45.0
        self.Kd_z = 90.0
        self.setpoint_z = 3.0
        self.current_z = 0.0
        self.integral_z = 0.0
        self.prev_error_z = 0.0

        # === X-Axis (Position) PID ===
        self.Kp_x = 1.0
        self.Ki_x = 0.0
        self.Kd_x = 5.0
        self.setpoint_x = 0.0
        self.current_x = 0.0
        self.integral_x = 0.0
        self.prev_error_x = 0.0

        # === Y-Axis (Position) PID ===
        self.Kp_y = 1.0
        self.Ki_y = 0.0
        self.Kd_y = 5.0
        self.setpoint_y = 0.0
        self.current_y = 0.0
        self.integral_y = 0.0
        self.prev_error_y = 0.0

        # === Node State ===
        self.motor_cmds = [0.0] * 4
        self.active = False
        self.dt = 0.02 # 50Hz update rate

    def odom_callback(self, msg: Odometry):
        self.current_z = msg.pose.pose.position.z
        self.current_x = msg.pose.pose.position.x
        self.current_y = -msg.pose.pose.position.y
        # self.get_logger().info(f"Odom update (x,y,z): {self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}")

    def pid_update(self):
        if not self.active:
            self.motor_cmds = [0.0] * 4
            act = Actuators()
            act.velocity = [0.0] * 4
            self.pub.publish(act)
            # Reset PID state when inactive
            self.integral_z = 0.0
            self.prev_error_z = 0.0
            self.integral_x = 0.0
            self.prev_error_x = 0.0
            self.integral_y = 0.0
            self.prev_error_y = 0.0
            return

        # === Z-Axis (Altitude) Calculation ===
        error_z = self.setpoint_z - self.current_z
        self.integral_z += error_z * self.dt
        self.integral_z = max(-100.0, min(self.integral_z, 100.0)) # Clamp integral
        derivative_z = (error_z - self.prev_error_z) / self.dt
        output_z = self.Kp_z * error_z + self.Ki_z * self.integral_z + self.Kd_z * derivative_z
        self.prev_error_z = error_z
        
        base_hover_thrust = 500.0 # This is a guess, adjust!
        thrust = base_hover_thrust + output_z

        # === X-Axis (Pitch) Calculation ===
        # Error is in world frame. We want to control pitch.
        # Positive error (setpoint > current) means we need to move in +X direction
        # This requires a positive pitch command (front down, rear up)
        # BUT, common mixing `thrust - pitch` means positive pitch *reduces* front motor speed.
        # Let's define output_x as the pitch command.
        error_x = self.setpoint_x - self.current_x
        self.integral_x += error_x * self.dt
        self.integral_x = max(-50.0, min(self.integral_x, 50.0)) # Clamp integral
        derivative_x = (error_x - self.prev_error_x) / self.dt
        # Note: The output sign depends on your drone's mixing and frame.
        # We'll assume positive output_x causes positive pitch (moves forward)
        output_x = self.Kp_x * error_x + self.Ki_x * self.integral_x + self.Kd_x * derivative_x
        self.prev_error_x = error_x
        pitch_cmd = output_x # Kp_x * ...
        
        # === Y-Axis (Roll) Calculation ===
        # Positive error (setpoint > current) means we need to move in +Y direction (right)
        # This requires a positive roll command (left up, right down)
        error_y = self.setpoint_y - self.current_y
        self.integral_y += error_y * self.dt
        self.integral_y = max(-50.0, min(self.integral_y, 50.0)) # Clamp integral
        derivative_y = (error_y - self.prev_error_y) / self.dt
        # We'll assume positive output_y causes positive roll (moves right)
        output_y = self.Kp_y * error_y + self.Ki_y * self.integral_y + self.Kd_y * derivative_y
        self.prev_error_y = error_y
        roll_cmd = output_y # Kp_y * ...

        # === Motor Mixing (X-Quad Configuration) ===
        # m0 (Front-Right), m1 (Rear-Left), m2 (Front-Left), m3 (Rear-Right)
        # Move Forward (+pitch_cmd): m0/m2 (front) decrease, m1/m3 (rear) increase
        # Move Right (+roll_cmd): m0/m3 (right) decrease, m1/m2 (left) increase
        
        m0 = thrust - pitch_cmd - roll_cmd # Front-Right
        m1 = thrust + pitch_cmd + roll_cmd # Rear-Left
        m2 = thrust - pitch_cmd + roll_cmd # Front-Left
        m3 = thrust + pitch_cmd - roll_cmd # Rear-Right

        # Clamp all motor commands
        m0 = max(0.0, min(m0, 1000.0))
        m1 = max(0.0, min(m1, 1000.0))
        m2 = max(0.0, min(m2, 1000.0))
        m3 = max(0.0, min(m3, 1000.0))

        self.motor_cmds = [m0, m1, m2, m3]

        act = Actuators()
        # The order in act.velocity must match the vehicle's motor mapping
        # We assume the order [m0, m1, m2, m3] is correct.
        act.velocity = self.motor_cmds
        self.pub.publish(act)

class PIDPositionGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.set_scale = 100 # Scale for float sliders

        layout = QVBoxLayout()
        
        # === Z-Axis (Altitude) Controls ===
        layout.addWidget(self._create_section_label("Z-Axis (Altitude)"))
        self.kp_z_slider, self.kp_z_label = self._create_slider_label("Kp_z", node.Kp_z, 0, 300)
        self.ki_z_slider, self.ki_z_label = self._create_slider_label("Ki_z", node.Ki_z, 0, 100)
        self.kd_z_slider, self.kd_z_label = self._create_slider_label("Kd_z", node.Kd_z, 0, 200)
        self.sp_z_slider, self.sp_z_label = self._create_slider_label("Setpoint Z", node.setpoint_z, 0, 10)
        
        for slider, label in [(self.kp_z_slider, self.kp_z_label), (self.ki_z_slider, self.ki_z_label),
                              (self.kd_z_slider, self.kd_z_label), (self.sp_z_slider, self.sp_z_label)]:
            layout.addWidget(label)
            layout.addWidget(slider)

        # === X-Axis Controls ===
        layout.addWidget(self._create_section_label("X-Axis (Forward/Backward)"))
        self.kp_x_slider, self.kp_x_label = self._create_slider_label("Kp_x", node.Kp_x, 0, 50)
        self.ki_x_slider, self.ki_x_label = self._create_slider_label("Ki_x", node.Ki_x, 0, 20)
        self.kd_x_slider, self.kd_x_label = self._create_slider_label("Kd_x", node.Kd_x, 0, 50)
        self.sp_x_slider, self.sp_x_label = self._create_slider_label("Setpoint X", node.setpoint_x, -10, 10)
        
        for slider, label in [(self.kp_x_slider, self.kp_x_label), (self.ki_x_slider, self.ki_x_label),
                              (self.kd_x_slider, self.kd_x_label), (self.sp_x_slider, self.sp_x_label)]:
            layout.addWidget(label)
            layout.addWidget(slider)

        # === Y-Axis Controls ===
        layout.addWidget(self._create_section_label("Y-Axis (Left/Right)"))
        self.kp_y_slider, self.kp_y_label = self._create_slider_label("Kp_y", node.Kp_y, 0, 50)
        self.ki_y_slider, self.ki_y_label = self._create_slider_label("Ki_y", node.Ki_y, 0, 20)
        self.kd_y_slider, self.kd_y_label = self._create_slider_label("Kd_y", node.Kd_y, 0, 50)
        self.sp_y_slider, self.sp_y_label = self._create_slider_label("Setpoint Y", node.setpoint_y, -10, 10)

        for slider, label in [(self.kp_y_slider, self.kp_y_label), (self.ki_y_slider, self.ki_y_label),
                              (self.kd_y_slider, self.kd_y_label), (self.sp_y_slider, self.sp_y_label)]:
            layout.addWidget(label)
            layout.addWidget(slider)

        # === Status & Info Labels ===
        layout.addWidget(self._create_section_label("Status"))
        self.position_label = QLabel("Current Pos (x,y,z): 0.00, 0.00, 0.00 m")
        self.setpoint_label_z = QLabel(f"Setpoint Z: {node.setpoint_z:.2f} m")
        self.setpoint_label_x = QLabel(f"Setpoint X: {node.setpoint_x:.2f} m")
        self.setpoint_label_y = QLabel(f"Setpoint Y: {node.setpoint_y:.2f} m")
        self.motor_label = QLabel("Motor Cmds: [0.0, 0.0, 0.0, 0.0]")
        self.status_label = QLabel("Status: STOPPED")
        self.status_label.setStyleSheet("font-weight: bold")

        layout.addWidget(self.position_label)
        layout.addWidget(self.setpoint_label_x)
        layout.addWidget(self.setpoint_label_y)
        layout.addWidget(self.setpoint_label_z)
        layout.addWidget(self.motor_label)
        layout.addWidget(self.status_label)

        # === Control Buttons ===
        self.start_btn = QPushButton("Start Motors")
        self.stop_btn = QPushButton("Stop Motors")
        self.start_btn.clicked.connect(self.start_motors)
        self.stop_btn.clicked.connect(self.stop_motors)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.stop_btn)
        layout.addLayout(button_layout)

        self.setLayout(layout)
        self.setWindowTitle("X500 Position PID Control")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(int(self.node.dt * 1000)) # 50Hz

    def _create_section_label(self, text):
        """Helper to create a styled section label"""
        label = QLabel(text)
        label.setStyleSheet("font-weight: bold; font-size: 14px; margin-top: 10px;")
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(line)
        
        # Need to return a QWidget to add to the main layout
        widget = QWidget()
        widget.setLayout(layout)
        return widget

    def _create_slider_label(self, name, initial, min_val, max_val):
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val * self.set_scale))
        slider.setMaximum(int(max_val * self.set_scale))
        slider.setValue(int(initial * self.set_scale))
        label = QLabel(f"{name}: {initial:.2f}")

        def on_value_change(val):
            real_val = val / self.set_scale
            label.setText(f"{name}: {real_val:.2f}")
            if name == "Kp_z":
                self.node.Kp_z = real_val
            elif name == "Ki_z":
                self.node.Ki_z = real_val
            elif name == "Kd_z":
                self.node.Kd_z = real_val
            elif name == "Setpoint Z":
                self.node.setpoint_z = real_val
                self.setpoint_label_z.setText(f"Setpoint Z: {real_val:.2f} m")
            # X-Axis
            elif name == "Kp_x":
                self.node.Kp_x = real_val
            elif name == "Ki_x":
                self.node.Ki_x = real_val
            elif name == "Kd_x":
                self.node.Kd_x = real_val
            elif name == "Setpoint X":
                self.node.setpoint_x = real_val
                self.setpoint_label_x.setText(f"Setpoint X: {real_val:.2f} m")
            # Y-Axis
            elif name == "Kp_y":
                self.node.Kp_y = real_val
            elif name == "Ki_y":
                self.node.Ki_y = real_val
            elif name == "Kd_y":
                self.node.Kd_y = real_val
            elif name == "Setpoint Y":
                self.node.setpoint_y = real_val
                self.setpoint_label_y.setText(f"Setpoint Y: {real_val:.2f} m")

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
        # Spin once to process odom callback
        rclpy.spin_once(self.node, timeout_sec=0)
        # Run the PID update logic
        self.node.pid_update()
        
        # Update UI labels
        self.position_label.setText(f"Current Pos (x,y,z): {self.node.current_x:.2f}, {self.node.current_y:.2f}, {self.node.current_z:.2f} m")
        cmds = self.node.motor_cmds
        self.motor_label.setText(f"Motor Cmds: [{cmds[0]:.1f}, {cmds[1]:.1f}, {cmds[2]:.1f}, {cmds[3]:.1f}]")
        
        # Update slider labels in case values were changed elsewhere (e.g. auto-tune)
        # Z-Axis
        self.kp_z_label.setText(f"Kp_z: {self.node.Kp_z:.2f}")
        self.kp_z_slider.setValue(int(self.node.Kp_z * self.set_scale))
        self.ki_z_label.setText(f"Ki_z: {self.node.Ki_z:.2f}")
        self.ki_z_slider.setValue(int(self.node.Ki_z * self.set_scale))
        self.kd_z_label.setText(f"Kd_z: {self.node.Kd_z:.2f}")
        self.kd_z_slider.setValue(int(self.node.Kd_z * self.set_scale))
        # X-Axis
        self.kp_x_label.setText(f"Kp_x: {self.node.Kp_x:.2f}")
        self.kp_x_slider.setValue(int(self.node.Kp_x * self.set_scale))
        self.ki_x_label.setText(f"Ki_x: {self.node.Ki_x:.2f}")
        self.ki_x_slider.setValue(int(self.node.Ki_x * self.set_scale))
        self.kd_x_label.setText(f"Kd_x: {self.node.Kd_x:.2f}")
        self.kd_x_slider.setValue(int(self.node.Kd_x * self.set_scale))
        # Y-Axis
        self.kp_y_label.setText(f"Kp_y: {self.node.Kp_y:.2f}")
        self.kp_y_slider.setValue(int(self.node.Kp_y * self.set_scale))
        self.ki_y_label.setText(f"Ki_y: {self.node.Ki_y:.2f}")
        self.ki_y_slider.setValue(int(self.node.Ki_y * self.set_scale))
        self.kd_y_label.setText(f"Kd_y: {self.node.Kd_y:.2f}")
        self.kd_y_slider.setValue(int(self.node.Kd_y * self.set_scale))


def main():
    rclpy.init()
    pid_node = PIDPositionNode()
    
    # The QTimer loop will call spin_once, so a background spin thread
    # is not strictly necessary for this simple app.
    
    app = QApplication(sys.argv)
    gui = PIDPositionGUI(pid_node)
    gui.show()
    
    exit_code = app.exec_()
    
    pid_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()