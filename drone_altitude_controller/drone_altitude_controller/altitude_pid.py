import sys
import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QPushButton, QLabel, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer


class PIDAltitudeNode(Node):
    def __init__(self):
        super().__init__("pid_altitude_controller")
        self.pub = self.create_publisher(Actuators, "/command/motor_speed", 10)
        self.sub = self.create_subscription(Odometry, "/x500/odom", self.odom_callback, 10)

        self.Kp = 100.0
        self.Ki = 40.0
        self.Kd = 100.0
        self.setpoint = 3.0
        
        self.base_thrust=500.0

        self.current_altitude = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.motor_cmd = 0.0
        self.active = False
        self.dt = 0.02
        self.last_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.current_altitude = msg.pose.pose.position.z

    def pid_update(self):
        if not self.active:
            self.stop_motors()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0 or dt > 0.5:
            dt = self.dt

        error = self.setpoint - self.current_altitude
        self.integral = max(-100, min(self.integral + error * dt, 100))
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        thrust = max(0, min(self.base_thrust + output, 1000))
        self.motor_cmd = thrust

        act = Actuators()
        act.velocity = [thrust] * 4
        self.pub.publish(act)

    def stop_motors(self):
        act = Actuators()
        act.velocity = [0.0] * 4
        self.pub.publish(act)
        self.integral = 0.0
        self.prev_error = 0.0
        self.motor_cmd = 0.0


class PIDAltitudeGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.kp_slider, self.kp_label = self.make_slider("Kp", node.Kp, (0, 300))
        self.ki_slider, self.ki_label = self.make_slider("Ki", node.Ki, (0, 100))
        self.kd_slider, self.kd_label = self.make_slider("Kd", node.Kd, (0, 200))
        self.sp_slider, self.sp_label = self.make_slider("Setpoint", node.setpoint, (0, 10))

        self.altitude_label = QLabel("Altitude: 0.00")
        self.motor_label = QLabel("Motor Cmd: 0.00")
        self.status_label = QLabel("Status: STOPPED")

        start_btn = QPushButton("Start")
        stop_btn = QPushButton("Stop")
        start_btn.clicked.connect(self.start)
        stop_btn.clicked.connect(self.stop)

        layout = QVBoxLayout()
        for s, l in [(self.kp_slider, self.kp_label),
                     (self.ki_slider, self.ki_label),
                     (self.kd_slider, self.kd_label),
                     (self.sp_slider, self.sp_label)]:
            layout.addWidget(l)
            layout.addWidget(s)

        layout.addWidget(self.altitude_label)
        layout.addWidget(self.motor_label)
        layout.addWidget(self.status_label)

        btns = QHBoxLayout()
        btns.addWidget(start_btn)
        btns.addWidget(stop_btn)
        layout.addLayout(btns)

        self.setLayout(layout)
        self.setWindowTitle("Altitude PID Control")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(int(node.dt * 1000))

    def make_slider(self, name, value, limits):
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(limits[0] * 100)
        slider.setMaximum(limits[1] * 100)
        slider.setValue(int(value * 100))

        label = QLabel(f"{name}: {value:.2f}")

        def changed(v):
            real = v / 100
            label.setText(f"{name}: {real:.2f}")
            if name == "Kp":
                self.node.Kp = real
            elif name == "Ki":
                self.node.Ki = real
            elif name == "Kd":
                self.node.Kd = real
            else:
                self.node.setpoint = real

        slider.valueChanged.connect(changed)
        return slider, label

    def start(self):
        self.node.active = True
        self.status_label.setText("Status: RUNNING")

    def stop(self):
        self.node.active = False
        self.status_label.setText("Status: STOPPED")

    def update_ui(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.node.pid_update()
        self.altitude_label.setText(f"Altitude: {self.node.current_altitude:.2f}")
        self.motor_label.setText(f"Motor Cmd: {self.node.motor_cmd:.2f}")


def main():
    rclpy.init()
    node = PIDAltitudeNode()

    app = QApplication(sys.argv)
    gui = PIDAltitudeGUI(node)
    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
