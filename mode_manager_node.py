#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8
import serial
import math

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # --- FYSISKE KONSTANTER (T200 Thrustere) ---
        self.c_fwd = (3.71 * 9.81) / 160000.0
        self.c_rev = (2.92 * 9.81) / 160000.0

        # --- SYSTEM TILSTAND ---
        self.current_mode = 0  # 0=Kill(Rød), 1=Manual(Gul), 2=Auto(Grønn)

        # --- SERIELL TILKOBLING ---
        self.serial_port_name = '/dev/ttyUSB0' # Standard for Nano. Endre tilbake til ACM0 om nødvendig.
        self.baud_rate = 115200
        self.arduino = None
        self.connect_serial()

        # --- SUBSCRIBERS ---
        # Lytter på ferdig allokerte krefter (fra Thrust Allocation)
        self.sub_forces = self.create_subscription(Float64MultiArray, '/thruster_forces', self.force_callback, 10)
        
        # Lytter på System Modus (fra Mode Manager)
        self.sub_mode = self.create_subscription(Int8, '/system_mode_status', self.mode_callback, 10)

        self.get_logger().info("IO Primus: Arduino Bridge oppdatert (Motor + LED kontroll).")

    def connect_serial(self):
        try:
            self.arduino = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0)
            self.get_logger().info(f"Koblet til Arduino på {self.serial_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Klarte ikke koble til Arduino: {e}")

    def mode_callback(self, msg):
        self.current_mode = msg.data

    # --- MOTOR KONTROLL (TIL ARDUINO) ---
    def calculate_pwm(self, force):
        if force >= 0:
            pwm = 1500 + math.sqrt(force / self.c_fwd)
        else:
            pwm = 1500 - math.sqrt(abs(force) / self.c_rev)
        return max(1100, min(1900, int(pwm)))

    def force_callback(self, msg):
        if not self.arduino or not self.arduino.is_open:
            return

        forces = msg.data
        if len(forces) != 4:
            return

        pwm1 = self.calculate_pwm(forces[0])
        pwm2 = self.calculate_pwm(forces[1])
        pwm3 = self.calculate_pwm(forces[2])
        pwm4 = self.calculate_pwm(forces[3])

        # Format: <pwm1,pwm2,pwm3,pwm4,mode>\n
        serial_msg = f"<{pwm1},{pwm2},{pwm3},{pwm4},{self.current_mode}>\n"
        self.arduino.write(serial_msg.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino and node.arduino.is_open:
            # Send stopp-signal + Rødt lys (0) ved nedstengning
            node.arduino.write(b"<1500,1500,1500,1500,0>\n")
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()