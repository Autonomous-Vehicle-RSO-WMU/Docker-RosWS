import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32
from sensor_msgs.msg import Imu,MagneticField
from sensor_msgs.msg import NavSatFix,NavSatStatus

import serial 

from my_robot_base.utils.quarternion_calc import calc_orientation #TODO: Use header instead?

class arduino(Node):
        def __init__(self):
            super().__init__('arduino_node_ros')

            # --- 1. Declare Parameters ---
            self.declare_parameter('port', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_14532303232351F050C0-if00')
            self.declare_parameter('baudrate', 115200)
            self.declare_parameter('poll_rate_hz', 20.0)
        
            # --- 2. Get Params Sent by Launch/Config ---
            port_name = self.get_parameter('port').get_parameter_value().string_value
            baud_rate = self.get_parameter('baudrate').get_parameter_value().integer_value
            poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)

            if poll_rate_hz <= 0.0:
                raise ValueError('poll_rate_hz must be greater than 0.0')

            # --- 3. Initialize Serial Connection Using Params ---
            self.arduinoData = serial.Serial(port_name, baud_rate)

            self.timer = self.create_timer(1.0 / poll_rate_hz, self.timer_callback)
            self.IMUpublisher=self.create_publisher(Imu,'/imu/data',10)
            self.Magnetopublisher=self.create_publisher(MagneticField,'/imu/mag',10)
            self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
            self.get_logger().info(f'Arduino polling at {poll_rate_hz:.1f} Hz on {port_name}')
        def timer_callback(self):
               if self.arduinoData.in_waiting:
                try:
                    line=self.arduinoData.readline().decode().strip()
                    if(', ' in line):
                        print(line)
                        line=line.split(', ')
                        if len(line)!=12:
                              return# skipping over incomplete data
                        accel=line[0:3]
                        gyro=line[3:6]
                        magn=line[6:9]
                        gps=line[9:]

                        # Ensure GPS data is in valid range before converting and publishing
                        lat = float(gps[0]) / 1e7
                        lon = float(gps[1]) / 1e7

                        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                            self.get_logger().warn(f"Dropping invalid GPS fix lat={lat}, lon={lon}, raw={gps}")
                            return

                        print(accel,gyro,magn,gps)
                        msg=Imu()
                        msg.header.stamp = self.get_clock().now().to_msg()  # <-- timestamp
                        msg.header.frame_id = "imu_link" #TODO: Don't hardcode frame here?
                        msg.linear_acceleration.x = float(accel[0])
                        msg.linear_acceleration.y = float(accel[1])
                        msg.linear_acceleration.z = float(accel[2])
                        msg.linear_acceleration_covariance[0] = -1  # unknown

                        # Angular velocity (gyroscope)
                        msg.angular_velocity.x = float(gyro[0])
                        msg.angular_velocity.y = float(gyro[1])
                        msg.angular_velocity.z = float(gyro[2])
                        msg.angular_velocity_covariance[0] = -1  # unknown

                        # # Orientation unknown (if not available)
                        msg.orientation_covariance[0] = -1
                        

                        msg2=MagneticField()
                        msg2.header.stamp = self.get_clock().now().to_msg()
                        msg2.header.frame_id = "imu_link" #TODO: Don't hardcode frame here?
                        msg2.magnetic_field.x=float(magn[0])
                        msg2.magnetic_field.y=float(magn[1])
                        msg2.magnetic_field.z=float(magn[2])
                        

                        ax = float(accel[0])
                        ay = float(accel[1])
                        az = float(accel[2])
                        mx = float(magn[0])
                        my = float(magn[1])
                        mz = float(magn[2])

                        quarternion = calc_orientation(ax, ay, az, mx, my, mz)

                        msg.orientation.x = quarternion[0]
                        msg.orientation.y = quarternion[1]
                        msg.orientation.z = quarternion[2]
                        msg.orientation.w = quarternion[3]
                        
                        msg3=NavSatFix()
                        msg3.header.stamp = self.get_clock().now().to_msg()
                        msg3.header.frame_id = "gps_link" #TODO: Don't hardcode frame here?
                        msg3.latitude=lat
                        msg3.longitude=lon
                        msg3.altitude=float(gps[2]) / 1000.0

                        self.IMUpublisher.publish(msg)
                        self.Magnetopublisher.publish(msg2)
                        self.gps_publisher.publish(msg3)
                except Exception as e:
                      pass
                      

def main():
    print('Hi from igvc_bot_arduino_sensors.')
    rclpy.init()
    serial_node=arduino()
    try:
        rclpy.spin(serial_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        serial_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
