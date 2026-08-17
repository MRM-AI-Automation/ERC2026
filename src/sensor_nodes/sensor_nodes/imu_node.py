import rclpy
from rclpy.node import Node
from aruco_msgs.msg import ImuData
from geometry_msgs.msg import Vector3


import time
import pickle
import os
import serial


class BNO055Node(Node):
    def __init__(self):
        super().__init__('ex_imu_node')

        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.line="123"
        
        self.imu_pub_ = self.create_publisher(ImuData, '/imu_data', 10)

        self.initialize_imu()


     
        self.timer_ = self.create_timer(0.01, self.publish_imu)

    def initialize_imu(self):
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(1)
        if(self.serial_port):
            print("into the serial port1")
        else:
            print("serial port issue")
        self.serial_port.dtr = False   # GPIO0 HIGH
        self.serial_port.rts = True    # EN LOW (reset)
        time.sleep(0.1)

        self.serial_port.rts = False   # EN HIGH (run)
        time.sleep(0.1)
        self.serial_port.dtr = True 
        self.serial_port.close()
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(1)
        if(self.serial_port):
            print("into the serial port reseted")
        else:
            print("serial port issue")



        
        while True:
            self.line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            print(self.line)
            
            if(len(self.line)!=0 and str(self.line[0])=="2"):
                break
        self.get_logger().info("Yaw usable")
    
    def publish_imu(self):
        
        self.line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
        if(self.line[0]!="2"):
            self.initialize_imu()
        else: 
            data=self.line[2:-1]
            xcv=data.split("_")
            for i in xcv:
                xcv[xcv.index(i)]=float(i)
            
            
            self.yaw=xcv[0]
            self.pitch=xcv[1]
            self.roll=xcv[2]
            print(f"r: {self.roll:7.2f} p: {self.pitch:7.2f} y: {self.yaw:7.2f}", end='\r')

            imu_msg_ = ImuData()
            imu_msg_.orientation.x = self.roll
            imu_msg_.orientation.y = self.pitch
            imu_msg_.orientation.z = self.yaw

            imu_msg_.acceleration.x = 0.0
            imu_msg_.acceleration.y = 0.0
            imu_msg_.acceleration.z = 0.0

            # Publish
            self.imu_pub_.publish(imu_msg_)



def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_calibration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
