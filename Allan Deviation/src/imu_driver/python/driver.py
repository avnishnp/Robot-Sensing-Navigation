#!/usr/bin/env python3

import rospy
import serial
# import sensor_msgs.msg
# import geometry_msgs.msg
# from geometry_msgs.msg import Vector3
import std_msgs.msg
# from sensor_msgs.msg import IMU
# from sensor_msgs.msg import MagField
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from imu_driver.msg import *
# from gps import *
from math import radians, sin, pi
import numpy as np
import math


def euler_to_quaternion(yaw, pitch, roll):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

# def degree_radian(yaw,pitch,roll):
#   yaw=float(math.radians(yaw))
#   pitch=float(math.radians(pitch))
#   roll=float(math.radians(roll))
#   return yaw,pitch,roll


# def write_register(ser,register,data):
#   return register,data
  

if __name__ == '__main__':
    
    rospy.init_node('driver_node_gps', anonymous=True)
    msg=imu_msg()
    ser_port=rospy.get_param('~port_number')
    ser = serial.Serial(ser_port, 115200, timeout=3.0)  
    # ser_baud=rospy.get_param('~baudrate',4800)
    # sample_rate=rospy.get_param('~sample_rate',1.0)
    ser.write(b"$VNWRG,07,40*XX")
    msg.Header.seq= 0

    pub = rospy.Publisher('imu', imu_msg, queue_size=10)
    imu_frame="IMU1_Frame"

    # Configuring and setting values for IMU
    # write_register(ser,register=6,data="14") # for YMR
    # write_register(ser,register=7,data="40") # for output rate 

    # rate = rospy.Rate(40.0) #40Hz

    try: 
            while not rospy.is_shutdown(): 
                lines = ser.readline().decode("ASCII").strip("\r")
                # lines=lines[4:][:-3]
                lines=lines[1:][:-3]
                line=lines.split(",")
                #print(line)
                if line[0] == "VNYMR": 
                    # print(lines)
                    #Rotations
                    yaw=math.radians(float(line[1]))
                    pitch=math.radians(float(line[2]))
                    roll=math.radians(float(line[3]))

                    #Magnetic Field
                    mag_x=float(line[4])
                    mag_y=float(line[5])
                    mag_z=float(line[6])

                    msg.MagField.magnetic_field.x=mag_x
                    msg.MagField.magnetic_field.y=mag_y
                    msg.MagField.magnetic_field.z=mag_z

                    rospy.loginfo(msg.Header)

                    quaternion=euler_to_quaternion(yaw,pitch,roll)
                    msg.IMU.orientation.x=quaternion[0]
                    msg.IMU.orientation.y=quaternion[1]
                    msg.IMU.orientation.z=quaternion[2]
                    msg.IMU.orientation.w=quaternion[3]


                    #publish Accx,Accy,Accz (m/s^2)
                    acc_x=float(line[7])
                    acc_y=float(line[8])
                    acc_z=float(line[9])
                   
                    msg.IMU.linear_acceleration.x=acc_x
                    msg.IMU.linear_acceleration.y=acc_y
                    msg.IMU.linear_acceleration.z=acc_z


                    angu_z,check=line[12].split('*')
                    #feed Imu angular_velocity 
                    ang_x=float(line[10])
                    ang_y=float(line[11])
                    ang_z=float(angu_z)

                    msg.IMU.angular_velocity.x=ang_x
                    msg.IMU.angular_velocity.y=ang_y
                    msg.IMU.angular_velocity.z=ang_z

                    #rospy.loginfo(msg.IMU)
                    now=rospy.Time.now()
                    msg.Header.stamp=now
                    msg.Header.frame_id=imu_frame
                    # checksum=str(check(line[13]))
                    #rospy.loginfo(msg)
                    # print('---')
                    
                    msg.IMU.header.frame_id=imu_frame
                    msg.MagField.header.frame_id=imu_frame

                    msg.IMU.header.stamp = now
                    msg.MagField.header.stamp = now
                    msg.IMU.header.seq += 1
                    msg.MagField.header.seq += 1
                    rospy.loginfo(msg)
                    pub.publish(msg)
    except serial.serialutil.SerialException:
        ser.close()

    







