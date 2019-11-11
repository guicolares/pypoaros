#!/usr/bin/env python
from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg
from dji_sdk.msg import TransparentTransmissionData
from std_msgs.msg import String
import time
import sys
import math
import rospy
import os
import socket
from subprocess import call
#buzzer
from gpiozero import Buzzer

sdkPermissionService = False
activation_service = None

HORIZ_ATT  = 0x00
HORIZ_VEL  = 0x40
HORIZ_POS  = 0x80
VERT_VEL   = 0x00
VERT_POS   = 0x10
VERT_TRU   = 0x20
YAW_ANG    = 0x00
YAW_RATE   = 0x08
HORIZ_GND  = 0x00
HORIZ_BODY = 0x02
STABLE_OFF = 0x00
STABLE_ON  = 0x01

def display_main_menu():
    print "----------- < Main menu > ----------"
    print "[a] Request to obtain control"
    print "[b] Release control"
    print "[c] Takeoff"
    print "[d] Landing"
    print "[e] Gimbal control"
    print "[f] Draw circle sample"
    print "[g] Arm Test"
    print "[h] Disarm Test"
    print "[i] Send Data to Mobile"
    print "[j] Exit"
    print "\nuse `rostopic echo` to query drone status"
    print "----------------------------------------"
    print "input: "


def callback_mob(data):
    print 'mobile callback'
    print data.data

def attitude_control(flag, x, y, z, yaw):
    attitude_control_service(flag = flag, x = x, y = y, z = z, yaw = yaw)


def gimbal_angle_control(yaw = 0, roll = 0, pitch = 0, duration = 0, absolute_or_incremental = True, yaw_cmd_ignore = False, roll_cmd_ignore = False, pitch_cmd_ignore = False):
    gimbal_angle_control_service(yaw = yaw, roll = roll, pitch = pitch, duration = duration,
            absolute_or_incremental = absolute_or_incremental, yaw_cmd_ignore = yaw_cmd_ignore, roll_cmd_ignore = roll_cmd_ignore, pitch_cmd_ignore = pitch_cmd_ignore)

def gimbal_speed_control(self, yaw_rate = 0, roll_rate = 0, pitch_rate = 0):
    gimbal_speed_control_service(yaw_rate = yaw_rate, roll_rate = roll_rate, pitch_rate = pitch_rate)

def callback_global(data):
    global altitude
    altitude = data.altitude

def init_services():
    rospy.wait_for_service("dji_sdk/activation")
    rospy.wait_for_service("dji_sdk/attitude_control")
    rospy.wait_for_service("dji_sdk/camera_action_control")
    rospy.wait_for_service("dji_sdk/drone_task_control")
    rospy.wait_for_service("dji_sdk/gimbal_angle_control")
    rospy.wait_for_service("dji_sdk/gimbal_speed_control")
    rospy.wait_for_service("dji_sdk/global_position_control")
    rospy.wait_for_service("dji_sdk/local_position_control")
    rospy.wait_for_service("dji_sdk/sdk_permission_control")
    rospy.wait_for_service("dji_sdk/velocity_control")
    rospy.wait_for_service("dji_sdk/drone_arm_control")
    rospy.wait_for_service("dji_sdk/virtual_rc_enable_control")
    rospy.wait_for_service("dji_sdk/virtual_rc_data_control")
    rospy.wait_for_service("dji_sdk/sync_flag_control")
    rospy.wait_for_service("dji_sdk/send_data_to_remote_device")

    global sdk_permission_control_service
    global drone_task_control_service
    global drone_arm_service
    global gimbal_angle_control_service
    global gimbal_speed_control_service
    global attitude_control_service
    global send_data_to_remote_device

    activation_service = rospy.ServiceProxy("dji_sdk/activation", dji_sdk.srv.Activation)
    attitude_control_service = rospy.ServiceProxy("dji_sdk/attitude_control", dji_sdk.srv.AttitudeControl)
    drone_task_control_service = rospy.ServiceProxy("dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)
    gimbal_angle_control_service = rospy.ServiceProxy("dji_sdk/gimbal_angle_control", dji_sdk.srv.GimbalAngleControl)
    gimbal_speed_control_service = rospy.ServiceProxy("dji_sdk/gimbal_speed_control", dji_sdk.srv.GimbalSpeedControl)
    global_position_control_service = rospy.ServiceProxy("dji_sdk/global_position_control", dji_sdk.srv.GlobalPositionControl)
    local_position_control_service = rospy.ServiceProxy("dji_sdk/local_position_control", dji_sdk.srv.LocalPositionControl)
    sdk_permission_control_service = rospy.ServiceProxy("dji_sdk/sdk_permission_control", dji_sdk.srv.SDKPermissionControl)
    velocity_control_service = rospy.ServiceProxy("dji_sdk/velocity_control", dji_sdk.srv.VelocityControl)
    drone_arm_service = rospy.ServiceProxy("dji_sdk/drone_arm_control", dji_sdk.srv.DroneArmControl)
    send_data_to_remote_device = rospy.ServiceProxy("dji_sdk/send_data_to_remote_device", dji_sdk.srv.SendDataToRemoteDevice)

def init_subscribers():
    rospy.Subscriber("/dji_sdk/data_received_from_remote_device", TransparentTransmissionData, callback_mob)
    
if __name__ == "__main__":
    rospy.init_node("dji_sdk_palestra", anonymous=True)
    init_services()
    init_subscribers()
    display_main_menu()
while True:
        main_operate_code = sys.stdin.read(1)
        if main_operate_code == 'a':
            sdk_permission_control_service(control_enable = 1)
        elif main_operate_code == 'b':
            sdk_permission_control_service(control_enable = 0)
        elif main_operate_code == 'c':
            drone_task_control_service(task=4)
        elif main_operate_code == 'd':
            drone_task_control_service(task=6)
        elif main_operate_code == 'e':
            gimbal_angle_control(0, 0, 0, 20)
            time.sleep(2)
            gimbal_angle_control(0, 0, 1800, 20)
            time.sleep(2)
            gimbal_angle_control(0, 0, -1800, 20)
            time.sleep(2)
            gimbal_angle_control(300, 0, 0, 20)
            time.sleep(2)
            gimbal_angle_control(-300, 0, 0, 20)
            time.sleep(2)
            gimbal_angle_control(0, 300, 0, 20)
            time.sleep(2)
            gimbal_angle_control(0, -300, 0, 20)
            time.sleep(2)
            gimbal_speed_control(100, 0, 0)
            time.sleep(2)
            gimbal_speed_control(-100, 0, 0)
            time.sleep(2)
            gimbal_speed_control(0, 0, 200)
            time.sleep(2)
            gimbal_speed_control(0, 0, -200)
            time.sleep(2)
            gimbal_speed_control(0, 200, 0)
            time.sleep(2)
            gimbal_speed_control(0, -200, 0)
            time.sleep(2)
            gimbal_angle_control(0, 0, 0, 20)
            
        elif main_operate_code == 'f':
            R = 2
            V = 2
            for i in range(300):
                vx = V * math.sin((V/R)*i/50.0)
                vy = V * math.cos((V/R)*i/50.0)

                attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|STABLE_ON, vx, vy, 0, 0)
                time.sleep(0.02)
        elif main_operate_code == 'g':
            drone_arm_service(arm=1)
        elif main_operate_code == 'h':
            drone_arm_service(arm=0)
        elif main_operate_code == 'i':
            send_data_to_remote_device('pypoa2019')
        elif main_operate_code == 'j':
            break
        else:
            display_main_menu()
