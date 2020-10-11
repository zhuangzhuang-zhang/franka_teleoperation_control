#!/usr/bin/env python
import os, os.path, sys
import time

def findPIDfromNode(val):
    # print(val)
    items = val.split(" ")
    pid = ""
    i = 0
    for item in items:
        if i is 0 or item is '':
            i = i + 1
            continue
        else:
            pid = item
            break
    return pid


node_names = [
    'usb_cam-test.launch',
    'image_view/image_view',
    'franka_modern_driver_v2_dual.launch',
    'franka_driver/franka1_modern_driver',
    'franka_driver/franka2_modern_driver',
    'franka_driver/joint_pub_node',
    'usb_cam/usb_cam',
    'davinci_driver/davinci_driver_node',
    'robot_state_publisher/robot_state_publisher',
    'rviz/rviz',
    'record_rcm/record_rcm',
    'omega_dual_robot.launch',
    'omega_touch_robot.launch',
    'servo_control_class/franka1_omega1_node',
    'servo_control_class/franka2_omega2_node'
    #'detect_server/script/ssd_main_show.py'
]

sh_names = [
    'master_dual_omega.sh',
    'master_omega_touch.sh'
]




def main():
	for sh_name in sh_names:
		#print(sh_name)
		shs = os.popen('ps -ef|grep ' + sh_name + '|grep -v grep').read()
		shs = shs.split("\n")
		
		for sh in shs:
			if(sh != ""):
				sh_pid = findPIDfromNode(sh)
				os.system("kill " + sh_pid)
				#print("killing " + sh_pid + " " + sh)


	for node_name in node_names:
		#print(node_name)
		vals = os.popen('ps -ef|grep ' + node_name + '|grep -v grep').read()
		vals = vals.split("\n")
		for val in vals:
			if val != "":
				val_pid = findPIDfromNode(val)
				os.system("kill " + val_pid)
				#print("killing " + val_pid + " " + val)

if __name__ == '__main__':
	main()

