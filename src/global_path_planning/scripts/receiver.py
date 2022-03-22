#!/usr/bin/env python
from sqlite3 import DatabaseError
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import threading
import socket
import time

class Receiver(object):
    def __init__(self, addr='127.0.0.1', port=23334):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = addr
        self.port = port
        self.sock.settimeout(1.0)
        self.sock.bind((self.addr, self.port))
        self.thread = threading.Thread(target=self.receive_data)
        self.thread.start()
        self.timeout = False
        # publisher
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("/global_path_planning", Path, self.pathCallback)
        rospy.init_node('joy_receiver')
        rospy.spin()

    def receive_data(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                splitted = data.decode("utf-8").split(',')
                goal_x, goal_y = splitted[0], splitted[1]
                robot_x, robot_y = -9999, -9999
                if len(splitted) == 4:
                    robot_x, robot_y = splitted[2], splitted[3]
                goal = PoseStamped()
                goal.pose.position.x = int(goal_x)
                goal.pose.position.y = int(goal_y)
                goal.pose.orientation.x = int(robot_x)
                goal.pose.orientation.y = int(robot_y)
                self.pub.publish(goal)
                self.timeout = False
            except socket.timeout:
                self.timeout = True
            time.sleep(0.01)
    
    def pathCallback(self, msg):
        path_str = ''
        for pose in msg.poses:
            # print(pose.pose.position.x, pose.pose.position.y)
            path_str += str(pose.pose.position.x) + ',' + str(pose.pose.position.y) + ';'
        path_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        path_sock.sendto(bytes(path_str, 'ascii'), ('127.0.0.1', 23333))

if __name__ == '__main__':
    try:
        Receiver()
    except rospy.ROSInterruptException:
        pass