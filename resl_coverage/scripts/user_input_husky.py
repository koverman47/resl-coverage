#!/usr/bin/env python

import rospy
from pynput.keyboard import Key, KeyCode, Listener
from geometry_msgs.msg import Twist

pub_twist = None
twist = None

def on_press(key):
    global twist
    if key == KeyCode.from_char('p'):
        twist.linear.z = 1.0
    elif key == KeyCode.from_char('l'):
        twist.linear.z = -1.0
    if key == KeyCode.from_char('q'):
        twist.linear.y = 1.0
    elif key == KeyCode.from_char('w'):
        twist.linear.y = -1.0
    if key == KeyCode.from_char('a'):
        twist.linear.x = 1.0
    elif key == KeyCode.from_char('s'):
        twist.linear.x = -1.0

def on_release(key):
    global twist
    if key == KeyCode.from_char('p'):
        twist.linear.z = 0.
    if key == KeyCode.from_char('l'):
        twist.linear.z = 0.
    if key == KeyCode.from_char('q'):
        twist.linear.y = 0.0
    elif key == KeyCode.from_char('w'):
        twist.linear.y = 0.0
    if key == KeyCode.from_char('a'):
        twist.linear.x = 0.0
    elif key == KeyCode.from_char('s'):
        twist.linear.x = 0.0
    if key == Key.esc:
        twist.linear.x = 0.
        twist.linear.y = 0.
        twist.linear.z = 0.
    pass

def main():
    global twist, pub_com
    name = rospy.get_namespace()
    rospy.init_node('user_input')
    twist = Twist()
    pub_twist = rospy.Publisher(name + 'cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()
    while not rospy.is_shutdown():
        pub_twist.publish(twist)
        rate.sleep()
    listener.stop()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
