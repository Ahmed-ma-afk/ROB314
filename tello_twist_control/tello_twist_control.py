#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('tello_twist_control')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

from time import sleep

if sys.platform == 'winmedium2':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading a letter from the keyboard and Publishing control to Twist!

Please press CTRL-C to quit.
"""

moveBindings = {
        '+x':(1,0,0,0),
        '-x':(-1,0,0,0),
        '+y':(0,1,0,0),
        '-y':(0,-1,0,0),
        '+z':(0,0,1,0),
        '-z':(0,0,-1,0),
        '+b1':(1,0,1,0),
        '-b1':(-1,0,-1,0),
        '+b2':(1,0,-1,0),
        '-b2':(-1,0,1,0)
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for a subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'winmedium2':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'winmedium2':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'winmedium2':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('tello_twist_control')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))

        Big = 6
        medium = 3
        small = 2
        dict = {'A': ['+z']*Big+['+x']*medium+['-z']*Big+['+z']*medium+['-x']*medium,
                'B': ['+z']*Big+['+x']*medium+['-z']*medium+['-x']*medium+['+x']*medium+['-z']*medium+['-x']*medium,
                'K': ['+z']*Big+['-z']*medium+['+b1']*medium+['-b1']*medium+['+b2']*medium,
                'D': ['+z']*Big+['+b2']*medium+['-z']*small+['-b1']*medium,
                'M': ['+z']*Big+['+b2']*medium+['+b1']*medium+['-z']*Big,
                'N': ['+z']*Big+['+b2']*Big+['+z']*Big,
                'O': ['+z']*Big+['+x']*medium+['-z']*Big+['-x']*medium,
                'R': ['+z']*Big+['+x']*medium+['-z']*medium+['-x']*medium +['+b2']*medium,
                'V': ['-b2']*Big+['+b2']*Big+ ['+b1']*Big,
                'W':  ['-b2']*Big+['+b2']*Big+ ['+b1']*Big+['+b2']*Big+['+b1']*Big,
                'X': ['-b2']*Big+['+b2']*medium+['+b1']*medium+['-b1']*Big,
                'Y': ['+z']*medium+['-b2']*medium + ['+b2']*medium +['+b1']*medium,
                'Z': ['-x']*medium +['+b1']*Big + ['-x']*medium,
                '1': ['+z']*Big +['-b1']*small,
                '3': ['+x']*medium +['+z']*medium+ ['-x']*medium+['+x']*medium+['+z']*medium+['-x']*medium,
                '4': ['+z']*Big +['-z']*medium+ ['-x']*medium+['+z']*medium
                }
    
        cmp = 0
        word = input('Enter a letter: ')
        len_word = len(word)
        index_letter = 0

        while(1):
                exit = getKey(settings, key_timeout)
                if (exit == '\x03'):
                    break                  

                if cmp >= len(dict[word[index_letter]]) :
                    x = 1
                    y = 0
                    z = 0
                    th = 0
                    index_letter += 1
                    cmp = 0

                if(index_letter >= len_word):
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    break 

                if cmp < len(dict[word[index_letter]]) :
                    key = dict[word[index_letter]][cmp]

                    if key in moveBindings.keys():
                        x = moveBindings[key][0]
                        y = moveBindings[key][1]
                        z = moveBindings[key][2]
                        th = moveBindings[key][3]

                # Skip updating cmd_vel if key timeout and robot already stopped.
                pub_thread.update(x, y, z, th, speed, turn)
                sleep(0.5) # Apply twist for 0.5 seconds

                cmp+=1

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
