#!/usr/bin/env python

#from __future__ import print_function

import roslib#; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty, signal

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Commands: forward, backward, left, right, stop, quit
TODO: faster, slower (adjust speed by 10%)

CTRL-C to quit
"""

moveBindings = {
        'forward':(1,0,0,0),
        'left':(0,0,0,0.5),
        'right':(0,0,0,-0.5),
        'backward':(-1,0,0,0)
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'faster':(1.1,1),
        'slower':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

# def getKey(): # this is a blocking function
#     tty.setraw(sys.stdin.fileno())
#     select.select([sys.stdin], [], [], 0)
#     print('here1')
#     key = sys.stdin.read(1)
#     print('here2')
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     print('here3')
#     return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def signal_handler(signal, frame):
  sys.exit(0)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('test', disable_signals=True)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    # try:
    print(msg)
    print(vels(speed,turn))
    COMMAND = ''
    import os
    python_file_dir_path = os.path.dirname(os.path.realpath(__file__))
    print("file2motor.py is in: {}".format(python_file_dir_path))
    FILEPATH = python_file_dir_path + "/command.txt"
    while(1):
        try:
        # key = getKey() # blocking function!
            try:
                COMMAND = open(FILEPATH).read().strip()
                if COMMAND in moveBindings.keys():
                    x = moveBindings[COMMAND][0]
                    y = moveBindings[COMMAND][1]
                    z = moveBindings[COMMAND][2]
                    th = moveBindings[COMMAND][3]
                # elif COMMAND in speedBindings.keys():
                #     speed = speed * speedBindings[COMMAND][0]
                #     turn = turn * speedBindings[COMMAND][1]

                #     print(vels(speed,turn))
                #     if (status == 14):
                #         print(msg)
                #     status = (status + 1) % 15
                elif COMMAND == 'stop': # TELL ROBOT TO STOP
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                elif COMMAND == 'quit':
                    break

                twist = Twist()
                twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
                pub.publish(twist)

            except FileNotFoundError:
                print('File not found at', FILEPATH)            
            # elif (key == '\x03'):
            #     break

            # print(x,y,z,th)

        except KeyboardInterrupt:
            # rospy.signal_shutdown('CTRL-C pressed, shutting down')
            break

    # except Exception as e:
    #     print(e)

    # finally:
    #     twist = Twist()
    #     twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    #     pub.publish(twist)

    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
