#!/usr/bin/env python3


import rospy

from std_msgs.msg import String

import sys, select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty



def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('keyboard_command')
    pub = rospy.Publisher('keyboard_command', String, queue_size=1)

    try:

        while not rospy.is_shutdown():
            key = getKey(settings)
            rospy.loginfo(key)
            if key != '':
                pub.publish(key)
            if (key == '\x03'):
                break

    except Exception as e:
        print(e)

    finally:
        restoreTerminalSettings(settings)
