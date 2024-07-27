#!/usr/bin/python3

import rospy
from std_msgs.msg import Char


def main():
    """
    Main function to initialize the ROS node and handle user input to publish to the /keys topic.
    """
    rospy.init_node("key_mapper")
    pub = rospy.Publisher("/keys", Char, queue_size=1)

    while not rospy.is_shutdown():
        c = Char()
        input_char = input("Send input: ")

        if len(input_char) == 1:
            c.data = ord(input_char)
            pub.publish(c)
        else:
            print("Please input only one character.")


if __name__ == "__main__":
    main()
