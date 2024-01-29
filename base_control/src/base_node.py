import rospy
from pymavlink import mavutil
import time
from optparse import OptionParser
from geometry_msgs.msg import Quaternion


# 1 	Pitch
# 2 	Roll
# 3 	Throttle
# 4 	Yaw
# 5 	Forward
# 6 	Lateral

class Basic:

    def __init__(self, options) -> None:
        self.pixhawk_port_addr=options.port_addr
        self.mode=(options.auv_mode)               
        self.master = mavutil.mavlink_connection(self.pixhawk_port_addr, baud=115200)
        self.thruster_subs = rospy.Subscriber("/base/cmd_pwm", Quaternion, self.__callback__, queue_size=2)
        self.channel_ary=[1500]*8
        self.master.wait_heartbeat()
        
    def __callback__(self, msg):
        self.channel_ary[2] = msg.x
        self.channel_ary[3] = msg.y
        self.channel_ary[4] = msg.z
        self.channel_ary[5] = msg.w

    def arm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,1, 0, 0, 0, 0, 0, 0)
    
    def disarm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,0, 0, 0, 0, 0, 0, 0)
        print("sent disarm")

    def mode_switch(self):
        if self.mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(self.mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            exit(1)
        mode_id = self.master.mode_mapping()[self.mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        
    def set_rc_channel_pwm(self, id, pwm):
        """ Set RC channel pwm value
        Args:
            id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if id < 1:
            print("Channel does not exist.")
            return

        # We only have 8 channels
        #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
        if id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,                # target_system
                self.master.target_component,             # target_component
                *rc_channel_values)                  # RC channel list, in microseconds.


    def actuate(self):
        self.set_rc_channel_pwm(1, int(self.channel_ary[0]))
        self.set_rc_channel_pwm(2, int(self.channel_ary[1]))
        self.set_rc_channel_pwm(3, int(self.channel_ary[2]))
        self.set_rc_channel_pwm(4, int(self.channel_ary[3]))
        self.set_rc_channel_pwm(5, int(self.channel_ary[4]))
        self.set_rc_channel_pwm(6, int(self.channel_ary[5]))
        self.set_rc_channel_pwm(7, int(self.channel_ary[6]))
        self.set_rc_channel_pwm(8, int(self.channel_ary[7]))

if __name__ == "__main__":
    obj = Basic()
    parser = OptionParser()
    parser.add_option("-p", "--port", dest="port_addr",help="Pass Pixhawk Port Address", metavar="VAR")
    parser.add_option("-m", "--mode", dest="auv_mode",help="Pass Pixhawk Mode", metavar="VAR")
    (options, args) = parser.parse_args()
    

