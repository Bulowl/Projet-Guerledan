#!/usr/bin/env python
# coding=utf-8

# Author : Maël LE GALLIC
# Date : 06/06/2018

import rospy
from std_msgs.msg import Float64
from nmea_msgs.msg import Sentence

def checksum(msg):
    return "{0:0{1}X}".format(reduce((lambda a,b : a^b), map(ord, msg)),2)

class FakeRudder:
    def __init__(self):
        rospy.Subscriber('helm_angle_cmd', Float64, self.cb_helm_angle_cmd, queue_size=10000)
        self.pub_nmea = rospy.Publisher('nmea_sentence', Sentence, queue_size=10000)
        self.rate = rospy.Rate(rospy.get_param('~rate', 3))
        self.cur_helm_angle_cmd = None

    def cb_helm_angle_cmd(self, helm_angle_cmd_msg):
        self.cur_helm_angle_cmd = helm_angle_cmd_msg

    def sendIIRSA(self):
        if self.cur_helm_angle_cmd == None:
            return
        singlerudder = -self.cur_helm_angle_cmd.data
        msg_comp = "IIRSA," + str(singlerudder) + ",A,,,"
        self.sendSentence(msg_comp)

    def sendSentence(self, msg):
        sentence = Sentence()
        sentence.header.stamp = rospy.Time.now()
        sentence.sentence = "$" + msg + "*" + checksum(msg)
        self.pub_nmea.publish(sentence)

    def run(self):
        while not rospy.is_shutdown():
            self.sendIIRSA()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fake_rudder')
    FakeRudder().run()
