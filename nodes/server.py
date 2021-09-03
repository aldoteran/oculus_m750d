#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from oculus_sonar.cfg import OculusConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {masterMode}, {gamma}, {range_m},
                  {gain}, {vOfSound}, {salinity}, {pingRate}""".format(**config))
    print(config)
    print(level)
    return config

if __name__ == "__main__":
    rospy.init_node("oculus_dynamic_reconfig", anonymous = False)
    srv = Server(OculusConfig, callback)
    rospy.spin()

