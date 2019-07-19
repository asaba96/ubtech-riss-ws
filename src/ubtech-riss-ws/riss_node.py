#!/usr/bin/env python
from __future__ import print_function 
import rospy

from sensor_msgs.msg import Joy

import time 
import openadk
from openadk.rest import ApiException
import pprint

from openadk.models.motions_parameter import MotionsParameter
from openadk.models.motions_operation import MotionsOperation

ip = 'http://172.26.207.240:9090/v1'


def joy_cb(msg):
    if(msg.axes[3]>0):
        rospy.loginfo('[{}] Moving forward'.format(rospy.get_name()))
        motion_controller.act("Forward", 'slow')

    if(msg.axes[3]<0):
        rospy.loginfo('[{}] Moving backward'.format(rospy.get_name()))
        motion_controller.act("Backward", 'slow')
    return 



class motionControl():
    def __init__(self, _configuration):
        self.api_instance = openadk.MotionsApi(openadk.ApiClient(_configuration)) 

    def act(self, _action, _speed='fast', _repeat=3):
        timestamp = int(time.time())
        motion = MotionsParameter(name=_action, direction='both', speed=_speed, repeat=_repeat)
        body = MotionsOperation(motion=motion, operation='start', timestamp=timestamp)

        try: 
            # Update the motions
            api_response = self.api_instance.put_motions(body)
            rospy.loginfo('[{}] {}'.format(rospy.get_name(), api_response)) 
        except ApiException as e: 
            rospy.loginfo('[{}] Exception when calling MotionsApi->put_motions:{%s}\n'.format(rospy.get_name(), e )) 
            #print("Exception when calling MotionsApi->put_motions: %s\n" % e)
    
    def stop(self):
        self.act('stop')

def sleep(_sec):
    rospy.sleep(_sec)


if __name__ == '__main__':
    try:
        rospy.get_master().getPid()
    except:
        ros.loginfo("ros is offline, exit")

    rospy.init_node('riss_node')
    rospy.loginfo("ros init")
    configuration = openadk.Configuration()
    configuration.host = 'http://172.26.207.240:9090/v1'#ip
    motion_controller = motionControl(configuration)

    _sub = rospy.Subscriber('joy', Joy, joy_cb)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('[{}] shutting down...'.format(rospy.get_name()))
