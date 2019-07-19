#!/usr/bin/env python
from __future__ import print_function

import time
import rospy
import openadk
import pprint

from sensor_msgs.msg import Joy

from openadk.rest import ApiException
from openadk.models.motions_parameter import MotionsParameter
from openadk.models.motions_operation import MotionsOperation


class motionControl(object):
    def __init__(self, _configuration):
        self.api_instance = openadk.MotionsApi(openadk.ApiClient(_configuration))
        self._sub = rospy.Subscriber('joy', Joy, self.joy_cb)

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

    def joy_cb(self, msg):
        if(msg.axes[3]>0):
            rospy.loginfo('[{}] Moving forward'.format(rospy.get_name()))
            motion_controller.act("Forward", 'fast, 3)

        elif(msg.axes[3]<0):
            rospy.loginfo('[{}] Moving backward'.format(rospy.get_name()))
            motion_controller.act("Backward", 'fast', 3)
        
        elif (msg.axes[2]<0):
            rospy.loginfo('[{}] Turning right'.format(rospy.get_name()))
            motion_controller.act("TurnRight", 'normal', 1)
        
        elif (msg.axes[2]>0):
            rospy.loginfo('[{}] Turning left'.format(rospy.get_name()))
            motion_controller.act("TurnLeft", 'normal', 1)
        
        elif(msg.axes[1]>0):
            rospy.loginfo('[{}] Grasping forward'.format(rospy.get_name()))
            motion_controller.act("Forward_g", 'normal', 2)

        elif(msg.axes[1]<0):
            rospy.loginfo('[{}] Grasping backward'.format(rospy.get_name()))
            motion_controller.act("Backward_g", 'normal', 2)
        
        elif (msg.buttons[10] == 1):
            rospy.loginfo('[{}] Grasping left One'.format(rospy.get_name()))
            motion_controller.act("Left1_g", 'slow', 1)

        elif (msg.buttons[8] == 1):
            rospy.loginfo('[{}] Grasping left two'.format(rospy.get_name()))
            motion_controller.act("Left2_g", 'slow', 1)
                    
        elif (msg.buttons[11] == 1):
            rospy.loginfo('[{}] Grasping Right One'.format(rospy.get_name()))
            motion_controller.act("Right1_g", 'slow', 1)

        elif (msg.buttons[12] == 1):
            rospy.loginfo('[{}] Grasping'.format(rospy.get_name()))
            motion_controller.act("Grab_g", 'slow', 1)


        return


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

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('[{}] shutting down...'.format(rospy.get_name()))
