#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import DeleteModel, DeleteModelResponse


class CuriosityMarsRoverArmAndMast(object):
    def __init__(self):
        rospy.loginfo("CuriosityRover Arm And Mast Initialising...")

        # TODO: MoveIt for Arm
        self.publishers_curiosity_d = {}
        self.controller_ns = "curiosity_mars_rover"
        self.controller_command = "command"
        self.controllers_list = [   "mast_p_joint_position_controller",
                                    "mast_02_joint_position_controller",
                                    "mast_cameras_joint_position_controller",
                                    "arm_01_joint_position_controller",
                                    "arm_02_joint_position_controller",
                                    "arm_03_joint_position_controller",
                                    "arm_04_joint_position_controller",
                                    "arm_tools_joint_position_controller"
                                ]

        for controller_name in self.controllers_list:
            topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
            self.publishers_curiosity_d[controller_name] = rospy.Publisher(
                topic_name,
                Float64,
                queue_size=1)

        self.wait_publishers_to_be_ready()
        self.init_publisher_variables()
        self.init_state()

        arm_service_name = "/"+self.controller_ns+"/arm_service"
        self.arm_service = rospy.Service(arm_service_name, DeleteModel, self.arm_service_cb)
        mast_service_name = "/" + self.controller_ns + "/mast_service"
        self.mast_service = rospy.Service(mast_service_name, DeleteModel, self.mast_service_cb)
        rospy.logwarn("CuriosityRover Arm And Mast...READY")

    def arm_service_cb(self, req):
        arm_mode_requested = req.model_name
        response = DeleteModelResponse()
        response.success = self.set_arm_pose(arm_mode_requested)
        if response.success:
            response.status_message = "Executed Arm Mode="+str(arm_mode_requested)
        else:
            response.status_message = "Couldnt Execute Arm Mode=" + str(arm_mode_requested)

        return response

    def mast_service_cb(self, req):
        mast_mode_requested = req.model_name
        response = DeleteModelResponse()
        response.success = self.set_mast_pose(mast_mode_requested)
        if response.success:
            response.status_message = "Executed Mast Mode=" + str(mast_mode_requested)
        else:
            response.status_message = "Couldnt Execute Mast Mode=" + str(mast_mode_requested)

        return response

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        for controller_name, publisher_obj in self.publishers_curiosity_d.iteritems():
            publisher_ready = False
            while not publisher_ready:
                rospy.logwarn("Checking Publisher for ==>"+str(controller_name))
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = (pub_num > 0)
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")

    def init_publisher_variables(self):
        """
        We create variables for more pythonic access access to publishers
        and not need to access any more
        :return:
        """
        # Get the publishers for mast
        self.mast_p = self.publishers_curiosity_d[self.controllers_list[0]]
        self.mast_02 = self.publishers_curiosity_d[self.controllers_list[1]]
        self.mast_cameras = self.publishers_curiosity_d[self.controllers_list[2]]
        # Get the publishers for arm
        self.arm_01 = self.publishers_curiosity_d[self.controllers_list[3]]
        self.arm_02 = self.publishers_curiosity_d[self.controllers_list[4]]
        self.arm_03 = self.publishers_curiosity_d[self.controllers_list[5]]
        self.arm_04 = self.publishers_curiosity_d[self.controllers_list[6]]
        self.arm_tools = self.publishers_curiosity_d[self.controllers_list[7]]

        # Init Messages
        self.mast_p_pos_msg = Float64()
        self.mast_02_pos_msg = Float64()
        self.mast_cameras_pos_msg = Float64()
        self.arm_01_pos_msg = Float64()
        self.arm_02_pos_msg = Float64()
        self.arm_03_pos_msg = Float64()
        self.arm_04_pos_msg = Float64()
        self.arm_tools_pos_msg = Float64()

    def init_state(self):
        self.set_arm_pose("close")
        self.set_mast_pose("close")

    def set_arm_pose(self, mode_name):

        if mode_name == "close" or mode_name == "open":
            if mode_name == "close":
                self.arm_01_pos_msg.data = -1.57
                self.arm_02_pos_msg.data = -0.4
                self.arm_03_pos_msg.data = -1.1
                self.arm_04_pos_msg.data = -1.57
                self.arm_tools_pos_msg.data = -1.57

            if mode_name == "open":
                self.arm_01_pos_msg.data = 0.0
                self.arm_02_pos_msg.data = 0.0
                self.arm_03_pos_msg.data = 0.0
                self.arm_04_pos_msg.data = 0.0
                self.arm_tools_pos_msg.data = 0.0


            self.arm_01.publish(self.arm_01_pos_msg)
            self.arm_02.publish(self.arm_02_pos_msg)
            self.arm_03.publish(self.arm_03_pos_msg)
            self.arm_04.publish(self.arm_04_pos_msg)
            self.arm_tools.publish(self.arm_tools_pos_msg)

            return True
        else:
            rospy.logerr("Requested a Not supported ARM POSE")
            return False

    def set_mast_pose(self, mode_name):

        if mode_name == "close" or mode_name == "open":
            if mode_name == "close":
                self.mast_p_pos_msg.data = 1.57
                self.mast_02_pos_msg.data = -1.57
                self.mast_cameras_pos_msg.data = 0.0

            if mode_name == "open":
                self.mast_p_pos_msg.data = 0.0
                self.mast_02_pos_msg.data = -0.5
                self.mast_cameras_pos_msg.data = 0.0

            self.mast_p.publish(self.mast_p_pos_msg)
            self.mast_02.publish(self.mast_02_pos_msg)
            self.mast_cameras.publish(self.mast_cameras_pos_msg)
            return True
        else:
            rospy.logerr("Requested a Not supported MAST POSE")
            return False


if __name__ == "__main__":
    rospy.init_node("CuriosityRoverArmMast_node")
    curiosity_mars_rover_arm_mast_object = CuriosityMarsRoverArmAndMast()
    rospy.spin()


