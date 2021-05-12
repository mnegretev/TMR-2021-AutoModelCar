#!/usr/bin/env python
import rospy
import rospkg
import math
import numpy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest

def main():
    rospy.init_node("test1_start")
    print("Waiting for service /gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Service is now available")
    rospack  = rospkg.RosPack()
    pkg_path = rospack.get_path('autonomos_gazebo_simulation')
    
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_req   = SpawnModelRequest()
    angle = numpy.random.uniform(math.pi-0.3, math.pi+0.3)
    pos_x = numpy.random.uniform(0.4, 0.6) 

    spawn_req.model_name = "AutoModelMini"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x
    spawn_req.initial_pose.position.y = 3.3
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)

    spawn_req.model_name = "AutoModelParked1"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -1
    spawn_req.initial_pose.position.y = 3.64
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = -1
    spawn_req.initial_pose.orientation.w = 0
    spawn_model(spawn_req)

    spawn_req.model_name = "AutoModelParked2"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -1.5
    spawn_req.initial_pose.position.y = 3.64
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = -1
    spawn_req.initial_pose.orientation.w = 0
    spawn_model(spawn_req)

    spawn_req.model_name = "AutoModelParked3"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -2.88
    spawn_req.initial_pose.position.y = 3.64
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = -1
    spawn_req.initial_pose.orientation.w = 0
    spawn_model(spawn_req)

    spawn_req.model_name = "AutoModelParked4"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -3.38
    spawn_req.initial_pose.position.y = 3.64
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = -1
    spawn_req.initial_pose.orientation.w = 0
    spawn_model(spawn_req)


if __name__ == "__main__":
    main()
