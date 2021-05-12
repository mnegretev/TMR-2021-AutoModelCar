#!/usr/bin/env python
import rospy
import rospkg
import math
import numpy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest

def main():
    rospy.init_node("test2_start")
    print("Waiting for service /gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Service is now available")
    rospack  = rospkg.RosPack()
    pkg_path = rospack.get_path('autonomos_gazebo_simulation')
    
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_req   = SpawnModelRequest()
    
    angle = numpy.random.uniform(math.pi-0.3, math.pi+0.3)
    pos_x = numpy.random.uniform(-3.8,-1.2) 
    spawn_req.model_name = "AutoModel_Obstacle1"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x
    spawn_req.initial_pose.position.y = 3.3
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)

    angle = numpy.random.uniform(-math.pi/2.0-0.2, -math.pi/2.0+0.2)
    pos_y = numpy.random.uniform(-1,1) 
    spawn_req.model_name = "AutoModel_Obstacle2"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -5.4
    spawn_req.initial_pose.position.y = pos_y
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)

    angle = numpy.random.uniform(0, 3.0*math.pi/2.0)
    pos_x = -2.2 + 1.55*math.cos(angle);
    pos_y =  0.5 + 1.55*math.sin(angle);
    spawn_req.model_name = "AutoModel_Obstacle3"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x
    spawn_req.initial_pose.position.y = pos_y
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin((angle + math.pi/2)/2)
    spawn_req.initial_pose.orientation.w = math.cos((angle + math.pi/2)/2)
    spawn_model(spawn_req)

    angle = numpy.random.uniform(-math.pi/2.0-0.2, -math.pi/2.0+0.2)
    pos_y = numpy.random.uniform(-1.25,-1.5) 
    spawn_req.model_name = "AutoModel_Obstacle4"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = 0.65
    spawn_req.initial_pose.position.y = pos_y
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)
    
if __name__ == "__main__":
    main()
