#!/usr/bin/env python
import rospy
import rospkg
import math
import numpy
from std_msgs.msg import Int16
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetLinkStateRequest
from gazebo_msgs.srv import GetLinkStateResponse

def main():
    rospy.init_node("test2_start")
    print("Waiting for service /gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Service is now available")
    rospack  = rospkg.RosPack()
    pkg_path = rospack.get_path('autonomos_gazebo_simulation')
    
    spawn_model    = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    spawn_req      = SpawnModelRequest()
    get_link_req   = GetLinkStateRequest()

    angle = numpy.random.uniform(math.pi-0.3, math.pi+0.3)
    pos_x = numpy.random.uniform(1.2,3.8) 

    spawn_req.model_name = "AutoModelMini"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x
    spawn_req.initial_pose.position.y = 3.3
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)

    spawn_req.model_name = "StopSign"
    spawn_req.model_xml  = open(pkg_path + '/models/stop_sign_small/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x - 0.2
    spawn_req.initial_pose.position.y = 3.55
    spawn_req.initial_pose.position.z = 0.0
    spawn_req.initial_pose.orientation.z = math.sin(1.5708/2)
    spawn_req.initial_pose.orientation.w = math.cos(1.5708/2)
    spawn_model(spawn_req)
     
    spawn_req.model_name = "AutoModel_Obstacle1"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_obstacle/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x - 2
    spawn_req.initial_pose.position.y = 3.3
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = 1.0
    spawn_req.initial_pose.orientation.w = 0.0
    spawn_model(spawn_req)

    angle = numpy.random.uniform(0, 3.0*math.pi/2.0)
    pos_x = -2.2 + 1.55*math.cos(angle);
    pos_y =  0.5 + 1.55*math.sin(angle);
    spawn_req.model_name = "AutoModel_Obstacle2"
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
    spawn_req.model_name = "AutoModel_Obstacle3"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_static/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = 0.65
    spawn_req.initial_pose.position.y = pos_y
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)

    get_link_req.link_name = 'AutoModel_Obstacle1::base_link'
    initial_speed = -numpy.random.randint(80,120)
    pub_speed = rospy.Publisher("/AutoModel_Obstacle1/manual_control/speed",    Int16, queue_size=1)
    pub_steer = rospy.Publisher("/AutoModel_Obstacle1/manual_control/steering", Int16, queue_size=1)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        link_state = get_link_state(get_link_req)
        obstacle_x = link_state.link_state.pose.position.x
        obstacle_y = link_state.link_state.pose.position.y
        steering = 135 if obstacle_x < -4.18 else 90 - int(100*(3.3 - obstacle_y))
        speed = initial_speed if obstacle_y > 1.8 else 0
        pub_speed.publish(speed)
        pub_steer.publish(steering)
        loop.sleep()
    
if __name__ == "__main__":
    main()
