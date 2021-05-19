#include<iostream>
#include<ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include<math.h>
#include "object_detection/points_objects.h"
#include<string>
#include <vector>
#include <chrono>
#include <thread>

int LANE_DRIVING = 0;
int PASSING = 1;
int STOP = 2;
int MOVING_LEFT = 3;
int STOP_MOVING_RIGHT = 4;
int MOVING_RIGHT = 5;

static std::map<int, std::string>task_names{
     { LANE_DRIVING,"Lane driving"},
     { PASSING, "Passing cars" },
     { STOP,"Stop moving"},
     { MOVING_LEFT, "Moving to left lane"},
     { STOP_MOVING_RIGHT, " Moving aling"},
     { MOVING_RIGHT , "Returning right "}
};


class Task{
    public:
        std::string name;
        int ID;

        Task(int task_identifier){
            this->ID = task_identifier;
            this->name = task_names[this->ID];
        }
};


class Master{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber distance_center;
        ros::Subscriber on_off_sub;
        ros::Subscriber object_detection;
        ros::Publisher angle_pub;
        ros::Publisher speed_pub;

        std_msgs::Int16 angle_message;
        std_msgs::Int16 speed_message;

        std::vector<geometry_msgs::Point> found_objects;
        int num_found_objects;
        int dist_now;
        int angle_now;
        int angle_last;
        float kp_angle;
        float kd_angle;
        float error_angle;
        int u_angle;
        int angle_pd;
        int speed_pid;
        float kp_speed;
        int u_speed;
        std::vector<Task> task_pile; 
        int count_pass;
        std::chrono::steady_clock::time_point start;
        std::chrono::steady_clock::time_point end;
   
    public:
        //Constructor
        Master(Task task){
            //ros::Subscriber angle_now = nh.subscribe("/angle_line_now",1,angle_nowCallback);
            distance_center = nh_.subscribe("/distance_center_line",1, &Master::dist_center_clbk, this);
            object_detection = nh_.subscribe("/objects_points", 1, &Master::object_detec_clbk, this);
            angle_pub = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/steering",1000);
            speed_pub = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/speed",1000);
            num_found_objects = 0;
            dist_now = 0;
            angle_now = 0;
            angle_last = 0;
            kp_angle = 0.825;
            kd_angle = 0.0297;
            error_angle = 0.0;
            u_angle = 0;
            speed_pid = 0;
            kp_speed = 1.2645;
            u_speed = 0;
            count_pass = 0;
            this->add_task(task);
        }

        void dist_center_clbk(const std_msgs::Int16& dis_now_center){
            dist_now = static_cast<int>(dis_now_center.data);
            this->run();
            this->publish_policies();
            //ROS_INFO("Dist center %d", dist_now);
        }

        void object_detec_clbk(const object_detection::points_objects::ConstPtr& msg) {
            found_objects.clear();
            //ROS_INFO("%d", msg->another_field);
            for(auto point :  msg->points){
                found_objects.push_back(point);
            //ROS_INFO("first point: x=%.2f, y=%.2f", point.x, point.y);
            } 
            num_found_objects = found_objects.size();
        }

        void add_task(Task task){
            this->task_pile.push_back(task);
        }

        void remove_task(){
            if (this->task_pile.size() > 1)
                this->task_pile.pop_back();
        }
        Task get_current_task(void){
            return this->task_pile.back();
        }

        void task_assigner(void){
            // Get the current task   
            Task current_task = get_current_task();
            // Checks the number of detected obstacles.
            if (this->num_found_objects > 0){
                                
                //Checks each obstacle info
                for (auto obstacle : this->found_objects){
                    // Obstacle in front detected while driving
                    if (count_pass <= 0 && (obstacle.y <= 360.0 && obstacle.y >= 270.0)){

                       if (current_task.ID == LANE_DRIVING){
                            // Adds following routine
                            this->add_task(Task(STOP));
                            this->add_task(Task(STOP_MOVING_RIGHT));
                            this->add_task(Task(MOVING_RIGHT));
                            this->add_task(Task(MOVING_LEFT));
                            this->add_task(Task(PASSING));
                            count_pass++;
                            start =  std::chrono::steady_clock::now();
                            break;
                       }
                    }
                }
            }
        }
        
        void task_solver(void){
            Task current_task = this->get_current_task();
            ROS_INFO_STREAM("[Current task]: " <<current_task.name);
            if (current_task.ID == LANE_DRIVING){
                on_lane();
            }
            else if (current_task.ID == PASSING){
                end =  std::chrono::steady_clock::now();
                if (this->num_found_objects == 0){
                    on_lane();
                }
                else{
                    for (auto obstacle : this->found_objects){
                        if( (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > 4750) && (obstacle.y == 0.0 || (obstacle.y <= 360.0 && obstacle.y >= 300.0))){
                            stop_car();
                            this->remove_task();
                            break;
                        }
                        else{
                            on_lane();
                        }
                        found_objects.clear();
                    }
                }
            }
            else if(current_task.ID == MOVING_LEFT){
                angle_pd = 0;
                speed_pid = 150;
                for (auto obstacle : this->found_objects){
                        if((obstacle.y <= 315.0) && (obstacle.y >= 200.0)){
                            speed_pid = 0;
                            this->remove_task();
                            break;
                        }
                        found_objects.clear();
                    }
                           
            }
            else if(current_task.ID == MOVING_RIGHT){
                angle_pd = 180;
                speed_pid = 50;

                for (auto obstacle : this->found_objects){
                        if( obstacle.x <= 21.0 && (obstacle.y <= 345.0 && obstacle.y >= 180.0)){
                            speed_pid = 0;
                            this->remove_task();
                            start =  std::chrono::steady_clock::now();
                            break;
                        }
                        found_objects.clear();
                }
            }
            else if(current_task.ID == STOP_MOVING_RIGHT){
                end =  std::chrono::steady_clock::now();
                speed_pid = -100;
                angle_pd = 0;
                if(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > 1850){
                    speed_pid = 0;
                    angle_pd = 90;
                    this->remove_task();
                }
            }
            else if(current_task.ID == STOP){
                speed_pid = 0;
            }

        }

        void run(void){
            this->task_assigner();
            this->task_solver();
        }

        void on_lane(void){
            u_angle = static_cast<int>(kp_angle*static_cast<float>(dist_now) + kd_angle*static_cast<float>(dist_now - angle_last));
            angle_pd = 90 + u_angle;
            if(angle_pd<= 45)
	            angle_pd = 45;
            else if(angle_pd >= 135)
	            angle_pd = 135;
            u_speed = static_cast<int>(kp_speed * dist_now);
            speed_pid = - 435 + abs(u_speed);
            if(speed_pid < - 435)
	            speed_pid = - 435;
            else if(speed_pid > 0)
	            speed_pid = 0;
            angle_last = angle_pd;
        }

        void stop_car(void){
            u_angle = static_cast<int>(kp_angle * static_cast<float>(dist_now) + kd_angle * static_cast<float>(dist_now - angle_last));
            angle_pd = 90 + u_angle;
            if(angle_pd<= 45)
	            angle_pd = 45;
            else if(angle_pd >= 135)
	            angle_pd = 135;
            angle_last = angle_pd;
            while (speed_pid < 0)
            {
                speed_pid += 5;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }   
            
        }
        void publish_policies(){
            angle_message.data = angle_pd;
            angle_pub.publish(angle_message);
            speed_message.data = speed_pid;
            speed_pub.publish(speed_message);
        }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "Master");
    Master *control = new  Master(Task(LANE_DRIVING));
    ros::spin();
    return 0;
}