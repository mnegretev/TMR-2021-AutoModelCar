#include<iostream>
#include<ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include<math.h>
#include "object_detection/points_objects.h"
#include<string>
#include <vector>
#include<map>
#include <chrono>
#include <thread>

int LANE_DRIVING = 0;
int FOLLOWING = 1;
int MOVING_LEFT = 2;
int PASSING = 3;
int MOVING_RIGHT = 4;
int MOVING_RIGHT_LANE = 5;

static std::map<int, std::string>task_names{
     { LANE_DRIVING,"Lane driving"},
     { FOLLOWING,"Following"},
     { MOVING_LEFT, "Moving to left lane"},
     { PASSING, "Passing obstacle" },
     { MOVING_RIGHT, "Returning right lane"},
     { MOVING_RIGHT_LANE , "Returning right lane to lane"}
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
        bool passing_enabled;
        int time_long;
        std::vector<Task> task_pile; 
        Task *last_task;
        int count;
        int count_pass;
        int max_waiting_time;
        float dist_to_keep;
        int vel_decreasing_factor;
        int mid_speed;
        std::chrono::steady_clock::time_point start;
        std::chrono::steady_clock::time_point end;
   
    public:
        //Constructor
        Master(Task task, bool PASSING_ENABLED,int MAX_WAIT_TIME, float DIST_TO_KEEP){
            //ros::Subscriber angle_now = nh.subscribe("/angle_line_now",1,angle_nowCallback);
            distance_center = nh_.subscribe("/distance_center_line",1, &Master::dist_center_clbk, this);
            object_detection = nh_.subscribe("/objects_points", 1, &Master::object_detec_clbk, this);
            angle_pub = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/steering",1000);
            speed_pub = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/speed",1000);
            num_found_objects = 0;
            dist_now = 0;
            angle_now = 0;
            angle_last = 0;
            kp_angle = 0.825;//1.025
            kd_angle = 0.0297;
            error_angle = 0.0;
            u_angle = 0;
            speed_pid = 0;
            kp_speed = 2.4462;
            u_speed = 0;
            count = 0;
            count_pass = 0;
            mid_speed = 1035;
            vel_decreasing_factor = -15;
            dist_to_keep = DIST_TO_KEEP;
            max_waiting_time = MAX_WAIT_TIME; 
            passing_enabled = PASSING_ENABLED;
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
            std::map<float, float>points_order;
            //ROS_INFO("%d", msg->another_field);
            for(auto point :  msg->points){
                points_order.insert ( std::pair<float, float>(point.x, point.y) );
            //ROS_INFO("first point: x=%.2f, y=%.2f", point.x, point.y);
            } 
            auto first_point = points_order.begin();
            geometry_msgs::Point point_msg;
            point_msg.x = first_point->first;
            point_msg.y = first_point->second;
            found_objects.push_back(point_msg);
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
                    if ((obstacle.y > 70.0) && (obstacle.y < 110.0) && (obstacle.x < 119.0)){

                       if (current_task.ID == LANE_DRIVING){
                            // Adds following routine
                            this->add_task(Task(FOLLOWING));
                            last_task =  new Task(FOLLOWING);
                            break;
                       }
                    }
                     if ((current_task.ID == FOLLOWING) && (this->count > this->max_waiting_time)){
                            //Resets count
                            count_pass += 1;
                            this->count = 0;
                            this->add_task(Task(MOVING_RIGHT_LANE));
                            this->add_task(Task(MOVING_RIGHT));
                            this->add_task(Task(PASSING));
                            this->add_task(Task(MOVING_LEFT));
                            break;
                        }     
                }
            }
        }
        
        void task_solver(void){
            Task current_task = this->get_current_task();
            ROS_INFO_STREAM("[Current task]: " <<current_task.name);
            if (current_task.ID == LANE_DRIVING){
                for (auto obstacle : this->found_objects){
                    if ((obstacle.y > 55.0 && obstacle.y < 125.0) && (obstacle.x <= 200.0)){
                        mid_speed = 535;
                        break;
                    }
                }          
                on_lane();
            }
            else if (current_task.ID == FOLLOWING){
                if (count_pass > 4){
                    count_pass = 0;
                }
                // If no obstacles detected
                if (this->num_found_objects == 0){
                    on_lane();
                    this->remove_task();
                }
                else{
                    for (auto obstacle : this->found_objects){
                        // No obstacle in front, finish task
                        if (obstacle.y >= 225.0 && obstacle.y <= 315.0 || last_task->ID == MOVING_RIGHT_LANE){
                            on_lane();
                            this->count = 0;
                            this->remove_task();
                            break;
                        }
                        else if(obstacle.x > (this->dist_to_keep + 10.0)){
                            this->count = 0;
                            if(count_pass == 2 || count_pass == 3){
                                on_lane_front_object_cross(obstacle.x);
                            }
                            else{
                                on_lane_front_object(obstacle.x);
                            }
                            break;
                        }
                        else if (obstacle.x < (this->dist_to_keep - 10)){
                            this->count = 0;
                            if(count_pass == 2 || count_pass == 3){
                               on_lane_front_object_cross(obstacle.x);
                            }
                            else{
                                on_lane_front_object(obstacle.x);
                            }
                            break;
                        }
                        else{
                            // Counting
                            if (this->passing_enabled == true){
                                this->count += 1;
                            }
                            //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            on_lane_stop();
                            break;    
                        }
                    }
                }
            }
            else if (current_task.ID == MOVING_LEFT){
                for (auto obstacle : this->found_objects){
                    if ((obstacle.y >= 60.0) && (obstacle.y <= 135.0)){
                        if(count_pass == 4){
                            speed_pid = -250;
                            angle_pd = 170;
                        }
                        else{
                            speed_pid = -250;
                            angle_pd = 160;
                        }
                        break;
                    }
                    else if((obstacle.y >= 0.0 && obstacle.y < 60.0) || (obstacle.y >= 360.0 && obstacle.y <= 315.0)){
                        on_lane();
                        this->remove_task();
                        break;
                    }
                }
            }
            else if (current_task.ID ==  PASSING){
                for (auto obstacle : this->found_objects){
                    if ((obstacle.y >= 270.0) && (obstacle.y <= 345.0)){
                        speed_pid = -300;
                        angle_pd = 20;
                        this->remove_task();
                        start =  std::chrono::steady_clock::now();
                        break;
                    }
                    else {
                        if(count_pass > 2){
                            speed_pid = -200;
                            angle_pd = 65;
                        }
                        else{
                            on_lane();
                        }
                    }
                }
            }
            else if (current_task.ID ==   MOVING_RIGHT){
                time_long = 2500;
                if (count_pass == 4) {
                    time_long = 3500;
                }
                else if(count_pass == 2){
                    time_long = 1700;
                }
                else if(count_pass == 3){
                    time_long = 1850;
                }
                end =  std::chrono::steady_clock::now();
                    if ( (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > time_long ) ){
                        on_lane_right();
                        this->remove_task();
                        start =  std::chrono::steady_clock::now();
                    }
                    else{
                        speed_pid = -200;
                        angle_pd = 20;
                    }
            }
            else if (current_task.ID ==   MOVING_RIGHT_LANE){
                //time_long = 20000;
                /*
                if(count_pass == 3){
                    time_long = 25000;
                }
                */
                 end =  std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() >  5500){
                        on_lane_right(); 
                        this->remove_task();
                        mid_speed = 1035;
                    }
                    else{
                        on_lane_right();
                    }
            }
            last_task =  new Task(current_task.ID);
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
            speed_pid = - mid_speed + abs(u_speed);
            //speed_pid = -200;
            if(speed_pid < - mid_speed)
	            speed_pid = - mid_speed;
            else if(speed_pid > 0)
	            speed_pid = 0;
            angle_last = angle_pd;
        }

        void on_lane_right(void){
            u_angle = static_cast<int>(kp_angle*static_cast<float>(dist_now) + kd_angle*static_cast<float>(dist_now - angle_last));
            angle_pd = 90 + u_angle;
            if(angle_pd<= 0)
	            angle_pd = 0;
            else if(angle_pd >= 180)
	            angle_pd = 180;
            u_speed = static_cast<int>(kp_speed * dist_now);
	        speed_pid = -150;
            angle_last = angle_pd;
        }

         void on_lane_front_object_cross(float obstacle_dist){
            speed_pid =  decrement_speed(obstacle_dist);
            if(speed_pid < - 535)
	            speed_pid = - 535;
            else if(speed_pid >= 250)
	            speed_pid = 250;
            angle_pd = 90;
         }

        void on_lane_front_object(float obstacle_dist){
            u_angle = static_cast<int>(kp_angle * static_cast<float>(dist_now) + kd_angle * static_cast<float>(dist_now - angle_last));
            angle_pd = 90 + u_angle;
            if(angle_pd<= 45)
	            angle_pd = 45;
            else if(angle_pd >= 135)
	            angle_pd = 135;
            speed_pid =  decrement_speed(obstacle_dist);
            //speed_pid = -200;
            if(speed_pid < - 535)
	            speed_pid = - 535;
            else if(speed_pid >= 250)
	            speed_pid = 250;
            angle_last = angle_pd;
        }
        
        int decrement_speed(float obstacle_dist){
            int distance_diff, dist_to_obstacle;
            dist_to_obstacle = static_cast<int>(obstacle_dist);
            distance_diff = dist_to_obstacle  - dist_to_keep;
            return vel_decreasing_factor * distance_diff;
        }

        void on_lane_stop(void){
            u_angle = static_cast<int>(kp_angle*static_cast<float>(dist_now) + kd_angle*static_cast<float>(dist_now - angle_last));
            angle_pd = 90 + u_angle;
            if(angle_pd<= 45)
	            angle_pd = 45;
            else if(angle_pd >= 135)
	            angle_pd = 135;
            speed_pid = 0;
            angle_last = angle_pd;
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
    Master *control = new  Master(Task(LANE_DRIVING), true, 5, 115);
    ros::spin();
    return 0;
}