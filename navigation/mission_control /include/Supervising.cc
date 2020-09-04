#include "Supervising.h"

using std::cout;
using std::endl;

Supervising::Supervising(){
    danger              = false;
    route_outside       = false;
    route_map           = false;
    predict_outside     = false;
    predict_map         = false;
    buffer              = false;
    back_safe_left      = false;
    back_safe_right     = false;
    front_safe          = false;
    revolute_safe       = false;
    danger_assist       = false;

    limit_danger_radius     = 0.5;
    min_danger_longth       = 0.35;
    min_danger_width        = 0.25;
    limit_route_radius      = 5;
    limit_predict_radius    = 10;
    buffer_radius           = 2;
    back_safe_radius        = 1;
    front_safe_radius       = 1;
    revolute_safe_radius    = 0.5;
    danger_assist_radius    = 1;

    attempt_limit       = 4;
    attempt_times       = 0;
    traceback_index     = 0;

    angle_offset_       = 0.2;
    revolute_angle_     = PI/8;
    
    Ka = 1.0/3;
    Kb = tan(PI/18);

    narrow_danger       = false;
    narrow_buffer       = false;
    AUTO_mission_state_ = true;
    danger_obstacle_state_ = false;
    
    narrow_danger_longth = 0.3;
    narrow_danger_width = 0.2;
    narrow_buffer_radius = 0.4;

    traceback_route.header.frame_id = "/map";

}

Supervising::~Supervising(){
    printf("Supervising is closed\n");
}

void Supervising::AutoObstaclePercept(sensor_msgs::PointCloud obstacle_in_base, geometry_msgs::Twist Input_cmd){
    
    int danger_num          = 0;
    int route_outside_num   = 0;
    int route_map_num       = 0;
    int predict_outside_num = 0;
    int predict_map_num     = 0;
    int buffer_num          = 0;
    int front_safe_num      = 0;
    int first_quadrant_num  = 0;
    int fourth_quadrant_num = 0;
    int revolute_safe_num   = 0;
    int danger_assist_num   = 0;
    int back_safe_left_num = 0;
    int back_safe_right_num = 0;
    
    for(int Index = 0; Index < obstacle_in_base.points.size(); ++Index)
    {           
        double point_distance = hypot(obstacle_in_base.points[Index].x,obstacle_in_base.points[Index].y);

        double front_add_x_y = Kb*obstacle_in_base.points[Index].x + obstacle_in_base.points[Index].y + 0.3;
        double front_subtract_x_y = Kb*obstacle_in_base.points[Index].x - obstacle_in_base.points[Index].y + 0.3;

        double back_add_x_y = Kb*obstacle_in_base.points[Index].x + obstacle_in_base.points[Index].y - 0.3;
        double back_subtract_x_y = Kb*obstacle_in_base.points[Index].x - obstacle_in_base.points[Index].y - 0.3;

        double back_safe_add_x_y = Kb*obstacle_in_base.points[Index].x + obstacle_in_base.points[Index].y;
        double back_safe_subtract_x_y = Kb*obstacle_in_base.points[Index].x - obstacle_in_base.points[Index].y;


        double theta = atan2(obstacle_in_base.points[Index].y,obstacle_in_base.points[Index].x);

        
        if(Input_cmd.linear.x > 0){
            if(((point_distance <= limit_danger_radius) && ((front_add_x_y >= 0) && (front_subtract_x_y >= 0) && (obstacle_in_base.points[Index].x >=0)))
                                    || ((obstacle_in_base.points[Index].x >= -min_danger_longth) && (obstacle_in_base.points[Index].x <= min_danger_longth) 
                                    && (obstacle_in_base.points[Index].y >= -min_danger_width) && (obstacle_in_base.points[Index].y <= min_danger_width))) {
                danger_num++;
            }
        }else if(Input_cmd.linear.x < 0){
            if(((point_distance <= limit_danger_radius) && ((back_add_x_y <= 0) && (back_subtract_x_y <= 0) && (obstacle_in_base.points[Index].x <=0)))
                                    || ((obstacle_in_base.points[Index].x >= -min_danger_longth) && (obstacle_in_base.points[Index].x <= min_danger_longth) 
                                    && (obstacle_in_base.points[Index].y >= -min_danger_width) && (obstacle_in_base.points[Index].y <= min_danger_width))) {
                danger_num++;
            }
        }else if(Input_cmd.linear.x == 0){
            if((obstacle_in_base.points[Index].x >= -min_danger_longth) && (obstacle_in_base.points[Index].x <= min_danger_longth) 
                                    && (obstacle_in_base.points[Index].y >= -min_danger_width) && (obstacle_in_base.points[Index].y <= min_danger_width)) {
                danger_num++;
            }
        }
        
        if((point_distance <= limit_route_radius) && (point_distance > limit_danger_radius) && (front_add_x_y >= 0) && (front_subtract_x_y >= 0) 
                                                        && (obstacle_in_base.points[Index].x >=0) && (obstacle_in_base.points[Index].z == 1)){
            route_outside_num++;
        } 
        if((point_distance <= limit_route_radius) && (point_distance > limit_danger_radius) && (front_add_x_y >= 0) && (front_subtract_x_y >= 0) 
                                                        && (obstacle_in_base.points[Index].x >=0) && (obstacle_in_base.points[Index].z == 10)){
            route_map_num++;
        } 
        if((point_distance <= limit_predict_radius) && (point_distance > limit_route_radius) && (front_add_x_y >= 0) && (front_subtract_x_y >= 0) 
                                                        && (obstacle_in_base.points[Index].x >=0) && (obstacle_in_base.points[Index].z == 1)){
            predict_outside_num++;
        } 
        
        if((point_distance <= limit_predict_radius) && (point_distance > limit_route_radius) && (front_add_x_y >= 0) && (front_subtract_x_y >= 0) 
                                                        && (obstacle_in_base.points[Index].x >=0) && (obstacle_in_base.points[Index].z == 10)){
            predict_map_num++;
        }
        if((point_distance <= buffer_radius) && (front_add_x_y >= 0) && (front_subtract_x_y >= 0) && (obstacle_in_base.points[Index].x >=0)){
            buffer_num++;
            if(theta > 0 && theta <PI/2){
                first_quadrant_num++;
            }else if(theta < 0 && theta > -PI/2){
                fourth_quadrant_num++;
            }
        }

        if((point_distance <= front_safe_radius) && (front_add_x_y >= 0) && (front_subtract_x_y >= 0) && (obstacle_in_base.points[Index].x >=0)){
            front_safe_num++;
        }
        if(point_distance <= revolute_safe_radius){
            revolute_safe_num++;
        }
        if((point_distance <= danger_assist_radius) && (((back_add_x_y >= 0) && (front_subtract_x_y <= 0)) || ((front_add_x_y <= 0) && (back_subtract_x_y >= 0)))){
            danger_assist_num++;
        }  
        if((point_distance <= back_safe_radius) && (back_safe_add_x_y >= 0) && (obstacle_in_base.points[Index].x <=0)){
            back_safe_left_num++;
        }
        if((point_distance <= back_safe_radius) && (back_safe_subtract_x_y >= 0) && (obstacle_in_base.points[Index].x <=0)){
            back_safe_right_num++;
        }    
    }    

    if(danger_num > 0) danger = true;
    else if(danger_num <= 0) danger = false;
     
    if(route_outside_num > 0) route_outside = true;
    else if(route_outside_num == 0) route_outside = false;

    if(route_map_num > 0) route_map = true;
    else if(route_map_num == 0) route_map = false;

    if(predict_outside_num > 0) predict_outside = true;
    else if(predict_outside_num == 0) predict_outside = false;

    if(predict_map_num > 0) predict_map = true;
    else if(predict_map_num == 0) predict_map = false;

    if(buffer_num > 2) buffer = true;
    else if(buffer_num <= 2) buffer = false;

    if(front_safe_num > 0) front_safe = true;
    else if(front_safe_num == 0) front_safe = false;

    if(revolute_safe_num > 2) revolute_safe = true;
    else if(revolute_safe_num <= 2) revolute_safe = false;

    if(danger_assist_num > 2) danger_assist = true;
    else if(danger_assist_num <= 2) danger_assist = false;

    if(first_quadrant_num > fourth_quadrant_num) back_turn = 1;
    else if(first_quadrant_num < fourth_quadrant_num) back_turn = 2;
    else if(first_quadrant_num == fourth_quadrant_num) back_turn = 0;
    
    
    // cout << "danger_num :" << danger_num << endl;
    // cout << "route_outside_num :" << route_outside_num << endl;
    // cout << "predict_outside_num :" << predict_outside_num << endl;
    // cout << "buffer_num :" << buffer_num << endl;
    // cout << "front_safe_num :" << front_safe_num << endl;
    // cout << "first_quadrant_num :" << first_quadrant_num << endl;
    // cout << "fourth_quadrant_num :" << fourth_quadrant_num << endl;
    // cout << "revolute_safe_num :" << revolute_safe_num << endl;
    // cout << "danger_assist_num :" << danger_assist_num << endl;

    auto_area_state_[0] = danger_num;
    auto_area_state_[1] = danger_assist_num;
    auto_area_state_[2] = route_outside_num;
    auto_area_state_[3] = predict_outside_num;
    auto_area_state_[4] = buffer_num;
    auto_area_state_[5] = front_safe_num;
    auto_area_state_[6] = revolute_safe_num;

    danger_obstacle_state_ = danger;  

}

void Supervising::GenerateTracebackRoute(geometry_msgs::Point32 global_goal, geometry_msgs::Point32 vehicle_pose_now){
    static geometry_msgs::Point32 last_global_goal;
    static vector<geometry_msgs::Point32> vehicle_pose_1;
    static vector<geometry_msgs::Point32> vehicle_pose_2;

    if(fabs(global_goal.x - last_global_goal.x) > 0.01 || fabs(global_goal.y - last_global_goal.y) > 0.01){
        vehicle_pose_2.assign(vehicle_pose_1.begin(), vehicle_pose_1.end());
        vehicle_pose_1.clear();
    }else if(fabs(global_goal.x - last_global_goal.x) < 0.01 && fabs(global_goal.y - last_global_goal.y) < 0.01){
        if(vehicle_pose_1.size() == 0){
            vehicle_pose_1.push_back(vehicle_pose_now);
        }else if(vehicle_pose_1.size() != 0){
            if(hypot(fabs(vehicle_pose_now.x - vehicle_pose_1.back().x),fabs(vehicle_pose_now.y - vehicle_pose_1.back().y)) < 0.5){
                return;
            }else if(fabs(vehicle_pose_now.x - vehicle_pose_que.back().x) > 0.01 || fabs(vehicle_pose_now.y - vehicle_pose_1.back().y) > 0.01){
                vehicle_pose_1.push_back(vehicle_pose_now);
            }
        }
    }
    last_global_goal = global_goal;
    vehicle_pose_que.clear();
    // vehicle_pose_.insert(vehicle_pose_que.end(), vehicle_pose_3.begin(), vehicle_pose_3.end());
    vehicle_pose_que.insert(vehicle_pose_que.end(), vehicle_pose_2.begin(), vehicle_pose_2.end());
    vehicle_pose_que.insert(vehicle_pose_que.end(), vehicle_pose_1.begin(), vehicle_pose_1.end());
}

int Supervising::TracebackSupervise(vector<geometry_msgs::Point32> Input, geometry_msgs::Point32 vehicle_in_map){
    double reach_goal_distance = 1;
    if(hypot(vehicle_in_map.x-Input.front().x,vehicle_in_map.y-Input.front().y) < reach_goal_distance) {
        attempt_times = 0;
        traceback_index = 12*ROS_RATE;
        vehicle_pose_traceback.clear();
        return static_cast<int>(AUTO::P);
    }else{
        return static_cast<int>(AUTO::TRACEBACK);
    }
}

int Supervising::AutoSuperviseDecision(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Point32 vehicle_in_map,geometry_msgs::Twist Input_cmd,bool plan_state,bool wait_plan_state){

    static int index_;
    static int yaw_index;
    static bool revolute_flag = false;
    static bool revolute_1_flag = false;
    static bool revolute_2_flag = false;
    static vector<float> Input_cmd_linear_queue(4,0);
    static vector<float> Input_cmd_angular_queue(4,0);

    double yaw_now = vehicle_in_map.z;
    limit_danger_radius = 0.5 + Ka*Input_cmd.linear.x * Input_cmd.linear.x;

    Input_cmd_linear_queue.insert(Input_cmd_linear_queue.begin(),Input_cmd.linear.x);
    Input_cmd_linear_queue.pop_back();

    Input_cmd_angular_queue.insert(Input_cmd_angular_queue.begin(),Input_cmd.angular.z);
    Input_cmd_angular_queue.pop_back();

    if((Input_cmd_linear_queue[0] > 0 || Input_cmd_linear_queue[1] > 0)  && (Input_cmd_linear_queue[2] < 0 || Input_cmd_linear_queue[3] < 0)){
        attempt_times++;
    }

    // cout << "===================================" << endl;
    // // cout << "Input_cmd_linear_queue[" << Input_cmd_linear_queue[0] << "," << Input_cmd_linear_queue[1] << "," << Input_cmd_linear_queue[2]<< ","  
    // //                                                                                         << Input_cmd_linear_queue[3] << "]" << endl;
    // // cout << "Input_cmd_angular_queue[" << Input_cmd_angular_queue[0] << "," << Input_cmd_angular_queue[1] << "," << Input_cmd_angular_queue[2]<< ","  
    // //                                                                                         << Input_cmd_angular_queue[3] << "]" << endl;                                                                                        
    // cout << "index_ :" << index_ << endl;
    // cout << "attempt_times :" << attempt_times << endl;
    // cout << "traceback_index :" << traceback_index << endl;

    AutoObstaclePercept(obstacle_in_base,Input_cmd);

    if(attempt_times >= attempt_limit){

        AUTO_mission_state_ = false; //test
        attempt_times = 0;
        traceback_index = 0;
        return static_cast<int>(AUTO::P);

        if(traceback_index <= 10*ROS_RATE){
            if(!wait_plan_state){
                traceback_index++;
                return static_cast<int>(AUTO::P);

            }else if(wait_plan_state){
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::AUTO);
            }

        }else if(traceback_index > 10*ROS_RATE && traceback_index < 12*ROS_RATE){
            if(danger){
                return static_cast<int>(AUTO::P);
            }else{
                AUTO_mission_state_ = false;
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::P);
                // return TracebackSupervise(vehicle_pose_traceback,vehicle_in_map);
            }
        }else if(traceback_index >= 12*ROS_RATE){
            if(!wait_plan_state){
                return static_cast<int>(AUTO::P);               
            }else if(wait_plan_state){
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::AUTO);
            }
        }
    }else if(attempt_times < attempt_limit){

        traceback_route.header.stamp = ros::Time::now();
        traceback_route.header.frame_id = "/map";
        traceback_route.points.assign(vehicle_pose_que.begin(), vehicle_pose_que.end());
        vehicle_pose_traceback.assign(vehicle_pose_que.begin(), vehicle_pose_que.end());

        //****************The car go front************************//
        if(Input_cmd_linear_queue[0] > 0.02){

            index_ = 0;
            yaw_index = 0;
            if(danger){
                return static_cast<int>(AUTO::P);  
            }else if(!plan_state){
                AUTO_mission_state_ = false; //test
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::P);
                if(!buffer){
                    return static_cast<int>(AUTO::D1); 
                }else if(buffer){
                    return static_cast<int>(AUTO::P); 
                }
            }else if(danger_assist){
                return static_cast<int>(AUTO::D2); 
            }else if(route_outside){
                return static_cast<int>(AUTO::D3); 
            }else if(predict_outside){
                return static_cast<int>(AUTO::D4); 
            }else{
                return static_cast<int>(AUTO::AUTO); 
            }
        }
        //****************The car go back************************//
        if(Input_cmd_linear_queue[0] < -0.02){
            index_ = 0;
            yaw_index = 0;    
            revolute_flag = false;
            if(danger){
                return static_cast<int>(AUTO::P); 
            }else if(buffer){
                if(back_turn == 1) return static_cast<int>(AUTO::R1); 
                else if(back_turn == 2) return static_cast<int>(AUTO::R2); 
                else if(back_turn == 0) return static_cast<int>(AUTO::R3); 
                
            }else if(!plan_state){
                AUTO_mission_state_ = false; //test
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::P);
                return static_cast<int>(AUTO::R3); 
                
            }else if((!danger) && (!buffer)){
                return static_cast<int>(AUTO::AUTO); 
            }else{
                return static_cast<int>(AUTO::AUTO); 
            }
        }
        //****************The car stop************************//

        if(fabs(Input_cmd_linear_queue[0]) <= 0.02){
            if(revolute_flag) buffer = true;
            if(danger){
                return static_cast<int>(AUTO::P); 
            }else if(buffer){
                AUTO_mission_state_ = false;
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::P);
                if(!revolute_safe){
                    if(yaw_index == 0) {
                        revolute_pose_.x = vehicle_in_map.x;
                        revolute_pose_.y = vehicle_in_map.y;
                        revolute_pose_.z = vehicle_in_map.z;
                        yaw_index++;
                    }
                    if(index_ < 3){
                        /***************vehicle yaw convert*********************/
                        if(revolute_2_flag){
                            if(yaw_now - revolute_pose_.z <= 0) yaw_now = yaw_now;
                            else if(yaw_now - revolute_pose_.z > 0) yaw_now = yaw_now - 2*PI;
                        }
                        if(revolute_1_flag){
                            if(yaw_now - revolute_pose_.z >= 0) yaw_now = yaw_now;
                            else if(yaw_now - revolute_pose_.z < 0) yaw_now = yaw_now + 2*PI;
                        }
                        if((yaw_now - revolute_pose_.z >= 0) && (revolute_angle_ - yaw_now + revolute_pose_.z > angle_offset_)){
                            revolute_flag = true;
                            revolute_1_flag = true;
                            revolute_2_flag = false; 
                            return static_cast<int>(AUTO::REVOLUTE1);
                        }else if(revolute_angle_ - yaw_now + revolute_pose_.z <= angle_offset_){
                            if(index_ <= 1) {
                                if(back_safe_left){
                                    return static_cast<int>(AUTO::P); 
                                }else if(!back_safe_left){
                                    revolute_pose_.x = vehicle_in_map.x;
                                    revolute_pose_.y = vehicle_in_map.y;
                                    ConvertAnglePositive(revolute_pose_.z,vehicle_in_map);
                                    index_++;
                                    revolute_1_flag = false;
                                    revolute_2_flag = true;
                                    return static_cast<int>(AUTO::REVOLUTE2);
                                }
                            }else{
                                index_++;
                            }
                        }else if((yaw_now - revolute_pose_.z < 0) && (PI/2 + yaw_now - revolute_pose_.z > angle_offset_)){
                            return static_cast<int>(AUTO::REVOLUTE2);
                        }else if(PI/2 + yaw_now - revolute_pose_.z < angle_offset_){
                            if(back_safe_right){
                                return static_cast<int>(AUTO::P); 
                            }else if(!back_safe_right){
                                revolute_pose_.x = vehicle_in_map.x;
                                revolute_pose_.y = vehicle_in_map.y;
                                ConvertAngleNegative(revolute_pose_.z,vehicle_in_map);
                                index_++;
                                revolute_1_flag = true;
                                revolute_2_flag = false; 
                                return static_cast<int>(AUTO::REVOLUTE1);
                            }
                        }
                    }else{
                        if(back_turn == 1) return static_cast<int>(AUTO::R1);
                        else if(back_turn == 2) return static_cast<int>(AUTO::R2);
                        else if(back_turn == 0) return static_cast<int>(AUTO::R3);
                    }
                }else if(revolute_safe){
                    return static_cast<int>(AUTO::P);
                }
            }else if(!plan_state){ 
                AUTO_mission_state_ = false;
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::P);  
                return static_cast<int>(AUTO::D1);               
            }else{
                return static_cast<int>(AUTO::AUTO);
            }           
        }
    }
}

int Supervising::AstarSuperviseDecision(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Point32 vehicle_in_map,geometry_msgs::Twist Input_cmd,bool plan_state,bool wait_plan_state){

    static int index_;
    static int yaw_index;
    static bool revolute_flag = false;
    static bool revolute_1_flag = false;
    static bool revolute_2_flag = false;
    static vector<float> Input_cmd_linear_queue(4,0);
    static vector<float> Input_cmd_angular_queue(4,0);

    double yaw_now = vehicle_in_map.z;
    limit_danger_radius = 0.5 + Ka*Input_cmd.linear.x * Input_cmd.linear.x;

    Input_cmd_linear_queue.insert(Input_cmd_linear_queue.begin(),Input_cmd.linear.x);
    Input_cmd_linear_queue.pop_back();

    Input_cmd_angular_queue.insert(Input_cmd_angular_queue.begin(),Input_cmd.angular.z);
    Input_cmd_angular_queue.pop_back();

    if((Input_cmd_linear_queue[0] > 0 || Input_cmd_linear_queue[1] > 0)  && (Input_cmd_linear_queue[2] < 0 || Input_cmd_linear_queue[3] < 0)){
        attempt_times++;
    }

    // cout << "===================================" << endl;
    // // cout << "Input_cmd_linear_queue[" << Input_cmd_linear_queue[0] << "," << Input_cmd_linear_queue[1] << "," << Input_cmd_linear_queue[2]<< ","  
    // //                                                                                         << Input_cmd_linear_queue[3] << "]" << endl;
    // // cout << "Input_cmd_angular_queue[" << Input_cmd_angular_queue[0] << "," << Input_cmd_angular_queue[1] << "," << Input_cmd_angular_queue[2]<< ","  
    // //                                                                                         << Input_cmd_angular_queue[3] << "]" << endl;                                                                                        
    // cout << "index_ :" << index_ << endl;
    // cout << "attempt_times :" << attempt_times << endl;
    // cout << "traceback_index :" << traceback_index << endl;

    AutoObstaclePercept(obstacle_in_base,Input_cmd);

    if(attempt_times >= attempt_limit){

        narrow_refind_ = true;
        attempt_times = 0;
        traceback_index = 0;
        return static_cast<int>(AUTO::P);

        if(traceback_index <= 10*ROS_RATE){
            if(!wait_plan_state){
                traceback_index++;
                return static_cast<int>(AUTO::P);

            }else if(wait_plan_state){
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::AUTO);
            }

        }else if(traceback_index > 10*ROS_RATE && traceback_index < 12*ROS_RATE){
            if(danger){
                return static_cast<int>(AUTO::P);
            }else{
                narrow_refind_ = true;
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::P);
                // return TracebackSupervise(vehicle_pose_traceback,vehicle_in_map);
            }
        }else if(traceback_index >= 12*ROS_RATE){
            if(!wait_plan_state){
                return static_cast<int>(AUTO::P);               
            }else if(wait_plan_state){
                attempt_times = 0;
                traceback_index = 0;
                return static_cast<int>(AUTO::AUTO);
            }
        }
    }else if(attempt_times < attempt_limit){
        narrow_refind_ = false;
        traceback_route.header.stamp = ros::Time::now();
        traceback_route.header.frame_id = "/map";
        traceback_route.points.assign(vehicle_pose_que.begin(), vehicle_pose_que.end());
        vehicle_pose_traceback.assign(vehicle_pose_que.begin(), vehicle_pose_que.end());

        //****************The car go front************************//
        if(Input_cmd_linear_queue[0] > 0.02){

            index_ = 0;
            yaw_index = 0;
            if(danger){
                return static_cast<int>(AUTO::P);  
            }else if(!plan_state){

                if(!front_safe){
                    return static_cast<int>(AUTO::D1); 
                }else if(front_safe){
                    return static_cast<int>(AUTO::AUTO); 
                }
            }else if(danger_assist){
                return static_cast<int>(AUTO::D2); 
            }else if(route_outside){
                return static_cast<int>(AUTO::D3); 
            }else if(predict_outside){
                return static_cast<int>(AUTO::D4); 
            }else{
                return static_cast<int>(AUTO::AUTO); 
            }
        }
        //****************The car go back************************//
        if(Input_cmd_linear_queue[0] < -0.02){
            index_ = 0;
            yaw_index = 0;    
            revolute_flag = false;
            if(danger){
                return static_cast<int>(AUTO::P); 
            }else if(buffer){
                if(back_turn == 1) return static_cast<int>(AUTO::R1); 
                else if(back_turn == 2) return static_cast<int>(AUTO::R2); 
                else if(back_turn == 0) return static_cast<int>(AUTO::R3); 
                
            }else if(!plan_state){
                return static_cast<int>(AUTO::R3); 
                
            }else if((!danger) && (!buffer)){
                return static_cast<int>(AUTO::AUTO); 
            }else{
                return static_cast<int>(AUTO::AUTO); 
            }
        }
        //****************The car stop************************//

        if(fabs(Input_cmd_linear_queue[0]) <= 0.02){
            if(revolute_flag) buffer = true;
            if(danger){
                return static_cast<int>(AUTO::P); 
            }else if(buffer){
                if(!revolute_safe){
                    if(yaw_index == 0) {
                        revolute_pose_.x = vehicle_in_map.x;
                        revolute_pose_.y = vehicle_in_map.y;
                        revolute_pose_.z = vehicle_in_map.z;
                        yaw_index++;
                    }
                    if(index_ < 3){
                        /***************vehicle yaw convert*********************/
                        if(revolute_2_flag){
                            if(yaw_now - revolute_pose_.z <= 0) yaw_now = yaw_now;
                            else if(yaw_now - revolute_pose_.z > 0) yaw_now = yaw_now - 2*PI;
                        }
                        if(revolute_1_flag){
                            if(yaw_now - revolute_pose_.z >= 0) yaw_now = yaw_now;
                            else if(yaw_now - revolute_pose_.z < 0) yaw_now = yaw_now + 2*PI;
                        }
                        if((yaw_now - revolute_pose_.z >= 0) && (revolute_angle_ - yaw_now + revolute_pose_.z > angle_offset_)){
                            revolute_flag = true;
                            revolute_1_flag = true;
                            revolute_2_flag = false; 
                            return static_cast<int>(AUTO::REVOLUTE1);
                        }else if(revolute_angle_ - yaw_now + revolute_pose_.z <= angle_offset_){
                            if(index_ <= 1) {
                                if(back_safe_left){
                                    return static_cast<int>(AUTO::P); 
                                }else if(!back_safe_left){
                                    revolute_pose_.x = vehicle_in_map.x;
                                    revolute_pose_.y = vehicle_in_map.y;
                                    ConvertAnglePositive(revolute_pose_.z,vehicle_in_map);
                                    index_++;
                                    revolute_1_flag = false;
                                    revolute_2_flag = true;
                                    return static_cast<int>(AUTO::REVOLUTE2);
                                }
                            }else{
                                index_++;
                            }
                        }else if((yaw_now - revolute_pose_.z < 0) && (PI/2 + yaw_now - revolute_pose_.z > angle_offset_)){
                            return static_cast<int>(AUTO::REVOLUTE2);
                        }else if(PI/2 + yaw_now - revolute_pose_.z < angle_offset_){
                            if(back_safe_right){
                                return static_cast<int>(AUTO::P); 
                            }else if(!back_safe_right){
                                revolute_pose_.x = vehicle_in_map.x;
                                revolute_pose_.y = vehicle_in_map.y;
                                ConvertAngleNegative(revolute_pose_.z,vehicle_in_map);
                                index_++;
                                revolute_1_flag = true;
                                revolute_2_flag = false; 
                                return static_cast<int>(AUTO::REVOLUTE1);
                            }
                        }
                    }else{
                        if(back_turn == 1) return static_cast<int>(AUTO::R1);
                        else if(back_turn == 2) return static_cast<int>(AUTO::R2);
                        else if(back_turn == 0) return static_cast<int>(AUTO::R3);
                    }
                }else if(revolute_safe){
                    return static_cast<int>(AUTO::P);
                }
            }else if(!plan_state){  
                if(!front_safe)  
                    return static_cast<int>(AUTO::D1);   
                else return static_cast<int>(AUTO::AUTO);          
            }else{
                return static_cast<int>(AUTO::AUTO);
            }           
        }
    }
}


void Supervising::NarrowObstaclePercept(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Twist Input_cmd){
    
    int narrow_danger_num = 0;
    int narrow_buffer_num = 0;
    int narrow_first_quadrant_num = 0;
    int narrow_fourth_quadrant_num = 0;
    
    for(int Index = 0; Index < obstacle_in_base.points.size(); ++Index)
    {           
        double point_distance = hypot(obstacle_in_base.points[Index].x,obstacle_in_base.points[Index].y);

        double front_add_x_y = Kb*obstacle_in_base.points[Index].x + obstacle_in_base.points[Index].y + 0.3;
        double front_subtract_x_y = Kb*obstacle_in_base.points[Index].x - obstacle_in_base.points[Index].y + 0.3;

        double theta = atan2(obstacle_in_base.points[Index].y,obstacle_in_base.points[Index].x);

        
        if((obstacle_in_base.points[Index].x >= -narrow_danger_longth) && (obstacle_in_base.points[Index].x <= narrow_danger_longth) 
                                && (obstacle_in_base.points[Index].y >= -narrow_danger_width) && (obstacle_in_base.points[Index].y <= narrow_danger_width)){
            narrow_danger_num++;
        }

        if( Input_cmd.linear.x > 0.05)
        if((point_distance <= narrow_buffer_radius) && (front_add_x_y >= 0) && (front_subtract_x_y >= 0) && (obstacle_in_base.points[Index].x >=0)){
            narrow_buffer_num++;
            if(theta > 0 && theta <PI/2){
                narrow_first_quadrant_num++;
            }else if(theta < 0 && theta > -PI/2){
                narrow_fourth_quadrant_num++;
            }
        }
    }    

    if(narrow_danger_num > 2) narrow_danger = true;
    else if(narrow_danger_num <= 2) narrow_danger = false;

    if(narrow_buffer_num > 0) narrow_buffer = true;
    else if(narrow_buffer_num == 0) narrow_buffer = false;

    if(narrow_first_quadrant_num > narrow_fourth_quadrant_num) local_turn = 1;
    else if(narrow_first_quadrant_num < narrow_fourth_quadrant_num) local_turn = 2;
    else local_turn = 0;
       
    // cout << "narrow_danger_num :" << narrow_danger_num << endl;
    // cout << "narrow_first_quadrant_num :" << narrow_first_quadrant_num << endl;
    // cout << "narrow_fourth_quadrant_num :" << narrow_fourth_quadrant_num << endl;

    danger_obstacle_state_ = narrow_danger;

}

bool Supervising::NarrowPlannerRestart(sensor_msgs::PointCloud obstacle_in_base,vector<geometry_msgs::Point32> path_local,nav_msgs::OccupancyGrid Grid,geometry_msgs::Twist Input_cmd) {
    
    for(int i = 0; i < path_local.size(); i++) {
        if(fabs(path_local[i].x) > Grid.info.width*Grid.info.resolution/2 || fabs(path_local[i].y) > Grid.info.height*Grid.info.resolution/2) continue;

        geometry_msgs::Point32 temp_point;
        temp_point.x = path_local[i].x - Grid.info.origin.position.x;
        temp_point.y = path_local[i].y - Grid.info.origin.position.y;

        int row_num = temp_point.x / Grid.info.resolution;
        int col_num = temp_point.y / Grid.info.resolution;

        for(int j = row_num - 3; j <= row_num + 3; j++) {
            if(j < 0 || j > Grid.info.height - 1) continue;
            for(int k = col_num - 3; k <= col_num + 3; k++) {
                if(k < 0 || k > Grid.info.width - 1) continue; 
                int check_index = col_num * Grid.info.width + row_num;
                if(Grid.data[check_index] >= 50) return true;
            }
        }
    }

    NarrowObstaclePercept(obstacle_in_base,Input_cmd);
    if(narrow_buffer) return true;

    return false;
}

int Supervising::NarrowSuperviseDecision(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Twist Input_cmd) {

    NarrowObstaclePercept(obstacle_in_base,Input_cmd);

    if(narrow_danger) return static_cast<int>(NARROW::P);
    if(narrow_buffer) return static_cast<int>(NARROW::P);

    return static_cast<int>(NARROW::D1);

}
