//
// Created by LHospital
//

#include "../include/PIDController/PID_Controller.h"

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
// #include <Controller/master_control.h>
#include <libconfig.h++>

using namespace libconfig;


class SubscribeAndPublish  
{  
public:  
    SubscribeAndPublish(double kp, double kd, double ki) {
        pid_ = new PID_Controller(kp, kd, ki);
        is_stop_ = 0;
        count = 0;
        //Topic you want to publish  
        pub_servo_ = n_.advertise<std_msgs::Int32>("servo", 1000);  
        pub_speed_ = n_.advertise<std_msgs::Int32>("speed", 1000);

        //Topic1 you want to subscribe  
        sub_ = n_.subscribe("offset_from_lanecenter", 10, &SubscribeAndPublish::callback1, this); 
        //Topic2 you want to subscribe  
        sub2_ = n_.subscribe("chatter2", 10, &SubscribeAndPublish::callback2, this);
    }

    ~SubscribeAndPublish() {
        delete pid_;
    }

    void setPID(double kp, double kd, double ki) {
        if(kp == 0 && kd == 0 && ki == 0) {
            ROS_WARN_STREAM("PID Parameter is all zero, please check your parameter!");
        }
        pid_ -> setPID(kp, kd, ki);
    }

    void callback1(const std_msgs::Float32::ConstPtr& msg1) {  
        ROS_INFO("Steering Angle: [%f]", msg1 -> data); 
        double angle = pid_ -> compute(msg1 -> data);
        int servo_ang = (int32_t)angle;
        int speed = 1;
        // 如果is_stop_ == 0，则让小车停止行动
        if(is_stop_ == 1) {
            servo_ang = 0;
            speed = 0;
        }
        servo_.data = (int32_t)servo_ang;
        speed_.data = (int32_t)speed;
        pub_servo_.publish(servo_);
        pub_speed_.publish(speed_);
        ROS_INFO("angle: %d, speed: %d", (int32_t)servo_ang, (int32_t)speed); 
        ++count; 
    }  
    // TODO: 障碍物识别、交通灯识别的预留回调函数
    void callback2(const std_msgs::Bool::ConstPtr& msg2){}

private:  
    ros::NodeHandle n_;   
    ros::Publisher pub_servo_;
    ros::Publisher pub_speed_;  
    ros::Subscriber sub_;
    ros::Subscriber sub2_;
    // TODO: 将这个msg换掉，添加speed信息 
    // Controller::master_control ctrl_msg_;
    std_msgs::Int32 servo_;
    std_msgs::Int32 speed_;
    int count; 

    // 是否停止的flag
    int is_stop_;

    // PID控制器
    PID_Controller *pid_;

};//End of class SubscribeAndPublish  

int main(int argc, char **argv) {  
    //Initiate ROS  
    ros::init(argc, argv, "contorl_component");  

    // 读取参数
    Config conf;
    try {
        conf.readFile("src/Controller/conf/PID_parameters.conf");
    } catch(const FileIOException &fioex) {
        std::cout << "Error to READ file: " << "src/Controller/conf/PID_parameters.conf" << std::endl;
        // return(EXIT_FAILURE);
    } catch(const ParseException &pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
        // return(EXIT_FAILURE);
    }
    double kp, kd, ki;
    kp = conf.lookup("kp");
    kd = conf.lookup("kd");
    ki = conf.lookup("ki");

    // Create an object of class SubscribeAndPublish that will take care of everything  
    SubscribeAndPublish sap(kp, kd, ki);  
    //ros::spin();
    ros::MultiThreadedSpinner s(2);  //多线程
    ros::spin(s);

    return 0;  
}  
