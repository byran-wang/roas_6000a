/*This file is adoppted from                                                            */
/*https://docs.ros.org/en/groovy/api/turtlesim/html/teleop__turtle__key_8cpp_source.html*/

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include <stdio.h>
#include <termios.h>


#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_S 0x73

class KeybardController
{
public:
    KeybardController():linear_(0), angular_(0), target_linear_velocity_(1), target_angular_velocity_(1){
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1);
        nh_.param<double>("target_linear_velocity", target_linear_velocity_, 1);
        nh_.param<double>("target_angular_velocity", target_angular_velocity_, 1);

    }
    void KeyLoop();

private:
    ros::NodeHandle nh_;
    double linear_;
    double angular_;
    ros::Publisher vel_pub_;
    double target_linear_velocity_;
    double target_angular_velocity_;

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig){
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keybord");
    KeybardController kc;
    signal(SIGINT, quit);
    kc.KeyLoop();
    return(0);
}

void KeybardController::KeyLoop()
{
    char c;
    bool dirty=false;

    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~(ICANON | ECHO);

    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    puts("use arrow key to move the robot");

    while(1){
        if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
        }

        linear_ = angular_ = 0;
        switch (c)
        {
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            angular_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            angular_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            linear_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            linear_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_S:
            ROS_DEBUG("STOP");
            dirty = true;
            break;
        }

        geometry_msgs::Twist vel;
        vel.linear.x = linear_ * target_linear_velocity_;
        vel.angular.z = angular_ * target_angular_velocity_;

        if(dirty == true){
            vel_pub_.publish(vel);
            dirty = false;
        }

    }



}
