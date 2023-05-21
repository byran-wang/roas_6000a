#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include <stdio.h>
#include <termios.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

namespace ball_following_controller{


class BallFollowingController
{
public:

    BallFollowingController();
    void control();
    void call_back(const geometry_msgs::Point&);
private:
    const double _max_linear_speed;
    const double _max_ratation_speed;
    const double _target_distance;
    const double _target_angle;

    ros::NodeHandle _nh;
    ros::Rate _rate;
    double _p_v;
    double _p_w;
    double _current_distance;
    double _current_angle;
    // double _previous_distance;
    // double _previoud_angle;

    
    ros::Subscriber _ball_position_sub;
    ros::Publisher _vel_pub;
    
};

BallFollowingController::BallFollowingController(): _max_linear_speed(4), _max_ratation_speed(4), _p_v(0), _p_w(0), _target_distance(100), _target_angle(512/2), _nh(),_rate(10),
                                                    _current_distance(0), _current_angle(0)                                                    
{
    _ball_position_sub = _nh.subscribe("/ball_centroid_and_radius", 1, &BallFollowingController::call_back, this);
    _vel_pub = _nh.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1);

    _nh.param<double>("p_v", _p_v, 0.1);
    _nh.param<double>("p_w", _p_w, 0.01);
}

void BallFollowingController::call_back(const geometry_msgs::Point& msg){
    _current_distance = msg.z;
    _current_angle = msg.x;
}

void BallFollowingController::control(){

    while(_nh.ok()){

        if(_current_distance > 70){
            double v = _p_v * (_target_distance - _current_distance);
            double w = _p_w * (_target_angle - _current_angle);


            geometry_msgs::Twist vel;
            vel.linear.x = v;
            vel.angular.z = w;

            _vel_pub.publish(vel);
        }

        _rate.sleep();
        
    }


}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "ball_following");
    ball_following_controller::BallFollowingController bfc;
    bfc.control();
    return 0;
}