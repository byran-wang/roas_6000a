#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include <stdio.h>
#include <termios.h>

namespace ball_following_controller{


double yaw_from_pxie(){
    
}

class BallFollowingController
{
public:
    BallFollowingController();
    void control();

private:
    const double _max_linear_speed;
    const double _max_ratation_speed;
    const double _p_v;
    const double _p_w;
    const double _target_distance;
    const double _target_angle;

    double _current_distance;
    double _current_angle;
    double _previous_distance;
    double _previoud_angle;
}

BallFollowingController::BallFollowingController(): _max_linear_speed(0), _max_ratation_speed(0), _p_v(0), _p_w(0), _target_distance(0), _target_angle(0){}

void BallFollowingController::control(){
    double linear_v = _p_v * (_target_distance - _current_distance);
    double linear_w = _p_w * (_target_angle - _current_angle);


}

}