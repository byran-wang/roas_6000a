#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include <stdio.h>
#include <termios.h>
#include <tf/transform_listener.h>
#include "std_msgs/Int16.h"

class AreaFinder
{
public:
    AreaFinder():_rate(10),_ax(3.5), _bx(6.5), _abcy(-5.0)
    {
        _area_pub = _nh.advertise<std_msgs::Int16>("area", 1);
    }

    void run();
private:
    ros::NodeHandle _nh;
    ros::Publisher _area_pub;
    tf::TransformListener  _tf_listener;
    ros::Rate _rate;
    const double _ax; // 3.5
    const double _bx; // 7.5
    const double _abcy; // -4.8
};

void AreaFinder::run() {
    while(_nh.ok()){
        tf::StampedTransform robot_M; // robot relative to map
        try{
            _tf_listener.lookupTransform("map", "base_link", ros::Time(0), robot_M);
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.02).sleep();
            continue;
        }

        const double x = robot_M.getOrigin().getX();
        const double y = robot_M.getOrigin().getY();

        ROS_ERROR("robot x: %f", x);
        ROS_ERROR("robot y: %f", y);
        std_msgs::Int16 msg;

        if(y < _abcy)
            msg.data = 3; //D        
        else if(x < _ax)
            msg.data = 0; //A
        
        else if(x < _bx)
            msg.data = 1; //B

        else 
            msg.data = 2; //C

        _area_pub.publish(msg);
        _rate.sleep();
    

    }

    return;
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "identify_area");
    AreaFinder af;
    af.run();
    return 0;
}

