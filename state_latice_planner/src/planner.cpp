#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

enum Orientation 
{
     N, E, S, W
};

// prefix ++
Orientation& operator++(Orientation& orientation)
{
    orientation = static_cast<Orientation>(orientation + 1);

    if(orientation > W)
        orientation = N;

    return orientation;
}

// postfix ++
Orientation operator++(Orientation& orientation, int inc)
{
    Orientation cpy = orientation;
    ++orientation;
    return cpy;
}

// prefix --
Orientation& operator--(Orientation& orientation)
{
    orientation = static_cast<Orientation>(orientation - 1);

    if(orientation < N)
        orientation = W;
    
    return orientation;
}

// postfix --
Orientation operator--(Orientation& orientation, int dec)
{
    Orientation cpy = orientation;
    --orientation;
    return cpy;
}

struct State
{
    int x = 0, y = 0;
    Orientation orientation = N;
};

double deg2rad(double deg)
{
    return (deg/180.0) * M_PI;
}

double rad2deg(double rad)
{
    return (rad/M_PI) * 180.0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle handle;

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    double r = 1;
    double start_x = 0;
    double start_y = 0;
    double end_x = 1;
    double end_y = 1;
    double steps = 100;
    double increment = abs(end_x - start_x) / steps;
    for(double angle = 0.0; angle < 90; ++angle)
    {
        double center_x = end_x;
        double center_y = start_y;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.header.seq = 0;
        pose.pose.position.x = center_x - cos(deg2rad(angle));
        pose.pose.position.y = sin(deg2rad(angle));
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        path.poses.push_back(pose);
    }

    ros::Publisher path_publisher = handle.advertise<nav_msgs::Path>("path", 1000);
    int a = 0;
    for(int x = 0; x < 1000000; x++)
    {
        path_publisher.publish(path);
    }
    std::cout << "Hello World\n";
    std::cout << deg2rad(90.0) << "\n";
    return 0;
}