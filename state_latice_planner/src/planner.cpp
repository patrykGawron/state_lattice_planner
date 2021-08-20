#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Int8.h"

class Map
{
    public:
        nav_msgs::OccupancyGrid map_cpy;
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
        {
                ROS_INFO("Got map %d %d", msg->info.width, msg->info.height);
                map_cpy.data = msg->data;
                map_cpy.info = msg->info;
                map_cpy.header = msg->header;
        }
};


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

    float gCost, hCost, fCost;

    bool operator==(State& other)
    {
        return x == other.x && y == other.y && orientation == other.orientation;
    }
};

double deg2rad(double deg)
{
    return (deg/180.0) * M_PI;
}

double rad2deg(double rad)
{
    return (rad/M_PI) * 180.0;
}

struct Point2D
{
    double x, y;
};

nav_msgs::Path draw_cubic_bezier(Point2D p1, Point2D p2, Point2D p3, Point2D p4)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    double dt = 0.01;
    for(double t = 0; t <= 1; t += dt)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.header.seq = 0;

        // Third order Bezier Curve
        pose.pose.position.x = p1.x * (1 - t) * (1 - t) * (1 - t) + 3 * p2.x * t * (1 - t) * (1 - t) + 3 * p3.x * t * t * (1 - t) + p4.x * t * t * t;
        pose.pose.position.y = p1.y * (1 - t) * (1 - t) * (1 - t) + 3 * p2.y * t * (1 - t) * (1 - t) + 3 * p3.y * t * t * (1 - t) + p4.y * t * t * t;

        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        path.poses.push_back(pose);
    }
    return path;
}



float heuristics(State state, State end)
{
    float dist = sqrt(pow(end.x - state.x, 2) + pow(end.y - state.y, 2));

    return dist;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "planner");
    ros::NodeHandle handle;
    ros::Rate rate(2);
    
    std::cout << "State Lattice Planner started!\n";


    Map gmObject;
    ros::Subscriber sub_map = handle.subscribe("map", 10, &Map::map_callback, &gmObject);

    std::vector<std::string> names = {"forward_right", "forward_left", "forward", "backward_right", "backward_left", "backward"};
    std::vector<ros::Publisher> publishers;
    publishers.reserve(6);

    for(int i = 0; i < 6; i++)
    {
        publishers.push_back(handle.advertise<nav_msgs::Path>(names[i], 1000));
    }

    nav_msgs::Path forward_right = draw_cubic_bezier({0+7, 0+3}, {0+7, 1+3}, {0.75+7, 0.75+3}, {1+7, 1+3});
    nav_msgs::Path forward_left = draw_cubic_bezier({0+7, 0+3}, {0+7, 1+3}, {-0.75+7, 0.75+3}, {-1+7, 1+3});
    nav_msgs::Path forward = draw_cubic_bezier({0+7, 0+3}, {0+7, 1+3}, {0+7, 0.5+3}, {0+7, 1+3});
    nav_msgs::Path backward_right = draw_cubic_bezier({0+7, 0+3}, {0+7, -1+3}, {0.75+7, -0.75+3}, {1+7, -1+3});
    nav_msgs::Path backward_left = draw_cubic_bezier({0+7, 0+3}, {0+7, -1+3}, {-0.75+7, -0.75+3}, {-1+7, -1+3});
    nav_msgs::Path backward = draw_cubic_bezier({0+7, 0+3}, {0+7, -1+3}, {0+7, -0.5+3}, {0+7, -1+3});

    
    // SEARCHING
    State end;
    end.x = 19, end.y = 17, end.hCost = 0;
    State start;
    start.x = 7, start.y = 3, start.gCost = 0, start.hCost = heuristics(start, end), start.fCost = 0 + heuristics(start, end);

    std::vector<State> visited;
    std::vector<State> queue;
    std::vector<Point2D> neigh = {{1, 1}, {-1, 1}, {0, 1}, {1, -1}, {-1, -1}, {0, -1}};

    queue.push_back(start);



    while(ros::ok())
    {

        publishers[0].publish(forward_right);
        publishers[1].publish(forward_left);
        publishers[2].publish(forward);
        publishers[3].publish(backward_right);
        publishers[4].publish(backward_left);
        publishers[5].publish(backward);


        // SEARCHING
        
        rate.sleep();
    }



    std::cout << "Planner has finished!\n";

    return 0;
}