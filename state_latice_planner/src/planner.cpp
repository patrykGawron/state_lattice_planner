#include <iostream>
#include <ros/ros.h>

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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle handle;
    Orientation orie = N;

    std::cout << "Hello World\n";
    std::cout << --orie << "\n";
    return 0;
}