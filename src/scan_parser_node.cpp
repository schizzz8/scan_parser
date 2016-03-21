#include <stdlib.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid map;
ros::Publisher map_pub;

double min = -5;
double dim = 33;
double delta = 0.3125;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    size_t numRanges = msg->ranges.size();
    double angle_min = msg->angle_min;
    double angle_increment = msg->angle_increment;


    for(int idx = 0; idx < numRanges; idx++)
    {
        if(msg->ranges.at(idx) != 0.0 && msg->ranges.at(idx) <= 5.0)
        {
            double theta = angle_min + idx*angle_increment;
            double ro = msg->ranges.at(idx);

            double x = ro*cos(theta);
            double y = ro*sin(theta);

            double xp = x - min, yp = y - min;
            double xt = xp - fmod(xp,delta), yt = yp - fmod(yp,delta);
            int i = xt/delta, j = yt/delta;

            map.data.at(i + dim*j) = 100;

        }
    }

    map_pub.publish(map);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"scan_parser");
    ros::NodeHandle nh;

    ros::Subscriber scan_sub = nh.subscribe("/scan",1000,scanCallback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map",1);

    map.info.resolution = delta;
    map.info.width = dim;
    map.info.height = dim;
    for(int i = 0; i < dim*dim; i++)
        map.data.push_back(0);

    geometry_msgs::Pose origin;
    origin.position.x = min,origin.position.y = min,origin.position.z = 0;
    origin.orientation.x = 0,origin.orientation.y = 0,origin.orientation.z = 0,origin.orientation.w = 1;
    map.info.origin = origin;

    ros::spin();

    return EXIT_SUCCESS;
}
