#include <stdlib.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,nav_msgs::Odometry> SyncPolicy;


class ScanParser
{
public:
    ScanParser(): _scanSub(_nh,"/vrep/laser_scan",1),_odomSub(_nh,"/odom",1),_parseSub(SyncPolicy(10),_scanSub,_odomSub)
    {
        _min = -5;
        _dim = 129;
        _delta = 0.078125;

        _parseSub.registerCallback(boost::bind(&ScanParser::parseCallback,this,_1,_2));

        _mapPub = _nh.advertise<nav_msgs::OccupancyGrid>("/grid_map",1);
    }

    void parseCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        //ROS_INFO("Received scan with %lu measurements",scan_msg->ranges.size());
        //ROS_INFO("Received odometry msg at time %fs",odom_msg->header.stamp.toSec());
        nav_msgs::OccupancyGrid map;
        map.header.stamp = ros::Time::now();
        map.header.frame_id = "odom";
        map.info.resolution = _delta;
        map.info.width = _dim;
        map.info.height = _dim;
        map.info.origin = odom_msg->pose.pose;
        for(int i = 0; i < _dim*_dim; i++)
            map.data.push_back(0);

        size_t numRanges = scan_msg->ranges.size();
        double angle_min = scan_msg->angle_min;
        double angle_increment = scan_msg->angle_increment;

        for(int idx = 0; idx < numRanges; idx++)
        {
            if(scan_msg->ranges.at(idx) != 0.0 && scan_msg->ranges.at(idx) <= 5.0)
            {
                double theta = angle_min + idx*angle_increment;
                double ro = scan_msg->ranges.at(idx);

                double x = ro*cos(theta);
                double y = ro*sin(theta);

                double xp = x - _min, yp = y - _min;
                double xt = xp - fmod(xp,_delta), yt = yp - fmod(yp,_delta);
                int i = xt/_delta, j = yt/_delta;

                map.data.at(i + _dim*j) = 100;
            }
        }
        _mapPub.publish(map);

    }

private:
    double _min;
    double _dim;
    double _delta;

    ros::NodeHandle _nh;
    message_filters::Subscriber<sensor_msgs::LaserScan> _scanSub;
    message_filters::Subscriber<nav_msgs::Odometry> _odomSub;
    message_filters::Synchronizer<SyncPolicy> _parseSub;

    ros::Publisher _mapPub;

}; //ScanParser




int main(int argc, char* argv[])
{
    ros::init(argc,argv,"scan_parser");

    ROS_INFO("Starting scan_parser node...");

    ScanParser sp;

    ros::spin();

    return EXIT_SUCCESS;
}



