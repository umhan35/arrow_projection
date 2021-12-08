#pragma once

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class PathProjection {
public:
    PathProjection();

    void spin();

private:
    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener transformListener;
    void wait_transform();

    ros::Subscriber global_nav_plan_sub;
    void global_nav_plan_callback(const nav_msgs::Path &global_nav_plan_path_msg);

    ros::Publisher pub;
};