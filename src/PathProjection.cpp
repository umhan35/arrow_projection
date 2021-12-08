#include "arrow_projection/PathProjection.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const auto GLOBAL_NAV_PATH_FRAME_ID = "odom";
const auto FRAME_ID = "map";

const auto GLOBAL_PLAN_TOPIC = "/move_base/TrajectoryPlannerROS/global_plan";

const auto MARKER_LIFE_TIME = 10;

PathProjection::PathProjection() : transformListener(tfBuffer) {
    global_nav_plan_sub = n.subscribe(GLOBAL_PLAN_TOPIC, 1, &PathProjection::global_nav_plan_callback, this);
    pub = n.advertise<visualization_msgs::MarkerArray>( "/projector/nav/path", 0 );
}

visualization_msgs::Marker create_arrow_marker(int id, geometry_msgs::Point from, geometry_msgs::Point to) {
    visualization_msgs::Marker m;
    m.lifetime = ros::Duration(MARKER_LIFE_TIME);
    m.id = id;
    m.type = visualization_msgs::Marker::ARROW;
    m.header.frame_id = FRAME_ID;

    m.points.emplace_back(from);
    m.points.emplace_back(to);

    // NOT USED: modified rviz source code here: https://github.com/uml-robotics/rviz/commit/b7edac0598838885630088dad0b6e402523be0a4
    m.scale.x = 0.06; // shaft_diameter
    m.scale.y = 0.08; //
    m.scale.z = 0.08; // head_diameter

    // magenta
    m.color.r = 1;
    m.color.g = 0;
    m.color.b = 1;
    m.color.a = 1;

    m.action = visualization_msgs::Marker::ADD;

    return m;
}

visualization_msgs::Marker create_destination_sphere_marker(int id, geometry_msgs::Pose p) {
    visualization_msgs::Marker m;
    m.lifetime = ros::Duration(MARKER_LIFE_TIME);
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.header.frame_id = FRAME_ID;

    m.pose = p;

    m.scale.x = 0.12;
    m.scale.y = 0.12;
    m.scale.z = 0.12;

    // green
    m.color.r = 0;
    m.color.g = 1;
    m.color.b = 0;
    m.color.a = 1;

    m.action = visualization_msgs::Marker::ADD;

    return m;
}

std::vector<geometry_msgs::Point>
get_sparse_points(const std::vector<geometry_msgs::PoseStamped> &poses, const double DISTANCE_BETWEEN = 0.1, const double DESTINATION_DIAMETER = 0.12) {

    std::vector<geometry_msgs::Point> points;

    // from destination to starting point

    int curr_i = poses.size() - 1;
    do {
        auto curr_p = poses.at(curr_i).pose.position;

        points.emplace_back(curr_p);
        ROS_DEBUG("Added %d", curr_i);

        int prev_i = curr_i;
        try {
            // check distance between previous to current
            double distance;
            do {
                prev_i--;
                auto prev_p = poses.at(prev_i).pose.position;

                distance = sqrt(pow(curr_p.x - prev_p.x, 2) + pow(curr_p.y - prev_p.y, 2));
            } while (distance < DISTANCE_BETWEEN || (curr_i == poses.size() - 1 && distance < DISTANCE_BETWEEN + DESTINATION_DIAMETER));

            ROS_DEBUG("  Distance (%d-%d): %.2f)", prev_i, curr_i, distance);

        } catch (const std::out_of_range &_) {
            ROS_DEBUG("Finished. Caught out of range exception: %s", _.what());
        }

        curr_i = prev_i;
    } while (curr_i > 0);

    return points;
}

void PathProjection::global_nav_plan_callback(const nav_msgs::Path & global_nav_plan_path_msg) {
    ROS_DEBUG_STREAM("Received path_projection msg from " << GLOBAL_PLAN_TOPIC);

    // convert from odom pose to map pose (odom changes as roscore restarts)
    std::vector<geometry_msgs::PoseStamped> poses_in_map_frame;
    for (auto &p : global_nav_plan_path_msg.poses) {
        poses_in_map_frame.push_back(tfBuffer.transform(p, FRAME_ID));
    }

    visualization_msgs::MarkerArray markers;

    // destination sphere
    markers.markers.push_back(create_destination_sphere_marker(0, poses_in_map_frame.back().pose));

    // arrows
    auto points_of_arrows = get_sparse_points(poses_in_map_frame);
    for (int i = 0; i + 1 < points_of_arrows.size(); i += 2) {
        auto arrow = create_arrow_marker(i / 2 + 1, points_of_arrows.at(i + 1), points_of_arrows.at(i));
        markers.markers.push_back(arrow);
    }

    // publish
    pub.publish(markers);
    ROS_DEBUG("Published");
}

void PathProjection::wait_transform() {
    ROS_INFO("Checking transform from %s to %s...", GLOBAL_NAV_PATH_FRAME_ID, FRAME_ID);
    const auto INTERVAL = 5;
    while( ! tfBuffer.canTransform(FRAME_ID, GLOBAL_NAV_PATH_FRAME_ID, ros::Time(0), ros::Duration(INTERVAL))) {
        ROS_WARN("Transform from %s to %s is not available after %d seconds. Retrying...", GLOBAL_NAV_PATH_FRAME_ID, FRAME_ID, INTERVAL);
    };
    ROS_INFO("Transform from %s to %s is available now.", GLOBAL_NAV_PATH_FRAME_ID, FRAME_ID);
}

void PathProjection::spin() {
    wait_transform();
    ROS_INFO("PathProjection node ready");

    ros::spin();
}
