#pragma once

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

//localization using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"

using namespace std;

#define angle_resolution 5//in degrees
#define distance_to_travel 1
#define angle_to_travel 10

class localization {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_odometry;
    ros::Publisher pub_localization_marker;
    ros::Publisher pub_localization;
    ros::Subscriber sub_position;

    // to store, process and display laserdata
    bool init_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];
    bool valid[1000];

    //to store the map
    nav_msgs::GetMap::Response resp;
    geometry_msgs::Point min, max;
    float cell_size;
    int width_max;
    int height_max;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    bool init_odom;
    geometry_msgs::Point odom_current;
    float odom_current_orientation;
    geometry_msgs::Point odom_last;
    float odom_last_orientation;

    //to store the initial_position of the mobile robot
    bool init_position;
    bool new_position;
    geometry_msgs::Point initial_position;
    float initial_orientation;

    //to store the predicted and estimated position of the mobile robot
    bool localization_initialized;
    geometry_msgs::Point predicted_position;
    float predicted_orientation;
    geometry_msgs::Point estimated_position;
    float estimated_orientation;
    int score_max, score_total;

    float distance_traveled;
    float previous_distance_traveled;
    float angle_traveled;
    float previous_angle_traveled;

    //to store the hits of the laser in the frame of the map
    geometry_msgs::Point hit[1000];
    bool cell_occupied[1000];
    geometry_msgs::Point position;
    float orientation;

public:

    localization();

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update();

    void initialize_localization();

    void predict_position();

    void estimate_position();

    void find_best_position(float min_x, float max_x, float min_y, float max_y, float min_orientation,
                            float max_orientation);

    int sensor_model(float x, float y, float o);

    int sensor_model_with_valid(float x, float y, float o);

    int compute_score2(float x, float y, float o);

    int cell_value(float x, float y);

//CALLBACK
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

    void odomCallback(const nav_msgs::Odometry::ConstPtr &o);

    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &p);

// Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb);

// GRAPHICAL DISPLAY
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
// Draw the field of view and other references
    void reset_display();

    void display_localization(geometry_msgs::Point position, float orientation);

    void display_markers();

};

#endif
