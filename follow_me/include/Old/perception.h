#pragma once

#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#define detection_threshold 0.2 //threshold for motion detection
#define dynamic_threshold 75 //to decide if a cluster is static or dynamic

//threshold for clustering
#define cluster_threshold 0.2

//used for detection of leg
#define leg_size_min 0.05
#define leg_size_max 0.25
#define legs_distance_min 0.1
#define legs_distance_max 0.7

//used for uncertainty of leg
#define uncertainty_min_leg 0.5
#define uncertainty_max_leg 1
#define uncertainty_inc_leg 0.05

using namespace std;

class datmo 
{

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_datmo;
    ros::Publisher pub_datmo_marker;

    // to store, process and display both laserdata
    bool init_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    bool init_robot;
    bool stored_background;
    float background[1000];
    bool dynamic[1000];
    bool current_robot_moving;
    bool previous_robot_moving;

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000];// to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic

    //to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    //to perform detection of a moving person and store it
    int nb_persons_detected;
    geometry_msgs::Point person_detected[1000];
    bool person_dynamic[1000];
    geometry_msgs::Point moving_person_detected;//to store the coordinates of the moving person that we are tracking

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

public:

datmo(); 

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update();

// DETECT MOTION FOR BOTH LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background();
void reset_motion();
void detect_motion();

// CLUSTERING FOR LASER DATA
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering();

// DETECTION OF PERSONS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_legs();
void detect_persons();
void detect_moving_persons();

// CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void robot_movingCallback(const std_msgs::Bool::ConstPtr& state);

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb);

// Draw the field of view and other references
void populateMarkerReference();
void populateMarkerTopic();

};

#endif