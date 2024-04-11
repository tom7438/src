#pragma once

#ifndef DATMO_H
#define DATMO_H

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
#define dynamic_threshold 30 //to decide if a cluster is static or dynamic

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

//used for frequency
#define frequency_init 5
#define frequency_max 25

//used for uncertainty associated to the tracked person
#define uncertainty_min 0.4
#define uncertainty_max 1
#define uncertainty_inc 0.05

using namespace std;

class datmo 
{

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_datmo;
    ros::Publisher pub_datmo_marker, pub_motion_marker, pub_clusters_marker,
                   pub_legs_marker, pub_persons_marker, pub_tracked_person_marker;

    ros::Publisher pub_person_position;

    // to store, process and display laserdata
    bool init_laser, new_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    bool init_robot, new_robot;
    bool stored_background;
    float background[1000];
    bool dynamic[1000];
    bool current_robot_moving;
    bool previous_robot_moving;

    //to perform clustering
    int nb_clusters;// number of cluster
    int cluster_start[1000], cluster_end[1000];// to store the index of the start and the end of a cluster. For instance, cluster_start[3] = 8 means that current_scan[8] is the start of cluster 3.
    float cluster_size[1000];// to store the size (ie, the distance in meters between the start of the cluster and the end of the cluster) for each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle point of each cluster
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic. The percentage is an integer between 0 and 100.

    //to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    //to perform detection of persons and store them
    int nb_persons_detected;
    geometry_msgs::Point person_detected[1000];
    int leg_left[1000], leg_right[1000];
    bool person_dynamic[1000];

    //to perform tracking of a person
    bool is_person_tracked;
    bool associated;
    geometry_msgs::Point person_tracked;//to store the coordinates of the person that we are tracking
    int frequency;
    float uncertainty;

    // GRAPHICAL DISPLAY
    // Marker messages are filled by the respective display_x() functions.
    // All marker messages are published to a single marker topic, with different namespaces that can be 
    // selected in Rviz to display all or part of the markers.
    visualization_msgs::Marker marker_motion, marker_clusters, marker_legs, marker_persons, marker_tracked_person; 

public:

    datmo();
    datmo(char *goal_name);

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
    void perform_basic_clustering();
    void perform_advanced_clustering();
    int compute_nb_dynamic(int start, int end);

// DETECTION OF PERSONS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void detect_legs();
    void detect_persons();
    void detect_a_moving_person();

// TRACKING OF A PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void track_a_person();

// CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void robot_movingCallback(const std_msgs::Bool::ConstPtr& state);

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb);

// GRAPHICAL DISPLAY
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
// Draw the field of view and other references
    void display_motion();
    void display_clustering();
    void display_legs();
    void display_persons();
    void display_a_tracked_person();
    void populateMarkerReference();

};

#endif