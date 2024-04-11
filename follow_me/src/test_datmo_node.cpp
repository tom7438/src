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
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

using namespace std;

class test_datmo
{
private:
    ros::NodeHandle n;

    // communication with detection_node or datmo_node
    ros::Subscriber sub_goal_to_reach;

    // communication with rosbag to get the ideal detected or tracked position
    ros::Subscriber sub_ideal_goal_to_reach;

    ros::Publisher pub_test_datmo_marker;

    geometry_msgs::Point goal_to_reach;
    bool new_goal_to_reach;  // to check if a new /goal_to_reach is available or not
    bool init_goal_to_reach; // to check if a new /goal_to_reach is available or not

    geometry_msgs::Point ideal_goal_to_reach;
    bool new_ideal_goal_to_reach;  // to check if a new /goal_to_reach is available or not
    bool init_ideal_goal_to_reach; // to check if a new /goal_to_reach is available or not

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

public:
    test_datmo()
    {

        // communication with datmo
        sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &test_datmo::goal_to_reachCallback, this);
        sub_ideal_goal_to_reach = n.subscribe("ideal_goal_to_reach", 1, &test_datmo::ideal_goal_to_reachCallback, this);
        pub_test_datmo_marker = n.advertise<visualization_msgs::Marker>("test_datmo_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

        new_goal_to_reach = false;
        init_goal_to_reach = false;

        new_ideal_goal_to_reach = false;
        init_ideal_goal_to_reach = false;

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10); // this node will run at 10hz
        while (ros::ok())
        {
            ros::spinOnce(); // each callback is called once to collect new data: laser + robot_moving
            update();        // processing of data
            r.sleep();       // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update()
    {

        if (init_goal_to_reach && init_ideal_goal_to_reach && (new_goal_to_reach || new_ideal_goal_to_reach))
        {

            new_goal_to_reach = false;
            new_ideal_goal_to_reach = false;
            float dist = distancePoints(goal_to_reach, ideal_goal_to_reach);

            ROS_INFO("goal_to_reach(%f, %f) is located at %f from ideal_goal_to_reach(%f, %f)",
                goal_to_reach.x,
                goal_to_reach.y,
                dist,
                ideal_goal_to_reach.x,
                ideal_goal_to_reach.y);

            if (dist > 0.5)
            {

                ROS_WARN("goal_to_reach is too far from ideal_goal_to_reach");
                nb_pts = 0;

                //the ideal_goal_to_reach is dispayed in green color
                display[nb_pts] = ideal_goal_to_reach;
                colors[nb_pts].r = 0;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;

                //the goal_to_reach is dispayed in red color
                display[nb_pts] = goal_to_reach;
                colors[nb_pts].r = 1;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;


            }
        }

    } // update

    // CALLBACKS
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr &g)
    {
        // process the goal received from detector_node or datmo_node

        new_goal_to_reach = true;
        init_goal_to_reach = true;
        goal_to_reach = *g;
      //  ROS_INFO("new goal_to_reach(%f, %f) published", g->x, g->y);

    }

    void ideal_goal_to_reachCallback(const geometry_msgs::Point::ConstPtr &g)
    {
        // process the goal received from rosbag

        new_ideal_goal_to_reach = true;
        init_ideal_goal_to_reach = true;
        ideal_goal_to_reach = *g;
        //ROS_INFO("new ideal_goal_to_reach(%f, %f) published", g->x, g->y);

    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
    {

        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

// Draw the field of view and other references
void populateMarkerReference()
{

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "datmo_marker";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x = 0.02 * cos(-2.092350);
    v.y = 0.02 * sin(-2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x = 5.6 * cos(-2.092350);
    v.y = 5.6 * sin(-2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.092350 + 0.0057856218349117;
    // first and last beam are already included
    for (int i = 0; i < 723; i++, beam_angle += 0.0057856218349117)
    {
        v.x = 5.6 * cos(beam_angle);
        v.y = 5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x = 5.6 * cos(2.092350);
    v.y = 5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x = 0.02 * cos(2.092350);
    v.y = 0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_test_datmo_marker.publish(references);
}

void display_graphical_markers()
{

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "test_datmo_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    // ROS_INFO("%i points to display", nb_pts);
    for (int loop = 0; loop < nb_pts; loop++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;

        p.x = display[loop].x;
        p.y = display[loop].y;
        p.z = display[loop].z;

        c.r = colors[loop].r;
        c.g = colors[loop].g;
        c.b = colors[loop].b;
        c.a = colors[loop].a;

        // ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
        marker.points.push_back(p);
        marker.colors.push_back(c);
    }

    pub_test_datmo_marker.publish(marker);
    populateMarkerReference();
}

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_datmo_node");

    ROS_INFO("(test_datmo_node) waiting for a /goal_to_reach");
    test_datmo bsObject;

    ros::spin();

    return 0;
}
