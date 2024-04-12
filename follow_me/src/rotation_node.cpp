#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"

#define error_rotation_threshold M_PI / 15 // radians = 10 degres M_PI/18 normalement
#define rotation_speed_max M_PI / 6        // radians = 30 degres/s

#define kpr 0.4 // 0.5 normalement
#define kir 0
#define kdr 0

class rotation_node {
private:
    ros::NodeHandle n;

    // communication with detection_node or datmo_node
    //ros::Subscriber sub_goal_to_reach;

    // communication with rotation_to_do
    ros::Subscriber sub_rotation_to_do;

    // communication with odometry
    ros::Subscriber sub_odometry;
    ros::Publisher pub_odometry;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    geometry_msgs::Point goal_to_reach;
    bool new_goal_to_reach; // to check if a new /goal_to_reach is available or not

    // pid for rotation
    float rotation_to_do, rotation_done;
    float error_rotation;      // error in rotation
    bool cond_rotation;        // boolean to check if we still have to rotate or not
    float initial_orientation; // to store the initial orientation ie, before starting the pid for rotation control
    float current_orientation; // to store the current orientation: this information is provided by the odometer
    float error_integral_rotation;
    float error_previous_rotation;
    float rotation_speed;

    float translation_to_do;

    bool init_odom;

public:
    rotation_node() {

        // communication with cmd_vel to command the mobile robot
        pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // communication with odometry
        sub_odometry = n.subscribe("odom", 1, &rotation_node::odomCallback, this);

        // communication with datmo
        //sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &rotation_node::goal_to_reachCallback, this);
        sub_rotation_to_do = n.subscribe("rotation_to_do", 1, &rotation_node::rotation_to_doCallback, this);

        new_goal_to_reach = false;
        init_odom = false;
        cond_rotation = false;

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10); // this node will run at 10hz
        while (ros::ok()) {
            ros::spinOnce(); // each callback is called once to collect new data: laser + robot_moving
            update();        // processing of data
            r.sleep();       // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {

        if (init_odom) {

             // we receive a new /rotation_to_do
             ROS_INFO("\n\trotation_to_do = %f\n", rotation_to_do);
             if (new_goal_to_reach)
                 init_rotation();

            // we are performing a rotation
            if (cond_rotation) {
                compute_rotation();
                move_robot();
            }
        }

    } // update

    void init_rotation() {

        /*initial_orientation = current_orientation;
        error_rotation = 0;
        error_integral_rotation = 0;
        error_previous_rotation = 0;

        new_goal_to_reach = false;
        ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        translation_to_do = sqrt((goal_to_reach.x * goal_to_reach.x) + (goal_to_reach.y * goal_to_reach.y));

        if (translation_to_do) {

            // we compute the /rotation_to_dooal
            rotation_to_do = acos(goal_to_reach.x / translation_to_do);

            cond_rotation = fabs(rotation_to_do) > error_rotation_threshold;

            if (goal_to_reach.y < 0)
                rotation_to_do *= -1;

            // we initialize the pid for the control of rotation
            error_integral_rotation = 0;
            error_previous_rotation = 0;

            // ROS_INFO("rotation_to_do: %f, translation_to_do: %f", rotation_to_do*180/M_PI, translation_to_do);
            ROS_WARN("rotation_to_do: %f, translation_to_do: %f", rotation_to_do * 180 / M_PI, translation_to_do);
        } else {
            ROS_WARN("translation_to_do is equal to 0");
            rotation_to_do = 0;
        }*/
        new_goal_to_reach = false;

        cond_rotation = fabs(rotation_to_do) > error_rotation_threshold;

        // we initialize the pid for the control of rotation
        error_integral_rotation = 0;
        error_previous_rotation = 0;

        // ROS_INFO("rotation_to_do: %f, translation_to_do: %f", rotation_to_do*180/M_PI, translation_to_do);
        if (cond_rotation)
            ROS_WARN("rotation_to_do: %f", rotation_to_do * 180 / M_PI);
    } // init_rotation

    void compute_rotation() {

        ROS_INFO("current_orientation: %f, initial_orientation: %f", current_orientation * 180 / M_PI,
                 initial_orientation * 180 / M_PI);

        rotation_done = current_orientation - initial_orientation;

        // do not forget that rotation_done must always be between -M_PI and +M_PI
        if (rotation_done > M_PI) {
            ROS_WARN("rotation_done > 180 degrees: %f degrees -> %f degrees", rotation_done * 180 / M_PI,
                     (rotation_done - 2 * M_PI) * 180 / M_PI);
            rotation_done -= 2 * M_PI;
        } else if (rotation_done < -M_PI) {
            ROS_WARN("rotation_done < -180 degrees: %f degrees -> %f degrees", rotation_done * 180 / M_PI,
                     (rotation_done + 2 * M_PI) * 180 / M_PI);
            rotation_done += 2 * M_PI;
        }

        error_rotation = rotation_to_do - rotation_done;
        ROS_INFO("rotation_done : %f", rotation_done);
        // do not forget that error_rotation must always be between -M_PI and +M_PI
        if (error_rotation > M_PI) {
            ROS_WARN("error_rotation_done > 180 degrees: %f degrees -> %f degrees", error_rotation * 180 / M_PI,
                     (error_rotation - 2 * M_PI) * 180 / M_PI);
            error_rotation -= 2 * M_PI;
        } else if (error_rotation < -M_PI) {
            ROS_WARN("error_rotation < -180 degrees: %f degrees -> %f degrees", error_rotation * 180 / M_PI,
                     (error_rotation + 2 * M_PI) * 180 / M_PI);
            error_rotation += 2 * M_PI;
        }

        // cond_rotation = ...; cond_rotation is used to control if we stop or not the pid TO COMPLETE. take care that rotation_to_do could be negative
        cond_rotation = (fabs(error_rotation) > error_rotation_threshold);
        ROS_INFO("rotation_to_do: %f, rotation_done: %f, error_rotation: %f, cond_rotation: %i",
                 rotation_to_do * 180 / M_PI, rotation_done * 180 / M_PI, error_rotation * 180 / M_PI, cond_rotation);

        rotation_speed = 0;
        if (cond_rotation) {
            // Implementation of a PID controller for rotation_to_do;

            float error_derivation_rotation = error_previous_rotation - error_rotation;
            error_previous_rotation = error_rotation;
            ROS_INFO("error_derivation_rotation: %f", error_derivation_rotation);

            error_integral_rotation += error_rotation;
            ROS_INFO("error_integral_rotation: %f", error_integral_rotation);

            // control of rotation with a PID controller
            rotation_speed =
                    (kpr * error_rotation) + (kir * error_integral_rotation) + (kdr * error_derivation_rotation);
            ROS_WARN("rotation_speed: %f", rotation_speed * 180 / M_PI);
            //getchar();
        } else
            ROS_WARN("pid of rotation will stop");

    } // compute_rotation

    void move_robot() {
        ROS_INFO("MOVE ROBOT");

        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        // comment these 3 lines only when your pid works
        // pub_cmd_vel.publish(twist);
        // ROS_INFO("press enter to continue");

        twist.angular.z = rotation_speed;
        ROS_WARN("rotation speed = %f\n", rotation_speed);
        pub_cmd_vel.publish(twist);
        //getchar();

    } // move_robot

    // CALLBACKS
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void odomCallback(const nav_msgs::Odometry::ConstPtr &o) {

        init_odom = true;
        current_orientation = tf::getYaw(o->pose.pose.orientation);
    }

    void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr &g) {
        // process the goal received from moving_persons detector

        new_goal_to_reach = true;
        goal_to_reach = *g;
    }

    // Récupère le float directement envoyé par decision_node
    void rotation_to_doCallback(const std_msgs::Float32::ConstPtr &r) {
        // process the goal received from decision node

        new_goal_to_reach = true;
        //goal_to_reach = *r;
        std_msgs::Float32 tmp;
        tmp = *r;
        rotation_to_do = tmp.data;
        ROS_WARN("rotation_to_do dans callback = %f", rotation_to_do);
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "rotation_node");

    ROS_INFO("(rotation_node) waiting for a /goal_to_reach");
    rotation_node bsObject;

    ros::spin();

    return 0;
}
