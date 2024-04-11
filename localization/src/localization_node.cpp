#include <localization.h>

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void localization::update() {

    //ROS_INFO("odom = %i, laser = %i, position = %i", init_odom, init_laser, init_position);
    if (init_odom && init_laser && init_position) {
        if (!localization_initialized) {
            initialize_localization();
            localization_initialized = true;
        }
        previous_distance_traveled = distance_traveled;
        previous_angle_traveled = angle_traveled;

        distance_traveled = distancePoints(odom_current, odom_last);
        angle_traveled = odom_current_orientation - odom_last_orientation;
        if (angle_traveled < -M_PI)
            angle_traveled += 2 * M_PI;
        if (angle_traveled > M_PI)
            angle_traveled -= 2 * M_PI;

        if ((distance_traveled != previous_distance_traveled) || (angle_traveled != previous_angle_traveled))
            ROS_INFO("distance_traveled = %f, angle_traveled = %f since last localization", distance_traveled,
                     angle_traveled * 180 / M_PI);

        if ((distance_traveled > distance_to_travel) || (fabs(angle_traveled * 180 / M_PI) > angle_to_travel)) {
            predict_position();
            estimate_position();
        }
    }

}// update

int main(int argc, char **argv) {

    ros::init(argc, argv, "localization_node");

    localization bsObject;

    ros::spin();

    return 0;
}


