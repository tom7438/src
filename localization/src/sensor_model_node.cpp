//localization using lidar data
// written by O. Aycard
#include <localization.h>

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void localization::update() {

    if (init_laser && init_position && new_position) {

        new_position = false;
        int score_current = sensor_model(initial_position.x, initial_position.y, initial_orientation);
        ROS_INFO("initial_position(%f, %f, %f): score = %i", initial_position.x, initial_position.y,
                 initial_orientation * 180 / M_PI, score_current);
        reset_display();
        display_localization(initial_position, initial_orientation);
        display_markers();
    } else if (!init_laser || !init_position)
        ROS_INFO("waiting for laser data or initialization of position");

}// update

int main(int argc, char **argv) {

    ros::init(argc, argv, "sensor_model_complete_node");

    localization bsObject;

    ros::spin();

    return 0;
}


