#include <datmo.h>

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void datmo::update() 
{

      // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if (new_laser && new_robot)
    {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        //reset_display();

        // clustering
        perform_basic_clustering(); // to perform BASIC clustering
        display_clustering();

        // do not change this part
        new_laser = false;
        new_robot = false;
        previous_robot_moving = current_robot_moving;
    }
    else
    {
        if (!init_laser)
            ROS_WARN("waiting for laser data: run a rosbag");
        else if (!init_robot)
            ROS_WARN("waiting for robot_moving_node: rosrun follow_me robot_moving_node");
    }

} // update

int main(int argc, char **argv)
{

    ros::init(argc, argv, "detection_node");

    ROS_INFO("waiting for detection of a moving person");
    datmo bsObject;

    ros::spin();

    return 0;
}
