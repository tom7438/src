#include <datmo.h>

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void datmo::update() 
{

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser && new_robot ) 
    {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");        

        reset_display();
        //if the robot is not moving then we can perform moving person detection
        if ( !current_robot_moving ) 
        {
            ROS_INFO("robot is not moving");

            // if the robot was moving previously and now it is not moving then we store the background
            if ( previous_robot_moving && !current_robot_moving ) 
            {
                ROS_INFO("robot was moving");
                store_background();
            }
            else
            {
               ROS_INFO("robot was not moving");
               detect_motion();//to classify each hit of the laser as dynamic or not
            }
        }
        else
        {
            ROS_INFO("robot is moving");
            if ( !previous_robot_moving && current_robot_moving ) 
            {
                ROS_INFO("robot was not moving");
                reset_motion();            
            }
            else
            {
                ROS_INFO("robot was moving");
            }
        }        
        display_motion();

        //clustering
        //uncomment ONLY when you have implemented and tested the detection of motion
        reset_display();
        perform_clustering();//to perform clustering
        display_clustering();
        
        //detection of legs
        //uncomment ONLY when you have implemented and tested the clustering
        reset_display();
        detect_legs();//to detect legs using cluster
        display_legs();
        
        //detection of persons
        //uncomment ONLY when you have implemented and tested the detection of legs
        detect_persons();//to detect persons using legs detected
        display_persons();

        if(is_person_tracked)
            track_a_person();
        else
            detect_a_moving_person();
        
        display_graphical_markers();

        //do not change this part
        new_laser = false;
        new_robot = false;
        previous_robot_moving = current_robot_moving;       
        
    }
    else
    {
        if ( !init_laser )
            ROS_WARN("waiting for laser data: run a rosbag");
        else
            if ( !init_robot )
                ROS_WARN("waiting for robot_moving_node: rosrun follow_me robot_moving_node");
    }

}// update

int main(int argc, char **argv)
{

    ros::init(argc, argv, "detection_node");

    ROS_INFO("waiting for detection of a moving person");
    datmo bsObject;

    ros::spin();

    return 0;
}
