#include <datmo.h>

datmo::datmo() {

    sub_scan = n.subscribe("scan", 1, &datmo::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &datmo::robot_movingCallback, this);

    // communication with action
    //pub_datmo = n.advertise<geometry_msgs::Point>("goal_to_reach", 1); // Preparing a topic to publish the goal to reach.

    pub_datmo_marker = n.advertise<visualization_msgs::Marker>("datmo_marker",
                                                               1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_motion_marker = n.advertise<visualization_msgs::Marker>("motion_marker", 1);
    pub_clusters_marker = n.advertise<visualization_msgs::Marker>("clusters_marker", 1);
    pub_legs_marker = n.advertise<visualization_msgs::Marker>("legs_marker", 1);
    pub_persons_marker = n.advertise<visualization_msgs::Marker>("persons_marker", 1);
    pub_tracked_person_marker = n.advertise<visualization_msgs::Marker>("tracked_person_marker", 1);

    // person position
    pub_person_position = n.advertise<geometry_msgs::Point>("person_position", 1);

    new_laser = false;
    new_robot = false;
    init_laser = false;
    init_robot = false;

    previous_robot_moving = true;

    uncertainty = uncertainty_min;

    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }
}

datmo::datmo(char *goal_name) {

    sub_scan = n.subscribe("scan", 1, &datmo::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &datmo::robot_movingCallback, this);

    // communication with action
    pub_datmo = n.advertise<geometry_msgs::Point>(goal_name, 1); // Preparing a topic to publish the goal to reach.

    pub_datmo_marker = n.advertise<visualization_msgs::Marker>("datmo_marker",
                                                               1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    new_laser = false;
    new_robot = false;
    init_laser = false;
    init_robot = false;

    previous_robot_moving = true;

    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }
}

// DETECT MOTION FOR BOTH LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void datmo::store_background() {
    // store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop_hit = 0; loop_hit < nb_beams; loop_hit++)
        background[loop_hit] = r[loop_hit];

    ROS_INFO("background stored");

} // store_background

void datmo::reset_motion() {
    // for each hit, we reset the dynamic table

    ROS_INFO("reset motion");
    for (int loop_hit = 0; loop_hit < 1000; loop_hit++)
        dynamic[loop_hit] = false;
    ROS_INFO("reset_motion done");

} // reset_motion

void datmo::detect_motion() {
    // for each hit, compare the current range with the background to detect motion

    ROS_INFO("detecting motion");
    int diff;
    for (int loop_hit = 0; loop_hit < nb_beams; loop_hit++) {
        diff = background[loop_hit] - r[loop_hit];
        dynamic[loop_hit] = sqrt(pow(diff, 2.0)) > detection_threshold;
    }
    ROS_INFO("motion detected");

} // detect_motion

// CLUSTERING FOR LASER DATA
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
// Distance between two points
float datmo::distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
}

void datmo::perform_clustering() {

    ROS_INFO("performing clustering");

    perform_basic_clustering();
    perform_advanced_clustering();

    ROS_INFO("clustering performed");

} // perform_clustering

void datmo::perform_basic_clustering() {
    // TO COMPLETE
    // we perform the clustering as described in the lecture on perception
    // the data related to each cluster are stored in cluster_start, cluster_end and nb_cluster: see datmo.h for more details

    ROS_INFO("performing basic clustering");

    nb_clusters = 0;

    for (int loop_hit = 1; loop_hit < nb_beams; loop_hit++) {
        /*     if EUCLIDIAN DISTANCE between (the previous hit and the current one) is higher than "cluster_threshold"
                {//the current hit doesnt belong to the same cluster*/
        if (distancePoints(current_scan[loop_hit], current_scan[loop_hit - 1]) > detection_threshold) {
            cluster_end[nb_clusters] = loop_hit - 1;
            nb_clusters++;
            cluster_start[nb_clusters] = loop_hit;
        }
    }
    // Dont forget to update the different information for the last cluster
    cluster_end[nb_clusters] = nb_beams;

    ROS_INFO("basic clustering performed");

} // perform_basic_clustering

void datmo::perform_advanced_clustering() {
    /* for each cluster, we update:
        - cluster_size to store the size of the cluster ie, the euclidian distance between the first hit of the cluster and the last one
        - cluster_middle to store the middle of the cluster
        - cluster_dynamic to store the percentage of hits of the current cluster that are dynamic*/

    ROS_INFO("perform advanced clustering");

    for (int loop_cluster = 0; loop_cluster < nb_clusters; loop_cluster++) {
        int start = cluster_start[loop_cluster];
        int end = cluster_end[loop_cluster];
        cluster_size[loop_cluster] = distancePoints(current_scan[end], current_scan[start]);
        cluster_middle[loop_cluster].x = (current_scan[end].x + current_scan[start].x) / 2;
        cluster_middle[loop_cluster].y = (current_scan[end].y + current_scan[start].y) / 2;
        cluster_dynamic[loop_cluster] = (compute_nb_dynamic(start, end) * 100) / (end - start + 1);
    }

    ROS_INFO("advanced clustering performed");

} // perform_advanced_clustering

int datmo::compute_nb_dynamic(int start, int end) {
    //  return the number of points that are dynamic between start and end

    int nb_dynamic = 0;
    for (int i = start; i <= end; i++) {
        if (dynamic[i]) {
            nb_dynamic++;
        }
    }
    return (nb_dynamic);
} // compute_nb_dynamic

// DETECTION OF PERSONS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void datmo::detect_legs() {
    // TO COMPLETE
    // a leg is a cluster:
    // - with a size higher than "leg_size_min";
    // - with a size lower than "leg_size_max;
    // if more than "dynamic_threshold"% of its hits are dynamic the leg is considered to be dynamic
    // we update the array leg_cluster, leg_detected and leg_dynamic

    ROS_INFO("detecting legs");
    nb_legs_detected = 0;
    for (int loop = 0; loop < nb_clusters; loop++) // loop over all the clusters
    {

        if (cluster_size[loop] > leg_size_min && cluster_size[loop] < leg_size_max) {
            leg_cluster[nb_legs_detected] = loop;
            leg_dynamic[nb_legs_detected] = (cluster_dynamic[loop] > dynamic_threshold);
            leg_detected[nb_legs_detected] = cluster_middle[loop];
            nb_legs_detected++;
        }
    }
    ROS_INFO("legs detected");
} // detect_legs

void datmo::detect_persons() {

    // TO COMPLETE
    //  a person has two legs located at less than "legs_distance_max" one from the other
    //  a moving person (ie, person_dynamic array) has 2 legs that are dynamic
    //  we update the person_detected table to store the middle of the person
    //  we update the person_dynamic table to know if the person is moving or not

    ROS_INFO("detecting persons");
    nb_persons_detected = 0;

    for (int loop_leg_right = 0; loop_leg_right < nb_legs_detected; loop_leg_right++)
        for (int loop_leg_left = loop_leg_right + 1; loop_leg_left < nb_legs_detected; loop_leg_left++) {
            if (distancePoints(leg_detected[loop_leg_left], leg_detected[loop_leg_right]) <= legs_distance_max) {
                person_detected[nb_persons_detected].x =
                        (leg_detected[loop_leg_left].x + leg_detected[loop_leg_right].x) / 2;
                person_detected[nb_persons_detected].y =
                        (leg_detected[loop_leg_left].y + leg_detected[loop_leg_right].y) / 2;
                person_dynamic[nb_persons_detected] = leg_dynamic[loop_leg_left] && leg_dynamic[loop_leg_right];
                leg_left[nb_persons_detected] = loop_leg_left;
                leg_right[nb_persons_detected] = loop_leg_right;
                nb_persons_detected++;
            }
        }

    ROS_INFO("persons detected");

} // detect_persons

void datmo::detect_a_moving_person() {

    // we store the moving_person_detected in person_tracked
    // we update is_person_tracked
    // do not forget to publish person_tracked
    ROS_INFO("detecting a moving person");

    int distance_min = __INT_MAX__;

    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;

    int current_distance = 0;

    int dynamicper = 0;

    for (int loop_persons = 0; loop_persons < nb_persons_detected; loop_persons++) {
        current_distance = distancePoints(p, person_detected[loop_persons]);
        if (person_dynamic[loop_persons] && (current_distance < distance_min)) {
            dynamicper++;
            distance_min = current_distance;
            person_tracked = person_detected[loop_persons];
        }
    }

    if (nb_persons_detected > 0 && dynamicper > 0) {
        is_person_tracked = true;
        pub_datmo.publish(person_tracked);
        // Pour debug
        // ROS_INFO("person_tracked x : %f  y : %f", person_tracked.x, person_tracked.y);
    }

    ROS_INFO("a moving person detected");

} // detect_a_moving_person

// TRACKING OF A PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void datmo::track_a_person() {

    ROS_INFO("tracking a person");

    associated = false;
    float distance_min = uncertainty_max;
    int index_min;

    int current_distance = 0;

    geometry_msgs::Point reset;

    // association between the tracked person and the possible detection
    for (int loop_persons = 0; loop_persons < nb_persons_detected; loop_persons++) {
        // we search for the person_detected which is the closest one to the person_tracked
        // we store the related information in index_min and distance_min
        current_distance = distancePoints(person_tracked, person_detected[loop_persons]);
        if (current_distance < distance_min && current_distance < uncertainty) {
            index_min = loop_persons;
            distance_min = current_distance;
            associated = true;
        }
    }

    if (associated) {
        // update the information related to the person_tracked, frequency and uncertainty knowing that there is an association
        // should we publish or not person_tracked ?
        person_tracked = person_detected[index_min];
        uncertainty = uncertainty_min;
        frequency++;
        //pub_datmo.publish(person_tracked);
        pub_person_position(person_tracked);
    } else {
        // update the information related to the person_tracked, frequency and uncertainty knowing that there is no association
        // should we publish or not person_tracked ?
        uncertainty += uncertainty_inc;
        frequency--;
    }

    if (frequency == 0) {
        is_person_tracked = false;
        frequency = 1;
        uncertainty = uncertainty_min;
        reset.x = 0;
        reset.y = 0;
        person_tracked = reset;
        // Personne perdue
        pub_person_position(person_tracked);
    }

    // do not forget to update person_tracked according to the current association

    ROS_INFO("tracking of a person done");
}

// CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void datmo::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {

    new_laser = true;
    init_laser = true;

    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max) / angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
        if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
            r[loop] = scan->ranges[loop];
        else
            r[loop] = range_max;
        theta[loop] = beam_angle;

        // transform the scan in cartesian framewrok
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
        // Pour debug
        // ROS_INFO("laser[%i]: (%f, %f) -> (%f, %f)", loop, range[loop], beam_angle*180/M_PI, current_scan[loop].x, current_scan[loop].y);
    }

} // scanCallback

void datmo::robot_movingCallback(const std_msgs::Bool::ConstPtr &state) {

    new_robot = true;
    init_robot = true;
    current_robot_moving = state->data;

} // robot_movingCallback

// GRAPHICAL DISPLAY
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void datmo::display_motion() {

    ROS_INFO("\n");
    ROS_INFO("display motion");
    int nb_dyn = 0;
    std_msgs::ColorRGBA color;

    for (int loop_hit = 0; loop_hit < nb_beams; loop_hit++)
        if (dynamic[loop_hit]) {
            marker_motion.points.push_back(current_scan[loop_hit]);

            // choose color of marker point here
            color.r = 1;
            color.g = 1;
            color.b = 0;
            color.a = 1.0;
            marker_motion.colors.push_back(color);

            nb_dyn++;
        }

    // Fill remaining fields of marker message
    marker_motion.header.frame_id = "laser";
    marker_motion.header.stamp = ros::Time::now();
    marker_motion.ns = "datmo_marker_motion"; // change namespace so that the different markers do not replace each other (i.e. legs vs clusters)
    marker_motion.id = 0;
    marker_motion.type = visualization_msgs::Marker::POINTS;
    marker_motion.action = visualization_msgs::Marker::ADD;

    marker_motion.pose.orientation.w = 1;

    marker_motion.scale.x = 0.05;
    marker_motion.scale.y = 0.05;

    marker_motion.color.a = 1.0;

    pub_motion_marker.publish(marker_motion);

    marker_motion.points.clear();
    marker_motion.colors.clear();

    ROS_INFO("%i points are dynamic ", nb_dyn);
    ROS_INFO("motion displayed");

} // display_motion

void datmo::display_clustering() {

    ROS_INFO("\n");
    ROS_INFO("display clusters ");
    ROS_INFO("%d clusters have been detected.\n", nb_clusters);
    std_msgs::ColorRGBA color;

    for (int loop_cluster = 0; loop_cluster < nb_clusters; loop_cluster++) {

        int start = cluster_start[loop_cluster];
        int end = cluster_end[loop_cluster];

        /*
        ROS_INFO("cluster[%i] = middle(%f, %f): current_scan[%i](%f, %f) -> current_scan[%i](%f, %f), size: %f, nb_dynamic: %i, percentage_dynamic: %i",
                 loop_cluster,
                 cluster_middle[loop_cluster].x,
                 cluster_middle[loop_cluster].y,
                 start,
                 current_scan[start].x,
                 current_scan[start].y,
                 end,
                 current_scan[end].x,
                 current_scan[end].y,
                 cluster_size[loop_cluster],
                 compute_nb_dynamic(start, end),
                 cluster_dynamic[loop_cluster]);
        */

        // graphical display of the start of the current cluster in green
        marker_clusters.points.push_back(current_scan[start]);
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 1.0;
        marker_clusters.colors.push_back(color);

        // graphical display of the end of the current cluster in red
        marker_clusters.points.push_back(current_scan[end]);
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1.0;
        marker_clusters.colors.push_back(color);

        // graphical display of the middle of the current cluster in white if static and yellow if dynamic
        marker_clusters.points.push_back(cluster_middle[loop_cluster]);
        color.r = 1;
        color.g = 1;
        if (cluster_dynamic[loop_cluster] >= dynamic_threshold)
            color.b = 0;
        else
            color.b = 1;
        color.a = 1.0;
        marker_clusters.colors.push_back(color);
    }

    // Fill remaining fields of marker message
    marker_clusters.header.frame_id = "laser";
    marker_clusters.header.stamp = ros::Time::now();
    marker_clusters.ns = "datmo_marker_clusters"; // change namespace so that the different markers do not replace each other (i.e. legs vs clusters)
    marker_clusters.id = 0;
    marker_clusters.type = visualization_msgs::Marker::POINTS;
    marker_clusters.action = visualization_msgs::Marker::ADD;

    marker_clusters.pose.orientation.w = 1;

    marker_clusters.scale.x = 0.05;
    marker_clusters.scale.y = 0.05;

    marker_clusters.color.a = 1.0;

    pub_clusters_marker.publish(marker_clusters);

    marker_clusters.points.clear();
    marker_clusters.colors.clear();

    ROS_INFO("clusters displayed");

} // display_clusters

void datmo::display_legs() {

    ROS_INFO("\n");
    ROS_INFO("display legs");
    ROS_INFO("%d legs have been detected.\n", nb_legs_detected);

    std_msgs::ColorRGBA color;

    for (int loop_leg = 0; loop_leg < nb_legs_detected; loop_leg++) {
        int cluster = leg_cluster[loop_leg];

        if (leg_dynamic[loop_leg]) {
            ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i",
                     loop_leg,
                     cluster,
                     leg_detected[loop_leg].x,
                     leg_detected[loop_leg].y,
                     cluster_size[cluster],
                     cluster_dynamic[cluster]);
        } else {
            ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i",
                     loop_leg,
                     cluster,
                     leg_detected[loop_leg].x,
                     leg_detected[loop_leg].y,
                     cluster_size[cluster],
                     cluster_dynamic[cluster]);
        }

        for (int loop_hit = 0; loop_hit < nb_beams; loop_hit++)
            if (loop_hit >= cluster_start[cluster] && loop_hit <= cluster_end[cluster]) {

                // moving legs are yellow
                marker_legs.points.push_back(current_scan[loop_hit]);

                if (leg_dynamic[loop_leg]) {
                    color.r = 1;
                    color.g = 1;
                    color.b = 0;
                    color.a = 1.0;
                } else {
                    color.r = 1;
                    color.g = 1;
                    color.b = 1;
                    color.a = 1.0;
                }

                marker_legs.colors.push_back(color);
            }
    }

    // Fill remaining fields of marker message
    marker_legs.header.frame_id = "laser";
    marker_legs.header.stamp = ros::Time::now();
    marker_legs.ns = "datmo_marker_legs"; // change namespace so that the different markers do not replace each other (i.e. legs vs clusters)
    marker_legs.id = 0;
    marker_legs.type = visualization_msgs::Marker::POINTS;
    marker_legs.action = visualization_msgs::Marker::ADD;

    marker_legs.pose.orientation.w = 1;

    marker_legs.scale.x = 0.05;
    marker_legs.scale.y = 0.05;

    marker_legs.color.a = 1.0;

    pub_legs_marker.publish(marker_legs);

    marker_legs.points.clear();
    marker_legs.colors.clear();

    ROS_INFO("legs displayed");

} // display_legs

void datmo::display_persons() {

    ROS_INFO("\n");
    ROS_INFO("displaying persons");
    ROS_INFO("%d persons have been detected", nb_persons_detected);

    std_msgs::ColorRGBA color;

    for (int loop_persons = 0; loop_persons < nb_persons_detected; loop_persons++) {
        int left = leg_left[loop_persons];
        int right = leg_right[loop_persons];

        marker_persons.points.push_back(person_detected[loop_persons]);

        if (person_dynamic[loop_persons]) {
            ROS_INFO("moving person detected[%i](%f, %f): leg[%i](%f, %f) + leg[%i](%f, %f)",
                     loop_persons,
                     person_detected[loop_persons].x,
                     person_detected[loop_persons].y,
                     right,
                     leg_detected[right].x,
                     leg_detected[right].y,
                     left,
                     leg_detected[left].x,
                     leg_detected[left].y);

            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1.0;
        } else {
            ROS_INFO("%d left leg", cluster_dynamic[left]);
            ROS_INFO("%d right leg", cluster_dynamic[right]);
            ROS_INFO("static person detected[%i](%f, %f): leg[%i](%f, %f) + leg[%i](%f, %f)",
                     loop_persons,
                     person_detected[loop_persons].x,
                     person_detected[loop_persons].y,
                     right,
                     leg_detected[right].x,
                     leg_detected[right].y,
                     left,
                     leg_detected[left].x,
                     leg_detected[left].y);

            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1.0;
        }
        marker_persons.colors.push_back(color);
    }

    // Fill remaining fields of marker message
    marker_persons.header.frame_id = "laser";
    marker_persons.header.stamp = ros::Time::now();
    marker_persons.ns = "datmo_marker_persons"; // change namespace so that the different markers do not replace each other (i.e. legs vs clusters)
    marker_persons.id = 0;
    marker_persons.type = visualization_msgs::Marker::POINTS;
    marker_persons.action = visualization_msgs::Marker::ADD;

    marker_persons.pose.orientation.w = 1;

    marker_persons.scale.x = 0.05;
    marker_persons.scale.y = 0.05;

    marker_persons.color.a = 1.0;

    pub_persons_marker.publish(marker_persons);

    marker_persons.points.clear();
    marker_persons.colors.clear();

    ROS_INFO("persons displayed");

} // display_persons

void datmo::display_a_tracked_person() {

    ROS_INFO("\n");
    ROS_INFO("displaying the tracked person");

    std_msgs::ColorRGBA color;

    if (is_person_tracked)
        if (associated) {
            ROS_INFO("the tracked person has been detected at: (%f, %f) with frequency = %i and uncertainty = %f",
                     person_tracked.x,
                     person_tracked.y,
                     frequency,
                     uncertainty);

            marker_tracked_person.points.push_back(person_tracked);

            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1.0;

            marker_tracked_person.colors.push_back(color);
        } else if (!associated) {
            ROS_INFO(
                    "the tracked person has not been detected and is still at: (%f, %f) with frequency = %i and uncertainty = %f",
                    person_tracked.x,
                    person_tracked.y,
                    frequency,
                    uncertainty);

            marker_tracked_person.points.push_back(person_tracked);

            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1.0;

            marker_tracked_person.colors.push_back(color);
        } else
            ROS_WARN("the tracked person has been lost");

    // Fill remaining fields of marker message
    marker_tracked_person.header.frame_id = "laser";
    marker_tracked_person.header.stamp = ros::Time::now();
    marker_tracked_person.ns = "datmo_marker_tracked_person"; // change namespace so that the different markers do not replace each other (i.e. legs vs clusters)
    marker_tracked_person.id = 0;
    marker_tracked_person.type = visualization_msgs::Marker::POINTS;
    marker_tracked_person.action = visualization_msgs::Marker::ADD;

    marker_tracked_person.pose.orientation.w = 1;

    marker_tracked_person.scale.x = 0.05;
    marker_tracked_person.scale.y = 0.05;

    marker_tracked_person.color.a = 1.0;

    pub_tracked_person_marker.publish(marker_tracked_person);

    marker_tracked_person.points.clear();
    marker_tracked_person.colors.clear();

    ROS_INFO("the tracked person displayed");
}

// Draw the field of view and other references
void datmo::populateMarkerReference() {

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
    for (int i = 0; i < 723; i++, beam_angle += 0.0057856218349117) {
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

    pub_datmo_marker.publish(references);
}