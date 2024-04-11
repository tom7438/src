// localization using lidar data
//  written by O. Aycard
#include <localization.h>

localization::localization() {

    sub_scan = n.subscribe("scan", 1, &localization::scanCallback, this);
    sub_odometry = n.subscribe("odom", 1, &localization::odomCallback, this);
    pub_localization_marker = n.advertise<visualization_msgs::Marker>("localization_marker",
                                                                      1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    sub_position = n.subscribe("initialpose", 1, &localization::positionCallback, this);

    pub_localization = n.advertise<geometry_msgs::Point>("localization",
                                                         1); // Preparing a topic to publish the goal to reach.

    // get map via RPC
    nav_msgs::GetMap::Request req;
    ROS_INFO("Requesting the map...");
    while (!ros::service::call("static_map", req, resp)) {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }

    init_odom = false;
    init_laser = false;
    init_position = false;
    localization_initialized = false;

    width_max = resp.map.info.width;
    height_max = resp.map.info.height;
    cell_size = resp.map.info.resolution;
    min.x = resp.map.info.origin.position.x;
    min.y = resp.map.info.origin.position.y;
    max.x = min.x + width_max * cell_size;
    max.y = min.y + height_max * cell_size;

    ROS_INFO("map loaded");
    ROS_INFO("Map: (%f, %f) -> (%f, %f) with size: %f", min.x, min.y, max.x, max.y, cell_size);
    ROS_INFO("wait for initial pose");

    // INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10); // this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce(); // each callback is called once
        update();
        r.sleep(); // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

void localization::initialize_localization() {

    ROS_INFO("initialize localization");

    estimated_position = initial_position;
    estimated_orientation = initial_orientation;
    ROS_INFO("initial_position(%f, %f, %f): score = %i", initial_position.x, initial_position.y,
             initial_orientation * 180 / M_PI, score_max);
    reset_display();
    display_localization(initial_position, initial_orientation);
    display_markers();

    float min_x, max_x, min_y, max_y;
    min_x = initial_position.x - 1;
    max_x = initial_position.x + 1;
    min_y = initial_position.y - 1;
    max_y = initial_position.y + 1;
    // we search the position with the highest sensor_model in a square of 2x2 meters around the initial_position and with all possible orientations
    ROS_INFO("possible positions to tests: (%f, %f) -> (%f, %f)", min_x, min_y, max_x, max_y);
    find_best_position(min_x, max_x, min_y, max_y, -M_PI, M_PI);

    ROS_INFO("initialize localization done");
} // initialize_localization

void localization::predict_position() {
    // NOTHING TO DO HERE for the students

    ROS_INFO("predict_position");

    odom_last = odom_current;
    odom_last_orientation = odom_current_orientation;

    // prediction of the current position of the mobile robot
    predicted_orientation = estimated_orientation + angle_traveled;
    if (predicted_orientation < -M_PI)
        predicted_orientation += 2 * M_PI;
    if (predicted_orientation > M_PI)
        predicted_orientation -= 2 * M_PI;

    predicted_position.x = estimated_position.x + distance_traveled * cos(predicted_orientation);
    predicted_position.y = estimated_position.y + distance_traveled * sin(predicted_orientation);

    ROS_INFO("predict_position done");
}

void localization::estimate_position() {

    ROS_INFO("estimate_position");

    ROS_INFO("previous position: (%f, %f, %f)", estimated_position.x, estimated_position.y,
             estimated_orientation * 180 / M_PI);
    ROS_INFO("predicted position(%f, %f, %f): score = %i", predicted_position.x, predicted_position.y,
             predicted_orientation * 180 / M_PI, score_max);

    estimated_position = predicted_position;
    estimated_orientation = predicted_orientation;
    reset_display();
    display_localization(predicted_position, predicted_orientation);
    display_markers();

    float min_x, max_x, min_y, max_y, min_orientation, max_orientation;
    min_x = predicted_position.x - 0.5;
    max_x = predicted_position.x + 0.5;
    min_y = predicted_position.y - 0.5;
    max_y = predicted_position.y + 0.5;
    min_orientation = predicted_orientation - M_PI / 6;
    max_orientation = predicted_orientation + M_PI / 6;
    // we search the position with the highest sensor_model in a square of 1x1 meter around the predicted_position and with orientations around the predicted_orientation -M_PI/6 and +M_PI/6
    ROS_INFO("possible positions to tests: (%f, %f, %f) -> (%f, %f, %f)", min_x, min_y, min_orientation, max_x, max_y,
             max_orientation);
    find_best_position(min_x, max_x, min_y, max_y, min_orientation, max_orientation);

    ROS_INFO("estimate_position done");
}

void localization::find_best_position(float min_x, float max_x, float min_y, float max_y, float min_orientation,
                                      float max_orientation) {

    ROS_INFO("find_best_position");
    odom_last = odom_current;
    odom_last_orientation = odom_current_orientation;

    // loop over all the possible positions (x, y, theta) {
    //  *  * the increment on x and y is of 5cms and on theta is of 5 degrees
    for (float loop_x = min_x; loop_x < max_x; loop_x += 0.05) {
        for (float loop_y = min_y; loop_y < max_y; loop_y += 0.05) {
            for (float loop_o = min_orientation; loop_o < max_orientation; loop_o += (5 * M_PI / 180)) {
                if (cell_value(loop_x, loop_y) == 0) { // robair can only be at a free cell
                    int score_current = sensor_model(loop_x, loop_y, loop_o);
                    ROS_INFO("(%f, %f, %f): score = %i", loop_x, loop_y, loop_o * 180 / M_PI, score_current);
                    // we store the maximum score over all the possible positions in estimated_position and score_max
                    if (score_current > score_max) {
                        estimated_position.x = loop_x;
                        estimated_position.y = loop_y;
                        estimated_orientation = loop_o;
                        score_max = score_current;
                        reset_display();
                        display_localization(estimated_position, estimated_orientation);
                        display_markers();
                    }
                }
            }
        }
    }
    ROS_INFO("best_position found");
    ROS_INFO("best_position : (%f, %f, %f)\nScore de la meilleure position : %d\nMeilleur score possible : %d\n",
             estimated_position.x, estimated_position.y, estimated_orientation * 180 / M_PI, score_max, score_total);
    // Publish for notify decision_node
    pub_localization.publish(estimated_position);
}

int localization::sensor_model(float x, float y, float o) {
    // compute the score of the position (x, y, o)
    // for each hit of the laser, we compute its position in the map and check if it is occupied or not
    float uncertainty = 0.05;

    ROS_INFO("compute_score");
    // nb_pts = 0;
    int score_current = 0;
    float beam_angle = angle_min;
    score_total = 0;
    for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
        // TO COMPLETE
        hit[loop].x = x + (r[loop] * cos(o + beam_angle));
        hit[loop].y = y + (r[loop] * sin(o + beam_angle));

        cell_occupied[loop] = false;
        for (float loop_x = hit[loop].x - uncertainty; loop_x <= hit[loop].x + uncertainty; loop_x += uncertainty)
            for (float loop_y = hit[loop].y - uncertainty; loop_y <= hit[loop].y + uncertainty; loop_y += uncertainty)
                // the current hit of the laser corresponds to an occupied cell
                cell_occupied[loop] = cell_occupied[loop] || (cell_value(loop_x, loop_y) == 100);
        if (valid[loop]) {
            score_total++;
        }
        if (cell_occupied[loop] && valid[loop])
            score_current++;
    }

    ROS_INFO("score_current = %i", score_current);
    ROS_INFO("done");

    return (score_current);
}

int localization::sensor_model_with_valid(float x, float y, float o) {
    // compute the score of the position (x, y, o)
    // for each hit of the laser, we compute its position in the map and check if it is occupied or not
    float uncertainty = 0.05;

    nb_pts = 0;
    // we add the current hit to the hits to display
    display[nb_pts].x = x;
    display[nb_pts].y = y;
    display[nb_pts].z = 0;

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 1;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // we add the current hit to the hits to display
    display[nb_pts].x = x + cos(o);
    display[nb_pts].y = y + sin(o);
    display[nb_pts].z = 0;

    colors[nb_pts].r = 1;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // ROS_INFO("compute_score");
    // nb_pts = 0;
    int score_current = 0;
    float beam_angle = angle_min;
    for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
        //    if ( valid[loop] ) {
        //      ROS_INFO("loop = %i", loop);
        geometry_msgs::Point hit;
        hit.x = x + r[loop] * cos(o + beam_angle);
        hit.y = y + r[loop] * sin(o + beam_angle);

        // we add the current hit to the hits to display
        display[nb_pts].x = hit.x;
        display[nb_pts].y = hit.y;
        display[nb_pts].z = 0;

        bool cell_occupied = false;
        for (float loop_x = hit.x - uncertainty; loop_x <= hit.x + uncertainty; loop_x += uncertainty)
            for (float loop_y = hit.y - uncertainty; loop_y <= hit.y + uncertainty; loop_y += uncertainty)
                // the current hit of the laser corresponds to an occupied cell
                cell_occupied = cell_occupied || (cell_value(loop_x, loop_y) == 100);

        if ((valid[loop] && cell_occupied) /*|| ( !valid[loop] && cell_free )*/) {
            score_current++;
            // ROS_INFO("score_current = %i", score_current);

            // when matching is ok: the hit of the laser is green
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;
        }
            // the current hit of the laser corresponds to a free cell
        else if (valid[loop]) // pour supprimer les donnees non valides lors de l'affichage
        {

            // when matching is not ok: the hit of the laser is red
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;

            nb_pts++;
        } else {

            // when matching is not ok: the hit of the laser is red
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;
        }
    }

    // ROS_INFO("score_current = %i", score_current);
    // ROS_INFO("done");

    return (score_current);
}

int localization::compute_score2(float x, float y, float o) {
    // compute the score of the position (x, y, o)
    // for each hit of the laser, we compute its position in the map and check if it is occupied or not
    float uncertainty = 0.05;

    nb_pts = 0;
    // we add the current hit to the hits to display
    display[nb_pts].x = x;
    display[nb_pts].y = y;
    display[nb_pts].z = 0;

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 1;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // we add the current hit to the hits to display
    display[nb_pts].x = x + cos(o);
    display[nb_pts].y = y + sin(o);
    display[nb_pts].z = 0;

    colors[nb_pts].r = 1;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // ROS_INFO("compute_score");
    // nb_pts = 0;
    bool started = false;
    int best_start = 0;
    int best_end = 0;
    int score_current = 0;
    int current_start = 0;
    int current_end = -1;
    float beam_angle = angle_min;
    for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
        //    if ( valid[loop] ) {
        //      ROS_INFO("loop = %i", loop);
        geometry_msgs::Point hit;
        hit.x = x + r[loop] * cos(o + beam_angle);
        hit.y = y + r[loop] * sin(o + beam_angle);

        // we add the current hit to the hits to display
        display[nb_pts].x = hit.x;
        display[nb_pts].y = hit.y;
        display[nb_pts].z = 0;
        nb_pts++;

        bool cell_occupied = false;
        for (float loop_x = hit.x - uncertainty; loop_x <= hit.x + uncertainty; loop_x += uncertainty)
            for (float loop_y = hit.y - uncertainty; loop_y <= hit.y + uncertainty; loop_y += uncertainty)
                // the current hit of the laser corresponds to an occupied cell
                cell_occupied = cell_occupied || (cell_value(loop_x, loop_y) == 100);

        if (cell_occupied) {
            if (!started) {
                started = true;
                current_start = loop;
                current_end = loop;
            } else
                current_end = loop;
            // ROS_INFO("current_start = %i, current_end = %i, score_current: %i", current_start, current_end, current_end-current_start+1);
        } else {
            /*if ( started )
                ROS_WARN("current_start = %i, current_end = %i, score_current: %i", current_start, current_end, current_end-current_start+1);*/

            started = false;
            if (current_end - current_start + 1 > score_current) {
                score_current = current_end - current_start + 1;
                best_start = current_start;
                best_end = current_end;
                // ROS_WARN("best_start = %i, best_end = %i, score_current: %i", best_start, best_end, score_current);
            }
        }
    }
    nb_pts = 2;
    beam_angle = angle_min;
    for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {

        if (loop >= best_start && loop <= best_end) {
            // when matching is ok: the hit of the laser is green
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
            // the current hit of the laser corresponds to a free cell
        else {
            // when matching is not ok: the hit of the laser is red
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        nb_pts++;
    }

    // ROS_INFO("best_start = %i, best_end = %i", best_start, best_end);
    // ROS_INFO("score_current = %i", score_current);
    // ROS_INFO("done");

    return (score_current);
}

int localization::cell_value(float x, float y) {
    // returns the value of the cell corresponding to the position (x, y) in the map
    // returns 100 if cell(x, y) is occupied, 0 if cell(x, y) is free

    if ((min.x <= x) && (x <= max.x) && (min.y <= y) && (y <= max.y)) {
        float x_cell = (x - min.x) / cell_size;
        float y_cell = (y - min.y) / cell_size;
        int x_int = x_cell;
        int y_int = y_cell;
        // ROS_INFO("cell[%f = %d][%f = %d] = %d", x_cell, x_int, y_cell, y_int, map[x_int][y_int]);
        return (resp.map.data[width_max * y_int + x_int]);
    } else
        return (-1);
}

// CALLBACK
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void localization::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {

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

        valid[loop] = r[loop] < range_max;

        // transform the scan in cartesian framework
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

} // scanCallback

void localization::odomCallback(const nav_msgs::Odometry::ConstPtr &o) {

    init_odom = true;
    odom_current.x = o->pose.pose.position.x;
    odom_current.y = o->pose.pose.position.y;
    odom_current_orientation = tf::getYaw(o->pose.pose.orientation);

} // odomCallback

void localization::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &p) {

    init_position = true;
    new_position = true;
    initial_position.x = p->pose.pose.position.x;
    initial_position.y = p->pose.pose.position.y;
    initial_orientation = tf::getYaw(p->pose.pose.orientation);
}

// GRAPHICAL DISPLAY
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void localization::reset_display() {

    nb_pts = 0;
}

void localization::display_localization(geometry_msgs::Point position, float orientation) {

    // we add the current hit to the hits to display
    display[nb_pts].x = position.x;
    display[nb_pts].y = position.y;
    display[nb_pts].z = 0;

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 1;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // we add the current hit to the hits to display
    display[nb_pts].x = position.x + cos(orientation);
    display[nb_pts].y = position.y + sin(orientation);
    display[nb_pts].z = 0;

    colors[nb_pts].r = 1;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    for (int loop = 0; loop < nb_beams; loop++) {

        display[nb_pts].x = hit[loop].x;
        display[nb_pts].y = hit[loop].y;
        display[nb_pts].z = 0;

        if (cell_occupied[loop]) {
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        } else {
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        nb_pts++;
    }
}

void localization::display_markers() {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    for (int loop = 0; loop < nb_pts; loop++) {
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

    pub_localization_marker.publish(marker);
}

// Distance between two points
float localization::distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
}