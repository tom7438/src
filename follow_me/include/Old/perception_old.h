/***********************************************************************************************************************
 *
 * Copyright (c) 2022, LIG / MARVIN Team / O. Aycard
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of MARVIN nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#pragma once

#ifndef ROBAIR_ROBOT_DETECTION_H
#define ROBAIR_ROBOT_DETECTION_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class datmo
{
public:
    /**
     * \brief Default Class Constructor.
     */
    datmo();

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);
    
    /**
     * \brief Laser Data Processing.
     */
    void update();

    /**
     * \brief TODO.
     */
    void storeBackground();
    
    /**
     * \brief TODO.
     */
    void resetMotion();

    /**
     * \brief TODO.
     */
    void detectMotion();

    /**
     * \brief TODO.
     */
    void performClustering();

    /**
     * \brief TODO.
     */
    void detectLegs();

    /**
     * \brief TODO.
     */
    void detectPersons();

    /**
     * \brief TODO.
     */
    void detectMovingPerson();

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief TODO.
     */
    ros::Publisher pub_detection_node;
    /**
     * \brief TODO.
     */
    ros::Publisher pub_detection_marker;

    /**
     * \brief TODO.
     */
    //to perform detection of motion
    bool init_robot;
    
    /**
     * \brief TODO.
     */
    bool stored_background;
    
    /**
     * \brief TODO.
     */
    float background[1000];
    
    /**
     * \brief TODO.
     */
    bool dynamic[1000];

    /**
     * \brief TODO.
     */
    //to perform clustering
    int nb_cluster;// number of cluster
    
    /**
     * \brief TODO.
     */
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    
    /**
     * \brief TODO.
     */
    float cluster_size[1000];// to store the size of each cluster
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    
    /**
     * \brief TODO.
     */
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic

    /**
     * \brief TODO.
     */
    //to perform detection of legs and to store them
    int nb_legs_detected;
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point leg_detected[1000];
    
    /**
     * \brief TODO.
     */
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    
    /**
     * \brief TODO.
     */
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    /**
     * \brief TODO.
     */
    //to perform detection of a moving person and store it
    int nb_persons_detected;
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point person_detected[1000];
    
    /**
     * \brief TODO.
     */
    bool person_dynamic[1000];
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point moving_person_detected;//to store the coordinates of the moving person that we have detected

};

#endif