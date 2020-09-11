/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>


GlobalOptimization globalEstimator(1);
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path global_path;
double last_vio_t = -1;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::queue<nav_msgs::Odometry::ConstPtr> vioQueue;
std::mutex m_global;
Eigen::Matrix4d extrinsicPara;
Eigen::Vector4d media;
Eigen::Vector4d accept_origin;
Eigen::Vector3d transfered_pose;
float theta =0 / 180 * 3.1415926;
float cosTheta = cos(theta);
float sinTheta = sin(theta);
double lenth_count = 0;
bool isInitial = false;
double last_gps_t = 0;


void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    m_global.lock();
    gpsQueue.push(GPS_msg);
    m_global.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    vioQueue.push(pose_msg);
    nav_msgs::Odometry::ConstPtr first_pose = vioQueue.front();
    double t = first_pose->header.stamp.toSec(); 
    if (!isInitial)
    {
        while(!gpsQueue.empty())
        {
            sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
            double gps_t = GPS_msg->header.stamp.toSec();
            printf(" initial vio t: %f, gps t: %f \n", t, gps_t);
            // 100ms sync tolerance
            if(gps_t >= t - 0.1)
            {
                cout<<"global_init finished!"<<endl;
                isInitial = true;
                break;
            }
            else if(gps_t < t - 0.1){
                gpsQueue.pop();
            }
        }
        return;
    }
    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);
    //printf("vio_callback! \n");
    last_vio_t = t;
    //获取VIO输出pose
    Eigen::Vector3d vio_t(first_pose->pose.pose.position.x, first_pose->pose.pose.position.y, first_pose->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = first_pose->pose.pose.orientation.w;
    vio_q.x() = first_pose->pose.pose.orientation.x;
    vio_q.y() = first_pose->pose.pose.orientation.y;
    vio_q.z() = first_pose->pose.pose.orientation.z;
    //传入global estimator
    globalEstimator.inputOdom(t, vio_t, vio_q);

    m_global.lock();
    while(!gpsQueue.empty())
    {
        //获取最老的GPS数据及其时间
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        //printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 100ms sync tolerance
        if( (gps_t >= t - 0.1) && (gps_t <= t + 0.1))
        {   
            last_gps_t = gps_t;
            lenth_count ++;
            //add noise for ground truth
                double latitude = GPS_msg->latitude;
                double longitude = GPS_msg->longitude;
                double altitude = GPS_msg->altitude; 
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());

            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8);
            globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            vioQueue.pop();
            /*
            accept_origin<< global_t.x(),global_t.y(),global_t.z(),1;
            media = extrinsicPara * accept_origin;
            geometry_msgs::PoseStamped global_pose_stamped;
            global_pose_stamped.pose.position.x = media(0);
            global_pose_stamped.pose.position.y = media(1);
            global_pose_stamped.pose.position.z = media(2);
            geometry_msgs::PoseStamped global_pose_stamped;
            global_pose_stamped.pose.position.x = global_t.x();
            global_pose_stamped.pose.position.y = global_t.y();
            global_pose_stamped.pose.position.z = global_t.z();
            global_path.poses.push_back(global_pose_stamped);
            */
                // write result to file
            std::ofstream foutC("/home/weining/summer_intern/gps_aided_vins/src/GPS-aided-VINS-Mono-for-autunomous-driving/path_recorder/global_optimize.csv", ios::app);
            foutC.setf(ios::fixed, ios::floatfield);
            foutC.precision(0);
            foutC<<gps_t<<" ";
            foutC.precision(5);
            foutC <<globalEstimator.global_path.poses.back().pose.position.x<< " "
                    << globalEstimator.global_path.poses.back().pose.position.y<< " "
                    << globalEstimator.global_path.poses.back().pose.position.z << " "
                    << 0 << " "
                    << 0 << " "
                    << 0 << " "
                    << 1 <<endl;
            foutC.close();
            pub_global_path.publish(globalEstimator.global_path);
            break;
        }
        else if(gps_t < t - 0.1){
            gpsQueue.pop();
        }
            
        else if(gps_t > t + 0.1){
            vioQueue.pop();
            break;
        }
    }
    m_global.unlock();

    //read, extrinsic calibration and publish
    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    publish_car_model(t, global_t, global_q);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_optimize");
    ros::NodeHandle n("~");
    global_path = globalEstimator.global_path;
    global_path.header.frame_id = "world";
    extrinsicPara<< cosTheta, -sinTheta, 0, 0, 
                    sinTheta, cosTheta , 0, 0, 
                    0, 0, 1, 0,
                    0, 0, 0, 1;


    ros::Subscriber sub_GPS = n.subscribe("/navsat/fix", 100, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 1000, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    ros::spin();
    return 0;
}
