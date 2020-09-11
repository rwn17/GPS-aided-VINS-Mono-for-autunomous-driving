#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "LocalCartesian.hpp"

using namespace std;
using namespace Eigen;

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;
bool isOdom = 0;
GeographicLib::LocalCartesian geoConverter;
bool initGPS = 0;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) != EOF)
        {
            t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};
int idx = 1;
vector<Data> benchmark;

ros::Publisher pub_odom;
ros::Publisher pub_path;
nav_msgs::Path path;

int init = 0;
Quaterniond baseRgt;
Vector3d baseTgt;
tf::Transform trans;
Matrix4d extrinsicPara;
Vector4d media;
Vector4d accept_origin;
Vector3d transfered_pose;
float theta = 0 / 180 * 3.1415926 ;
float cosTheta = cos(theta);
float sinTheta = sin(theta);
bool init_flag = true;
double origin_x,origin_y,origin_z;
double relative_x,relative_y,relative_z;

void odom_callback(const nav_msgs::Odometry odom_msg)
{
 
    if (init_flag == true){
        init_flag = false;
        origin_x = odom_msg.pose.pose.position.x;
        origin_y = odom_msg.pose.pose.position.y;
        origin_z = odom_msg.pose.pose.position.z;
    }
    else{
        relative_x = odom_msg.pose.pose.position.x - origin_x;
        relative_y = odom_msg.pose.pose.position.y - origin_y;
        relative_z = odom_msg.pose.pose.position.z - origin_z;
    }
    extrinsicPara<< cosTheta, -sinTheta,0,1, 
                    sinTheta, cosTheta,0,0, 
                    0, 0, 1,1,
                    0,0,0,1;
    nav_msgs::Odometry odometry;
    odometry = odom_msg;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    pub_odom.publish(odometry);
    geometry_msgs::PoseStamped pose_stamped;

    accept_origin<< relative_x,
                    relative_y,
                    relative_z,
                    1;
    media = extrinsicPara * accept_origin;
    pose_stamped.pose.position.x = media(0);
    pose_stamped.pose.position.y = media(1);
    pose_stamped.pose.position.z = media(2);
    //pose_stamped.pose = odom_msg.pose.pose;
    pose_stamped.header = odom_msg.header;
    //pose_stamped.header.frame_id = "world";
    //pose_stamped.child_frame_id = "world";
    path.header = odom_msg.header;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
    //write ground truth to file 
    ofstream foutC("/home/weining/summer_intern/gps_aided_vins/src/GPS-aided-VINS-Mono-for-autunomous-driving/path_recorder/ground_truth.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << odom_msg.header.stamp.toSec() << " ";
    foutC.precision(5);
    foutC << media(0) << " "
            << media(1) << " "
            << media(2) << " "
            << 0 << " "
            << 0 << " "
            << 0 << " "
            << 1 << endl;
}

void gps_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude; 
    double position[3];
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, position[0], position[1], position[2]);
    extrinsicPara<< cosTheta, -sinTheta,0,1, 
                    sinTheta, cosTheta,0,0, 
                    0, 0, 1,1,
                    0,0,0,1;
    geometry_msgs::PoseStamped pose_stamped;
    accept_origin<< position[0],
                    position[1],
                    position[2],
                    1;
    media = extrinsicPara * accept_origin;
    pose_stamped.pose.position.x = media(0);
    pose_stamped.pose.position.y = media(1);
    pose_stamped.pose.position.z = media(2);
    //pose_stamped.pose = odom_msg.pose.pose;
    pose_stamped.header = GPS_msg->header;
    //pose_stamped.header.frame_id = "world";
    //pose_stamped.child_frame_id = "world";
    path.header = GPS_msg->header;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
    //write ground truth to file 
    ofstream foutC("/home/weining/summer_intern/gps_aided_vins/src/GPS-aided-VINS-Mono-for-autunomous-driving/path_recorder/ground_truth.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << GPS_msg->header.stamp.toSec() << " ";
    foutC.precision(5);
    foutC << media(0) << " "
            << media(1) << " "
            << media(2) << " "
            << 0 << " "
            << 0 << " "
            << 0 << " "
            << 1 << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_publisher");
    ros::NodeHandle n("~");
    /*
    string csv_file = readParam<string>(n, "data_name");
    std::cout << "load ground truth " << csv_file << std::endl;
    FILE *f = fopen(csv_file.c_str(), "r");
    if (f==NULL)
    {
      ROS_WARN("can't load ground truth; wrong path");
      //std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
      return 0;
    }
    char tmp[10000];
    if (fgets(tmp, 10000, f) == NULL)
    {
        ROS_WARN("can't load ground truth; no data available");
    }
    while (!feof(f))
        benchmark.emplace_back(f);
    fclose(f);
    benchmark.pop_back();
    ROS_INFO("Data loaded: %d", (int)benchmark.size());
    */
    pub_odom = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Subscriber sub_fix;
    if(isOdom == 1){
        sub_fix = n.subscribe("/navsat/odom", 1000, odom_callback);
    }
    else
    {
        cout<<"use fix"<<endl;
        sub_fix = n.subscribe("/gps/fix",1000, gps_callback);
    }
    
    ros::Rate r(20);
    ros::spin();
}
