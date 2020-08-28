#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;
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
float theta = 5.0 / 180 * 3.1415926 ;
float cosTheta = cos(theta);
float sinTheta = sin(theta);

void odom_callback(const nav_msgs::Odometry odom_msg)
{
    
    //ROS_INFO("odom callback!");
    /*
    if (odom_msg->header.stamp.toSec() > benchmark.back().t)
      return;
  
    for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= odom_msg->header.stamp.toSec(); idx++)
        ;


    if (init++ < SKIP)
    {
        baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
                              odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z) *
                  Quaterniond(benchmark[idx - 1].qw,
                              benchmark[idx - 1].qx,
                              benchmark[idx - 1].qy,
                              benchmark[idx - 1].qz).inverse();
        baseTgt = Vector3d{odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z} -
                  baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        return;
    }

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(benchmark[idx - 1].t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Vector3d tmp_V = baseRgt * Vector3d{benchmark[idx - 1].vx,
                                        benchmark[idx - 1].vy,
                                        benchmark[idx - 1].vz};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    pub_odom.publish(odometry);
    */
     
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

    accept_origin<< odom_msg.pose.pose.position.x,
                    odom_msg.pose.pose.position.y,
                    odom_msg.pose.pose.position.z,
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
    cout<<"ground truth xyz is :"<<pose_stamped.pose.position.x<<" "<<pose_stamped.pose.position.y<<" "<<endl;

    //write ground truth to file 
    ofstream foutC("/home/weining/summer_intern/vins-mono-catkin_ws/src/VINS-Mono/path_recorder/ground_truth.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << odom_msg.header.stamp.toSec() << " ";
    foutC.precision(5);
    foutC << media(0) << " "
            << media(1)<< " "
            << media(2) << " "
            << 0 << " "
            << 0 << " "
            << 0 << " "
            << 1 << endl;
    foutC.close();
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

    ros::Subscriber sub_odom = n.subscribe("/navsat/odom", 1000, odom_callback);
    
    ros::Rate r(20);
    ros::spin();
}
