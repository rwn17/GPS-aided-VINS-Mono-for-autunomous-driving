# GPS-aided-VINS-Mono-for-autunomous-driving
Project for summenr intern @ THU Autonomous Driving Lab. I'm still working on it.

## 1.Prerequisites 

My environment is Ubuntu 18.04, ROS Melodic,OpenCV 3.3.1, Eigen 3.3.1

## 2.Build this project on ROS
```
    cd ~/catkin_ws/src
    git clone https://github.com/rwn17/GPS-aided-VINS-Mono-for-autunomous-driving.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
 ```
 
 ## 3.Download the UrbanNav Dataset
 
 Download [UrbanNav Dataset](https://www.polyu-ipn-lab.com/urbannav). I only test it on **UrbanNav-HK-Data20200314** sequence, more tests and some modifications will be made in a couple of weeks.
 
 ## 4.Run the GPS-aided VINS-Mono system
 
 ```
     roslaunch vins_estimator ubannav.launch
     roslaunch vins_estimator vins_rviz.launch
     rosbag play YOUR_PATH_TO_DATASET
 ```
 PS: I made a typo on **ubannav.launch**,bug I'm too lazy to correct it :)
 
 ## 5.The modification I made
 
Firstly, I incoperate the global_fusion pkg from VINS-Fusion into VINS-Mono and publish the optimized path to TOPIC:**/global_optimize_node/global_path**, rviz could subscribe this topic and show the path with yellow line. I will improve the loss function of VINS-Fuision this weekend.

Secondly, I use original benchmark_publisher pkg to subscribe the **novatel/inspvax**  and publish the ground truth path. Rviz will subscribe the topic and show the ground truth path with red line.

I have already incorperate this two topic in the urbannav.launch file. So if everything goes right, you could see three line on Rviz. Just as figure shows.

## 6. Acknowledgements
This project is mainly based on [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) and [VINS-Mono](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert). I use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal).

## 7. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

 
