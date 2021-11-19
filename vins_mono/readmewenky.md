#  modification of vins_mono by zwk
## A Robust and Versatile Monocular Visual-Inertial State Estimator

* 1 envirment  **ubuntu20.04**  ros **noetic** opencv **>4.0**
  * add #include <opencv2/imgproc/types_c.h> #include <opencv2/imgproc/imgproc_c.h> in file that
    occurred a compilation error 
* 2 launch file using only one realsence_color.launch that set up rosbag and rviz node 