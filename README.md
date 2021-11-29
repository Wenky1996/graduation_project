#  gama fussion by zwk
## using vio gets pose and maping with rgbd camera D435i

* 1 envirment  **ubuntu20.04**  ros **noetic** opencv **>4.0**
    * add #include <opencv2/imgproc/types_c.h> #include <opencv2/imgproc/imgproc_c.h> in file that
      occurred a compilation error
* 2 launch file using only one realsence_color.launch that set up rosbag and rviz node 
* 3 save the map using command 
```
  rosservice call /save_map
```
