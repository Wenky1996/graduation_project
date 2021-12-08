// @Time : 2021/8/23 上午9:39
// @Author : WenkyJong
// @Site : MianYang SWUST
// @File : Rgbd2PointCloud_node.cpp
// @Contact: wenkyjong1996@gmail.com
// @desc:
#include <queue>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

//opencv
#include <opencv4/opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>

#include <thread>
#include <chrono>

#include "PointCloudProcess.h"

using std::queue;
using std::string;
cv_bridge::CvImageConstPtr ptr_color;
cv_bridge::CvImageConstPtr ptr_depth;

queue<sensor_msgs::ImageConstPtr> color_img_buf;
queue<sensor_msgs::ImageConstPtr> depth_img_buf;

const double camera_factor = 1000; //怎么获取该值

ros::Publisher pointcloudPublish;
PointCloudProcess pointCloudProcess(0.04);
void ImageHandler(const sensor_msgs::ImageConstPtr &msg_image);
void DepthHandler(const sensor_msgs::ImageConstPtr &msg_depth);
void Rgbd2Point();

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);

int main(int argc ,char **argv){
    ros::init(argc, argv, "rgbd2pointcloud");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    pointCloudProcess.SetOutlierParam(50,2.0);

    auto subImage = n.subscribe("/camera/color/image_raw", 100, ImageHandler);
    auto subDepth = n.subscribe("/camera/aligned_depth_to_color/image_raw",100,DepthHandler);
    //pointcloudPublish=n.advertise<sensor_msgs::PointCloud2>("fusion_pointcloud",100);
    pointcloudPublish=n.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points",100);

    std::thread rgbd2pointcloud_thread{Rgbd2Point};
    ros::spin();
    return 0;
}

void ImageHandler(const sensor_msgs::ImageConstPtr &msg_image){
    color_img_buf.push(msg_image);
}

void DepthHandler(const sensor_msgs::ImageConstPtr &msg_depth){
    depth_img_buf.push(msg_depth);
}

void Rgbd2Point(){
    while (true){
        if(!depth_img_buf.empty()&&!color_img_buf.empty()){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
            auto ptr_msg_color =color_img_buf.front();
            color_img_buf.pop();
            auto ptr_msg_depth =depth_img_buf.front();
            depth_img_buf.pop();
            ptr_color=cv_bridge::toCvCopy(ptr_msg_color,sensor_msgs::image_encodings::BGR8);
            ptr_depth=cv_bridge::toCvCopy(ptr_msg_depth,sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat rgb_image=ptr_color->image;
            cv::Mat depth_image =ptr_depth->image;
            for(int i=0;i<depth_image.rows;i++) {
                for (int j = 0; j < depth_image.cols; j++) {
                    float d = depth_image.ptr<float>(i)[j];
                    if (d < 1.0e-6) {             //the invalid depth value
                        continue;
                    }
                    pcl::PointXYZRGB p;
                    p.z = static_cast<double >(d) / camera_factor;
                    p.x = (j - 3.2896954345703125e+02) * p.z / 6.091984252929688e+02;
                    p.y = (i - 2.3813449096679688e+02) * p.z / 6.098977661132812e+02;
                    p.b = rgb_image.ptr<uchar>(i)[j * 3];
                    p.g = rgb_image.ptr<uchar>(i)[j * 3 + 1];
                    p.r = rgb_image.ptr<uchar>(i)[j * 3 + 2];
                    cloud->push_back(p);
                }
            }
            sensor_msgs::PointCloud2 pub_pointcloud;
            pointCloudProcess.VoexlFilter(cloud,cloud_pub);
            pcl::toROSMsg(*cloud_pub,pub_pointcloud);
            pub_pointcloud.header.frame_id="map";
            pub_pointcloud.header.stamp=ptr_msg_color->header.stamp;
            pointcloudPublish.publish(pub_pointcloud);
            ROS_INFO("point cloud size = %d",cloud_pub->points.size());
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
