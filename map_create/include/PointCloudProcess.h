/************************************
 @Time : 2021/11/23 下午2:28
 @Author : WenkyJong
 @Site : MianYang SWUST
 @File : PointCloudProcess.cpp
 @Contact: wenkyjong1996@gmail.com
 @desc:
 ****/
//
// Created by zwk on 2021/11/23.
//

#ifndef SRC_POINTCLOUDPROCESS_H
#define SRC_POINTCLOUDPROCESS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/filters/statistical_outlier_removal.h>


class PointCloudProcess {
public:
    PointCloudProcess(float voxel_param);
    inline void SetOutlierParam(int near_points,float outliers_threshold){
        near_points_=near_points;
        outliers_threshold_=outliers_threshold;
    }
    void VoexlFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out);
    void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out);
private:
    float voxel_param_;
    int near_points_;//临近点数
    float outliers_threshold_;//统计点数阈值
};

#endif //SRC_POINTCLOUDPROCESS_H
