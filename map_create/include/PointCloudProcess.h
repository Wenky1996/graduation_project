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


class PointCloudProcess {
public:
    PointCloudProcess(float voxel_param);
    void VoexlFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out);
private:
    float voxel_param_;
};


#endif //SRC_POINTCLOUDPROCESS_H
