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

#include "../include/PointCloudProcess.h"

PointCloudProcess::PointCloudProcess(float voxel_param) :voxel_param_{voxel_param}{

}

void PointCloudProcess::VoexlFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_in,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out) {
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(pc_in);
    sor.setLeafSize(voxel_param_,voxel_param_,voxel_param_);
    sor.filter(*pc_out);
}
