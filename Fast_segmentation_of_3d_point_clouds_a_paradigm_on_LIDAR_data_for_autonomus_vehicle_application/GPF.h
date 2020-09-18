
#ifndef _PLANE_FIT_H_
#define _PLANE_FIT_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>


class FloorSegment
{
public:
	FloorSegment();
	~FloorSegment();

	void Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud, 
			pcl::PointCloud<pcl::PointXYZI>::Ptr onlyfloor_cloud);
private:
	void estimate_plane(void);

	void extract_initial_seeds(const pcl::PointCloud<pcl::PointXYZI>& p_sorted);

private:

	int sensor_model_;
	int num_seg_;
	int num_iter_;
	int num_lpr_;

	double sensor_height_;
	double th_seeds_;
	double th_dist_;

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d 
	double d_;
	double th_dist_d_;

	double threshold_;
	
	Eigen::MatrixXf normal_;

};


#endif
