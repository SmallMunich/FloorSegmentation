#include <iostream>
#include <chrono>
#include "GPF.h"

int main(void)
{
	// read cloud data.
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr onlyfloor_cloud(new pcl::PointCloud<pcl::PointXYZI>());

	pcl::PCDReader reader;
	reader.read("apollo2.pcd", *cloudIn);

	std::cerr << "Read Cloud Data Points Size: " << cloudIn->points.size() << std::endl;
	// GPF 
	auto startTime = std::chrono::steady_clock::now();

	FloorSegment fs;
	fs.Run(cloudIn, nofloor_cloud, onlyfloor_cloud);
	
    auto endTime      = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cerr << "Ellapse-Time: " << ellapsedTime.count() << " milliseconds." << std::endl;

    std::cerr << "GPF Floor Cloud Data Size: " << onlyfloor_cloud->points.size() << std::endl;
    std::cerr << "GPF NoFloor Cloud Data Size: " << onlyfloor_cloud->points.size() << std::endl;

	// save floor cloud & nofloor cloud data.
	pcl::PCDWriter writer;
	onlyfloor_cloud->width = 1;
	onlyfloor_cloud->height = onlyfloor_cloud->points.size();
	writer.write("apollo2_onlyfloor.pcd", *onlyfloor_cloud);

	nofloor_cloud->width = 1;
	nofloor_cloud->height = nofloor_cloud->points.size();
	writer.write("apollo2_nofloor.pcd", *nofloor_cloud);

	return 0;
}