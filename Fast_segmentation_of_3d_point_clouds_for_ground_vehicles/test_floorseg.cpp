
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <string>

#include "FloorSegment.h"

int main(void)
{
    // Read PLY FILE FOR TEST
    pcl::PointCloud<pcl::PointXYZI> cloud;

    std::string ply_file = "2000line_downsampled.pcd";

    pcl::io::loadPCDFile(ply_file, cloud);

    auto startTime = std::chrono::steady_clock::now();

    // FloorSegmentParams Initialize
    floorSegmentParams params;
    params.visualize         = false;  // show with true
    params.r_min_square      = 0.1 * 0.1;
    params.r_max_square      = 100 * 100;
    params.n_bins            = 100;
    params.n_segments        = 180;
    params.max_dist_to_line  = 0.15;
    params.max_slope         = 1;
    params.max_error_square  = 0.01;
    params.long_threshold    = 2.0;
    params.max_long_height   = 0.1;
    params.max_start_height  = 0.2;
    params.sensor_height     = 1.73;
    params.line_search_angle = 0.2;
    params.n_threads         = 4;

    // Run FloorSegment
    floorSegmentation segmenter(params);

    std::vector<int> labels;
    segmenter.segment(cloud, &labels);

    std::cout << "Segmentation Finished." << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    for (size_t i = 0; i < cloud.size(); ++i)
    {
        if (1 == labels[i])
        {
            output_onlyfloor_cloud_ptr->push_back(cloud[i]);
        }
        else
        {
            output_nofloor_cloud_ptr->push_back(cloud[i]);
        }
    }

    auto endTime      = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "-----------------Message Flow Is As Follows.----------------" << std::endl;

    std::cout << "Ellapse-Time: " << ellapsedTime.count() << " milliseconds." << std::endl;

    std::cout << "segment cloud points size: " << cloud.points.size() << std::endl;
    std::cout << "segment floor points size: " << output_onlyfloor_cloud_ptr->points.size() << std::endl;
    std::cout << "segment nofloor points size: " << output_nofloor_cloud_ptr->points.size() << std::endl;

    std::cout << "Save Segmentation Result." << std::endl;
    // SAVE FLOOR/NOFLOOR PLY FILE
    pcl::io::savePCDFile("noBins_floor.pcd", *output_nofloor_cloud_ptr);
    pcl::io::savePCDFile("onlyBins_floor.pcd", *output_onlyfloor_cloud_ptr);

    return 0;
}

