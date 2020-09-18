#ifndef _FLOOR_SEGMENT_H_
#define _FLOOR_SEGMENT_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <mutex>

#include "CloudSegment.h"


struct floorSegmentParams
{
    floorSegmentParams()
      : visualize(false)
      , r_min_square(0.3 * 0.3)
      , r_max_square(20 * 20)
      , n_bins(30)
      , n_segments(180)
      , max_dist_to_line(0.15)
      , max_slope(1)
      , max_error_square(0.01)
      , long_threshold(2.0)
      , max_long_height(0.1)
      , max_start_height(0.2)
      , sensor_height(0.2)
      , line_search_angle(0.2)
      , n_threads(4)
    {
    }

    bool   visualize;
    double r_min_square;
    double r_max_square;
    int    n_bins;
    int    n_segments;
    double max_dist_to_line;
    double max_slope;
    double max_error_square;
    double long_threshold;
    double max_long_height;
    double max_start_height;
    double sensor_height;
    double line_search_angle;
    int    n_threads;
};

typedef pcl::PointCloud<pcl::PointXYZI>          pclPointCloud;
typedef std::pair<pcl::PointXYZI, pcl::PointXYZI> PointLine;

class floorSegmentation
{
public:
    floorSegmentation(const floorSegmentParams& params = floorSegmentParams());

    void segment(const pclPointCloud& cloud, std::vector<int>* segmentation);

private:
    void assignCluster(std::vector<int>* segmentation);

    void assignClusterThread(const unsigned int& start_index, const unsigned int& end_index,
                             std::vector<int>* segmentation);

    void insertPoints(const pclPointCloud& cloud);

    void insertionThread(const pclPointCloud& cloud, const size_t start_index, const size_t end_index);

    void getMinZPoints(pclPointCloud* out_cloud);

    void getLines(std::list<PointLine>* lines);

    void lineFitThread(const unsigned int start_index, const unsigned int end_index, std::list<PointLine>* lines,
                       std::mutex* lines_mutex);

    pcl::PointXYZI minZPointTo3d(const cloudBin::MinZPoint& min_z_point, const double& angle);

    void getMinZPointCloud(pclPointCloud* cloud);

    void visualizePointCloud(const pclPointCloud::ConstPtr& cloud, const std::string& id = "pointcloud");

    void visualizeLines(const std::list<PointLine>& lines);

    void visualize(const std::list<PointLine>& lines, const pclPointCloud::ConstPtr& cloud,
                   const pclPointCloud::ConstPtr& floor_cloud, const pclPointCloud::ConstPtr& nofloor_cloud);

private:
    const floorSegmentParams params_;

    std::vector<cloudSegment> segments_;

    std::vector<std::pair<int, int> > bin_index_;

    std::vector<cloudBin::MinZPoint> segment_coordinates_;

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};  // class floorSegmentation


#endif
