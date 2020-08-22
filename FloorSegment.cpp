
#include "FloorSegment.h"

#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>

floorSegmentation::floorSegmentation(const floorSegmentParams& params)
  : params_(params)
  , segments_(params.n_segments,
              cloudSegment(params.n_bins, params.max_slope, params.max_error_square, params.long_threshold,
                           params.max_long_height, params.max_start_height, params.sensor_height))
{
    if (params.visualize)
    {
        viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    }
}

void floorSegmentation::segment(const pclPointCloud& cloud, std::vector<int>* segmentation)
{
    std::cout << "Segment Cloud With " << cloud.size() << " Points." << std::endl;
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    segmentation->clear();
    segmentation->resize(cloud.size(), 0);
    bin_index_.resize(cloud.size());
    segment_coordinates_.resize(cloud.size());

    insertPoints(cloud);
    std::cout << "InsertPoints Finished" << std::endl;

    std::list<PointLine> lines;
    if (params_.visualize)
    {
        getLines(&lines);
    }
    else
    {
        getLines(NULL);
    }

    assignCluster(segmentation);
    std::cout << "assignCluster Finished" << std::endl;

    if (params_.visualize)
    {
        pclPointCloud::Ptr nofloor_cloud(new pclPointCloud());
        pclPointCloud::Ptr floor_cloud(new pclPointCloud());

        for (size_t i = 0; i < cloud.size(); ++i)
        {
            if (1 == segmentation->at(i))
            {
                floor_cloud->push_back(cloud[i]);
            }
            else
            {
                nofloor_cloud->push_back(cloud[i]);
            }
        }

        pclPointCloud::Ptr Min_Cloud(new pclPointCloud());
        getMinZPointCloud(Min_Cloud.get());
        visualize(lines, Min_Cloud, floor_cloud, nofloor_cloud);
    }

    std::chrono::high_resolution_clock::time_point end   = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli>      fp_ms = end - start;
    std::cout << "Elliapse Time " << fp_ms.count() << " ms." << std::endl;
}

void floorSegmentation::getLines(std::list<PointLine>* lines)
{
    std::mutex               lines_mutex;
    std::vector<std::thread> thread_vec(params_.n_threads);

    for (unsigned int i = 0; i < (size_t)params_.n_threads; ++i)
    {
        const unsigned int start_index = params_.n_segments / params_.n_threads * i;
        const unsigned int end_index   = params_.n_segments / params_.n_threads * (i + 1);
        thread_vec[i] =
            std::thread(&floorSegmentation::lineFitThread, this, start_index, end_index, lines, &lines_mutex);
    }

    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void floorSegmentation::lineFitThread(const unsigned int start_index, const unsigned int end_index,
                                      std::list<PointLine>* lines, std::mutex* lines_mutex)
{
    const bool   visualize = lines;
    const double seg_step  = 2 * M_PI / params_.n_segments;
    double       angle     = -M_PI + seg_step / 2 + seg_step * start_index;

    for (unsigned int i = start_index; i < end_index; ++i)
    {
        segments_[i].fitSegmentLines();

        if (visualize)
        {
            std::list<cloudSegment::Line> segments_lines;
            segments_[i].getLines(&segments_lines);

            for (auto line_iter = segments_lines.begin(); line_iter != segments_lines.end(); ++line_iter)
            {
                const pcl::PointXYZI start = minZPointTo3d(line_iter->first, angle);
                const pcl::PointXYZI end   = minZPointTo3d(line_iter->second, angle);
                lines_mutex->lock();
                lines->emplace_back(start, end);
                lines_mutex->unlock();
            }

            angle += seg_step;
        }
    }
}

void floorSegmentation::getMinZPointCloud(pclPointCloud* cloud)
{
    const double seg_step = 2 * M_PI / params_.n_segments;
    double       angle    = -M_PI + seg_step / 2;

    for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter)
    {
        for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter)
        {
            const pcl::PointXYZI min = minZPointTo3d(bin_iter->getMinZPoint(), angle);
            cloud->push_back(min);
        }

        angle += seg_step;
    }
}

pcl::PointXYZI floorSegmentation::minZPointTo3d(const cloudBin::MinZPoint& min_z_point, const double& angle)
{
    pcl::PointXYZI point;
    point.x = cos(angle) * min_z_point.d;
    point.y = sin(angle) * min_z_point.d;
    point.z = min_z_point.z;

    return point;
}

void floorSegmentation::assignCluster(std::vector<int>* segmentation)
{
    std::vector<std::thread> thread_vec(params_.n_threads);

    const size_t cloud_size = segmentation->size();

    for (unsigned int i = 0; i < (size_t)params_.n_threads; ++i)
    {
        const unsigned int start_index = cloud_size / params_.n_threads * i;
        const unsigned int end_index   = cloud_size / params_.n_threads * (i + 1);
        thread_vec[i] =
            std::thread(&floorSegmentation::assignClusterThread, this, start_index, end_index, segmentation);
    }

    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void floorSegmentation::assignClusterThread(const unsigned int& start_index, const unsigned int& end_index,
                                            std::vector<int>* segmentation)
{
    const double segment_step = 2 * M_PI / params_.n_segments;

    for (unsigned int i = start_index; i < end_index; ++i)
    {
        cloudBin::MinZPoint point_2d = segment_coordinates_[i];

        const int segment_index = bin_index_[i].first;

        if (segment_index >= 0)
        {
            double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);

            int steps = 1;

            while (-1 == dist && steps * segment_step < params_.line_search_angle)
            {
                int index_first = segment_index + steps;
                while (index_first >= params_.n_segments)
                    index_first -= params_.n_segments;

                int index_second = segment_index - steps;
                while (index_second < 0)
                    index_second += params_.n_segments;

                const double dist_first  = segments_[index_first].verticalDistanceToLine(point_2d.d, point_2d.z);
                const double dist_second = segments_[index_second].verticalDistanceToLine(point_2d.d, point_2d.z);

                if (dist_first > dist)
                {
                    dist = dist_first;
                }
                if (dist_second > dist)
                {
                    dist = dist_second;
                }

                ++steps;
            }

            if (dist < params_.max_dist_to_line && -1 != dist)
            {
                segmentation->at(i) = 1;
            }
        }
    }
}

void floorSegmentation::getMinZPoints(pclPointCloud* out_cloud)
{
    const double seg_step = 2 * M_PI / params_.n_segments;
    const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square)) / params_.n_bins;

    const double r_min = sqrt(params_.r_min_square);
    double       angle = -M_PI + seg_step / 2;

    for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter)
    {
        double dist = r_min + bin_step / 2;
        for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter)
        {
            pcl::PointXYZI point;
            if (bin_iter->hasPoint())
            {
                cloudBin::MinZPoint min_z_point(bin_iter->getMinZPoint());
                point.x = cos(angle) * min_z_point.d;
                point.y = sin(angle) * min_z_point.d;
                point.z = min_z_point.z;

                out_cloud->push_back(point);
            }

            dist += bin_step;
        }

        angle += seg_step;
    }
}

void floorSegmentation::insertPoints(const pclPointCloud& cloud)
{
    std::vector<std::thread> threads(params_.n_threads);

    const size_t points_per_thread = cloud.size() / params_.n_threads;

    for (unsigned int i = 0; i < (size_t)(params_.n_threads - 1); ++i)
    {
        const size_t start_index = i * points_per_thread;
        const size_t end_index   = (i + 1) * points_per_thread - 1;

        threads[i] = std::thread(&floorSegmentation::insertionThread, this, cloud, start_index, end_index);
    }

    const size_t start_index = (params_.n_threads - 1) * points_per_thread;
    const size_t end_index   = cloud.size() - 1;

    threads[params_.n_threads - 1] =
        std::thread(&floorSegmentation::insertionThread, this, cloud, start_index, end_index);

    for (auto it = threads.begin(); it != threads.end(); ++it)
    {
        it->join();
    }
}

void floorSegmentation::insertionThread(const pclPointCloud& cloud, const size_t start_index, const size_t end_index)
{
    const double segment_step = 2 * M_PI / params_.n_segments;
    const double bin_step     = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square)) / params_.n_bins;

    const double r_min = sqrt(params_.r_min_square);

    for (unsigned int i = start_index; i < end_index; ++i)
    {
        pcl::PointXYZI point(cloud[i]);

        const double range_square = point.x * point.x + point.y * point.y;
        const double range        = sqrt(range_square);

        if (range_square < params_.r_max_square && range_square > params_.r_min_square)
        {
            const double angle = std::atan2(point.y, point.x);

            const unsigned int bin_index     = (range - r_min) / bin_step;
            const unsigned int segment_index = (angle + M_PI) / segment_step;

            segments_[segment_index][bin_index].addPoint(range, point.z);

            bin_index_[i] = std::make_pair(segment_index, bin_index);
        }
        else
        {
            bin_index_[i] = std::make_pair<int, int>(-1, -1);
        }

        segment_coordinates_[i] = cloudBin::MinZPoint(range, point.z);
    }
}

void floorSegmentation::visualizePointCloud(const pclPointCloud::ConstPtr& cloud, const std::string& id)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr showcloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(size_t i=0; i < cloud->size(); ++i)
    {
        pcl::PointXYZ pt;
        pt.x = (*cloud)[i].x;
        pt.y = (*cloud)[i].y;
        pt.z = (*cloud)[i].z;

        showcloud->push_back(pt);
    }

    viewer_->addPointCloud(showcloud, id, 0);
}

void floorSegmentation::visualizeLines(const std::list<PointLine>& lines)
{
    size_t counter = 0;
    for (auto it = lines.begin(); it != lines.end(); ++it)
    {
        viewer_->addLine<pcl::PointXYZI>(it->first, it->second, std::to_string(counter++));
    }
}

void floorSegmentation::visualize(const std::list<PointLine>& lines, const pclPointCloud::ConstPtr& cloud,
                                  const pclPointCloud::ConstPtr& floor_cloud,
                                  const pclPointCloud::ConstPtr& nofloor_cloud)
{
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);

    visualizePointCloud(cloud, "Min_Cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "Min_Cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "Min_Cloud");

    visualizePointCloud(floor_cloud, "Floor_Cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f,
                                              "Floor_Cloud");

    visualizePointCloud(nofloor_cloud, "NoFloor_Cloud");
    visualizeLines(lines);

    while (!viewer_->wasStopped())
    {
        viewer_->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100000));
    }
}

