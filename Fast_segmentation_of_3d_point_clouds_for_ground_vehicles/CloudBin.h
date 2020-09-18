
#ifndef _CLOUD_BIN_H_
#define _CLOUD_BIN_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>

class cloudBin
{
public:
    struct MinZPoint
    {
        MinZPoint() : z(0), d(0)
        {
        }

        MinZPoint(const double& d, const double& z) : z(z), d(d)
        {
        }

        bool operator==(const MinZPoint& comp)
        {
            return z == comp.z && d == comp.d;
        }

        double z;
        double d;
    };

private:
    std::atomic<bool>   has_point_;
    std::atomic<double> min_z;
    std::atomic<double> min_z_range;

public:
    cloudBin();

    cloudBin(const cloudBin& segment);

    void addPoint(const pcl::PointXYZI& point);

    void addPoint(const double& d, const double& z);

    MinZPoint getMinZPoint();

    inline bool hasPoint()
    {
        return has_point_;
    }

};  // class cloudBin


#endif