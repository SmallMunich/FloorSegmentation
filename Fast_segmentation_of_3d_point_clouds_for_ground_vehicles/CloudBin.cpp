
#include "CloudBin.h"

#include <limits>


cloudBin::cloudBin() : min_z(std::numeric_limits<double>::max()), has_point_(false)
{
}

cloudBin::cloudBin(const cloudBin& segment) : min_z(std::numeric_limits<double>::max()), has_point_(false)
{
}

void cloudBin::addPoint(const pcl::PointXYZI& point)
{
    const double d = sqrt(point.x * point.x + point.y * point.y);
    addPoint(d, point.z);
}

void cloudBin::addPoint(const double& d, const double& z)
{
    has_point_ = true;
    if (z < min_z)
    {
        min_z       = z;
        min_z_range = d;
    }
}

cloudBin::MinZPoint cloudBin::getMinZPoint()
{
    MinZPoint point;

    if (has_point_)
    {
        point.z = min_z;
        point.d = min_z_range;
    }
    return point;
}
