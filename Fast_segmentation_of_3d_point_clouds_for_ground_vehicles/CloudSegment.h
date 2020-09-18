
#ifndef _CLOUD_SEGMENT_FLOOR_H_
#define _CLOUD_SEGMENT_FLOOR_H_

#include <list>
#include <map>

#include "CloudBin.h"

class cloudSegment
{
public:
    typedef std::pair<cloudBin::MinZPoint, cloudBin::MinZPoint> Line;
    typedef std::pair<double, double>                           LocalLine;

private:
    const double max_slope_;
    const double max_error_;
    const double long_threshold_;
    const double max_long_height_;
    const double max_start_height_;
    const double sensor_height_;

    std::vector<cloudBin> bins_;
    std::list<Line>       lines_;

    LocalLine fitLocalLine(const std::list<cloudBin::MinZPoint>& points);

    double getMeanError(const std::list<cloudBin::MinZPoint>& points, const LocalLine& line);

    double getMaxError(const std::list<cloudBin::MinZPoint>& points, const LocalLine& line);

    Line localLineToLine(const LocalLine& local_line, const std::list<cloudBin::MinZPoint>& line_points);

public:
    cloudSegment(const unsigned int& n_bins, const double& max_slope, const double& max_error,
                 const double& long_threshold, const double& max_long_height, const double& max_start_height,
                 const double& sensor_height);

    double verticalDistanceToLine(const double& d, const double& z);

    void fitSegmentLines();

    inline cloudBin& operator[](const size_t& index)
    {
        return bins_[index];
    }

    inline std::vector<cloudBin>::iterator begin()
    {
        return bins_.begin();
    }

    inline std::vector<cloudBin>::iterator end()
    {
        return bins_.end();
    }

    bool getLines(std::list<Line>* lines);
};

#endif