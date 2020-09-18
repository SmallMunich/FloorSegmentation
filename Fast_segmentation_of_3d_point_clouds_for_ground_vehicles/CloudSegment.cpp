
#include "CloudSegment.h"

cloudSegment::cloudSegment(const unsigned int& n_bins, const double& max_slope, const double& max_error,
                           const double& long_threshold, const double& max_long_height, const double& max_start_height,
                           const double& sensor_height)
  : bins_(n_bins)
  , max_slope_(max_slope)
  , max_error_(max_error)
  , long_threshold_(long_threshold)
  , max_long_height_(max_long_height)
  , max_start_height_(max_start_height)
  , sensor_height_(sensor_height)
{
}

void cloudSegment::fitSegmentLines()
{
    auto line_start = bins_.begin();

    while (!line_start->hasPoint())
    {
        ++line_start;
        if (line_start == bins_.end())
            return;
    }

    bool   is_long_line      = false;
    double cur_ground_height = -sensor_height_;

    std::list<cloudBin::MinZPoint> current_line_points(1, line_start->getMinZPoint());
    LocalLine                      cur_line = std::make_pair(0, 0);

    for (auto line_iter = line_start + 1; line_iter != bins_.end(); ++line_iter)
    {
        if (line_iter->hasPoint())
        {
            cloudBin::MinZPoint cur_point = line_iter->getMinZPoint();

            if (cur_point.d - current_line_points.back().d > long_threshold_)
            {
                is_long_line = true;
            }

            if (current_line_points.size() > 1)
            {
                double excepted_z = std::numeric_limits<double>::max();

                if (is_long_line && current_line_points.size() > 2)
                {
                    excepted_z = cur_line.first * cur_point.d + cur_line.second;
                }

                current_line_points.push_back(cur_point);

                cur_line = fitLocalLine(current_line_points);

                const double error = getMaxError(current_line_points, cur_line);

                if (error > max_error_ || std::fabs(cur_line.first) > max_slope_ ||
                    is_long_line && std::fabs(excepted_z - cur_point.z) > max_long_height_)
                {
                    current_line_points.pop_back();

                    if (current_line_points.size() > 2)
                    {
                        const LocalLine new_line = fitLocalLine(current_line_points);
                        lines_.push_back(localLineToLine(new_line, current_line_points));
                        cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
                    }
                    is_long_line = false;

                    current_line_points.erase(current_line_points.begin(), current_line_points.end());

                    --line_iter;
                }
                else
                {
                }
            }
            else
            {
                if (cur_point.d - current_line_points.back().d < long_threshold_ &&
                    std::fabs(current_line_points.back().z - cur_ground_height) < max_start_height_)
                {
                    current_line_points.push_back(cur_point);
                }
                else
                {
                    current_line_points.clear();
                    current_line_points.push_back(cur_point);
                }
            }
        }
    }

    if (current_line_points.size() > 2)
    {
        const LocalLine new_line = fitLocalLine(current_line_points);
        lines_.push_back(localLineToLine(new_line, current_line_points));
    }
}

cloudSegment::Line cloudSegment::localLineToLine(const LocalLine&                      local_line,
                                                 const std::list<cloudBin::MinZPoint>& line_points)
{
    Line         line;
    const double first_d  = line_points.front().d;
    const double second_d = line_points.back().d;

    const double first_z  = local_line.first * first_d + local_line.second;
    const double second_z = local_line.first * second_d + local_line.second;

    line.first.z  = first_z;
    line.first.d  = first_d;
    line.second.z = second_z;
    line.second.d = second_d;
    return line;
}

double cloudSegment::verticalDistanceToLine(const double& d, const double& z)
{
    static const double kMargin  = 0.1;
    double              distance = -1;

    for (auto it = lines_.begin(); it != lines_.end(); ++it)
    {
        if (it->first.d - kMargin < d && it->second.d + kMargin > d)
        {
            const double delta_z    = it->second.z - it->first.z;
            const double delta_d    = it->second.d - it->first.d;
            const double excepted_z = (d - it->first.d) / delta_d * delta_z + it->first.z;
            distance                = std::fabs(z - excepted_z);
        }
    }
    return distance;
}

double cloudSegment::getMeanError(const std::list<cloudBin::MinZPoint>& points, const LocalLine& line)
{
    double error_sum = 0;
    for (auto it = points.begin(); it != points.end(); ++it)
    {
        const double residual = (line.first * it->d + line.second) - it->z;
        error_sum += residual * residual;
    }
    return error_sum / points.size();
}

double cloudSegment::getMaxError(const std::list<cloudBin::MinZPoint>& points, const LocalLine& line)
{
    double max_error = 0;
    for (auto it = points.begin(); it != points.end(); ++it)
    {
        const double residual = (line.first * it->d + line.second) - it->z;
        const double error    = residual * residual;
        if (error > max_error)
        {
            max_error = error;
        }
    }
    return max_error;
}

cloudSegment::LocalLine cloudSegment::fitLocalLine(const std::list<cloudBin::MinZPoint>& points)
{
    const unsigned int n_points = points.size();
    Eigen::MatrixXd    X(n_points, 2);
    Eigen::VectorXd    Y(n_points);
    unsigned int       counter = 0;

    for (auto iter = points.begin(); iter != points.end(); iter++)
    {
        X(counter, 0) = iter->d;
        X(counter, 1) = 1;
        Y(counter)    = iter->z;
        ++counter;
    }

    Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
    LocalLine       line_result;
    line_result.first  = result(0);
    line_result.second = result(1);

    return line_result;
}

bool cloudSegment::getLines(std::list<Line>* lines)
{
    if (lines_.empty())
    {
        return false;
    }
    else
    {
        *lines = lines_;
        return true;
    }
}

