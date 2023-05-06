#ifndef DTW_H
#define DTW_H

#include <vector>
#include <Eigen/Eigen>
#include <limits>

class DTW {
public:
    DTW() {}
    ~DTW() {}

    double calculateDTWDistance(const std::vector<Eigen::Vector3d>& current_points, const std::vector<Eigen::Vector3d>& target_points);

private:
    double euclideanDistance(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);
};

#endif
