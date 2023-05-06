#include "DTW.h"

double DTW::calculateDTWDistance(const std::vector<Eigen::Vector3d>& current_points, const std::vector<Eigen::Vector3d>& target_points) {
    int n = current_points.size();
    int m = target_points.size();
    Eigen::MatrixXd dtw_matrix = Eigen::MatrixXd::Zero(n + 1, m + 1);

    // Initialize the first row and column with a large number
    double inf = std::numeric_limits<double>::infinity();
    for (int i = 1; i <= n; ++i) {
        dtw_matrix(i, 0) = inf;
    }
    for (int j = 1; j <= m; ++j) {
        dtw_matrix(0, j) = inf;
    }
    dtw_matrix(0, 0) = 0;

    // Compute the DTW distance
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= m; ++j) {
            double cost = euclideanDistance(current_points[i - 1], target_points[j - 1]);
            dtw_matrix(i, j) = cost + std::min({dtw_matrix(i - 1, j), dtw_matrix(i, j - 1), dtw_matrix(i - 1, j - 1)});
        }
    }
    return dtw_matrix(n, m);
}

double DTW::euclideanDistance(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
    return (point1 - point2).norm();
}
