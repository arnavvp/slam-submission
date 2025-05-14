#pragma once

#include <Eigen/Dense>
#include "dv_msgs/msg/indexed_cone.hpp"

std::vector<int> performDataAssociation(
    const std::vector<double>& mu_t,
    const std::vector<dv_msgs::msg::IndexedCone>& conesFromPerception,
    const Eigen::Matrix2d& Q_cov);

double normalize_angle(double angle);
