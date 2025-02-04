#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include "frc/apriltag/AprilTagFields.h"

#include <frc/geometry/Transform3d.h>
#include <string>

namespace consts::yearspecific {
inline const frc::AprilTagFieldLayout TAG_LAYOUT =
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape);
}  // namespace consts::yearspecific

namespace consts::vision {
inline const std::string CAM_NAME{"cam"};
inline const frc::Transform3d ROBOT_TO_CAM{
    frc::Translation3d{0.265256_m, 0.2770_m, 0.209751_m},
    frc::Rotation3d{0_rad, -20_deg, -20_deg}};

inline const Eigen::Matrix<double, 3, 1> SINGLE_TAG_STD_DEV{4, 4, 8};
inline const Eigen::Matrix<double, 3, 1> MULTI_TAG_STD_DEV{0.5, 0.5, 1};
}  // namespace consts::vision