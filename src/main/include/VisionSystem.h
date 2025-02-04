#pragma once

#include <optional>
#include <vector>

#include "Constants.h"
#include "frc/geometry/Pose2d.h"
#include "Camera.h"

class VisionSystem {
 public:
  VisionSystem() = default;
  void UpdateCameraPositionVis(frc::Pose3d robotPose);
  void SimulationPeriodic(frc::Pose2d simRobotPose);
  std::vector<std::optional<photon::EstimatedRobotPose>>
  GetCameraEstimatedPoses(frc::Pose3d robotPose);
  std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> GetPoseStdDevs(
      const std::vector<std::optional<photon::EstimatedRobotPose>>& poses);

 private:
  std::array<frc::Pose3d, 4> cameraLocations;
  nt::StructArrayPublisher<frc::Pose3d> cameraLocationsPub{
      nt::NetworkTableInstance::GetDefault()
          .GetTable("Vision")
          ->GetStructArrayTopic<frc::Pose3d>("CameraLocations")
          .Publish()};

  std::array<Camera, 1> cameras{
      Camera{consts::vision::CAM_NAME, consts::vision::ROBOT_TO_CAM,
             consts::vision::SINGLE_TAG_STD_DEV,
             consts::vision::MULTI_TAG_STD_DEV, true}};
};
