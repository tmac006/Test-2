
#include "VisionSystem.h"

#include <frc/geometry/Pose2d.h>

#include <vector>

void VisionSystem::UpdateCameraPositionVis(frc::Pose3d robotPose) {
  cameraLocations[0] = robotPose.TransformBy(consts::vision::ROBOT_TO_CAM);
 

  cameraLocationsPub.Set(cameraLocations);
}

std::vector<std::optional<Eigen::Matrix<double, 3, 1>>>
VisionSystem::GetPoseStdDevs(
    const std::vector<std::optional<photon::EstimatedRobotPose>>& poses) {
  std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> allStdDevs;
  int i = 0;
  for (auto& cam : cameras) {
    if (poses[i].has_value()) {
      auto stddev =
          cam.GetEstimationStdDevs(poses[i].value().estimatedPose.ToPose2d());
      allStdDevs.push_back(stddev);
    } else {
      allStdDevs.push_back(std::nullopt);
    }
    i++;
  }
  return allStdDevs;
}

std::vector<std::optional<photon::EstimatedRobotPose>>
VisionSystem::GetCameraEstimatedPoses(frc::Pose3d robotPose) {
  std::vector<std::optional<photon::EstimatedRobotPose>> allPoses;
  for (auto& cam : cameras) {
    allPoses.push_back(cam.GetEstimatedGlobalPose(robotPose));
  }
  return allPoses;
}

void VisionSystem::SimulationPeriodic(frc::Pose2d simRobotPose) {
  for (auto& cam : cameras) {
    cam.SimPeriodic(simRobotPose);
  }
}