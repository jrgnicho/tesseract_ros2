#ifndef TESSERACT_ROSUTILS_CONVERSIONS_H
#define TESSERACT_ROSUTILS_CONVERSIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_eigen/tf2_eigen.h>
//#include <tesseract_process_planners/process_definition.h>
//#include <tesseract_process_planners/process_planner.h>
#include <iostream>
#include <fstream>

namespace tesseract_rosutils
{
/**
 * @brief Convert STD Vector to Eigen Vector
 * @param vector The STD Vector to be converted
 * @return Eigen::VectorXd
 */
inline Eigen::VectorXd toEigen(const std::vector<double>& vector)
{
  return Eigen::VectorXd::Map(vector.data(), static_cast<long>(vector.size()));
}

/**
 * @brief Converts JointState position to Eigen vector in the order provided by joint_names
 * @param joint_state The JointState
 * @param joint_names The vector joint names used to order the output
 * @return Eigen::VectorXd in the same order as joint_names
 */
inline Eigen::VectorXd toEigen(const sensor_msgs::msg::JointState& joint_state,
                               const std::vector<std::string>& joint_names)
{
  Eigen::VectorXd position;
  position.resize(static_cast<long>(joint_state.position.size()));
  int i = 0;
  for (const auto& joint_name : joint_names)
  {
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    assert(it != joint_state.name.end());
    size_t index = static_cast<size_t>(std::distance(joint_state.name.begin(), it));
    position[i] = joint_state.position[index];
    ++i;
  }

  return position;
}

inline Eigen::MatrixXd toEigen(const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                               const std::vector<std::string>& joint_names)
{
  Eigen::MatrixXd trajectory;
  trajectory.resize(joint_trajectory.points.size(), joint_trajectory.points.front().positions.size());
  for (std::size_t j = 0; j < joint_trajectory.points.size(); j++)
  {
    const auto traj_point = joint_trajectory.points[j];
    Eigen::VectorXd position;
    position.resize(static_cast<long>(traj_point.positions.size()));
    int i = 0;
    for (const auto& joint_name : joint_names)
    {
      auto it = std::find(joint_trajectory.joint_names.begin(), joint_trajectory.joint_names.end(), joint_name);
      assert(it != joint_trajectory.joint_names.end());
      size_t index = static_cast<size_t>(std::distance(joint_trajectory.joint_names.begin(), it));
      position[i] = traj_point.positions[index];
      ++i;
    }
    trajectory.row(j) = position;
  }
  return trajectory;
}


/**
 * @brief Convert a vector of waypoints into a pose array
 * @param waypoints A vector of waypoints
 * @return Pose Array
 */
// geometry_msgs::msg::PoseArray toPoseArray(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints)
//{
//  geometry_msgs::msg::PoseArray pose_array;
//  for (const auto& wp : waypoints)
//  {
//    if (wp->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT)
//    {
//      const tesseract_motion_planners::CartesianWaypoint::Ptr& cwp =
//          std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(wp);
//      geometry_msgs::msg::Pose pose = tf2::toMsg(cwp->cartesian_position_);
//      pose_array.poses.push_back(pose);
//    }
////    else
////    {
////      ROS_ERROR("toPoseArray only support Cartesian Waypoints at this time.");  // BUG: Throw something here
////    }
//  }

//  return pose_array;
//}

/**
 * @brief Convert a process definition into a single pose array
 * @param process_definition A process definition
 * @return Pose Array
 */
// geometry_msgs::msg::PoseArray toPoseArray(const tesseract_process_planners::ProcessDefinition& process_definition)
//{
//  geometry_msgs::msg::PoseArray full_path;
//  for (size_t i = 0; i < process_definition.segments.size(); ++i)
//  {
//    geometry_msgs::msg::PoseArray poses = toPoseArray(process_definition.segments[i].approach);
//    full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());

//    poses = toPoseArray(process_definition.segments[i].process);
//    full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());

//    poses = toPoseArray(process_definition.segments[i].departure);
//    full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());

//    if (i < process_definition.transitions.size())
//    {
//      poses = toPoseArray(process_definition.transitions[i].transition_from_end);
//      full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());
//    }
//  }

//  return full_path;
//}

/**
 * @brief Convert a joint trajector to csv formate and write to file
 * @param joint_trajectory Joint trajectory to be writen to file
 * @param file_path The location to save the file
 * @return true if successful
 */
inline bool toCSVFile(const trajectory_msgs::msg::JointTrajectory& joint_trajectory, const std::string& file_path)
{
  std::ofstream myfile;
  myfile.open(file_path);

  // Write Joint names as header
  std::copy(joint_trajectory.joint_names.begin(),
            joint_trajectory.joint_names.end(),
            std::ostream_iterator<std::string>(myfile, ","));
  myfile << ",\n";
  for (const auto& point : joint_trajectory.points)
  {
    std::copy(point.positions.begin(), point.positions.end(), std::ostream_iterator<double>(myfile, ","));
    myfile << "," + std::to_string(point.time_from_start.sec + point.time_from_start.nanosec / 1e9) + ",\n";
  }
  myfile.close();
  return true;
}
}  // namespace tesseract_rosutils
#endif  // TESSERACT_ROSUTILS_CONVERSIONS_H
