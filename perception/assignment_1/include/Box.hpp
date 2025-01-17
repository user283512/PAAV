#include <Eigen/Dense>      // Eigen::Vector3f
#include <Eigen/Geometry>   // Eigen::Quaternionf

namespace lidar_obstacle_detection
{
  struct BoxQ
  {
    Eigen::Vector3f bbox_transform;
    Eigen::Quaternionf bbox_quaternion;
    float cube_length;
    float cube_width;
    float cube_height;
  };

  struct Box
  {
    Eigen::Vector3f min_pos;
    Eigen::Vector3f max_pos;
  };
}

