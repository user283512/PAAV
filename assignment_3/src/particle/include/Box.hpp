#include <Eigen/Geometry> 

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
    Eigen::Vector3f pos_min;
    Eigen::Vector3f pos_max;
  };
}