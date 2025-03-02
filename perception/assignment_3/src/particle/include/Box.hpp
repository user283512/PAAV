#ifndef BOX_HPP_
#define BOX_HPP_

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
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
  };
}
#endif