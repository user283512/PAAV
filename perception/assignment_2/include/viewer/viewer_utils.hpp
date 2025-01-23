#include <Eigen/Geometry>

namespace viewer
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

    bool isPointInside(float x, float y, float z)
    {
      return x >= x_min && x <= x_max &&
             y >= y_min && y <= y_max &&
             z >= z_min && z <= z_max;
    }
  };

  struct Color
  {
    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {
    }
  };

  enum class CameraAngle
  {
    XY,
    TopDown,
    Side,
    FPS
  };

}
