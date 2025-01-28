#ifndef LIDAR_OBSTACLE_DETECTION_RENDERER_HPP
#define LIDAR_OBSTACLE_DETECTION_RENDERER_HPP

#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Box.hpp"

namespace lidar_obstacle_detection
{
  struct Color
  {
    float r, g, b;
    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {
    }
  };

  static bool lidarActivated;

  enum class CameraAngle
  {
    XY,
    TopDown,
    Side,
    FPS
  };

  class Renderer
  {
  private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    unsigned long long rays_counter_;

  public:
    static void setLidarStatus();

    static bool getLidarStatus();

    void resetCam(const std::string &cloudName);

    Renderer();

    void updatePose(const std::string &id,
                    const Eigen::Affine3f &pose);

    static void keyboardCallback(const pcl::visualization::KeyboardEvent &event);

    void removeShape(const std::string &id);

    void RenderHighway();

    void updateShape(const std::string &id,
                     float opacity);

    void RenderRays(const Eigen::Vector3f &origin,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          const std::string &id);

    void addCircle(float centroid_x,
                   float centroid_y,
                   const std::string &id,
                   float radius,
                   int r,
                   int g,
                   int b);

    void ClearRays();

    void ClearViewer();

    void addText(float centroid_x,
                 float centroid_y,
                 int id);

    void RenderPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          const std::string &name,
                          Color color = Color(1, 1, 1));

    void RenderPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                          const std::string &name,
                          Color color = Color(-1, -1, -1));

    void RenderBox(Box box,
                   int id,
                   Color color = Color(1, 0, 0),
                   float opacity = 1.0);

    void RenderBox(BoxQ box,
                   int id,
                   Color color = Color(1, 0, 0),
                   float opacity = 1.0);

    void InitCamera(CameraAngle view_angle);

    bool WasViewerStopped() const;

    void SpinViewerOnce() const;
  };
}

#endif
