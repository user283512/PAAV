/**
 * The original author of the rendering code is Aaron Brown (https://github.com/awbrown90).
 * His code has been slightly modified to make it more structured.
 */

#ifndef LIDAR_OBSTACLE_DETECTION_RENDERER_HPP
#define LIDAR_OBSTACLE_DETECTION_RENDERER_HPP

#include <vector>
#include <string>
#include <cstdint>

#include <Eigen/Dense>                          // Eigen::Vector3f
#include <pcl/visualization/pcl_visualizer.h>   // pcl::visualization::PCLVisualizer e pcl::visualization::KeyboardEvent
#include <pcl/point_cloud.h>                    // pcl::PointCloud
#include <pcl/point_types.h>                    // pcl::PointXYZ, pcl::PointXYZI

#include "../include/Box.hpp"

namespace lidar_obstacle_detection
{
  struct Color
  {
    float r, g, b;
    Color(float r, float g, float b)
        : r{ r }, g{ g }, b{ b }
    {}
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
  public:
    Renderer() 
      : _viewer{ new pcl::visualization::PCLVisualizer("3D Viewer") },
        _rays_counter{ 0 }
    {}

    static void setLidarStatus();
    static bool getLidarStatus();
    static void keyboardCallback(const pcl::visualization::KeyboardEvent &event);
    void removeShape(int id);
    void RenderHighway();
    void RenderRays(const Eigen::Vector3f& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void addCircle(float centroid_x, float centroid_y, int id);
    void ClearRays();
    void ClearViewer();
    void addText(float centroid_x, float centroid_y, float centroid_z, const std::string& id);
    void RenderPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::string& name,
      const Color& color = Color(1.f, 1.f, 1.f)
    );
    void RenderPointCloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      const std::string& name,
      const Color& color = Color(-1.f, -1.f, -1.f)
    );
    void RenderBox(const Box& box, int id, const Color& color = Color(1.f, 0.f, 0.f), float opacity = 1.0f);
    void RenderBox(const BoxQ& box, int id, const Color& color = Color(1.f, 0.f, 0.f), float opacity = 1.0f);
    void InitCamera(CameraAngle view_angle);
    bool WasViewerStopped() const;
    void SpinViewerOnce() const;

  private:
    pcl::visualization::PCLVisualizer::Ptr _viewer;
    uint64_t _rays_counter;
  };
}

#endif