/**
 * The original author of the rendering code is Aaron Brown (https://github.com/awbrown90).
 * His code has been slightly modified to make it more structured.
 */

#include <iostream>
#include <format>

#include <pcl/common/common.h>                  // Per pcl::ModelCoefficients
#include <pcl/visualization/cloud_viewer.h>     // Per visualizzare le nuvole di punti
#include <pcl/visualization/point_cloud_color_handlers.h> // Per color handler
#include <pcl/visualization/pcl_visualizer.h>   // Per shape rendering properties

#include "../include/Renderer.hpp"

namespace lidar_obstacle_detection
{
  void Renderer::addCircle(float centroid_x, float centroid_y, int id)
  {
    pcl::ModelCoefficients c_coeff;
    c_coeff.values.resize(3);      // We need 3 values
    c_coeff.values[0] = centroid_x;
    c_coeff.values[1] = centroid_y;
    c_coeff.values[2] = 0.4f;      // radius

    char buff[32]{};
    std::format_to_n(buff, sizeof(buff), "c_{}", id);

    _viewer->addCircle(c_coeff, buff, 0); 
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 1000, buff);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, buff);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 10000.0, buff);
  }

  void Renderer::addText(float centroid_x, float centroid_y, float centroid_z, std::string_view id)
  {
    _viewer->addText3D(id.data(), pcl::PointXYZ(centroid_x, centroid_y, 0), 0.5f, 255, 0, 0, "", 0);
  }

  void Renderer::removeShape(int id)
  {
    char buff[32]{};
    std::format_to_n(buff, sizeof(buff), "c_{}", id);
    _viewer->removeShape(buff, 0);
  }

  void Renderer::keyboardCallback(const pcl::visualization::KeyboardEvent &event) 
  {
    if(event.getKeySym() == "v" && event.keyUp() )
	    setLidarStatus();
  }
  
  void Renderer::setLidarStatus()
  {
    lidarActivated^= true;
  }

  bool Renderer::getLidarStatus()
  {
    return lidarActivated;
  }

  void Renderer::RenderHighway()
  {
    /* units in meters */ 
    float roadLength = 50.0f;
    float roadWidth = 12.0f;
    float roadHeight = 0.2f;

    _viewer->addCube(
      -roadLength/2.f, roadLength/2.f, 
      -roadWidth/2.f, roadWidth/2.f, 
      -roadHeight, 0.0f, 
      0.2f, 0.2f, 0.2f, 
      "highwayPavement"
    );
    _viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 
      "highwayPavement"
    );
    _viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 
      0.2f, 0.2f, 0.2f,
      "highwayPavement"
    );
    _viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 
      1.0f, 
      "highwayPavement"
    );

    _viewer->addLine(
      pcl::PointXYZ(-roadLength / 2,-roadWidth / 6, 0.01f), 
      pcl::PointXYZ(roadLength / 2, -roadWidth / 6, 0.01f),
      1.0f, 1.0f, 0.0f,
      "line1"
    );

    _viewer->addLine(
      pcl::PointXYZ(-roadLength / 2, roadWidth / 6, 0.01f),
      pcl::PointXYZ(roadLength / 2, roadWidth / 6, 0.01f),
      1.0f, 1.0f, 0.0f, 
      "line2"
    );
  }

  void Renderer::RenderRays(const Eigen::Vector3f& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    for(pcl::PointXYZ point : cloud->points)
    {
      char buff[32]{};
      std::format_to_n(buff, sizeof(buff), "ray{}", _rays_counter);

      _viewer->addLine(
        pcl::PointXYZ(origin.x(), origin.y(), origin.z()), 
        point,
        1.0f, 0.0f, 0.0f, 
        buff
      );
      ++_rays_counter;
    }
  }

  void Renderer::ClearRays()
  {
    char buff[32]{};
    for(; _rays_counter > 0; _rays_counter--)
    {
      std::fill_n(buff, sizeof(buff), 0);
      std::format_to_n(buff, sizeof(buff), "ray{}", _rays_counter);
      _viewer->removeShape(buff);
    }
  }

  void Renderer::RenderPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    std::string_view name, 
    const Color& color
  )
  {
    _viewer->addPointCloud<pcl::PointXYZ>(cloud, name.data());
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name.data());
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name.data());
  }

  void Renderer::RenderPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
    std::string_view name, 
    const Color& color
  )
  {
    if(color.r == -1)
    {
      /* Select color based off of cloud intensity */
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
      _viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name.data());
    }
    else
    {
      /* Select color based off input value */ 
      _viewer->addPointCloud<pcl::PointXYZI>(cloud, name.data());
      _viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 
        color.r, color.g, color.b, 
        name.data());
    }

    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name.data());
  }

  /* Draw wire frame box with filled transparent color */
  void Renderer::RenderBox(const Box& box, int id, const Color& color, float opacity)
  {
    if(opacity > 1.0f) opacity = 1.0f;
    if(opacity < 0.0f) opacity = 0.0f;

    std::string cube = std::format("box{}", id);
    _viewer->addCube(
      box.min_pos.x(), box.max_pos.x(),
      box.min_pos.y(), box.max_pos.y(),
      box.min_pos.z(), box.max_pos.z(),
      color.r, color.g, color.b, 
      cube
    );

    _viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      cube
    );
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = std::format("boxFill{}", id);
    _viewer->addCube(
      box.min_pos.x(), box.max_pos.x(),
      box.min_pos.y(), box.max_pos.y(),
      box.min_pos.z(), box.max_pos.z(),
      color.r, color.g, color.b, 
      cubeFill
    );
    _viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 
      cubeFill
    );
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3f, cubeFill);
  }

  void Renderer::RenderBox(const BoxQ& box, int id, const Color& color, float opacity)
  {
    if(opacity > 1.0f) opacity = 1.0f;
    if(opacity < 0.0f) opacity = 0.0f;

    std::string cube = std::format("box{}", id);
    _viewer->addCube(
      box.bbox_transform, 
      box.bbox_quaternion, 
      box.cube_length, 
      box.cube_width, 
      box.cube_height, 
      cube
    );
    _viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      cube
    );
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = std::format("boxFill{}", id);
    _viewer->addCube(
      box.bbox_transform, 
      box.bbox_quaternion,
      box.cube_length, 
      box.cube_width, 
      box.cube_height, 
      cubeFill
    );
    _viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 
      cubeFill
    );
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3f, cubeFill);
  }

  void Renderer::InitCamera(CameraAngle view_angle)
  {
    _viewer->setBackgroundColor(0, 0, 0);

    /* Set camera position and angle */
    _viewer->initCameraParameters();

    /* Distance away in meters */ 
    int distance = 16;

    switch(view_angle)
    {
      case CameraAngle::XY:
        _viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;

      case CameraAngle::TopDown:
        _viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;

      case CameraAngle::Side:
        _viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;

      case CameraAngle::FPS:
        _viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
        break;

      default:
        throw std::logic_error("unknown CameraAngle");
    }

    if(view_angle != CameraAngle::FPS)
      _viewer->addCoordinateSystem(1.0f);

	  _viewer->registerKeyboardCallback(keyboardCallback);
    lidarActivated = true;
  }

  void Renderer::ClearViewer()
  {
    _viewer->removeAllPointClouds();
    _viewer->removeAllShapes();
  }

  bool Renderer::WasViewerStopped() const
  {
    return _viewer->wasStopped();
  }

  void Renderer::SpinViewerOnce() const
  {
    _viewer->spinOnce();
  }
};