
//
// The original author of the rendering code is Aaron Brown (https://github.com/awbrown90).
// His code has been slightly modified to make it more structured.
//

#include "Renderer.hpp"

namespace lidar_obstacle_detection
{
  Renderer::Renderer() 
  : viewer_{ new pcl::visualization::PCLVisualizer("3D Viewer") }, 
    rays_counter_{ 0 }
  {}

  void Renderer::addCircle(float centroid_x, float centroid_y,std::string id, float radius, int r, int g, int b)
  {
    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize (3);    // We need 3 values
    circle_coeff.values[0] = centroid_x;
    circle_coeff.values[1] = centroid_y;
    circle_coeff.values[2] = radius; //radius

    viewer_->addCircle(circle_coeff,id,0);  

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 1, id);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id);
  }

  void Renderer::addText(float centroid_x, float centroid_y, int id)
  {
    char str_id[32]{};
    std::snprintf(str_id, sizeof(str_id), "%d", id);

  	viewer_->addText3D(str_id, pcl::PointXYZ(centroid_x, centroid_y, 0), 0.3, 255, 255, 255, "", 0);
  }

  void Renderer::resetCam(std::string cloudName)
  {
    viewer_->resetCameraViewpoint(cloudName);
    viewer_->addCoordinateSystem(1.0,cloudName);
  }

  bool Renderer::updatePose(const std::string& id, const Eigen::Affine3f& pose)
  {
    viewer_->updateShapePose(id, pose);
    return false;
  }

  void Renderer::updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string id) 
  {
    viewer_->updatePointCloud(cloud,id);
  }	
  
  void Renderer::updateShape (const std::string &id, float opacity)
  {
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);
  }

  void Renderer::removeShape(std::string id)
  {
    viewer_->removeShape(id,0);
  }

  void Renderer::keyboardCallback(const pcl::visualization::KeyboardEvent &event) 
  {
    if(event.getKeySym() == "v" && event.keyUp() ) 
    {
	    setLidarStatus();
    }
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
    // units in meters
    float roadLength = 50.0;
    float roadWidth = 12.0;
    float roadHeight = 0.2;

    viewer_->addCube(
      -roadLength/2.0f, roadLength/2.0f, 
      -roadWidth/2.0f, roadWidth/2.0f, 
      -roadHeight, 0.0f, 
      0.2f, 0.2f, 0.2f, 
      "highwayPavement"
    );

    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 
      "highwayPavement"
    );

    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 
      0.2f, 0.2f, 0.2f, 
      "highwayPavement"
    );

    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 
      1.0f, 
      "highwayPavement"
    );

    viewer_->addLine(
      pcl::PointXYZ(-roadLength/2,-roadWidth/6, 0.01), 
      pcl::PointXYZ(roadLength/2, -roadWidth/6, 0.01),
      1.0, 1.0, 0.0, 
      "line1"
    );

    viewer_->addLine(
      pcl::PointXYZ(-roadLength/2, roadWidth/6, 0.01),
      pcl::PointXYZ(roadLength/2, roadWidth/6, 0.01), 
      1.0, 1.0, 0.0, 
      "line2"
    );
  }


  void Renderer::RenderRays(const Eigen::Vector3f& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    for(pcl::PointXYZ point : cloud->points)
    {
      char label[32]{};
      std::snprintf(label, sizeof(label), "ray%u", static_cast<uint32_t>(rays_counter_));

      viewer_->addLine(
        pcl::PointXYZ(origin.x(), origin.y(), origin.z()), 
        point,
        1.0, 0.0, 0.0, 
        label
      );
      
      ++rays_counter_;
    }
  }

  void Renderer::ClearRays()
  {
    while(rays_counter_-- > 0)
    {
      char label[32]{};
      std::snprintf(label, sizeof(label), "ray%u", static_cast<uint32_t>(rays_counter_));

      viewer_->removeShape(label);
    }
  }

  void Renderer::RenderPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    const std::string& name, 
    const Color& color
  )
  {
    viewer_->addPointCloud<pcl::PointXYZ>(cloud, name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
  }

  void Renderer::RenderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
    const std::string& name, 
    const Color& color
  )
  {
    if(color.r == -1)
    {
      // Select color based off of cloud intensity
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
      viewer_->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
      // Select color based off input value
      viewer_->addPointCloud<pcl::PointXYZI>(cloud, name);
      viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 
        color.r, color.g, color.b, 
        name
      );
    }

    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
  }

  // Draw wire frame box with filled transparent color
  void Renderer::RenderBox(const Box& box, const int id, const Color& color, float opacity)
  {
    if(opacity > 1.0) opacity = 1.0;
    if(opacity < 0.0) opacity = 0.0;

    char cube[32]{};
    std::snprintf(cube, sizeof(cube), "box%d", id);

    viewer_->addCube(
      box.pos_min.x(), box.pos_max.x(), 
      box.pos_min.y(), box.pos_max.y(), 
      box.pos_min.z(), box.pos_max.z(), 
      color.r, color.g, color.b, 
      cube
    );

    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      cube
    );

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    char cubeFill[32]{};
    std::snprintf(cubeFill, sizeof(cubeFill), "boxFill%d", id);

    viewer_->addCube(
      box.pos_min.x(), box.pos_max.x(),
      box.pos_min.y(), box.pos_max.y(),
      box.pos_min.z(), box.pos_max.z(),
      color.r, color.g, color.b, 
      cubeFill
    );

    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 
      cubeFill
    );

    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 
      color.r, color.g, color.b, 
      cubeFill
    );

    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 
      opacity*0.3, 
      cubeFill
    );
  }

  void Renderer::RenderBox(const BoxQ& box, const int id, const Color& color, float opacity)
  {
    if(opacity > 1.0) opacity = 1.0;
    if(opacity < 0.0) opacity = 0.0;

    char cube[32]{};
    std::snprintf(cube, sizeof(cube), "box%d", id);

    viewer_->addCube(
      box.bbox_transform, box.bbox_quaternion, 
      box.cube_length, box.cube_width, box.cube_height, 
      cube
    );
    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      cube
    );
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    char cubeFill[32]{};
    std::snprintf(cubeFill, sizeof(cubeFill), "boxFill%d", id);

    viewer_->addCube(
      box.bbox_transform, box.bbox_quaternion,
      box.cube_length, box.cube_width, box.cube_height, 
      cubeFill
    );
    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 
      cubeFill
    );
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
  }

  void Renderer::InitCamera(CameraAngle view_angle)
  {
    viewer_->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer_->initCameraParameters();

    // distance away in meters
    constexpr int distance = 16;

    switch(view_angle)
    {
      case CameraAngle::XY:
      {
        //viewer_->setCameraPosition(54, 40, 0,1, 1, 0);
        //viewer_->addCoordinateSystem(1.0,54, 40, 0);
        viewer_->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
      }
      case CameraAngle::TopDown:
      {
        viewer_->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
      }
      case CameraAngle::Side:
      {
        viewer_->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
      }
      case CameraAngle::FPS:
      {
        viewer_->setCameraPosition(-10, 0, 0, 0, 0, 1);
        break;
      }
      default:
      {
        throw std::logic_error("unknown CameraAngle");
      }
    }

    if(view_angle != CameraAngle::FPS)
    {
      viewer_->addCoordinateSystem(1.0);
    }
	  viewer_->registerKeyboardCallback(keyboardCallback);
    lidarActivated = true;
  }

  void Renderer::ClearViewer()
  {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
  }

  bool Renderer::WasViewerStopped() const
  {
    return viewer_->wasStopped();
  }



  void Renderer::SpinViewerOnce() const
  {
    viewer_->spinOnce();
  }
}
