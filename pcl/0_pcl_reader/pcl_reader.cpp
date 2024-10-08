#include <iostream>
#include <filesystem>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

namespace fs = std::filesystem;
using stringstream      = std::stringstream;
using CloudViewer       = pcl::visualization::CloudViewer;
using PCLVisualizer     = pcl::visualization::PCLVisualizer;
using PointCloud_XYZ    = pcl::PointCloud<pcl::PointXYZ>;

static int user_data = 0;
    
void viewerPsycho(PCLVisualizer& viewer)
{
    static unsigned count = 0;
    stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    

int main(int argc, char **argv)
{
    PointCloud_XYZ::Ptr cloud (new PointCloud_XYZ);
    
    bool load = pcl::io::loadPCDFile(argv[1], *cloud);
    if (load == -1) 
    {
        PCL_ERROR("Errore nel caricamento del file PCD \n");
        return -1;
    }
    
    CloudViewer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
    //This will get called once per visualization iteration in a separated thread
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped());
    
    return 0;
}