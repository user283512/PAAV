#include <vector>
#include <unordered_set>

#include "../include/tree_utilities.hpp"

using my_visited_set_t = std::unordered_set<int>;

// TODO (optional)
// This function builds the clusters following a euclidean clustering approach
// 	- Input:
// 		+ cloud: Point cloud to be explored
// 		+ tree: kd tree for searching neighbors
// 		+ distanceTol: Distance tolerance to build the clusters
// 		+ setMinClusterSize: Minimum cluster size
// 		+ setMaxClusterSize: Max cluster size
// 	- Output:
// 		+ cluster: at the end of this function we will have a set of clusters
std::vector<pcl::PointIndices> euclideanCluster(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    my_pcl::KdTree *tree,
    float distanceTol,
    int setMinClusterSize,
    int setMaxClusterSize)
{
  my_visited_set_t visited{};              // already visited points
  std::vector<pcl::PointIndices> clusters; // vector of PointIndices that will contain all the clusters
  std::vector<int> cluster;                // vector of int that is used to store the points that the function proximity will give me back
  // for every point of the cloud
  //   if the point has not been visited (use the function called "find")
  //     find clusters using the proximity function
  //
  //     if we have more clusters than the minimum
  //       Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)
  //     end if
  //   end if
  // end for
  return clusters;
}
