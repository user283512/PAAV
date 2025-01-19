#include <vector>
#include <unordered_set>

#include "../include/tree_utilities.hpp"

using my_visited_set_t = std::unordered_set<int>;

// TODO (optional)
// This function computes the nearest neighbors and builds the clusters
// 	- Input:
// 		+ cloud: Point cloud to be explored
// 		+ target_ndx: i-th point to visit
// 		+ tree: kd tree for searching neighbors
// 		+ distanceTol: Distance tolerance to build the clusters
// 		+ visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
// 		+ cluster: Here we add points that will represent the cluster
// 		+ max: Max cluster size
// 	- Output:
// 		+ visited: already visited points
// 		+ cluster: at the end of this function we will have one cluster
void proximity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
               int target_ndx,
               my_pcl::KdTree *tree,
               float distanceTol,
               my_visited_set_t &visited,
               std::vector<int> &cluster,
               int max)
{
  if (cluster.size() < max)
  {
    cluster.push_back(target_ndx);
    visited.insert(target_ndx);

    std::vector<float> point{cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};

    // get all neighboring indices of point
    std::vector<int> neighborNdxs = tree->search(point, distanceTol);

    for (int neighborNdx : neighborNdxs)
    {
      // if point was not visited
      if (visited.find(neighborNdx) == visited.end())
      {
        proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
      }

      if (cluster.size() >= max)
      {
        return;
      }
    }
  }
}
