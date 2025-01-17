/* \author Aaron Brown */

#include "Renderer.hpp"

namespace my_pcl
{
	// Structure to represent node of kd tree
	struct Node
	{
		std::vector<float> point;
		int id;
		Node *left;
		Node *right;

		Node(std::vector<float> arr, int setId)
				: point(arr), id(setId), left(NULL), right(NULL)
		{
		}
	};

	struct KdTree
	{
		Node *root;
		int dimension;
		KdTree()
				: root(NULL)
		{
			dimension = 2;
		}

		void set_dimension(int new_dimension);

		int get_split_by_index(int depth);

		bool within_sphere(std::vector<float> target, std::vector<float> point, float distanceTol);
		bool within_box(std::vector<float> target, std::vector<float> point, float distanceTol);

		Node *insert(Node *node, int depth, std::vector<float> point, int id);

		void insert(std::vector<float> point, int id);

		void search(std::vector<int> &neighbors, std::vector<float> target, float distanceTol, Node *node, int depth);

		// return a list of point ids in the tree that are within distance of target
		std::vector<int> search(std::vector<float> target, float distanceTol);
	};
}
