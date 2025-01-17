#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace my_pcl
{
	/**
	 * Structure to represent node of kd tree
	 */
	struct Node
	{
		std::vector<float> point;
		int id;
		Node* left;
		Node* right;

		Node(const std::vector<float>& arr, int id)
		:	point{ arr }, 
			id{ id }, 
			left{ nullptr }, 
			right{ nullptr }
		{}
	};


	class KdTree
	{
	public:
		KdTree()
			: root{ nullptr },
				dimension{ 2 }
		{}

		void set_dimension(int new_dimension);

		int get_split_by_index(int depth);

		bool within_sphere(std::vector<float> target, std::vector<float> point, float distanceTol);

		bool within_box(std::vector<float> target, std::vector<float> point, float distanceTol);

		Node* insert(Node* node, int depth, std::vector<float> point, int id);

		void insert(std::vector<float> point, int id);

		void search(std::vector<int>& neighbors, std::vector<float> target, float distanceTol, Node* node, int depth);
		
		/**
		 * return a list of point ids in the tree that are within distance of target
		 */
		std::vector<int> search(std::vector<float> target, float distanceTol);

		Node* root;
		int dimension;
	};
}
