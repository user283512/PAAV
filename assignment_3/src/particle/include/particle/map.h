#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map 
{
public:
	struct single_landmark_s
	{
		int id_i ; // Landmark ID
		float x_f; // Landmark x-position in the map (global coordinates)
		float y_f; // Landmark y-position in the map (global coordinates)

		single_landmark_s(int id = 0, float x = 0, float y = 0)
			: id_i{ id },
				x_f{ x },
				y_f{ y }
		{}
	};

	std::vector<single_landmark_s> landmark_list ; // List of landmarks in the map
};

#endif /* MAP_H_ */
