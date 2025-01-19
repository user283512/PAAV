// #define DATASET_2

float g_ransac_threshold = 0.2f;  // 20cm
float g_cluster_tolerance = 0.3f; // 50cm
int g_cluster_min_size = 100;
int g_cluster_max_size = 10'000;

// #ifdef DATASET_1
// #elif defined DATASET_2
// #endif