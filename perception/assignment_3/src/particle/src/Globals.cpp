// TODO: define the proper number of particles
int g_num_particles = 0;
int g_num_reflectors = 8;

// TODO: define the proper noise values
double g_sigma_init[3] = {0, 0, 0};         //[x,y,theta] initialization noise.
double g_sigma_pos[3] = {0.05, 0.05, 0.05}; //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
double g_sigma_landmark[2] = {0.4, 0.4};    //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]