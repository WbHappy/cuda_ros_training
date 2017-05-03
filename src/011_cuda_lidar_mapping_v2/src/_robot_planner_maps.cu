#include "../include/_robot_planner_maps.cuh"

int roundUpTo(int divider, int input)
{
    return ((input + divider - 1 ) / divider) * divider;
}


_RobotPlannerMaps::_RobotPlannerMaps()
{
    dev_heightmap = GpuMapI16();
    dev_costmap = GpuMapI16();
}


_RobotPlannerMaps::~_RobotPlannerMaps()
{
    freeConstSizeMemory();
    freeMaps();
    freeLaserScan();
}




void _RobotPlannerMaps::allocateConstSizeMemory()
{
    gpuErrchk(cudaMalloc((void**)&dev_dk_matrix, 16 * sizeof(double)) );
}

void _RobotPlannerMaps::allocateMaps(float target_east, float target_north)
{
    target_east *= map_scale;
    target_north *= map_scale;

    map_orient = atan2(target_north, target_east);
    map_offset_pix = roundUpTo(map_pow2_divider, map_meters_offset*map_scale);


    int dist_xy = (int) (sqrt(target_east*target_east + target_north*target_north));
    int map_size_x = roundUpTo(map_pow2_divider, dist_xy) + 2 * roundUpTo(map_pow2_divider, map_offset_pix);
    int map_size_y = 2 * roundUpTo(map_pow2_divider, map_offset_pix);


    printf("angle form East = %f\n", map_orient);
    printf("dist_xy = %d\n", dist_xy);
    printf("map_size_x = %d\n", map_size_x);
    printf("map_size_y = %d\n", map_size_y);

    dev_heightmap.allocate(map_size_y, map_size_x);
    dev_costmap.allocate(map_size_y, map_size_x);

    dev_heightmap.fill(UNKNOWN);
    dev_heightmap.drawCircle(init_circle_height, map_offset_pix, map_offset_pix, init_circle_radius); // -40 - height form rover center to bottom of its wheels
    dev_costmap.fill(0);


    host_heightmap.allocate(map_size_y, map_size_x);
    host_costmap.allocate(map_size_y, map_size_x);

    host_heightmap.fill(127);
    host_costmap.fill(127);


}

void _RobotPlannerMaps::allocateLaserScan(int laser_rays)
{
    this->laser_rays = laser_rays;
    gpuErrchk(cudaMalloc((void**)&dev_laser_scan, laser_rays * sizeof(float)) );

    gpuErrchk(cudaMalloc((void**)&dev_debug, laser_rays * sizeof(float)) );
    host_debug = (float*)malloc(laser_rays * sizeof(float));
}





void _RobotPlannerMaps::freeConstSizeMemory()
{
    gpuErrchk( cudaFree(dev_dk_matrix) );
}

void _RobotPlannerMaps::freeMaps()
{
    dev_heightmap.release();
    dev_costmap.release();

    host_heightmap.release();
    host_costmap.release();
}

void _RobotPlannerMaps::freeLaserScan()
{
    gpuErrchk( cudaFree(dev_laser_scan) );

    gpuErrchk( cudaFree(dev_debug) );
}





void _RobotPlannerMaps::resizeMaps(float target_east, float target_north)
{
    freeMaps();
    allocateMaps(target_east, target_north);
}





void _RobotPlannerMaps::cudaDebugInfo()
{

    gpuErrchk( cudaMemcpy(host_debug, dev_debug, laser_rays * sizeof(float), cudaMemcpyDeviceToHost) );

    printf("==== CUDA DEBUG ====\n");
    for(int i = 0; i < laser_rays; i++)
    {
        printf("%f\n", host_debug[i]);
    }
    printf("====================\n");
}
