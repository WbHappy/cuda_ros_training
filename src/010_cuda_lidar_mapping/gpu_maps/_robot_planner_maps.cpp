#include "_robot_planner_maps.hpp"

_RobotPlannerMaps::_RobotPlannerMaps()
{
    dev_heightmap = _GpuMap_UI8();
    dev_costmap = _GpuMap_UI8();

    map_orient = 0.0;       // Clock counter-wise angle between East and vector from Start to End points
    map_scale = 10.0;        // One meter in real world is equal to this number of fields

    map_pow2_divider = 32;   // Map size must be divisible by this number
    map_min_offset = 100;     // Minimum number of fields between Start/Stop points and edge of map

    cmap_refresh_size = 128;  // Size of square, within which costmap is refreshed in fields

}


////////////////////////////////////////////////////////////////////////////////


int roundUpTo(int divider, int input)
{
    return ((input + divider - 1 ) / divider) * divider;
}


////////////////////////////////////////////////////////////////////////////////

// Allocation of maps in GPU and CPU memeory.
// Some class-fields such as map_orient are calculated.
// It is worth to notice, that EAST == X, and NORTH == Y :)

void _RobotPlannerMaps::allocateMaps(float target_east, float target_north)
{
    this->map_orient = atan2(target_north, target_east);

    int dist_xy = (int) (sqrt(target_east*target_east + target_north*target_north));
    int map_size_x = roundUpTo(map_pow2_divider, dist_xy) + 2 * roundUpTo(map_pow2_divider, map_min_offset);
    int map_size_y = 2 * roundUpTo(map_pow2_divider, map_min_offset);

    printf("angle form East = %f\n", map_orient);
    printf("dist_xy = %d\n", dist_xy);
    printf("map_size_x = %d\n", map_size_x);
    printf("map_size_y = %d\n", map_size_y);

    dev_heightmap.allocate(map_size_y, map_size_x);
    dev_costmap.allocate(map_size_y, map_size_x);

    host_heightmap = cv::Mat(map_size_y, map_size_x, CV_8UC1);
    host_costmap = cv::Mat(map_size_y, map_size_x, CV_8UC1);
}


////////////////////////////////////////////////////////////////////////////////


void _RobotPlannerMaps::resizeMaps(float target_east, float target_north)
{
    freeMaps();
    allocateMaps(target_east, target_north);
}


////////////////////////////////////////////////////////////////////////////////


void _RobotPlannerMaps::freeMaps()
{
    dev_heightmap.free();
    dev_costmap.free();

    host_heightmap.release();
    host_costmap.release();
}
