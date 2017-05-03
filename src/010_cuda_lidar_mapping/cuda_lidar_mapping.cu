#include "cuda_lidar_mapping.cuh"

struct PointInt2D
{
    int x;
    int y;
};

__device__ PointInt2D mapRealToGPU(float point_x, float point_y, float map_orient, float map_scale, float map_offset_pix)
{

    PointInt2D map_pose;
    float point_orient = atan2f(point_y, point_x);
    float point_dist = sqrtf(point_x*point_x + point_y*point_y);

    map_pose.x = (int) (sinf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);
    map_pose.y = (int) (cosf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);

    return map_pose;
}

__global__ void lidarMappingKernel(float* laser_scan, int laser_rays, double* dk_matrix, uint8_t* heightmap, int map_x, int map_y, float map_orient, float map_scale, float map_offset_pix, float* debug)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    float laser_angle = ((float)tid)/laser_rays*SCAN_ANGLE - 0.5*SCAN_ANGLE;


    float rotZ[16];
    float dk_laser_sensor[16];

    rotZ[0] = cosf(laser_angle);
    rotZ[1] = -sinf(laser_angle);
    rotZ[2] = 0;
    rotZ[3] = 0;

    rotZ[4] = sinf(laser_angle);
    rotZ[5] = cosf(laser_angle);
    rotZ[6] = 0;
    rotZ[7] = 0;

    rotZ[8] = 0;
    rotZ[9] = 0;
    rotZ[10] = 1;
    rotZ[11] = 0;

    rotZ[12] = 0;
    rotZ[13] = 0;
    rotZ[14] = 0;
    rotZ[15] = 1;

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < 4; k++){
                dk_laser_sensor[i*4 + j] += dk_matrix[i*4 + k] * rotZ[k*4 + j];
            }
        }
    }

    float point_x = dk_laser_sensor[0] * laser_scan[tid] + dk_laser_sensor[3];
    float point_y = dk_laser_sensor[4] * laser_scan[tid] + dk_laser_sensor[7];
    float point_z = dk_laser_sensor[8] * laser_scan[tid] + dk_laser_sensor[11];

    PointInt2D point_map = mapRealToGPU(point_x, point_y, map_orient, map_scale, map_offset_pix);

    // atomicExch(&heightmap[point_map.y * map_x + point_map.x], (uint8_t)255);
    heightmap[point_map.x * map_y + point_map.y] = (uint8_t) 255;

    debug[tid] = point_map.x * map_y + point_map.y;


}


CudaLidarMapping::CudaLidarMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros)
{
    this->_rpm = _rpm;
    this->_ros = _ros;
}


////////////////////////////////////////////////////////////////////////////////


void CudaLidarMapping::copyInputToDevice()
{
    gpuErrchk( cudaMemcpy(_rpm->dev_laser_scan, &_ros->laser_scan.ranges.front(), _rpm->laser_rays * sizeof(float), cudaMemcpyHostToDevice) );
    gpuErrchk( cudaMemcpy(_rpm->dev_dk_matrix, _rpm->dk_matrix.m, 16*sizeof(double), cudaMemcpyHostToDevice) );

}


////////////////////////////////////////////////////////////////////////////////


void CudaLidarMapping::executeKernel()
{
    lidarMappingKernel <<< _rpm->laser_rays, 1 >>> (
        _rpm->dev_laser_scan,
        _rpm->laser_rays,
        _rpm->dev_dk_matrix,
        _rpm->dev_heightmap.data,
        _rpm->dev_heightmap.size_x,
        _rpm->dev_heightmap.size_y,
        _rpm->map_orient,
        _rpm->map_scale,
        _rpm->map_offset_pix,
        _rpm->dev_debug
    );

    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

}


////////////////////////////////////////////////////////////////////////////////


void CudaLidarMapping::copyOutputToHost()
{
    gpuErrchk( cudaMemcpy(_rpm->host_heightmap.data, _rpm->dev_heightmap.data, _rpm->dev_heightmap.size() * sizeof(uint8_t), cudaMemcpyDeviceToHost) );
}


////////////////////////////////////////////////////////////////////////////////


void CudaLidarMapping::display()
{
    cv::namedWindow("win1", 0);
    cv::imshow("win1", _rpm->host_heightmap);
    cv::waitKey(10);
}
