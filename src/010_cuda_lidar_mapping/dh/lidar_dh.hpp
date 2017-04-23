#ifndef LIDAR_DH_HPP_
#define LIDAR_DH_HPP_

#include "ht_matrix.hpp"

class LidarDH{

    double a1;
    double th2;
    double d2;
    double al3;

public:
    LidarDH();

    HTMatrix dkWorldToRover(double tx, double ty, double tz, double qx, double qy, double qz, double qw);

    HTMatrix dkRoverToLidar(double th2);

    HTMatrix dkWorldToLidar(double tx, double ty, double tz, double qx, double qy, double qz, double qw, double th2);

};

#endif
