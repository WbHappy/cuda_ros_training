#include "lidar_dh.hpp"

LidarDH::LidarDH(){
    double a1 = 0.3;    // Dist between rover center and tower z-axis
    double d2 = 0.7;    // Height between rover center and Lidar
    double al3 = 0.2;   // Lidar's const angle of rotation in x axis    //TODO CHANGE TO Y!
}

HTMatrix LidarDH::dkWorldToRover(double tx, double ty, double tz, double qx, double qy, double qz, double qw)
{
    HTMatrix G;

    G.set(1 - 2*qy*qy - 2*qz*qz,   2*qx*qy - 2*qz*qw,       2*qx*qz + 2*qy*qw,       tx,
          2*qx*qy + 2*qz*qw,       1 - 2*qx*qx - 2*qz*qz,   2*qy*qz - 2*qx*qw,       ty,
          2*qx*qz - 2*qy*qw,       2*qy*qz + 2*qx*qw,       1 - 2*qx*qx - 2*qy*qy,   tz,
                   0,                       0,                        0,              1);

    return G;
}

HTMatrix LidarDH::dkRoverToLidar(double th2)
{


    HTMatrix A1, A2, A3, A4;

    A1 = transX(-a1);
    A2 = rotZ(th2) * transZ(d2);
    // A3 = rotX(al3);
    A3 = rotZ(PI/2) * rotX(al3);
    A4 = rotZ(-PI/2);

    HTMatrix A = A1*A2*A3*A4;

    return A;
}

HTMatrix LidarDH::dkWorldToLidar(double tx, double ty, double tz, double qx, double qy, double qz, double qw, double th2)
{
    return dkWorldToRover(tx, ty, tz, qx, qy, qz, qw) * dkRoverToLidar(th2);
}
