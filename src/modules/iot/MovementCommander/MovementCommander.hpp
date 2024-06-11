#ifndef MovementCommander_HPP
#define MovementCommander_HPP

#include <cmath>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include "GPS/GPSController.hpp"


class MovementCommander {



public:
    MovementCommander();
    ~MovementCommander();

    int Movingtest();

    int activation1(int expectedn);
    int repostest();
    int locpostest();
    int gototest1(double x,double y,double z);
    int gototest2(double x,double y,double z);
    double* repos(float x,float y,float z,GPSController gpsC);

private:
    const float airspeed = 3;
};


#endif /* GPS_CONTROLLER_HPP */
