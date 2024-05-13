#ifndef GPS_CONTROLLER_HPP
#define GPS_CONTROLLER_HPP

#include <cmath>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/uORB.h>


class GPSController {
private:

    uORB::Subscription gps_sub {ORB_ID(sensor_gps)};

    struct sensor_gps_s gps_s;

    double* Startpoint = new double[3];






public:




    GPSController();
    ~GPSController();


    bool getposition(double *latitude, double *longitude, double *altitude);
    double* getDistances();
    double* createWaypoint(double  x,double  y,double z);

    double metersToLongitude(double meters);
    double metersToLatitude(double meters);
    double longitudeToMeters(double longitude);
    double latitudeToMeters(double latitude);

    int GPStest();


    double* getstart();
};

#endif /* GPS_CONTROLLER_HPP */
