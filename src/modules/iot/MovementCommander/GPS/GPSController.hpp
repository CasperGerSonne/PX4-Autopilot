#ifndef GPS_CONTROLLER_HPP
#define GPS_CONTROLLER_HPP

#include <cmath>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/uORB.h>
#include <tuple>

class GPSController {
private:

    uORB::Subscription gps_sub {ORB_ID(sensor_gps)};

    struct sensor_gps_s gps_s;

    double* WaypointsX = new double[10];
    double* WaypointsY = new double[10];
    double* WaypointsZ = new double[10];

    int maxwaypoints = 10;



public:
    int waypointCount = 0;



    GPSController();
    ~GPSController();


    bool getposition(double *latitude, double *longitude, double *altitude);
    double* getDistances();
    int createWaypoint(double  x,double  y,double z);

    double metersToLongitude(double meters);
    double metersToLatitude(double meters);
    double longitudeToMeters(double longitude);
    double latitudeToMeters(double latitude);

    void generateExampleWaypoints();
    void resetWaypoints();
};

#endif /* GPS_CONTROLLER_HPP */
