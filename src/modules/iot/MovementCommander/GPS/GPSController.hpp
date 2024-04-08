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
    px4_pollfd_struct_t gps_fds[1];




    double* getDistancesFromStartOrWaypoint(bool Start);


public:
    int maxwaypoints = 10;
    double* WaypointsX = new double[maxwaypoints];
    double* WaypointsY = new double[maxwaypoints];
    double* WaypointsZ = new double[maxwaypoints];

    GPSController();
    ~GPSController();

    int waypointCount = 0;

    bool getposition(double *latitude, double *longitude, double *altitude, int poll_T_ms);
    double* getDistances();
    int createWaypoint(double  x,double  y,double z);
    double metersToLongitude(double meters);
    double metersToLatitude(double meters);
    double longitudeToMeters(double longitude);
    double latitudeToMeters(double latitude);
    void generateExampleWaypoints();
};

#endif /* GPS_CONTROLLER_HPP */
