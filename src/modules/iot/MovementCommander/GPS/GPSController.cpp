#include <commander/Commander.hpp>
#include "GPSController.hpp"

GPSController::GPSController() {

    //GPSController::gps_sub = orb_subscribe(ORB_ID(sensor_gps));
    //orb_set_interval(gps_sub, 5000); // callback interval

    // get starting location
    //double* x = new double;
    //double* y = new double;
    //double* z = new double;
    //if (!getposition(x,y,z,1000)){
    //    PX4_ERR("Waypoint not created bcause of poll timeoutn");
    //};

    //createWaypoint(latitudeToMeters(*x),longitudeToMeters(*y) ,-*z);
//
    //gps_fds[0].fd = gps_sub;
    //gps_fds[0].events = POLLIN;
}

GPSController::~GPSController() {
    // Destructor implementation
}


void GPSController::generateExampleWaypoints(){
    for(int i=waypointCount; i< maxwaypoints; i++){
        createWaypoint(WaypointsX[0] + cos(i),WaypointsX[1] + sin(i),-i);
    }
}

int GPSController::createWaypoint(double x,double y,double z){
    if(waypointCount >= 10){
        PX4_ERR("Waypoint not created bcause of too many waypoints\n");
        return 1;
    }else if(z>0){
        PX4_ERR("Waypoint not created z greater than 0 is underground\n");
        return 1;
    }

    WaypointsX[waypointCount] = x;
    WaypointsY[waypointCount] = y;
    WaypointsZ[waypointCount] = z;

    waypointCount += 1;
    return 0;
}

bool GPSController::getposition(double *latitude,double *longitude,double *altitude,int poll_T_ms){
    // Set callback interval to 1 second
    int poll_ret = px4_poll(gps_fds, 1, poll_T_ms);

    if (poll_ret == 0) {
        PX4_WARN("Timeout: No data received");
        return 0;

    } else if(poll_ret < 0) {
        PX4_ERR("Error: poll failed");
        return 0;

    }


    gps_sub.copy(&gps_s);


    *latitude = gps_s.latitude_deg / 1e7; // Convert from 1e7 scale to degrees
    *longitude = gps_s.longitude_deg / 1e7; // Convert from 1e7 scale to degrees
    *altitude = gps_s.altitude_msl_m / 1e3;

    return 0;

}

double* GPSController::getDistances(){
    double* distances = new double[waypointCount-1];
    if (waypointCount > 1) {
        double firstX = WaypointsX[0];
        double firstY = WaypointsY[0];
        double firstZ = WaypointsZ[0];

        for (int i = 1; i < waypointCount; ++i) {
            double dx = WaypointsX[i] - firstX;
            double dy = WaypointsY[i] - firstY;
            double dz = WaypointsZ[i] - firstZ;

            // Calculate distance using Euclidean distance formula
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            distances[i-1] = distance;


        }
    }else{
        PX4_ERR("No waypoints created");
    }
    return distances;
}

double GPSController::metersToLongitude(double meters) {
    // Calculate the circumference of the Earth at the given latitude
    return meters / (6371000 * cos(WaypointsX[0] * (M_PI / 180)) * (M_PI / 180));
}

double GPSController::metersToLatitude(double meters) { return meters / 111132.92; }

double GPSController::latitudeToMeters(double latitude) {
    return latitude * 111132.92;
}

double GPSController::longitudeToMeters(double longitude) {
    double latitude = WaypointsX[0] * (M_PI / 180); // Convert latitude to radians
    return longitude * (6371000 * cos(latitude) * (M_PI / 180));
}


