#include <commander/Commander.hpp>
#include "GPSController.hpp"

GPSController::GPSController() {

    //double* x = new double;
    //double* y = new double;
    //double* z = new double;
//
    //if (!getposition(x,y,z)){
    //    PX4_ERR("Waypoint not created bcause of poll timeoutn");
    //}
//
//
    //createWaypoint(latitudeToMeters(*x),longitudeToMeters(*y) ,-*z);

}

GPSController::~GPSController() {
    // Destructor implementation
}


void GPSController::generateExampleWaypoints(){
    for(int i=waypointCount; i< maxwaypoints; i++){
        createWaypoint(-i,0,0);
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

    WaypointsX[waypointCount] = x + WaypointsX[0];
    WaypointsY[waypointCount] = y + WaypointsY[0];
    WaypointsZ[waypointCount] = z + WaypointsZ[0];

    waypointCount += 1;
    return 0;
}

void GPSController::resetWaypoints(){
    waypointCount = 1;
    for(int i=1; i< maxwaypoints; i++){
        createWaypoint(0,0,0);
    }
    waypointCount = 1;

}

bool GPSController::getposition(double *latitude,double *longitude,double *altitude){

	gps_sub.copy(&gps_s);

    *latitude = gps_s.latitude_deg;
    *longitude = gps_s.longitude_deg;
    *altitude = gps_s.altitude_msl_m;

    return true;

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


