#include <commander/Commander.hpp>
#include "GPSController.hpp"

GPSController::GPSController() {

    double* x = new double;
    double* y = new double;
    double* z = new double;

    if (!getposition(x,y,z)){
        PX4_ERR("Waypoint not created bcause of poll timeoutn");
    }

    Startpoint[0] = *x;
    Startpoint[1] = *y;
    Startpoint[2] = *z;



}

GPSController::~GPSController() {
    // Destructor implementation
}

double* GPSController::getstart(){
    return Startpoint;
}



double* GPSController::createWaypoint(double x,double y,double z){


    double* res = new double[3];
    res[0] = metersToLatitude(x) + Startpoint[0];
    res[1] = metersToLongitude(y) + Startpoint[1];
    res[2] = z + Startpoint[2];

    return res;
}

bool GPSController::getposition(double *latitude,double *longitude,double *altitude){

	gps_sub.copy(&gps_s);

    *latitude = gps_s.latitude_deg;
    *longitude = gps_s.longitude_deg;
    *altitude = gps_s.altitude_msl_m;

    return true;

}




double GPSController::metersToLongitude(double meters) {
    // Calculate the circumference of the Earth at the given latitude
    return meters / (6371000 * cos(Startpoint[0] * (M_PI / 180)));
}

double GPSController::metersToLatitude(double meters) { return meters / 111111.0; }

double GPSController::latitudeToMeters(double latitude) {
    return latitude * 111111.0;
}

double GPSController::longitudeToMeters(double longitude) {
    double latitude = Startpoint[0] * (M_PI / 180); // Convert latitude to radians
    return longitude * (6371000 * cos(latitude) * (M_PI / 180));
}


