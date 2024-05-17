#include <commander/Commander.hpp>
#include "GPSController.hpp"

GPSController::GPSController() {


    if (!getposition(&Startpoint[0],&Startpoint[1],&Startpoint[2])){
        PX4_ERR("Waypoint not created bcause of poll timeoutn");
    }

}

GPSController::~GPSController() {
    // Destructor implementation
}

double* GPSController::getstart(){
    return Startpoint;
}



double* GPSController::createWaypoint(double dlat,double dlon,double dalt){


    double* res = new double[3];
    res[0] = metersToLatitude(dlat) + Startpoint[0];
    res[1] = metersToLongitude(dlon) + Startpoint[1];
    res[2] = dalt + Startpoint[2];

    return res;
}

bool GPSController::getposition(double *latitude,double *longitude,double *altitude){
    while (!gps_sub.updated()){
        px4_sleep(0.3);
    }
	gps_sub.copy(&gps_s);

    *latitude = gps_s.latitude_deg;
    *longitude = gps_s.longitude_deg;
    *altitude = gps_s.altitude_ellipsoid_m;

    return true;

}

double GPSController::metersToLongitude(double meters) {
    // Calculate the circumference of the Earth at the given latitude
    printf("longitude multiplier %f",cos(Startpoint[0] * (M_PI / 180)));
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

int GPSController::GPStest(){
    // Subscirbe to "sensor_gyro", then set a polling interval of 200ms
		int gps_subs = orb_subscribe(ORB_ID(sensor_gps));
		orb_set_interval(gps_subs, 200);

		// Configure a POSIX POLLIN system to sleep the current thread until
		// data appears on the topic
		px4_pollfd_struct_t fds_gps;
		fds_gps.fd = gps_subs;
		fds_gps.events = POLLIN;
		hrt_abstime starttime = hrt_absolute_time();
		 // Loop a specified number of times
		for(int i = 1; i <= 400; i++){
			// Allow the POSIX POLLIN system to poll for data, with 1000ms timeout
			int poll_ret = px4_poll(&fds_gps, 1, 1000);

			// If px4_poll returns 0, then the poll system timed out! Throw an error.
			if(poll_ret == 0)
			{
			PX4_ERR("Got no data within a second");
			}

			// If it didn't return 0, we got data!
			else{
				// Double check that the data we recieved was in the right format (I think - need to check)
				if(fds_gps.revents & POLLIN){

					// Create a sensor_gps_s struct to store the data we recieved
					struct sensor_gps_s gps;

					// Copy the data over to the struct
					orb_copy(ORB_ID(sensor_gps), gps_subs, &gps);


					// Finally, print the data!
					printf("%lld |  %+2.6f  |  %+2.6f  |  %+2.2f  \n", hrt_absolute_time()-starttime, (double)gps.latitude_deg, (double)gps.longitude_deg, (double)gps.altitude_ellipsoid_m);
				}
			}
		}

    // Tell the user that the application is ending...
    PX4_INFO("Hovergames gyro exit");

    // Typical C exit :)
    return 0;
}

