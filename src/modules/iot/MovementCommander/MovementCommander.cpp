#include <commander/Commander.hpp>
#include "MovementCommander.hpp"
#include <stdio.h>
#include "UartCommander/Serial_port/serial_port.h"
#include "GPS/GPSController.hpp"
#include <cmath>

#include <uORB/topics/vehicle_local_position.h>
bool uintcontains(uint8_t value, uint8_t* list,int length){
	for (int i = 0; i< length ; i++){
		if(value == list[i]){
			return true;
		}
	}
	return false;
}
static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}



// our includes:

MovementCommander::MovementCommander(){

}

MovementCommander::~MovementCommander(){

}
int MovementCommander::activation1(int expectedn){

    	Serial_Port serial_port("/dev/ttyS1", 115200);
	serial_port.start();

	GPSController gpsC = *new GPSController();

	const double N = 2.0; // change to correspond to enviroment
	const int PowerMesurrementDB = -66;
	const int PowerMesurrementDC = -55;
	int bytes_read = 0;
	int totbytes = 0;

	double DistDB;
	double DistDC;
	double DistCB;

	double* RSSIDB = new double[5];
	double* RSSIDC = new double[5];

	double movedDistance = 0;



	bool isflying = false;
	hrt_abstime lastbytesread= 0;

	uint8_t* data = new uint8_t[expectedn];
	uint8_t* buffer = new uint8_t[expectedn];

	struct timespec ts;
	uint64_t Unix_epoch_time;
	printf("epoch_time : RSSIBD : RSSIDC : DistDB : DistDC : DistBC : flying : movement\n");
	while(true){
		//stay in goto struct point


		//wait for 200 or recieve message
		bytes_read = serial_port._read_port(*buffer,expectedn,200);


		lastbytesread = hrt_absolute_time();
		//add to data if having read something
		if (bytes_read > 0){
			if((hrt_absolute_time()-lastbytesread) >= 50000){

				totbytes = 0;
			}

			if (totbytes + bytes_read > expectedn){
				totbytes = 0;
			}

			for (int i = 0; i< bytes_read;i++){
				data[i+totbytes] = buffer[i];
			}

			totbytes += bytes_read;


		}else{continue;}


		if ((totbytes != expectedn)){
			continue;
		}
		if (data[0] != 0 & data[expectedn-1]!=255){
			continue;
		}



		// calculate distannces
		DistDB = pow(10.0,((PowerMesurrementDB + (double)data[1])/(10.0*N)));
		DistDC = pow(10.0,((PowerMesurrementDC + (double)data[2])/(10.0*N)));

		if (isflying){

			DistCB = (cos(asin(0/DistDB))*DistDB) + (cos(asin(0/DistDC))*DistDC);
		}else{
			DistCB = DistDB + DistDC;
		}

		totbytes = 0;

		if (data[2] == 1){

			if (isflying){

				gpsC.createRelativeWaypoint(-movedDistance,0,0);
				movedDistance = 0;


				//send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);


				isflying = false;



			}else{



			}
		}
		//board is too far away
		else{
			if (!isflying){

				//
				//const char *takeoff = "takeoff";
				////
				//char* takeoffPtr = const_cast<char*>(takeoff);
				////
				//Commander::custom_command(1, &takeoffPtr);


				gpsC.createRelativeWaypoint( (-DistCB/2) - movedDistance ,0,0);
				if (data[1] > data[2]){
					movedDistance = movedDistance +1;
				}
				else{
					movedDistance = movedDistance -1;
				}


				isflying = true;

			}else{

				gpsC.createRelativeWaypoint((-DistCB/2) - movedDistance ,0,0);
				if (data[1] > data[2]){
					movedDistance = movedDistance +1;
				}
				else{
					movedDistance = movedDistance -1;
				}




			}
		}

		clock_gettime(CLOCK_REALTIME, &ts);

		Unix_epoch_time = ts.tv_sec;

		printf("%llu : %d : %d : %f : %f : %f : %d : %f \n",Unix_epoch_time, data[1], data[2], DistDB, DistDC, DistCB, isflying, movedDistance);

	}

	return 0;

}


double* MovementCommander::repos(float x, float y,float z,GPSController gpsC){

		double* waypoint = gpsC.createRelativeWaypoint(x,y,z);

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
										airspeed,
										vehicle_command_s::SPEED_TYPE_AIRSPEED,
										NAN,
										NAN,
										waypoint[0],
										waypoint[1],
										NAN
										);


		return waypoint;
}
double mean(double* arr, n){
	double c = 0;
	for (int i = 0; i<n,i++){
		c = c + arr[i];
	}
	return c/n
}
int MovementCommander::repostest(){



	GPSController gpsC = *new GPSController();

	const double N = 2.0; // change to correspond to enviroment
	const int PowerMesurrement = -57.1857;

	uint8_t dataDC[41] = { 1, 1, 86, 82, 76, 70, 70, 72, 82, 79, 87, 89, 81, 82, 87, 86, 90, 80, 85, 88, 92, 94, 90, 74, 76, 81, 88, 85, 86, 87, 1, 1,1, 1};
	uint8_t dataDB[41] = { 69, 70, 75, 82, 81, 81, 85, 79, 67, 72, 67, 72, 68, 71, 69, 67, 80, 73, 70, 77, 71, 73, 75, 73, 71, 70, 73, 76, 73, 83, 69, 70, 69, 70};

	double DistDB;
	double DistDC;
	double* DistCB = new double[5];
	double avgDistCB;
	int CBcount = 0;

	double movedDistance = 0;

	double* waypoint = gpsC.getstart();
	double * distanceToStart;

	bool isflying = false;

	struct timespec ts;
	uint64_t Unix_epoch_time;

	double lastadded = 0;
	printf("epoch_time : RSSIBD : RSSIDC : DistDB : DistDC : DistBC : flying : dx : dy : dz\n");
	for(int i = 0; i < 34; i++){

		DistDB = pow(10.0,((PowerMesurrement + (double)dataDB[i])/(10.0*N)));
		DistDC = pow(10.0,((PowerMesurrement + (double)dataDC[i])/(10.0*N)));

		if (CBcount = 0 ){
			DistCB[CBcount] = DistDB + DistDC;
			lastadded = DistCB[CBcount];
			CBcount++;

		}
		else{
			if (abs(lastadded - (DistDB + DistDC)) <= 4){
				DistCB[CBcount] = DistDB + DistDC;
				CBcount++;
			} else if (lastadded - (DistDB + DistDC) >){
				lastadded += 4;
			} else{
				lastadded -= 4
			}
		}

		if (CBcount != 4){
			continue;
		} else{
			avgDistCB = mean(DistCB,5);
			CBcount = 0;
		}


		// calculate distannces


		if (dataDC[i] == 1){

			if (isflying){

				waypoint = repos(0,0,0,gpsC);
				movedDistance = 0;


				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);


				isflying = false;
		}
		//board is too far away
		else{
			if (!isflying){

				const char *takeoff = "takeoff";
				char* takeoffPtr = const_cast<char*>(takeoff);
				Commander::custom_command(1, &takeoffPtr);


				waypoint = repos( (-avgDistCB/2),0,0,gpsC);
				movedDistance = movedDistance + ((-avgDistCB/2) - movedDistance);

				isflying = true;



			}else{

				waypoint = repos((-avgDistCB/2) ,0,0,gpsC);
				movedDistance = movedDistance + ((-avgDistCB/2) - movedDistance);




			}
		}

		clock_gettime(CLOCK_REALTIME, &ts);

		Unix_epoch_time = ts.tv_sec;

		distanceToStart = gpsC.calculateDistances(waypoint,gpsC.getstart());

		printf("%llu : %d : %d : %f : %f : %f : %d : %f : %f : %f : %f\n",Unix_epoch_time, dataDB[i], dataDC[i], DistDB, DistDC, DistCB, isflying, movedDistance, distanceToStart[0],distanceToStart[1],distanceToStart[2]);

	}

	return 0;

}
