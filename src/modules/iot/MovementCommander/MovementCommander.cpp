#include <commander/Commander.hpp>
#include "MovementCommander.hpp"
#include <stdio.h>
#include "UartCommander/Serial_port/serial_port.h"
#include "GPS/GPSController.hpp"
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

	const uint8_t N = 2; // change to correspond to enviroment
	const int PowerMesurrement = -49.34;
	int bytes_read = 0;
	int totbytes = 0;



	double DistDB;
	double DistDC;


	double DistCB;

	double* waypoint;

	bool isflying = false;
	hrt_abstime lastbytesread= 0;


	uint8_t* data = new uint8_t[expectedn];
	uint8_t* buffer = new uint8_t[expectedn];


	while(true){
		//stay in goto struct point


		//wait for 200 or recieve message
		bytes_read = serial_port._read_port(*buffer,expectedn,200);


		lastbytesread = hrt_absolute_time();
		//add to data if having read something
		if (bytes_read > 0){
			if((hrt_absolute_time()-lastbytesread) >= 50000){
				printf("reset because end of message with totbytes = %d\n",totbytes);
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

		for (int i = 0; i < totbytes; i++) {
			printf("%c: ", static_cast<char>(data[i]));
		}

		printf("\n");
		DistDB = 10^((PowerMesurrement + data[1])/10*N);
		DistDC = 10^((PowerMesurrement + data[2])/10*N);
		printf("Rssi0 %d\n", data[1]);
		printf("Rssi1 %d\n", data[2]);

		printf("DistDB %f\n", DistDB);
		printf("DistDC %f\n", DistDC);

		DistCB = (cos(asin(1.5/DistDB)*DistDB)) + (cos(asin(1.5/DistDC))*DistDC);
		printf("thought distance between base and board: %f",DistCB);
		//assuming 1d motion and the drone being perfect on line
		totbytes = 0;

		if (data[2] < -60){

			if (isflying){
				printf("Is flying landing\n");
				waypoint = gpsC.createWaypoint(0,0,1.5);
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
									0.5,
									vehicle_command_s::SPEED_TYPE_AIRSPEED,
									NAN,
									NAN,
									waypoint[0],
									waypoint[1],
									waypoint[2]
									);

				sleep(2);
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
				sleep(5);
				isflying = false;
				continue;
			}else{
				printf("on ground no need to move\n");
				continue;
			}
		}
		//board is too far away
		else{
			if (!isflying){
				printf("Taking off\n");
				const char *takeoff = "takeoff";
				char* takeoffPtr = const_cast<char*>(takeoff);
				Commander::custom_command(1, &takeoffPtr);

				sleep(5);

				waypoint = gpsC.createWaypoint(-DistCB,0,1.5);
				printf("moving to %f",waypoint[0]);
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
									0.5,
									vehicle_command_s::SPEED_TYPE_AIRSPEED,
									NAN,
									NAN,
									waypoint[0],
									waypoint[1],
									waypoint[2]
									);
				isflying = true;
				continue;
			}else{
				printf("isflying doing movement\n");
				waypoint = gpsC.createWaypoint(-DistCB,0,1.5);
				printf("moving to %f",waypoint[0]);
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
									0.5,
									vehicle_command_s::SPEED_TYPE_AIRSPEED,
									NAN,
									NAN,
									waypoint[0],
									waypoint[1],
									waypoint[2]
									);
				continue;
			}
		}
	}

	return 0;


}
//int MovementCommander::gototest(){
//
//
//    		goto_setpoint_s goto_setpoint_s1{};
//		goto_setpoint_s goto_setpoint_s2{};
//		orb_advert_t goto_pub1 = orb_advertise(ORB_ID(goto_setpoint), &goto_setpoint_s1);
//		orb_advert_t goto_pub2 = orb_advertise(ORB_ID(goto_setpoint), &goto_setpoint_s2);
//		goto_setpoint_s1.position[0] = 0;
//		goto_setpoint_s1.position[1] = 0;
//		goto_setpoint_s1.position[2] = 0;
//		goto_setpoint_s1.heading = 0;
//		goto_setpoint_s1.max_horizontal_speed = 1;
//		goto_setpoint_s1.max_vertical_speed = 1;
//		goto_setpoint_s1.max_heading_rate = 0;
//		goto_setpoint_s1.flag_control_heading = true;
//		goto_setpoint_s1.flag_set_max_horizontal_speed = true;
//		goto_setpoint_s1.flag_set_max_vertical_speed = true;
//		goto_setpoint_s1.flag_set_max_heading_rate = true;
//
//		goto_setpoint_s2.position[0] = 0;
//		goto_setpoint_s2.position[1] = 0;
//		goto_setpoint_s2.position[2] = -2;
//		goto_setpoint_s2.heading = 0;
//		goto_setpoint_s2.max_horizontal_speed = 1;
//		goto_setpoint_s2.max_vertical_speed = 1;
//		goto_setpoint_s2.max_heading_rate = 0;
//		goto_setpoint_s2.flag_control_heading = true;
//		goto_setpoint_s2.flag_set_max_horizontal_speed = true;
//		goto_setpoint_s2.flag_set_max_vertical_speed = true;
//		goto_setpoint_s2.flag_set_max_heading_rate = true;
//
//		for (int i = 1; i< argc; i++){
//			if(!strcmp(argv[i], "-x")){
//				goto_setpoint_s2.position[0] = atoi(argv[i+1]);
//			}
//			if(!strcmp(argv[i], "-y")){
//				goto_setpoint_s2.position[1] = atoi(argv[i+1]);
//			}
//			if(!strcmp(argv[i], "-z")){
//				goto_setpoint_s2.position[2] = atoi(argv[i+1]);
//			}
//
//		}
//		printf("Takeoff");
//		const char *takeoff = "takeoff";
//		char* takeoffPtr = const_cast<char*>(takeoff);
//		Commander::custom_command(1, &takeoffPtr);
//
//		sleep(7);
//		printf("first send move\n");
//		goto_setpoint_s2.timestamp = hrt_absolute_time();
//		orb_publish(ORB_ID(goto_setpoint), goto_pub2 , &goto_setpoint_s2);
//
//		for(int i = 0; i<50;i++){
//			goto_setpoint_s1.timestamp = hrt_absolute_time();
//			orb_publish(ORB_ID(goto_setpoint), goto_pub1 , &goto_setpoint_s1);
//			usleep(200000);
//		}
//
//		printf("Return with goto\n");
//		goto_setpoint_s2.timestamp = hrt_absolute_time();
//		goto_setpoint_s2.position[0] = - goto_setpoint_s2.position[0];
//		goto_setpoint_s2.position[1] = - goto_setpoint_s2.position[1] ;
//		goto_setpoint_s2.position[2] = goto_setpoint_s2.position[2] ;
//		orb_publish(ORB_ID(goto_setpoint), goto_pub2 , &goto_setpoint_s2);
//
//		for (int i = 0; i<20;i++){
//			goto_setpoint_s1.timestamp = hrt_absolute_time();
//			orb_publish(ORB_ID(goto_setpoint), goto_pub1 , &goto_setpoint_s1);
//			usleep(200000);
//		}
//
//		sleep(4);
//		printf("Landing");
//		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
//		return 0;
//}

int MovementCommander::repos(float x, float y,float z){
    		GPSController gpsC = *new GPSController();
		double* startpoint = gpsC.getstart();
		float latmeters = x;
		float longmeters = y;
		float alt = z;



		double* waypoint = gpsC.createWaypoint(latmeters,longmeters,alt);
		printf("startpoint: lat %lf; lon %lf; alt %lf\n", startpoint[0],startpoint[1],startpoint[2]);
		printf("coordinate: lat %lf; lon %lf; alt %lf\n", waypoint[0],waypoint[1],waypoint[2]);

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
										0.5,
										vehicle_command_s::SPEED_TYPE_AIRSPEED,
										NAN,
										NAN,
										waypoint[0],
										waypoint[1],
										waypoint[2]
										);
		printf("Donne\n");

		return 0;
}
