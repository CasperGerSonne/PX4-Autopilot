/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/



#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include "iot.h"
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>


// Globals:





extern "C" __EXPORT int iot_main(int argc, char *argv[])
{
	return iot::main(argc, argv);
}


iot::iot():ModuleParams(nullptr){

};

iot *iot::instantiate(int argc, char *argv[])
{

	iot *instance = new iot();

	instance->initPoint();
	printf("Z coordinate: %f",static_cast<double>(instance->_goto_point.position[2]));
	return instance;
}

int iot::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int iot::custom_command(int argc, char *argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:



	else if (!strcmp(argv[0], "rxtest1")){

		int expected = 11;
		if (argc>1){
			expected = atoi(argv[1]);
		}
		UartCommander uartCom = *new UartCommander();
		uartCom.Uart_Rxtest1(expected);
	}
	else if (!strcmp(argv[0], "rxtest2")){
		int expected = 11;
		if (argc>1){
			expected = atoi(argv[1]);
		}
		printf("%d\n",expected);
		UartCommander uartCom = *new UartCommander();
		uartCom.Uart_Rxtest2(expected);
	}
	else if (!strcmp(argv[0], "goto")){




		goto_setpoint_s goto_setpoint_s1{};
		goto_setpoint_s goto_setpoint_s2{};
		orb_advert_t goto_pub1 = orb_advertise(ORB_ID(goto_setpoint), &goto_setpoint_s1);
		orb_advert_t goto_pub2 = orb_advertise(ORB_ID(goto_setpoint), &goto_setpoint_s2);
		goto_setpoint_s1.position[0] = 0;
		goto_setpoint_s1.position[1] = 0;
		goto_setpoint_s1.position[2] = 0;
		goto_setpoint_s1.heading = 0;
		goto_setpoint_s1.max_horizontal_speed = 1;
		goto_setpoint_s1.max_vertical_speed = 1;
		goto_setpoint_s1.max_heading_rate = 0;
		goto_setpoint_s1.flag_control_heading = true;
		goto_setpoint_s1.flag_set_max_horizontal_speed = true;
		goto_setpoint_s1.flag_set_max_vertical_speed = true;
		goto_setpoint_s1.flag_set_max_heading_rate = true;

		goto_setpoint_s2.position[0] = 0;
		goto_setpoint_s2.position[1] = 0;
		goto_setpoint_s2.position[2] = -2;
		goto_setpoint_s2.heading = 0;
		goto_setpoint_s2.max_horizontal_speed = 1;
		goto_setpoint_s2.max_vertical_speed = 1;
		goto_setpoint_s2.max_heading_rate = 0;
		goto_setpoint_s2.flag_control_heading = true;
		goto_setpoint_s2.flag_set_max_horizontal_speed = true;
		goto_setpoint_s2.flag_set_max_vertical_speed = true;
		goto_setpoint_s2.flag_set_max_heading_rate = true;

		for (int i = 1; i< argc; i++){
			if(!strcmp(argv[i], "-x")){
				goto_setpoint_s2.position[0] = atoi(argv[i+1]);
			}
			if(!strcmp(argv[i], "-y")){
				goto_setpoint_s2.position[1] = atoi(argv[i+1]);
			}
			if(!strcmp(argv[i], "-z")){
				goto_setpoint_s2.position[2] = atoi(argv[i+1]);
			}

		}
		printf("Takeoff");
		const char *takeoff = "takeoff";
		char* takeoffPtr = const_cast<char*>(takeoff);
		Commander::custom_command(1, &takeoffPtr);

		sleep(7);
		printf("first send move\n");
		goto_setpoint_s2.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(goto_setpoint), goto_pub2 , &goto_setpoint_s2);

		for(int i = 0; i<50;i++){
			goto_setpoint_s1.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(goto_setpoint), goto_pub1 , &goto_setpoint_s1);
			usleep(200000);
		}

		printf("Return with goto\n");
		goto_setpoint_s2.timestamp = hrt_absolute_time();
		goto_setpoint_s2.position[0] = - goto_setpoint_s2.position[0];
		goto_setpoint_s2.position[1] = - goto_setpoint_s2.position[1] ;
		goto_setpoint_s2.position[2] = goto_setpoint_s2.position[2] ;
		orb_publish(ORB_ID(goto_setpoint), goto_pub2 , &goto_setpoint_s2);

		for (int i = 0; i<20;i++){
			goto_setpoint_s1.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(goto_setpoint), goto_pub1 , &goto_setpoint_s1);
			usleep(200000);
		}

		sleep(4);
		printf("Landing");
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
		return 0;
	}

	else if (!strcmp(argv[0], "activation")){

		Serial_Port serial_port("/dev/ttyS1", 115200);
        	serial_port.start();

		GPSController gpsC = *new GPSController();


		int bytes_read = 0;
        	int totbytes = 0;
		int expectedn = 1;

		float ccDistBase = 0;

		double* waypoint;

		bool isflying = false;

		if (argc>1){
			expectedn = atoi(argv[1]);
		}

		uint8_t* data = new uint8_t[expectedn];
        	uint8_t* buffer = new uint8_t[expectedn];




		while(true){
			//stay in goto struct point


			//wait for 200 or recieve message
			bytes_read = serial_port._read_port(*buffer,expectedn,200);

			//add to data if having read something
			if (bytes_read > 0){
				for (int i = 0; i<  totbytes + bytes_read;i++){
					data[i+totbytes] = buffer[i];
				}
				totbytes += bytes_read;
				printf("Read message\n");
			}else{continue;}


			if (!(totbytes == expectedn)){

				continue;

			}

			ccDistBase = data[0]; // in dm
			printf("Got a distance in dm %.f\n", static_cast<double>(ccDistBase));
			totbytes = 0;
			if ((ccDistBase <=  10)){

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

					waypoint = gpsC.createWaypoint(-ccDistBase/10,0,1.5);
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
					printf("isflying doing movement");
					waypoint = gpsC.createWaypoint(-ccDistBase/10,0,1.5);
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




	else if (!strcmp(argv[0], "repos")){





		GPSController gpsC = *new GPSController();
		double* startpoint = gpsC.getstart();

		float latmeters = 0;
		float longmeters = 0;
		float alt = 0;

		for (int i = 1; i< argc; i++){
			if(!strcmp(argv[i], "-x")){
				printf("latarg: %s\n",argv[i+1]);
				latmeters = atoi(argv[i+1]);
			}
			if(!strcmp(argv[i], "-y")){
				printf("lonarg: %s\n",argv[i+1]);
				longmeters = atoi(argv[i+1]);
			}
			if(!strcmp(argv[i], "-z")){
				printf("altarg: %s\n",argv[i+1]);
				alt  =  atoi(argv[i+1]);
			}
		}

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


	else if (!strcmp(argv[0], "GPStest")){
		// Subscirbe to "sensor_gyro", then set a polling interval of 200ms
		int gps_sub = orb_subscribe(ORB_ID(sensor_gps));
		orb_set_interval(gps_sub, 200);

		// Configure a POSIX POLLIN system to sleep the current thread until
		// data appears on the topic
		px4_pollfd_struct_t fds_gps;
		fds_gps.fd = gps_sub;
		fds_gps.events = POLLIN;
		hrt_abstime starttime = hrt_absolute_time();
		 // Loop a specified number of times
    for(int i = 1; i <= 400; i++)
    {
        // Allow the POSIX POLLIN system to poll for data, with 1000ms timeout
        int poll_ret = px4_poll(&fds_gps, 1, 1000);

        // If px4_poll returns 0, then the poll system timed out! Throw an error.
        if(poll_ret == 0)
        {
            PX4_ERR("Got no data within a second");
        }

        // If it didn't return 0, we got data!
        else
        {
            // Double check that the data we recieved was in the right format (I think - need to check)
            if(fds_gps.revents & POLLIN)
            {

                // Create a sensor_gps_s struct to store the data we recieved
                struct sensor_gps_s gps;

                // Copy the data over to the struct
                orb_copy(ORB_ID(sensor_gps), gps_sub, &gps);


                // Finally, print the data!
                printf("%lld |  %+2.6f  |  %+2.6f  |  %+2.2f  \n", hrt_absolute_time()-starttime, (double)gps.latitude_deg, (double)gps.longitude_deg, (double)gps.altitude_msl_m);
            }
        }
    }

    // Tell the user that the application is ending...
    PX4_INFO("Hovergames gyro exit");

    // Typical C exit :)
    return 0;

	}


	return print_usage("unknown command");
}


int iot::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

void iot::initPoint(){
	_goto_point.position[0] = 0;
	_goto_point.position[1] = 0;
	_goto_point.position[2] = -2.5;
	_goto_point.heading = 90*M_DEG_TO_RAD;
	_goto_point.max_horizontal_speed = 1;
	_goto_point.max_vertical_speed = 1;
	_goto_point.max_heading_rate = 2;
	_goto_point.flag_control_heading = true;
	_goto_point.flag_set_max_horizontal_speed = true;
	_goto_point.flag_set_max_vertical_speed = true;
	_goto_point.flag_set_max_heading_rate = true;
}

void iot::changePoint(float x, float y , float z ){

	if (z>0){
		PX4_WARN("z greater than 0 is underground");
	}else{

	_goto_point.position[0] = x;
	_goto_point.position[1] = y;
	_goto_point.position[2] = z;

	}
}

void iot::publishPoint(){
	orb_advert_t goto_setpoint_pub = orb_advertise(ORB_ID(goto_setpoint), &_goto_point);
	_goto_point.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(goto_setpoint), goto_setpoint_pub, &_goto_point);
}


void iot::run()
{
	//const char *takeoff = "takeoff";
	//char* takeoffPtr = const_cast<char*>(takeoff);
	//Commander::custom_command(1, &takeoffPtr);
	//sleep(7);
	//Serial_Port serial_port("/dev/ttyS1", 115200);
	//serial_port.start();
	//int bytes_read;
	//uint8_t* data = new uint8_t[6];
	//uint8_t distances[6];
	//while (!should_exit()) {
	//	bytes_read = serial_port._read_port(*data,1,200);
	//	printf("Bytes recieved %d\n",bytes_read);
	//	if (bytes_read >= 1){
//
	//		for (int i = 0; i < bytes_read; ++i) {
	//			distances[i] = data[i];
	//			printf("Distance %d is %u\n",i,distances[i]);
	//		}
//
//
	//		uint8_t furthest = 0;
	//		for (int i = 0;i < 6; i++){
	//			furthest = mymax(furthest,distances[i]);
	//		}
	//		if (furthest > 5){
	//			changePoint(0,furthest/2,-1.5);
	//		}else{
	//			changePoint(0,0,-1.5);
	//		}
	//	}else{changePoint(0,0,-1.5);}
//
//
	//	printf("X: %f  Y: %f  Z: %f\n",(double)_goto_point.position[0],(double)_goto_point.position[1],(double)_goto_point.position[2]);
//
//
	//	publishPoint();
//
//
	//}
	while (!should_exit()) {
		sleep(1);
	}
	sleep(10);



}



int iot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


