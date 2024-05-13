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
#include "MovementCommander/MovementCommander.hpp"
#include "MovementCommander/UartCommander/UartCommander.hpp"

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

	//instance->initPoint();
	//printf("Z coordinate: %f",static_cast<double>(instance->_goto_point.position[2]));
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
	//else if (!strcmp(argv[0], "goto")){
	//	MovementCommander movecom = *new MovementCommander();
	//	movecom.gototest();
//
	//}

	else if (!strcmp(argv[0], "act1d")){

		int expectedn;

		if (argc > 1){
			expectedn = atoi(argv[1]);
		}
		else{
			PX4_ERR("Expected recieved rssi bytes not determined");
			return 1;
		}

		MovementCommander movecom = *new MovementCommander();
		movecom.activation1(expectedn);
		return 0;

	}

	else if (!strcmp(argv[0], "repos")){
		MovementCommander movecom = *new MovementCommander();
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
		movecom.repos(latmeters,longmeters,alt);

	}


	else if (!strcmp(argv[0], "GPStest")){
		GPSController gpscontroller = *new GPSController();
		gpscontroller.GPStest();

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


