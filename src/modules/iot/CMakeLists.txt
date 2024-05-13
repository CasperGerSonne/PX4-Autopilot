
add_subdirectory(MovementCommander)

px4_add_module(
	MODULE modules__iot
	MAIN iot
	COMPILE_FLAGS
	SRCS
		iot.cpp
	DEPENDS
	MovementCommander


	)

