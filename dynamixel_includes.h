/* Include interfaces */

#ifndef DYNAMIXEL_INCLUDES_H_
#define DYNAMIXEL_INCLUDES_H_

//#define SIMULATE_DYNAMIXEL

#ifdef SIMULATE_DYNAMIXEL
	#include "stub/dynamixel_stub.h"
#else
	#include "/home/rebecca/projects/ECE591_WS/src/dynamixel-workbench/dynamixel_workbench_toolbox/include/dynamixel_workbench_toolbox/dynamixel_workbench.h"
#endif

#endif /*DYNAMIXEL_INCLUDES_H_*/
