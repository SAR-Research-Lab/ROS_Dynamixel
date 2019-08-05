/* Stub files for dynamixel */
#include "dynamixel_stub.h"

static int32 global_servoPosition = 195; //This is the center position of the left-right motion 

bool init(const char* device_name = "/dev/ttyUSB0", uint32_t baud_rate = 57600, const char **log = NULL)
{
	return(true); 
}


bool ping(uint8_t id, const char **log = NULL)
{
	return(true);
}


bool torqueOn(uint8_t id, const char **log = NULL)
{
	return(true); 
}


bool goalPosition(uint8_t id, int value, const char **log = NULL)
{
	global_servoPosition = value; 
	return(true); 
}

bool getPresentPositionData(uint8_t id, int32_t* data, const char **log = NULL)
{
	*data = global_servoPosition; 
	return(true); 

}
