/* Stub interfaces */

#ifndef DYNAMIXEL_STUB_H_
#define DYNAMIXEL_STUB_H_

#include "dynamixel_driver_stub.h"

class DynamixelWorkbench : public DynamixelDriver
{
 public:
  DynamixelWorkbench();
  ~DynamixelWorkbench();

  bool torqueOn(uint8_t id, const char **log = NULL);

  bool goalPosition(uint8_t id, int value, const char **log = NULL);

  bool getPresentPositionData(uint8_t id, int32_t* data, const char **log = NULL);
};

#endif /*DYNAMIXEL_STUB_H_*/
