#ifndef DYNAMIXEL_DRIVER_STUB_H
#define DYNAMIXEL_DRIVER_STUB_H


class DynamixelDriver
{
 public:
  DynamixelDriver();
  ~DynamixelDriver();

  bool init(const char* device_name = "/dev/ttyUSB0", 
            uint32_t baud_rate = 57600, 
            const char **log = NULL);

  bool ping(uint8_t id,
            const char **log = NULL);
};

#endif //DYNAMIXEL_DRIVER_STUB_H
