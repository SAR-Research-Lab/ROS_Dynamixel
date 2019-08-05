
// This header defines the standard ROS classes.
#include <ros/ros.h>
#include "dynamixel_includes.h"
#include "dynamixel_control/ServoPosition.h"

static uint8_t left_right_id; 
static uint8_t forward_back_id;
static DynamixelWorkbench my_dynamixel; 
static bool increment = true; 

/*****************************************************
 *              Periodic Timer Callback              *
 *              for getting current position of      *
 *              Dynamixel.                           *
 *****************************************************/
void readCurPositionTimer(const ros::TimerEvent& event)
{
  const char* log;  
  int32_t     current_position; 

  /* Local init */
  current_position = 0U; 

  /* Read and print the current position */
  my_dynamixel.getPresentPositionData(left_right_id, &current_position, &log); 

  if(increment == true)
  {
    ROS_INFO("Incrementing");     
    current_position = current_position + 10;
    
    if(current_position >= 350)
    {
      increment = false;  
    }
  }
  else
  {
    ROS_INFO("Decrementing");     
    current_position = current_position - 10; 
    
    if(current_position <= 50)
    {
      increment = true; 
    }
  }

  my_dynamixel.goalPosition(left_right_id, current_position); 

  ROS_INFO("Current position: %d", current_position); 
}

/*****************************************************
 *              Main Entry Point                     *
 *****************************************************/
int main(int argc, char **argv) 
{
  bool               result;
  uint32_t           baud_rate;
  const char*        log;
  const char*        device_name; 
  
  /* Local Inits */
  result          = false;
  baud_rate       = 1000000;
  device_name     = "/dev/ttyUSB0";
  left_right_id   = 2U;
  forward_back_id = 1U;  
  
  /* Initialize the ROS system. */
  ros::init(argc, argv, "dynamixel_control_rebecca");

  /* Establish this program as a ROS node. */
  ros::NodeHandle nh;

  /* Send some output as a log message. */
  ROS_INFO_STREAM("Starting Dynamixel Node!");

  /* Initialize the necessary USB port for dynamixel communication */
  if(my_dynamixel.init(device_name, baud_rate, &log) == true) 
  {
    /* Ping Dynamixel to see if it is on */
    if((my_dynamixel.ping(left_right_id, &log) == true) && (my_dynamixel.ping(forward_back_id, &log)))
    {
      /* Turn on the torque */       
      my_dynamixel.torqueOn(left_right_id, &log); 
      my_dynamixel.torqueOn(forward_back_id, &log);     
      
      /* Start a timer to read the current position every 10ms */
      ros::Timer timer = nh.createTimer(ros::Duration(0.1), readCurPositionTimer);

      /* Loop forever */
      ros::spin();
    }
    else
    {
      ROS_ERROR("Couldn't ping Dynamixels. Double check that your system is powered on!"); 
    }
  }
  else
  {
    ROS_ERROR("Couldn't open USB port! Double check that your USB connector is plugged in!"); 
  }
}
