/*****************************************************
 *              Header Includes                      *
 *****************************************************/
#include <ros/ros.h>
#include "dynamixel_includes.h"
#include "dynamixel_control/ServoPosition.h"

/*****************************************************
 *              Prototypes                           *
 *****************************************************/
static void Ros_Init(int argc, char **argv); 
static bool Dynamixel_Init(void); 

/*****************************************************
 *             File Scope Variables                  *
 *****************************************************/
static uint8_t left_right_id; 
static uint8_t forward_back_id;
static DynamixelWorkbench my_dynamixel; 
static dynamixel_control::ServoPosition myServoPosition;
static ros:: Publisher pub; 

/*****************************************************
 *              Initializations for ROS              *
 *****************************************************/
static void Ros_Init(int argc, char **argv)
{

  /* Initialize the ROS system. */
  ros::init(argc, argv, "dynamixel_control_rebecca");

  /* Establish this program as a ROS node. */
  ros::NodeHandle nh;

  /* Set up the servo publisher and subscriber */
  pub = nh.advertise<dynamixel_control::ServoPosition>("dynamixel_control_rebecca", 100); 
}

/*****************************************************
 *              Initializations for the              *
                Dynamixels                           *
 *****************************************************/
static bool Dynamixel_Init(void)
{
  bool        result;
  uint32_t    baud_rate;
  const char* log;
  const char* device_name; 

  /* Local inits */ 
  result      = false;  
  baud_rate   = 1000000;
  device_name = "/dev/ttyUSB0";

  /* Initialize the necessary USB port for dynamixel communication */
  if(my_dynamixel.init(device_name, baud_rate, &log) == true) 
  {
    /* Ping Dynamixel to see if it is on */
    if((my_dynamixel.ping(left_right_id, &log) == true) && (my_dynamixel.ping(forward_back_id, &log)))
    {
      /* Send some output as a log message. */
      ROS_INFO_STREAM("Starting Dynamixel Node!"); 
     
      /* Turn on the torque */       
      my_dynamixel.torqueOn(left_right_id, &log); 
      my_dynamixel.torqueOn(forward_back_id, &log); 
      
      result = true; 
    }
  }

  return(result); 
}

/*****************************************************
 *              Main Entry Point                     *
 *****************************************************/
int main(int argc, char **argv) 
{  
  const char* log;  
  
  /* Local Inits */
  left_right_id   = 2U;
  forward_back_id = 1U;  
  
  /* Initialize ROS nodes and variables */  
  Ros_Init(argc, argv);

  /* Set up the rate */
  ros:: Rate rate(100); 
  
  /* Initialize the necessary USB port for dynamixel communication */
  if(Dynamixel_Init() == true)     
  {
    /* Loop forever */    
    while(ros::ok())
    { 
      /* Get the current position of the servo and publish it */
      my_dynamixel.getPresentPositionData(left_right_id, &myServoPosition.servoCurrentPosition, &log);
      pub.publish(myServoPosition); 
  
      ROS_INFO("Current position: %d", myServoPosition.servoCurrentPosition);
      rate.sleep();
     }      
  }
  else
  {
    ROS_ERROR("Couldn't connect to servo! Check that the USB is plugged in and the servo is powered on!"); 
  }
}
