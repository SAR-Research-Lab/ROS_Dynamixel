/*****************************************************
 *              Header Includes                      *
 *****************************************************/
#include <ros/ros.h>
#include "dynamixel_includes.h"
#include "dynamixel_control/ServoPosition.h"

/*****************************************************
 *              Prototypes                           *
 *****************************************************/
static void set_servo_position(int32_t requested_degree); 
static void Ros_Init(int argc, char **argv); 
static bool Dynamixel_Init(void); 

/*****************************************************
 *             File Scope Variables                  *
 *****************************************************/
static uint8_t left_right_id; 
static uint8_t forward_back_id;
static DynamixelWorkbench my_dynamixel; 
static ros:: Subscriber sub;

/*****************************************************
 *            Definitions                            *
 *****************************************************/
#define SERVO_DEGREE_MIN    -130.0f
#define SERVO_DEGREE_MAX    -50.0f
#define SERVO_DEGREE_CENTER -90.0f
#define RAD2DEG(x) x * (180.0/3.141592653589793238463)
#define DEG2RAD(x) x * (3.141592653589793238463/180.0)

/*****************************************************
 *              Callback for accepting               *
 *              the servo position goal.             *
 *****************************************************/
void goalPositionCallback(dynamixel_control::ServoPosition requestedServoPosition)
{
  float       currentDegreeValue; 
  float       requestedDegreeValue; 
  float       servoRadValue; 
  float       radianValue;
  const char* log;

  if(requestedServoPosition.servoSetCenter == 1)
  {
    set_servo_position(SERVO_DEGREE_CENTER);
  }
  else
  {
    my_dynamixel.getRadian(left_right_id, &radianValue, &log); 
    currentDegreeValue = RAD2DEG(radianValue);

    /* Convert degrees to radians */
    if(requestedServoPosition.servoClockWiseRotation == 0)
    {
        requestedDegreeValue = currentDegreeValue + requestedServoPosition.servoDegreeRotation; 
    }
    else
    {
        requestedDegreeValue = currentDegreeValue - requestedServoPosition.servoDegreeRotation; 
    }

    set_servo_position(requestedDegreeValue);

    ROS_INFO("Requested final degree: %f", requestedDegreeValue);
  }
}

/*****************************************************
 *              Initializations for ROS              *
 *****************************************************/
static void Ros_Init(int argc, char **argv)
{

  /* Initialize the ROS system. */
  ros::init(argc, argv, "dynamixel_control_rebecca");

  /* Establish this program as a ROS node. */
  ros::NodeHandle nh;

  /* Set up the servo subscriber */
  sub = nh.subscribe<dynamixel_control::ServoPosition>("dynamixel_control_rebecca", 100, goalPositionCallback);
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
 *              Set the requested
 *              position of the servo                           
 *****************************************************/
static void set_servo_position(int32_t requested_degree)
{    
    float       currentDegreeValue; 
    float       requestedDegreeValue; 
    float       servoRadValue; 
    float       radianValue;
    const char* log;

    /* Bounds check */
    if(requested_degree < SERVO_DEGREE_MIN)
    {
        requested_degree = SERVO_DEGREE_MIN; 
    }
    else if(requested_degree > SERVO_DEGREE_MAX)
    {
        requested_degree = SERVO_DEGREE_MAX; 
    }
    else
    {
        /* Do nothing */
    }
 
    /* Get the current position of the servo */
    my_dynamixel.getRadian(left_right_id, &radianValue, &log); 
    currentDegreeValue = RAD2DEG(radianValue);
    
    /* Determine which way to turn */
    if(requested_degree > currentDegreeValue)
    {
        while(currentDegreeValue <= requested_degree)
        {
            currentDegreeValue = currentDegreeValue + 1;
                        
            /* Convert back to radians */
            servoRadValue = DEG2RAD(currentDegreeValue); 
 
            /* Send the position to the servo */
            my_dynamixel.goalPosition(left_right_id, servoRadValue, &log);
            
            /* Get the new position */
            my_dynamixel.getRadian(left_right_id, &radianValue, &log); 
            currentDegreeValue = RAD2DEG(radianValue);
        }
    }
    else
    {
        while(currentDegreeValue >= requested_degree)
        {
            currentDegreeValue = currentDegreeValue - 1;
                        
            /* Convert back to radians */
            servoRadValue = DEG2RAD(currentDegreeValue); 
 
            /* Send the position to the servo */
            my_dynamixel.goalPosition(left_right_id, servoRadValue, &log);
            
            /* Get the new position */
            my_dynamixel.getRadian(left_right_id, &radianValue, &log); 
            currentDegreeValue = RAD2DEG(radianValue);
        }
    }
}

/*****************************************************
 *              Main Entry Point                     *
 *****************************************************/
int main(int argc, char **argv) 
{  
  const char* log;
  float       radianValue; 
  int32_t     servoCurrentPosition;    
  
  /* Local Inits */
  left_right_id   = 2U;
  forward_back_id = 1U;  
  
  /* Initialize ROS nodes and variables */  
  Ros_Init(argc, argv);

  /* Set up the rate */
  ros:: Rate rate(10); 
  
  /* Initialize the necessary USB port for dynamixel communication */
  if(Dynamixel_Init() == true)     
  {
    /* Start up servo centered */
    set_servo_position(SERVO_DEGREE_CENTER);

    /* Loop forever */    
    while(ros::ok())
    {    
      /* Trigger callback to fire and grab latest data and update the servo */
      ros::spinOnce(); 
   
      /* Wait for some time */
      rate.sleep();
     }      
  }
  else
  {
    ROS_ERROR("Couldn't connect to servo! Check that the USB is plugged in and the servo is powered on!"); 
  }
}
