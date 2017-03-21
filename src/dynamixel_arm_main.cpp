/**
 * @file dynamixel_arm_main.cpp
 *
 * @brief arm controller of the aerial manipulator robot system
 *
 *  *
 * @author Guangyu Zhang, <gyzhang13@sina.cn>

 */
#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "../include/dynamixel_arm_controller.h"                             // Uses Dynamixel SDK library

// Control table address
// Control table address is different in Dynamixel model

#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634
#define ADDR_PRO_INDIRECTDATA_FOR_READ          642

// Data Byte Length
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          8
#define LEN_PRO_INDIRECTDATA_FOR_READ           8

#define BAUDRATE                        115200
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller


                                                         // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define DXL_MINIMUM_POSITION_VALUE      -100000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      100000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
   dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_WRITE, LEN_PRO_INDIRECTDATA_FOR_WRITE);

   // Initialize Groupsyncread instance
   dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_INDIRECTDATA_FOR_READ);

  dynamixel::DynamixelController dynamixelController(portHandler, packetHandler, &groupSyncWrite, &groupSyncRead);

  int index = 0;
  int dxl1_goal_position[2] = {-13000, 13000};
  int dxl2_goal_position[2] = {-52000, 52000}; // Goal position
  int dxl1_goal_velocity[2] = {-1000, 1000};
  int dxl2_goal_velocity[2] = {-500, 500}; // Goal position

  int32_t dxl1_present_position = 0, dxl2_present_position = 0;              // Present position
  int32_t dxl1_present_velocity = 0, dxl2_present_velocity = 0;

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }



  // Add parameter storage for Dynamixel#1 present position value
  dynamixelController.arm_initial();


  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;
    // set pos and vel
    dynamixelController.dxl1_pos = dxl1_goal_position[index];
    dynamixelController.dxl2_pos = dxl2_goal_position[index];
    dynamixelController.dxl1_vel = dxl1_goal_velocity[index];
    dynamixelController.dxl2_vel = dxl2_goal_velocity[index];
    dynamixelController.set_targets();

    do
    {
   	//get pos and vel
    	dynamixelController.get_status();
    	dxl1_present_position = dynamixelController.dxl1_pre_pos;
    	dxl1_present_velocity = dynamixelController.dxl1_pre_vel;
		dxl2_present_position = dynamixelController.dxl2_pre_pos;
		dxl2_present_velocity = dynamixelController.dxl2_pre_vel;
//
        printf("PresPos_1:%03d\t   PresPos_2:%03d\n",  dxl1_present_position,   dxl2_present_position);
        printf("Presvel_1:%03d\t   PresVel_2:%03d\n",  dxl1_present_velocity,   dxl2_present_velocity);

    }while((abs(dxl1_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl2_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

  dynamixelController.torque_disable();

  // Close port
  portHandler->closePort();

  return 0;
}

