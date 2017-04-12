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
#include <math.h>
#include <sys/time.h>



#include "dynamixel_arm_controller.h"                             // Uses Dynamixel SDK library
#include "autopilot_interface.h"
#include "serial_port.h"

// Control table address
// Control table address is different in Dynamixel model

#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634
#define ADDR_PRO_INDIRECTDATA_FOR_READ          642

// Data Byte Length
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          8
#define LEN_PRO_INDIRECTDATA_FOR_READ           8

#define BAUDRATE                        115200
#define DEVICENAME                      "/dev/Dypro_usb"      // Check which port is being used on your controller


                                                         // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define DXL_MINIMUM_POSITION_VALUE      -131584             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      131584              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

#define DXL_MOVING_STATUS_THRESHOLD     100                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
int getch();
int kbhit(void);
long getCurrentTime();
void traj_generator(double T, double *coeff, double i_pos, double f_pos);

const double PI = 3.1415926535898;
const double RADS_TO_DXL = 131584/PI;
const double DXL_TO_RADS = PI/131584;
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
  double traj_coeff1[6];
  double traj_coeff2[6];
  double dxl1_igpos[2] = {-PI/4, PI/4};
  double dxl1_fgpos[2] = {PI/4, -PI/4};
  double dxl2_igpos[2] = {-PI/2, PI/2};
  double dxl2_fgpos[2] = {PI/2, -PI/2};
  double T = 3;
  double time_stamp;
  double current_time;
  bool time_flag;
  double t;


//  int dxl1_goal_position[2] = {-13000, 13000};
//  int dxl2_goal_position[2] = {-52000, 52000}; // Goal position
//  int dxl1_goal_velocity[2] = {-1000, 1000};
//  int dxl2_goal_velocity[2] = {-500, 500}; // Goal position
  int dxl1_goal_position[2] = {-23000, 23000};
  int dxl2_goal_position[2] = {-62000, 62000}; // Goal position
  int dxl1_goal_velocity[2] = {-5000, 5000};
  int dxl2_goal_velocity[2] = {-8000, 8000}; // Goal position
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

  char *uart_name = (char*)"/dev/px4_usb";
  int baudrate = 115200;
  Autopilot_Interface *autopilot_interface_quit;
  Serial_Port *serial_port_quit;
  /*
   * Construct a Serial_Port object
   * This object handles the opening and closing of the Robai cyton epsilon300 controller
   *  computer's serial port over which it will communicate to an autopilot.  It has
   * methods to read and write a mavlink_message_t object.  To help with read
   * and write in the context of pthreading, it gaurds port operations with a
   * pthread mutex lock (int fd).
   *
   */
  Serial_Port serial_port(uart_name, baudrate);

  /*
   * Construct an Autopilot_Interface objiect,This object will start
   *  two threads for read and write MAVlink message of
   * commands to move the robotic arm and its status.
   *
   */
  Autopilot_Interface autopilot_interface(&serial_port);

  serial_port_quit         = &serial_port;
  autopilot_interface_quit = &autopilot_interface;


  /*
   * This is where the port is opened, and read and write threads are started.
   */
  serial_port.start();
  autopilot_interface.start();
  // Add parameter storage for Dynamixel#1 present position value
  dynamixelController.arm_initial();
  double dxl1_init_pos;
  double dxl2_init_pos;
  dynamixelController.get_status();
  dxl1_init_pos =DXL_TO_RADS*dynamixelController.dxl1_pre_pos;
  dxl2_init_pos = DXL_TO_RADS*dynamixelController.dxl2_pre_pos;
  traj_generator(T, traj_coeff1, dxl1_init_pos, dxl1_igpos[index]);
  traj_generator(T, traj_coeff2, dxl1_init_pos, dxl2_igpos[index]);
  sleep(3);
  t=0;
  current_time = getCurrentTime()/1.0e6f;
  time_stamp = current_time;

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;
    // set pos and vel
    sleep(2);

    	current_time = getCurrentTime()/1.0e6f;
       if (current_time-time_stamp >= 0.02f)
       {
    	   time_stamp = current_time;
    	   t = t +0.02;
    	   dynamixelController.dxl1_pos = RADS_TO_DXL*(traj_coeff1[0]+traj_coeff1[1]*t+traj_coeff1[2]*t*t +\
    	           						  traj_coeff1[3]*pow(t,3)+traj_coeff1[4]*pow(t,4)+traj_coeff1[5]*pow(t,5));
    	   dynamixelController.dxl2_pos = RADS_TO_DXL*(traj_coeff1[0]+traj_coeff1[1]*t+traj_coeff1[2]*t*t +\
    	       			   	   	   	   	  traj_coeff1[3]*pow(t,3)+traj_coeff1[4]*pow(t,4)+traj_coeff1[5]*pow(t,5));

    	   dynamixelController.set_targets();
    	   if (t >= T)
    	   {
    		   t = 0;
    		   if (index == 0)
    		    {
    		      index = 1;
    		    }
    		    else
    		    {
    		      index = 0;
    		    }
    		   traj_generator(T, traj_coeff1, dxl1_igpos[index],dxl1_fgpos[index]);
    		   traj_generator(T, traj_coeff2, dxl2_igpos[index],dxl2_fgpos[index]);
    	   }
       }

       dynamixelController.get_status();
       dxl1_present_position = dynamixelController.dxl1_pre_pos;
       dxl2_present_position = dynamixelController.dxl2_pre_pos;
       dxl1_present_velocity = dynamixelController.dxl1_pre_vel;
       dxl2_present_velocity = dynamixelController.dxl2_pre_vel;
       printf("PresPos_1:%03d\t   PresPos_2:%03d\n",  dxl1_present_position,   dxl2_present_position);
       printf("Presvel_1:%03d\t   PresVel_2:%03d\n",  dxl1_present_velocity,   dxl2_present_velocity);

       pthread_mutex_lock(&(autopilot_interface.joints_lock));
       autopilot_interface.mani_joints.joint_posi_1 = dxl1_present_position;
       autopilot_interface.mani_joints.joint_posi_2 = dxl2_present_position;
       autopilot_interface.mani_joints.joint_rate_1 = dxl1_present_velocity;
       autopilot_interface.mani_joints.joint_rate_2 = dxl2_present_velocity;
       pthread_mutex_unlock(&(autopilot_interface.joints_lock));



  }
  // --------------------------------------------------------------------------
  //   Join threads of serial port
  // --------------------------------------------------------------------------
  pthread_join (autopilot_interface.read_tid, NULL);
  pthread_join (autopilot_interface.write_tid, NULL);

  dynamixelController.torque_disable();

  // Close port
  portHandler->closePort();

  return 0;
}

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
void  traj_generator(double T, double *coeff, double i_pos, double f_pos)
{
	coeff[0] = 0;
	coeff[1] = 0;
	coeff[2] = 0;
	coeff[3] = (20*(f_pos-i_pos))/(2*pow(T,3));
	coeff[4] = (30*(f_pos-i_pos))/(2*pow(T,4));
	coeff[5] = (12*(f_pos-i_pos))/(2*pow(T,5));


}
long getCurrentTime()
{
   struct timeval tv;
   gettimeofday(&tv,NULL);
   return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
