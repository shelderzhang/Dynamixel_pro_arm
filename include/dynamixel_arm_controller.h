/*
 * dynamixel_controller.h
 *
 *  Created on: Mar 16, 2017
 *      Author: gyz
 */

#ifndef INCLUDE_DYNAMIXEL_ARM_CONTROLLER_H_
#define INCLUDE_DYNAMIXEL_ARM_CONTROLLER_H_
#include "dynamixel_sdk.h"
// Control table address
#include <pthread.h>
#include <unistd.h>
namespace dynamixel

{
	void* read_armstatus_thread(void*args);

class WINDECLSPEC DynamixelController
{
	 public:
  DynamixelController(PortHandler *port, PacketHandler *ph, GroupSyncWrite *wr,  GroupSyncRead *rd);
  ~DynamixelController();

  int dxl1_pos;
  int dxl2_pos;
  int dxl1_vel;
  int dxl2_vel;
  int32_t dxl1_pre_pos;
  int32_t dxl2_pre_pos;
  int32_t dxl1_pre_vel;
  int32_t dxl2_pre_vel;
  int torque_enable();
  int arm_initial();
  int set_targets();
  int get_status();
  int torque_disable();
  void read_armstatus_thread_main();
  pthread_mutex_t joint_status_lock;
  void start();
  void stop();

 private:
  PortHandler    *port_hd;
  PacketHandler  *ph_hd;
  GroupSyncWrite *group_write;
  GroupSyncRead *group_read;


  int dxl_comm_result;             // Communication result
  bool dxl_addparam_result;                // addParam result
  bool dxl_getdata_result;                 // GetParam result


  uint8_t dxl_error;                          // Dynamixel error

  uint8_t dxl1_param_indirect_data_for_write[8];
  uint8_t dxl2_param_indirect_data_for_write[8];

  uint16_t ADDR_PRO_INDIRECTADDRESS_FOR_WRITE;                       // EEPROM region
  uint16_t ADDR_PRO_INDIRECTADDRESS_FOR_READ;                        // EEPROM region
  uint16_t ADDR_PRO_INDIRECTDATA;

  uint16_t ADDR_PRO_TORQUE_ENABLE;                 // Control table address is different in Dynamixel model
  uint16_t ADDR_PRO_GOAL_POSITION;
  uint16_t ADDR_PRO_GOAL_VELOCITY;
  uint16_t ADDR_PRO_PRESENT_POSITION;
  uint16_t ADDR_PRO_PRESENT_VELOCITY;
  uint16_t ADDR_PRO_PRESENT_CURRENT;

  // Data Byte Length
  uint16_t LEN_PRO_GOAL_POSITION;
  uint16_t LEN_PRO_GOAL_VELOCITY;
  uint16_t LEN_PRO_PRESENT_POSITION;
  uint16_t LEN_PRO_PRESENT_VELOCITY;
  uint16_t LEN_PRO_PRESENT_CURRENT;

  // Default setting
  int DXL1_ID;                   // Dynamixel#1 ID: 1
  int DXL2_ID;                   // Dynamixel#2 ID: 2

                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

  int TORQUE_ENABLE;                   // Value for enabling the torque
  int TORQUE_DISABLE;                   // Value for disabling the torque
  int DXL_MINIMUM_POSITION_VALUE;             // Dynamixel will rotate between this value
  int DXL_MAXIMUM_POSITION_VALUE;             // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
  int DXL_MOVING_STATUS_THRESHOLD;                  // Dynamixel moving status threshold

  int indirectdata_parameter(int DXL_ID);
  int add_parameters();
  bool time_to_exit;
  pthread_t read_armstatus_tid;

};

}





#endif /* INCLUDE_DYNAMIXEL_ARM_CONTROLLER_H_ */
