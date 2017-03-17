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


namespace dynamixel

{

class WINDECLSPEC DynamixelController
{
	 public:
  DynamixelController(PortHandler *port, PacketHandler *ph, GroupSyncWrite *pos_wr, GroupSyncWrite *vel_wr, GroupSyncRead *pos_rd, GroupSyncRead *vel_rd);
  ~DynamixelController(){};

  int dxl1_pos;
  int dxl2_pos;
  int dxl1_vel;
  int dxl2_vel;
  int32_t dxl1_pre_pos;
  int32_t dxl2_pre_pos;
  int32_t dxl1_pre_vel;
  int32_t dxl2_pre_vel;
  int torque_enable();
  int add_parameters();
  int set_position();
  int set_velocity();
  int get_position();
  int get_velocity();
  int torque_disable();


 private:
  PortHandler    *port_hd;
  PacketHandler  *ph_hd;
  GroupSyncWrite *pos_write;
  GroupSyncWrite *vel_write;
  GroupSyncRead *pos_read;
  GroupSyncRead *vel_read;

  int dxl_comm_result;             // Communication result
  bool dxl_addparam_result;                // addParam result
  bool dxl_getdata_result;                 // GetParam result


  uint8_t dxl_error;                          // Dynamixel error

  uint8_t dxl1_param_goal_position[4];
  uint8_t dxl2_param_goal_position[4];

  uint8_t dxl1_param_goal_velocity[4];
  uint8_t dxl2_param_goal_velocity[4];

  int32_t dxl1_present_position;
  int32_t dxl2_present_position;              // Present position

  int32_t dxl1_present_velocity;
  int32_t dxl2_present_velocity;

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




};

}





#endif /* INCLUDE_DYNAMIXEL_ARM_CONTROLLER_H_ */
