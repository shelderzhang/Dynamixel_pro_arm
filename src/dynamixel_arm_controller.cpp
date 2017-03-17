/*
 * dynamixel_arm_controller.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: gyz
 */


#include <algorithm>
#include "../include/dynamixel_arm_controller.h"


using namespace dynamixel;

DynamixelController::DynamixelController(PortHandler *port, PacketHandler *ph, GroupSyncWrite *pos_wr, GroupSyncWrite *vel_wr, GroupSyncRead *pos_rd, GroupSyncRead *vel_rd)
{
	  port_hd = port;
	  ph_hd = ph;
	  pos_write = pos_wr;
	  vel_write = vel_wr;
	  pos_read = pos_rd;
	  vel_read = vel_rd;

	  dxl_addparam_result = false;                // addParam result
	  dxl_getdata_result = false;                 // GetParam result


	  dxl_error = 0;                          // Dynamixel error


	  ADDR_PRO_TORQUE_ENABLE = 562;                 // Control table address is different in Dynamixel model
	  ADDR_PRO_GOAL_POSITION = 596;
	  ADDR_PRO_GOAL_VELOCITY = 600;
	  ADDR_PRO_PRESENT_POSITION = 611;
	  ADDR_PRO_PRESENT_VELOCITY = 615;
	  ADDR_PRO_PRESENT_CURRENT = 621;

	    // Data Byte Length
	  LEN_PRO_GOAL_POSITION = 4;
	  LEN_PRO_GOAL_VELOCITY = 4;
	  LEN_PRO_PRESENT_POSITION = 4;
	  LEN_PRO_PRESENT_VELOCITY = 4;
	  LEN_PRO_PRESENT_CURRENT = 2;

	    // Default setting
	  DXL1_ID = 1;                   // Dynamixel#1 ID: 1
	  DXL2_ID = 2;                   // Dynamixel#2 ID: 2

	                                                              // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

	  TORQUE_ENABLE = 1;                   // Value for enabling the torque
	  TORQUE_DISABLE = 0;                   // Value for disabling the torque
	  DXL_MINIMUM_POSITION_VALUE = -100000;             // Dynamixel will rotate between this value
	  DXL_MAXIMUM_POSITION_VALUE = 100000;             // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
	  DXL_MOVING_STATUS_THRESHOLD = 20;                  // Dynamixel moving status threshold



}
int DynamixelController::torque_enable()
{
	  // Enable Dynamixel#1 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }
	  else
	  {
	    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
	  }

	  // Enable Dynamixel#2 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }
	  else
	  {
	    printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
	  }

	  return 0;
}
int DynamixelController::add_parameters()
{
	// Add parameter storage for Dynamixel#1 present position value
	  dxl_addparam_result = pos_read->addParam(DXL1_ID);
	  if (dxl_addparam_result != true)
	  {
	    fprintf(stderr, "[ID:%03d] posgroupSyncRead addparam failed \n", DXL1_ID);
	    return 0;
	  }

	  // Add parameter storage for Dynamixel#2 present position value
	  dxl_addparam_result = pos_read->addParam(DXL2_ID);
	  if (dxl_addparam_result != true)
	  {
	    fprintf(stderr, "[ID:%03d] posgroupSyncRead addparam failed \n", DXL2_ID);
	    return 0;
	  }

	  // Add parameter storage for Dynamixel#1 present velocity value
	  dxl_addparam_result = vel_read->addParam(DXL1_ID);
	  if (dxl_addparam_result != true)
	  {
	    fprintf(stderr, "[ID:%03d] velgroupSyncRead addparam failed", DXL1_ID);
	    return 0;
	  }

	  // Add parameter storage for Dynamixel#2 present velocity value
	  dxl_addparam_result = vel_read->addParam(DXL2_ID);
	  if (dxl_addparam_result != true)
	  {
	    fprintf(stderr, "[ID:%03d] velgroupSyncRead addparam failed", DXL2_ID);
	    return 0;
	  }
	  return 0;
}

int DynamixelController::set_position()
{
	// Allocate goal position value into byte array
	    dxl1_param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl1_pos));
	    dxl1_param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl1_pos));
	    dxl1_param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl1_pos));
	    dxl1_param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl1_pos));

	    dxl2_param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl2_pos));
	    dxl2_param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl2_pos));
	    dxl2_param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl2_pos));
	    dxl2_param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl2_pos));

	    // Add Dynamixel#1 goal position value to the Syncwrite storage
	    dxl_addparam_result = pos_write->addParam(DXL1_ID, dxl1_param_goal_position);
	    if (dxl_addparam_result != true)
	    {
	      fprintf(stderr, "[ID:%03d] posgroupSyncWrite addparam failed \n", DXL1_ID);
	      return 0;
	    }

	    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	    dxl_addparam_result = pos_write->addParam(DXL2_ID, dxl2_param_goal_position);
	    if (dxl_addparam_result != true)
	    {
	      fprintf(stderr, "[ID:%03d] posgroupSyncWrite addparam failed \n", DXL2_ID);
	      return 0;
	    }

	    // Syncwrite goal position
	    dxl_comm_result = pos_write->txPacket();
	    if (dxl_comm_result != COMM_SUCCESS) ph_hd->printTxRxResult(dxl_comm_result);

	    // Clear syncwrite parameter storage
	    pos_write->clearParam();
	    return 0;
}
int DynamixelController::set_velocity()
{
	// Allocate goal velocity value into byte array
		    dxl1_param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl1_vel));
		    dxl1_param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl1_vel));
		    dxl1_param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl1_vel));
		    dxl1_param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl1_vel));

		    dxl2_param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl2_vel));
		    dxl2_param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl2_vel));
		    dxl2_param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl2_vel));
		    dxl2_param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl2_vel));

		    // Add Dynamixel#1 goal velocity value to the Syncwrite storage
		    dxl_addparam_result = vel_write->addParam(DXL1_ID, dxl1_param_goal_velocity);
		    if (dxl_addparam_result != true)
		    {
		      fprintf(stderr, "[ID:%03d] velgroupSyncWrite addparam failed \n", DXL1_ID);
		      return 0;
		    }

		    // Add Dynamixel#2 goal velocity value to the Syncwrite parameter storage
		    dxl_addparam_result = vel_write->addParam(DXL2_ID, dxl2_param_goal_velocity);
		    if (dxl_addparam_result != true)
		    {
		      fprintf(stderr, "[ID:%03d] velgroupSyncWrite addparam failed \n", DXL2_ID);
		      return 0;
		    }

		    // Syncwrite goal velocity
		    dxl_comm_result = vel_write->txPacket();
		    if (dxl_comm_result != COMM_SUCCESS) ph_hd->printTxRxResult(dxl_comm_result);

		    // Clear syncwrite parameter storage
		    vel_write->clearParam();
		    return 0;
}

int DynamixelController::get_position()
{
	// Syncread present position
	      dxl_comm_result = pos_read->txRxPacket();
	      if (dxl_comm_result != COMM_SUCCESS) ph_hd->printTxRxResult(dxl_comm_result);

	      // Check if posgroupSyncRead data of Dynamixel#1 is available
	      dxl_getdata_result = pos_read->isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d] posgroupSyncRead getdata failed \n", DXL1_ID);
	        return 0;
	      }

	      // Check if posgroupSyncRead data of Dynamixel#2 is available
	      dxl_getdata_result = pos_read->isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d] posgroupSyncRead getdata failed \n", DXL2_ID);
	        return 0;
	      }

	      // Get Dynamixel#1 present position value
	      dxl1_pre_pos = pos_read->getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	      // Get Dynamixel#2 present position value
	      dxl2_pre_pos = pos_read->getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

		  return 0;
}
int DynamixelController::get_velocity()
{
    dxl_comm_result = vel_read->txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) ph_hd->printTxRxResult(dxl_comm_result);
      // Check if posgroupSyncRead data of Dynamixel#1 is available
      dxl_getdata_result = vel_read->isAvailable(DXL1_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] velgroupSyncRead getdata failed \n", DXL1_ID);
        return 0;
      }

      // Check if posgroupSyncRead data of Dynamixel#2 is available
      dxl_getdata_result = vel_read->isAvailable(DXL2_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] velgroupSyncRead getdata failed \n", DXL2_ID);
        return 0;
      }
      // Get Dynamixel#1 present position value
      dxl1_pre_vel = vel_read->getData(DXL1_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);

           // Get Dynamixel#2 present position value
      dxl2_pre_vel = vel_read->getData(DXL2_ID, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);
	  return 0;
}

int DynamixelController::torque_disable()
{
	  // Disable Dynamixel#1 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }

	  // Disable Dynamixel#2 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }
}


