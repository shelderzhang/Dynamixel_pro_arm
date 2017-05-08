/*
 * dynamixel_arm_controller.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: gyz
 */


#include <algorithm>
#include "dynamixel_arm_controller.h"
#include <math.h>


using namespace dynamixel;



DynamixelController::DynamixelController(PortHandler *port, PacketHandler *ph, GroupSyncWrite *wr, GroupSyncRead *rd)
{
	  port_hd = port;
	  ph_hd = ph;
	  group_write = wr;
	  group_read = rd;


	  dxl_addparam_result = false;                // addParam result
	  dxl_getdata_result = false;                 // GetParam result


	  dxl_error = 0;                          // Dynamixel error
	  ADDR_PRO_INDIRECTADDRESS_FOR_WRITE = 49;                  // EEPROM region
	  ADDR_PRO_INDIRECTADDRESS_FOR_READ = 57;                  // EEPROM region
	  ADDR_PRO_INDIRECTDATA = 638;

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

	  read_armstatus_tid  = 0; // read thread id
	  time_to_exit   = false;  // flag to signal thread exit

	  int result = pthread_mutex_init(&joint_status_lock, NULL);
	  if ( result != 0 )
	   {
	       printf("\n DynamixelController joint_status_lock mutex init failed\n");
	   }


}
DynamixelController::~DynamixelController()
{
	pthread_mutex_destroy(&joint_status_lock);
}

int DynamixelController::torque_enable()
{
	  // Enable Dynamixel#1 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	    fprintf(stderr, "[ID:%03d] torque_enable failed \n", DXL1_ID);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }
	  else
	  {
	    printf("Dynamixel#%d torque enable \n", DXL1_ID);
	  }

	  // Enable Dynamixel#2 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	    fprintf(stderr, "[ID:%03d] torque_enable failed \n", DXL2_ID);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }
	  else
	  {
	    printf("Dynamixel#%d torque enable \n", DXL2_ID);
	  }

	  return 0;
}

int DynamixelController::arm_initial()
{
	torque_disable();
	indirectdata_parameter(DXL1_ID);
	indirectdata_parameter(DXL2_ID);
	torque_enable();
	add_parameters();
	return 0;
}
int DynamixelController::indirectdata_parameter(int DXL_ID)
{
	int i;
	// INDIRECTDATA parameter storages replace LED, goal position, present position and moving status storages
	for (i = 0; i<4;i++ )
	{
		dxl_comm_result = ph_hd->write2ByteTxRx(port_hd, DXL_ID, (ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 2*i), (ADDR_PRO_GOAL_POSITION + i), &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
		    ph_hd->printTxRxResult(dxl_comm_result);
		    fprintf(stderr, "[ID:%03d] POS write indirectdata_parameter failed \n", DXL_ID);
		}
	    else if (dxl_error != 0)
		{
			 ph_hd->printRxPacketError(dxl_error);
	    }

	}
//	for (i = 0; i<4;i++ )
//	{
//		dxl_comm_result = ph_hd->write2ByteTxRx(port_hd, DXL_ID, (ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 8+2*i), (ADDR_PRO_GOAL_VELOCITY + i), &dxl_error);
//		if (dxl_comm_result != COMM_SUCCESS)
//		{
//		    ph_hd->printTxRxResult(dxl_comm_result);
//		}
//	    else if (dxl_error != 0)
//		{
//			 ph_hd->printRxPacketError(dxl_error);
//	    }
//
//	}

	for (i = 0; i<4;i++ )
	{
		dxl_comm_result = ph_hd->write2ByteTxRx(port_hd, DXL_ID, (ADDR_PRO_INDIRECTADDRESS_FOR_READ+ 2*i), (ADDR_PRO_PRESENT_POSITION + i), &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
		    ph_hd->printTxRxResult(dxl_comm_result);
		    fprintf(stderr, "[ID:%03d] POS read indirectdata_parameter failed \n", DXL_ID);
		}
	    else if (dxl_error != 0)
		{
			 ph_hd->printRxPacketError(dxl_error);
	    }

	}
	for (i = 0; i<4;i++ )
	{
		dxl_comm_result = ph_hd->write2ByteTxRx(port_hd, DXL_ID, (ADDR_PRO_INDIRECTADDRESS_FOR_READ + 8+2*i), (ADDR_PRO_PRESENT_VELOCITY + i), &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
		    ph_hd->printTxRxResult(dxl_comm_result);
		    fprintf(stderr, "[ID:%03d] VEL read indirectdata_parameter failed \n", DXL_ID);
		}
	    else if (dxl_error != 0)
		{
			 ph_hd->printRxPacketError(dxl_error);
	    }

	}

	return 0;

}

int DynamixelController::add_parameters()
{
	// Add parameter storage for Dynamixel#1 present position value
	  dxl_addparam_result = group_read->addParam(DXL1_ID);
	  if (dxl_addparam_result != true)
	  {
	    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed \n", DXL1_ID);
	    return 0;
	  }

	  // Add parameter storage for Dynamixel#2 present position value
	  dxl_addparam_result = group_read->addParam(DXL2_ID);
	  if (dxl_addparam_result != true)
	  {
	    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed \n", DXL2_ID);
	    return 0;
	  }

	  return 0;
}


int DynamixelController::set_targets()
{
	// Allocate goal position value into byte array
	    dxl1_param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(dxl1_pos));
	    dxl1_param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(dxl1_pos));
	    dxl1_param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(dxl1_pos));
	    dxl1_param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(dxl1_pos));
//	    dxl1_param_indirect_data_for_write[4] = DXL_LOBYTE(DXL_LOWORD(dxl1_vel));
//	    dxl1_param_indirect_data_for_write[5] = DXL_HIBYTE(DXL_LOWORD(dxl1_vel));
//	    dxl1_param_indirect_data_for_write[6] = DXL_LOBYTE(DXL_HIWORD(dxl1_vel));
//	    dxl1_param_indirect_data_for_write[7] = DXL_HIBYTE(DXL_HIWORD(dxl1_vel));

	    dxl2_param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(dxl2_pos));
	    dxl2_param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(dxl2_pos));
	    dxl2_param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(dxl2_pos));
	    dxl2_param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(dxl2_pos));
//	    dxl2_param_indirect_data_for_write[4] = DXL_LOBYTE(DXL_LOWORD(dxl2_vel));
//	    dxl2_param_indirect_data_for_write[5] = DXL_HIBYTE(DXL_LOWORD(dxl2_vel));
//	    dxl2_param_indirect_data_for_write[6] = DXL_LOBYTE(DXL_HIWORD(dxl2_vel));
//	    dxl2_param_indirect_data_for_write[7] = DXL_HIBYTE(DXL_HIWORD(dxl2_vel));

	    // Add Dynamixel#1 goal position value to the Syncwrite storage
	    dxl_addparam_result = group_write->addParam(DXL1_ID, dxl1_param_indirect_data_for_write);
	    if (dxl_addparam_result != true)
	    {
	      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed \n", DXL1_ID);
	      return 0;
	    }

	    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	    dxl_addparam_result = group_write->addParam(DXL2_ID, dxl2_param_indirect_data_for_write);
	    if (dxl_addparam_result != true)
	    {
	      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed \n", DXL2_ID);
	      return 0;
	    }

	    // Syncwrite goal position
	    dxl_comm_result = group_write->txPacket();
	    if (dxl_comm_result != COMM_SUCCESS) ph_hd->printTxRxResult(dxl_comm_result);

	    // Clear syncwrite parameter storage
	    group_write->clearParam();
	    return 0;
}


int DynamixelController::get_status()
{
	// Syncread present position
	      dxl_comm_result = group_read->txRxPacket();
	      if (dxl_comm_result != COMM_SUCCESS) ph_hd->printTxRxResult(dxl_comm_result);

	      // Check if posgroupSyncRead data of Dynamixel#1 is available
	      dxl_getdata_result = group_read->isAvailable(DXL1_ID, ADDR_PRO_INDIRECTDATA, LEN_PRO_PRESENT_POSITION);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d]  groupSyncRead POS getdata failed \n", DXL1_ID);
	        return 0;
	      }

	      dxl_getdata_result = group_read->isAvailable(DXL1_ID, ADDR_PRO_INDIRECTDATA+LEN_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_VELOCITY);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d] groupSyncRead VEL getdata failed \n", DXL1_ID);
	        return 0;
	      }

	      // Get Dynamixel#1 present position value
	      dxl1_pre_pos = group_read->getData(DXL1_ID, ADDR_PRO_INDIRECTDATA, LEN_PRO_PRESENT_POSITION);
	      dxl1_pre_vel = group_read->getData(DXL1_ID, ADDR_PRO_INDIRECTDATA+LEN_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_VELOCITY);


	      // Check if posgroupSyncRead data of Dynamixel#2 is available
	      dxl_getdata_result = group_read->isAvailable(DXL2_ID, ADDR_PRO_INDIRECTDATA, LEN_PRO_PRESENT_POSITION);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d] groupSyncRead POS getdata failed \n", DXL2_ID);
	        return 0;
	      }

	      dxl_getdata_result = group_read->isAvailable(DXL2_ID, ADDR_PRO_INDIRECTDATA+LEN_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_VELOCITY);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d] groupSyncRead VEL getdata failed \n", DXL2_ID);
	        return 0;
	      }


	      // Get Dynamixel#2 present position value
	      dxl2_pre_pos = group_read->getData(DXL2_ID, ADDR_PRO_INDIRECTDATA, LEN_PRO_PRESENT_POSITION);
	      dxl2_pre_vel = group_read->getData(DXL2_ID, ADDR_PRO_INDIRECTDATA+LEN_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_VELOCITY);

		  return 0;
}

int DynamixelController::torque_disable()
{
	  // Disable Dynamixel#1 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	    fprintf(stderr, "[ID:%03d] torque_disable failed \n", DXL1_ID);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }
	  else
	  {
	    printf("Dynamixel#%d  torque disable \n", DXL1_ID);
	  }

	  // Disable Dynamixel#2 Torque
	  dxl_comm_result = ph_hd->write1ByteTxRx(port_hd, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    ph_hd->printTxRxResult(dxl_comm_result);
	    fprintf(stderr, "[ID:%03d] torque_disable failed \n", DXL2_ID);
	  }
	  else if (dxl_error != 0)
	  {
	    ph_hd->printRxPacketError(dxl_error);
	  }
	  else
	  {
	    printf("Dynamixel#%d  torque disable \n", DXL2_ID);
	  }

	  return 0;
}




