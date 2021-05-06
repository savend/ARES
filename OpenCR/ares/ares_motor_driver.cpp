/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */
/* Modified for ARES Robot: Audric Strumpler */

#include "ares_motor_driver.h"

AresMotorDriver::AresMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_rear_wheel_id_(DXL_LEFT_REAR_ID), right_rear_wheel_id_(DXL_RIGHT_REAR_ID),
  left_front_wheel_id_(DXL_LEFT_FRONT_ID), right_front_wheel_id_(DXL_RIGHT_FRONT_ID)
{
}

AresMotorDriver::~AresMotorDriver()
{
  closeDynamixel();
}

bool AresMotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setTorque(left_rear_wheel_id_, true);
  setTorque(right_rear_wheel_id_, true);
  setTorque(left_front_wheel_id_, true);
  setTorque(right_front_wheel_id_, true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  groupSyncReadError_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, 70, 1);

  return true;
}

bool AresMotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void AresMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_rear_wheel_id_, false);
  setTorque(right_rear_wheel_id_, false);
  setTorque(left_front_wheel_id_, false);
  setTorque(right_front_wheel_id_, false);

  // Close port
  portHandler_->closePort();
}

bool AresMotorDriver::readEncoder(int32_t &left_rear_value, int32_t &right_rear_value, int32_t &left_front_value, int32_t &right_front_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_rear_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_rear_wheel_id_);
  if (dxl_addparam_result != true)
    return false;
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_front_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_front_wheel_id_);
  if (dxl_addparam_result != true)
    return false;


  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_rear_value  = groupSyncReadEncoder_->getData(left_rear_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_rear_value = (-1)*groupSyncReadEncoder_->getData(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  left_front_value  = groupSyncReadEncoder_->getData(left_rear_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_front_value = (-1)*groupSyncReadEncoder_->getData(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();
  return true;
}

  bool AresMotorDriver::readError(int &left_rear_value, int &right_rear_value, int &left_front_value, int &right_front_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadError_->addParam(left_rear_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadError_->addParam(right_rear_wheel_id_);
  if (dxl_addparam_result != true)
    return false;
  dxl_addparam_result = groupSyncReadError_->addParam(left_front_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadError_->addParam(right_front_wheel_id_);
  if (dxl_addparam_result != true)
    return false;


  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadError_->isAvailable(left_rear_wheel_id_, 70, 1);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadError_->isAvailable(right_rear_wheel_id_, 70, 1);
  if (dxl_getdata_result != true)
    return false;
  dxl_getdata_result = groupSyncReadError_->isAvailable(left_front_wheel_id_, 70, 1);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadError_->isAvailable(right_front_wheel_id_, 70, 1);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_rear_value  = groupSyncReadError_->getData(left_rear_wheel_id_,  70, 1);
  right_rear_value = groupSyncReadError_->getData(right_rear_wheel_id_, 70, 1);
  left_front_value  = groupSyncReadError_->getData(left_rear_wheel_id_,  70, 1);
  right_front_value = groupSyncReadError_->getData(right_rear_wheel_id_, 70, 1);


  groupSyncReadError_->clearParam();
  return true;
}

  bool AresMotorDriver::aresReboot(void)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error

  dxl_comm_result = packetHandler_->reboot(portHandler_, left_rear_wheel_id_, &dxl_error);
  dxl_comm_result = packetHandler_->reboot(portHandler_, left_front_wheel_id_, &dxl_error);
  dxl_comm_result = packetHandler_->reboot(portHandler_, right_rear_wheel_id_, &dxl_error);
  dxl_comm_result = packetHandler_->reboot(portHandler_, right_front_wheel_id_, &dxl_error);

  //init();
  
  return true;
}

bool AresMotorDriver::controlMotor(int64_t left_rear_wheel_value, int64_t right_rear_wheel_value, int64_t left_front_wheel_value, int64_t right_front_wheel_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(left_rear_wheel_id_, (uint8_t*)&left_rear_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(right_rear_wheel_id_, (uint8_t*)&right_rear_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(left_front_wheel_id_, (uint8_t*)&left_front_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(right_front_wheel_id_, (uint8_t*)&right_front_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}
