/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */
/* Modified for ARES Robot: Audric Strumpler, Gabin Pratx */

#include "ares.h"


/*******************************************************************************
  Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(9600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(sound_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);
  nh.subscribe(headlights_status_sub);
  nh.subscribe(ventilator_status_sub);
  nh.subscribe(reinitialize_motors_status_sub);

  nh.advertise(sensor_state_pub);
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);
  nh.advertise(ambient_temp_pub);
  nh.advertise(object_temp_pub);
  nh.advertise(o2_concentration_pub);
  nh.advertise(environment_temp_pub);
  nh.advertise(environment_humidity_pub);
  nh.advertise(environment_pressure_pub);
  nh.advertise(emergency_state_pub);

  tf_broadcaster.init(nh);

  motor_driver.init();

  // Setting for IMU
  sensors.init();

  //setup for the sensors
  ir_temp_sensor.begin();

  char log_msg[100];

  if (!o2_sensor.begin(O2_SENSOR_I2C_ADDRESS))
  {
    sprintf(log_msg, "O2-sensor I2C connection failled !");
    nh.loginfo(log_msg);
  }

  // Digital PinMode declaration on the OpenCR
  pinMode(RELAIS_PIN_HEADLIGHTS, OUTPUT);
  digitalWrite(RELAIS_PIN_HEADLIGHTS, HIGH);

  pinMode(RELAIS_PIN_VENTILATOR, OUTPUT);
  digitalWrite(RELAIS_PIN_VENTILATOR, LOW);

  pinMode(BATTERY_LED_PIN, OUTPUT);
  digitalWrite(BATTERY_LED_PIN, HIGH);

  pinMode(EMERGENCY_SWITCH_INTERRUPT_PIN, INPUT_PULLDOWN);
  //attachInterrupt(2, emergencyCallback, CHANGE);   // when emergency button is pressed (voltage is falling on the Pin) or released (voltage rising back )

  // Init diagnosis
  diagnosis.init();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

  pinMode(LED_WORKING_CHECK, OUTPUT);

  setup_end = true;
}


/*******************************************************************************
  Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    if ((t - tTime[6]) > CONTROL_MOTOR_TIMEOUT)
    {

      goal_velocity[0] = zero_velocity[0];
      goal_velocity[1] = zero_velocity[1];
      controlAres();
    }
    else {



      controlAres();
    }
    tTime[0] = t;
  }

  if ((t - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
  {
    publishCmdVelFromRC100Msg();
    tTime[1] = t;
  }

  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg();
    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t - tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }

  if ((t - tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishVersionInfoMsg();
    publishO2Mesurement();
    publishIRtempMesurement();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t - tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  // Send log message after ROS connection
  sendLogMsg();

  // Receive data from RC100
  bool clicked_state = controllers.getRCdata(goal_velocity_from_rc100);
  if (clicked_state == true)
    tTime[6] = millis();

  // Check push button pressed for simple test drive
  driveTest(diagnosis.getButtonPress(3000));

  // Update the IMU unit
  sensors.updateIMU();

  // TODO
  // Update sonar data
  // sensors.updateSonar(t);

  int motor_error_1 = 0;
  int motor_error_2 = 0;
  int motor_error_3 = 0;
  int motor_error_4 = 0;
  char log_msg[50];


  motor_driver.readError(motor_error_1, motor_error_2, motor_error_3, motor_error_4);
  if (motor_error_1 != 0 || motor_error_2 != 0 || motor_error_3 != 0 || motor_error_4 != 0)
  {
    sprintf(log_msg,  "motor 1 : %d, motor 2 : %d, motor 3 : %d, motor 4 : %d", motor_error_1, motor_error_2, motor_error_3, motor_error_4);
    //char log_msg[50];
    //sprintf(log_msg, "Goal velocity to control Ares");
    nh.loginfo(log_msg);
  }

  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());

  // Show LED status
  diagnosis.showLedStatus(nh.connected());

  // Update Voltage
  battery_state = diagnosis.updateVoltageCheck(setup_end);

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());

  publishEmergencyState();

}

/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

/*******************************************************************************
  Callback function for sound msg
*******************************************************************************/
void soundCallback(const ares_msgs::Sound& sound_msg)
{
  sensors.makeSound(sound_msg.value);
}

/*******************************************************************************
  Callback function for motor_power msg
*******************************************************************************/
void motorPowerCallback(const std_msgs::Bool& power_msg)
{
  bool dxl_power = power_msg.data;

  //motor_driver.setTorque(dxl_power);
}

/*******************************************************************************
  Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);
}

/*******************************************************************************
  Callback function for headlights
  ATTENTION inverted logic: HIGH means LOW   and   LOW means HIGH
*******************************************************************************/
void headlightsCallback(const std_msgs::Bool& headlights_status_msg)
{
  bool headlights_status = headlights_status_msg.data;

  if (headlights_status == true)
  {
    digitalWrite(RELAIS_PIN_HEADLIGHTS, LOW);
  }
  else digitalWrite(RELAIS_PIN_HEADLIGHTS, HIGH);
}

/*******************************************************************************
  Callback function for ventilator
  ATTENTION inverted logic: HIGH means LOW   and   LOW means HIGH
*******************************************************************************/
void ventilatorCallback(const std_msgs::Bool& ventilator_status_msg)
{
  bool ventilator_status = ventilator_status_msg.data;

  if (ventilator_status == true)
  {
    digitalWrite(RELAIS_PIN_VENTILATOR, HIGH);
  }
  else digitalWrite(RELAIS_PIN_VENTILATOR, LOW);
}

/*******************************************************************************
  Callback function for reinitializing the motors on command
*******************************************************************************/
void motorsReinitializationCallback(const std_msgs::Bool& reinitialize_motors_msg)
{
  motor_driver.aresReboot(); // reinitialize the motors on command
  motor_driver.init();
}


/*******************************************************************************
  Publish msgs (CMD Velocity data from RC100 : angular velocity, linear velocity)
*******************************************************************************/
void publishCmdVelFromRC100Msg(void)
{
  cmd_vel_rc100_msg.linear.x  = goal_velocity_from_rc100[LINEAR];
  cmd_vel_rc100_msg.angular.z = goal_velocity_from_rc100[ANGULAR];

  cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
}

/*******************************************************************************
  Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
  Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}

void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery = sensors.checkVoltage();

  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_rear_encoder, sensor_state_msg.right_rear_encoder, sensor_state_msg.left_front_encoder, sensor_state_msg.right_front_encoder);

  if (dxl_comm_result == true)
    updateMotorInfo(sensor_state_msg.left_rear_encoder, sensor_state_msg.right_rear_encoder, sensor_state_msg.left_front_encoder, sensor_state_msg.right_front_encoder);
  else
    return;

  sensor_state_msg.bumper = sensors.checkPushBumper();

  sensor_state_msg.cliff = sensors.getIRsensorData();

  // TODO
  // sensor_state_msg.sonar = sensors.getSonarData();

  sensor_state_msg.illumination = sensors.getIlluminationData();

  sensor_state_msg.button = sensors.checkPushButton();

  //  sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
  Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
  Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 4.0f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;

  battery_state_pub.publish(&battery_state_msg);
}

/*******************************************************************************
  Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
  Publish the IR_temperature_mesurement (ambient_temerature, object_temperature) std_msgs
*******************************************************************************/
void publishIRtempMesurement(void)
{
  amb_temp_msg.data = ir_temp_sensor.GetAmbientTemp_Celsius();
  ambient_temp_pub.publish(&amb_temp_msg);

  obj_temp_msg.data = ir_temp_sensor.GetObjectTemp_Celsius();
  object_temp_pub.publish(&obj_temp_msg);
}

/*******************************************************************************
  Publish the O2_volume_mesurement std_msgs
*******************************************************************************/
void publishO2Mesurement(void)
{
  o2_msg.data = o2_sensor.ReadOxygenData(COLLECT_NUMBER_AVG_O2);
  o2_concentration_pub.publish(&o2_msg);
}

/*******************************************************************************
  Publish the Environment_parameters_mesurement (temperature, pressure, humidity) std_msgs
*******************************************************************************/
void publishEnvParametersMesurement(void)
{
  if (!env_sensor.performReading())
  {
    char log_msg[50];
    sprintf(log_msg, "Could not perform reading environment sensor");
    nh.loginfo(log_msg);
  }
  env_temp_msg.data = env_sensor.temperature;
  environment_temp_pub.publish(&env_temp_msg);

  env_pres_msg.data = (env_sensor.pressure / 100.0);
  environment_pressure_pub.publish(&env_pres_msg);

  env_hum_msg.data = env_sensor.humidity;
  environment_humidity_pub.publish(&env_hum_msg);
}

/*******************************************************************************
  Get if emergencyButton pressed, Publish warning flag, and Reinitialize the motors
*******************************************************************************/
void publishEmergencyState (void)
{
  static long previous_millis = millis();
  static bool button_state;                                                           // tells if the button is pressed or released
  static bool previous_button_state = digitalRead(EMERGENCY_SWITCH_INTERRUPT_PIN);    // tells if the button was pressed or released
  static bool emergency_state = false;                                                // warning flag rised when emergency button was pressed

  button_state = digitalRead(EMERGENCY_SWITCH_INTERRUPT_PIN);
  
  if (button_state != previous_button_state)
  {
    if (millis() - previous_millis >= DEBOUNCE_TIME)
    {
      if (digitalRead(EMERGENCY_SWITCH_INTERRUPT_PIN) == LOW)
      {
        emergency_state = true; // warning: emergency button was pressed !
      }
      else if (digitalRead(EMERGENCY_SWITCH_INTERRUPT_PIN) == HIGH)
      {
        motor_driver.aresReboot(); // reinitialize the motors after an emergency stop
        motor_driver.init();
        emergency_state = false;
      }
      emergency_state_msg.data = emergency_state;
      emergency_state_pub.publish(&emergency_state_msg); //publishing the warning

      previous_button_state = button_state;
      previous_millis = millis();

    }
  }
}


/*******************************************************************************
  Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_link");

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "chassis");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_link");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/chassis");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg);

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
  Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
  Update the joint states
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};

  joint_states_pos[LEFT_REAR]  = last_rad[LEFT_REAR];
  joint_states_pos[RIGHT_REAR] = last_rad[RIGHT_REAR];
  joint_states_pos[LEFT_FRONT]  = last_rad[LEFT_FRONT];
  joint_states_pos[RIGHT_FRONT] = last_rad[RIGHT_FRONT];

  joint_states_vel[LEFT_REAR]  = last_velocity[LEFT_REAR];
  joint_states_vel[RIGHT_REAR] = last_velocity[RIGHT_REAR];
  joint_states_vel[LEFT_FRONT]  = last_velocity[LEFT_FRONT];
  joint_states_vel[RIGHT_FRONT] = last_velocity[RIGHT_FRONT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
  CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
  Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_rear_tick, int32_t right_rear_tick, int32_t left_front_tick, int32_t right_front_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0, 0, 0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }

    last_tick[LEFT_REAR] = left_rear_tick;
    last_tick[RIGHT_REAR] = right_rear_tick;
    last_tick[LEFT_FRONT] = left_front_tick;
    last_tick[RIGHT_FRONT] = right_front_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_rear_tick;

  last_diff_tick[LEFT_REAR] = current_tick - last_tick[LEFT_REAR];
  last_tick[LEFT_REAR]      = current_tick;
  last_rad[LEFT_REAR]       += TICK2RAD * (double)last_diff_tick[LEFT_REAR];

  current_tick = right_rear_tick;

  last_diff_tick[RIGHT_REAR] = current_tick - last_tick[RIGHT_REAR];
  last_tick[RIGHT_REAR]      = current_tick;
  last_rad[RIGHT_REAR]       += TICK2RAD * (double)last_diff_tick[RIGHT_REAR];

  current_tick = left_front_tick;

  last_diff_tick[LEFT_FRONT] = current_tick - last_tick[LEFT_FRONT];
  last_tick[LEFT_FRONT]      = current_tick;
  last_rad[LEFT_FRONT]       += TICK2RAD * (double)last_diff_tick[LEFT_FRONT];

  current_tick = right_front_tick;

  last_diff_tick[RIGHT_FRONT] = current_tick - last_tick[RIGHT_FRONT];
  last_tick[RIGHT_FRONT]      = current_tick;
  last_rad[RIGHT_FRONT]       += TICK2RAD * (double)last_diff_tick[RIGHT_FRONT];
}

/*******************************************************************************
  Calculate the odometry (TO VERIFY BECAUSE OF THE FOUR WHEELS)
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT_REAR];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT_REAR];
  //wheel_l = TICK2RAD * (double)((last_diff_tick[LEFT_REAR]+last_diff_tick[LEFT_FRONT])/2); //average of the two motors
  //wheel_r = TICK2RAD * (double)l((last_diff_tick[RIGHT_REAR]+last_diff_tick[RIGHT_FRONT])/2); //average of the two motors

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                       0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT_REAR]  = wheel_l / step_time;
  last_velocity[RIGHT_REAR] = wheel_r / step_time;
  last_velocity[LEFT_FRONT]  = wheel_l / step_time; //TO VERIFY
  last_velocity[RIGHT_FRONT] = wheel_r / step_time; //TO VERIFY
  last_theta = theta;

  return true;
}

/*******************************************************************************
  Ares test drive using push buttons
*******************************************************************************/
void driveTest(uint8_t buttons)
{


  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[4] = {0, 0, 0, 0};

  motor_driver.readEncoder(current_tick[LEFT_REAR], current_tick[RIGHT_REAR], current_tick[LEFT_FRONT], current_tick[RIGHT_FRONT]);

  if (buttons & (1 << 0))
  {
    move[LINEAR] = true;
    saved_tick[RIGHT_REAR] = current_tick[RIGHT_REAR];


    diff_encoder = TEST_DISTANCE / ((WHEEL_RADIUS * 2 * 3.141592) / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
    tTime[6] = millis();
  }
  else if (buttons & (1 << 1))
  {
    move[ANGULAR] = true;
    saved_tick[RIGHT_REAR] = current_tick[RIGHT_REAR];

    diff_encoder = (TEST_RADIAN * TURNING_RADIUS) / ((WHEEL_RADIUS * 2 * 3.141592) / 4096);
    tTime[6] = millis();
  }

  if (move[LINEAR])
  {
    if (abs(saved_tick[RIGHT_REAR] - current_tick[RIGHT_REAR]) <= diff_encoder)
    {
      goal_velocity_from_button[LINEAR]  = 0.05;
      tTime[6] = millis();
    }
    else
    {
      goal_velocity_from_button[LINEAR]  = 0.0;
      move[LINEAR] = false;
    }
  }
  else if (move[ANGULAR])
  {
    if (abs(saved_tick[RIGHT_REAR] - current_tick[RIGHT_REAR]) <= diff_encoder)
    {
      goal_velocity_from_button[ANGULAR] = -0.7;
      tTime[6] = millis();
    }
    else
    {
      goal_velocity_from_button[ANGULAR]  = 0.0;
      move[ANGULAR] = false;
    }
  }
}

/*******************************************************************************
  Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;

  if (isConnected)
  {
    if (variable_flag == false)
    {
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
  Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;

  if (isConnected)
  {
    if (wait_flag == false)
    {
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
  Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
  Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
  Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];

  String name             = "ARES";
  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with " + name;

  const char* init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

/*******************************************************************************
  Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
  Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"left_rear_wheel_joint", (char*)"right_rear_wheel_joint", (char*)"left_front_wheel_joint", (char*)"right_front_wheel_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

/*******************************************************************************
  Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

  sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);

}

/*******************************************************************************
  Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("EXTERNAL SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("OpenCR SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  float* quat = sensors.getOrientation();

  DEBUG_SERIAL.println("IMU : ");
  DEBUG_SERIAL.print("    w : "); DEBUG_SERIAL.println(quat[0]);
  DEBUG_SERIAL.print("    x : "); DEBUG_SERIAL.println(quat[1]);
  DEBUG_SERIAL.print("    y : "); DEBUG_SERIAL.println(quat[2]);
  DEBUG_SERIAL.print("    z : "); DEBUG_SERIAL.println(quat[3]);

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("DYNAMIXELS");
  DEBUG_SERIAL.println("---------------------------------------");
  //  DEBUG_SERIAL.println("Torque : " + String(motor_driver.getTorque()));

  int32_t encoder[WHEEL_NUM] = {0, 0, 0, 0};
  motor_driver.readEncoder(encoder[LEFT_REAR], encoder[RIGHT_REAR], encoder[LEFT_FRONT], encoder[RIGHT_FRONT]);

  DEBUG_SERIAL.println("Encoder(left_rear) : " + String(encoder[LEFT_REAR]));
  DEBUG_SERIAL.println("Encoder(right_rear) : " + String(encoder[RIGHT_REAR]));
  DEBUG_SERIAL.println("Encoder(left_front) : " + String(encoder[LEFT_FRONT]));
  DEBUG_SERIAL.println("Encoder(right_front) : " + String(encoder[RIGHT_FRONT]));

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Ares");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Odometry : ");
  DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}

/*******************************************************************************
  Control bike speed
*******************************************************************************/
void controlAres(void)
{



  bool dxl_comm_result = false;

  double wheel1_spd_cmd, wheel2_spd_cmd, wheel3_spd_cmd, wheel4_spd_cmd;
  double lin_vel1, lin_vel2, lin_vel3, lin_vel4;


  wheel1_spd_cmd = goal_velocity[LINEAR] - (sqrt(WHEEL_POS_FROM_CENTER_X_1 * WHEEL_POS_FROM_CENTER_X_1 + WHEEL_POS_FROM_CENTER_Y_1 * WHEEL_POS_FROM_CENTER_Y_1) * goal_velocity[ANGULAR]) * cos(atan(WHEEL_POS_FROM_CENTER_Y_1 / WHEEL_POS_FROM_CENTER_X_1));

  wheel2_spd_cmd = goal_velocity[LINEAR] + (sqrt(WHEEL_POS_FROM_CENTER_X_2 * WHEEL_POS_FROM_CENTER_X_2 + WHEEL_POS_FROM_CENTER_Y_2 * WHEEL_POS_FROM_CENTER_Y_2) * goal_velocity[ANGULAR]) * cos(atan(WHEEL_POS_FROM_CENTER_Y_2 / WHEEL_POS_FROM_CENTER_X_2));

  wheel3_spd_cmd = goal_velocity[LINEAR] - (sqrt(WHEEL_POS_FROM_CENTER_X_3 * WHEEL_POS_FROM_CENTER_X_3 + WHEEL_POS_FROM_CENTER_Y_3 * WHEEL_POS_FROM_CENTER_Y_3) * goal_velocity[ANGULAR]) * cos(atan(WHEEL_POS_FROM_CENTER_Y_3 / WHEEL_POS_FROM_CENTER_X_3));

  wheel4_spd_cmd = goal_velocity[LINEAR] + (sqrt(WHEEL_POS_FROM_CENTER_X_4 * WHEEL_POS_FROM_CENTER_X_4 + WHEEL_POS_FROM_CENTER_Y_4 * WHEEL_POS_FROM_CENTER_Y_4) * goal_velocity[ANGULAR]) * cos(atan(WHEEL_POS_FROM_CENTER_Y_4 / WHEEL_POS_FROM_CENTER_X_4));

  lin_vel1 = wheel1_spd_cmd * VELOCITY_CONSTANT_VALUE;
  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel2 = -1 * wheel2_spd_cmd * VELOCITY_CONSTANT_VALUE;
  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel3 = wheel3_spd_cmd * VELOCITY_CONSTANT_VALUE;
  if (lin_vel3 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel3 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel3 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel3 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel4 = -1 * wheel4_spd_cmd * VELOCITY_CONSTANT_VALUE;
  if (lin_vel4 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel4 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel4 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel4 = -LIMIT_X_MAX_VELOCITY;
  }


  dxl_comm_result = motor_driver.controlMotor((int64_t)lin_vel1, (int64_t)lin_vel2, (int64_t)lin_vel3, (int64_t)lin_vel4);
  if (dxl_comm_result == false)
    return;


}
