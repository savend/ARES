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

#ifndef ARES_H
#define ARES_H
#define NOETIC_SUPPORT

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ares_msgs/SensorState.h"
#include "ares_msgs/Sound.h"
#include "ares_msgs/VersionInfo.h"


#include <math.h>

//***********ARES CONFIG******************

#include <RC100.h>

#include "ares_motor_driver.h"
#include "ares_sensor.h"
#include "ares_controller.h"
#include "ares_diagnosis.h"


#define WHEEL_RADIUS                    0.075     // meter
#define WHEEL_SEPARATION                0.16      // meter (BURGER => 0.16, WAFFLE => 0.287)
#define TURNING_RADIUS                  0.080     //****** // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                    0.105     //****** // meter (BURGER : 0.105, WAFFLE : 0.220)
// #define ROBOT_LENGTH                    0.165     // meter

#define WHEEL_POS_FROM_CENTER_X_1       -0.100    // meter
#define WHEEL_POS_FROM_CENTER_Y_1       -0.128    // meter
#define WHEEL_POS_FROM_CENTER_X_2       0.100     // meter
#define WHEEL_POS_FROM_CENTER_Y_2       -0.128    // meter
#define WHEEL_POS_FROM_CENTER_X_3       -0.100    // meter
#define WHEEL_POS_FROM_CENTER_Y_3       0.128     // meter
#define WHEEL_POS_FROM_CENTER_X_4       0.100     // meter
#define WHEEL_POS_FROM_CENTER_Y_4       0.128     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define VELOCITY_CONSTANT_VALUE         1263.632956882  // V = r * w = r * RPM * 0.10472
                                                        //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                        // Goal RPM = V * 1263.632956882

#define CONTROL_PERIOD                  8000

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
//#define MAX_LINEAR_VELOCITY             (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
//#define MAX_ANGULAR_VELOCITY            (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s
#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI



// Function prototypes
//void receiveRemoteControlData(void);
//void controlMotorSpeed(void);


//***********CORE CONFIG******************

#define FIRMWARE_VER "1.0.0"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        4

#define LEFT_REAR                        0
#define RIGHT_REAR                       1
#define LEFT_FRONT                       2
#define RIGHT_FRONT                      3

#define LINEAR                           0
#define ANGULAR                          1

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

// #define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void soundCallback(const ares_msgs::Sound& sound_msg);
void motorPowerCallback(const std_msgs::Bool& power_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishCmdVelFromRC100Msg(void);
void publishImuMsg(void);
void publishMagMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);

void controlAres(void);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

ros::Subscriber<ares_msgs::Sound> sound_sub("sound", soundCallback);

ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);

ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Ares
ares_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Ares
ares_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Ares
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Ares using RC100 remote controller
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

// Odometry of Ares
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Ares
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Ares
#if defined NOETIC_SUPPORT
sensor_msgs::BatteryStateNoetic battery_state_msg;
#else
sensor_msgs::BatteryState battery_state_msg;
#endif
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Ares
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Ares
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
AresMotorDriver motor_driver;

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0, 0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0, 0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0, 0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
AresSensor sensors;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
AresController controllers;
float zero_velocity[2] = {0.0, 0.0};
float goal_velocity[2] = {0.0, 0.0};
float goal_velocity_from_button[2] = {0.0, 0.0};
float goal_velocity_from_cmd[2] = {0.0, 0.0};
float goal_velocity_from_rc100[2] = {0.0, 0.0};

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
AresDiagnosis diagnosis;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

#endif // ARES_H_
