// System includes
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// ROS includes
#include <custom_msgs/EulerMotion.h>
#include <custom_msgs/Lockout.h>
#include <std_msgs/Float32.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/ResetMotors.h>
#include <ros/ros.h>

#include "control-loop-main.hpp"
#include "pid.hpp"

typedef custom_msgs::EulerMotion VecWithCurl;

ros::Publisher output_vec_pub;

VecWithCurl latest_controller, latest_position;
float latest_depth;
VecWithCurl plant_output;
bool lockout_flag;

// Flag raised when input data is ready
bool data_ready_flag = false;
bool controller_data_ready_flag = false;
bool accel_data_ready_flag = false;
bool depth_data_ready_flag = false;

void controllerRosCallback(const custom_msgs::EulerMotion::ConstPtr &controller_request)
{
  // Store data locally and raise the flag
  latest_controller = *controller_request;
  controller_data_ready_flag = true;
}

void accelGyroRosCallback(const custom_msgs::EulerMotion::ConstPtr &position)
{
  // Store data locally and raise the flag
  latest_position = *position;
  accel_data_ready_flag = true;
}

void depthRosCallback(const std_msgs::Float32::ConstPtr &depth)
{
  latest_depth = depth->data;
  depth_data_ready_flag = true;
}

bool lockoutCallback(custom_msgs::Lockout::Request &req, custom_msgs::Lockout::Response &res)
{
  lockout_flag = req.lockout;
  ROS_INFO("Lockout updated: %s\n", lockout_flag ? "locked" : "unlocked");
  data_ready_flag = true;
  return true;
}

double desired_z, desired_r, desired_p, desired_w;
double prev_w, current_w;
ros::Time prev_time;

pid pid_z = pid(8, 1, 2), pid_pitch(6, 1, 1), pid_roll(-1, -0.1, 0.1), pid_yaw(2, 0, 2);
// pid pid_z = pid(1, -0.1, 0.1), pid_pitch(1, 0.1, 0.1), pid_roll(GG, 0, 0), pid_yaw(1, 0.1, 0.1);

bool first_run = true;

// DC blocking filter coefficient of 0.9 was chosen by trial and error
// DcBlockingFilter accel_z_filter = DcBlockingFilter(0.9);

void loop()
{
  // When any new data is ready, this will be called

  if (lockout_flag)
  {
    // Handle lockout if needed
  }

  if (desired_w == -999)
  {
    if (latest_position.yaw != 0)
    {
      // Set the yaw to the current value so the robot doesn't try to pivot itself immediately
      desired_w = latest_position.yaw;
    }
    else
    {
      // Do nothing unless yaw data is ready
      ROS_INFO("Wating for init data from accelerometer");
      return;
    }
  }

  // Determine the time since last run
  ros::Time time = ros::Time::now();
  double dt_s = (time - prev_time).toSec();
  prev_time = time;

  // This performs an integral
  desired_z += latest_controller.z * dt_s;
  desired_r = latest_controller.roll;
  desired_p = latest_controller.pitch;
  desired_w += latest_controller.yaw * dt_s;

  double w_diff = latest_position.yaw - prev_w;
    // Change in yaw was greater than pi, so we must have overflowed and jumped
    // from pi to -pi or vice versa
  if (std::abs(w_diff) > M_PI) {
    // Note that pi was chosen in the above line because it's half the maximum
    // change, so pi is the maximum distance from a small change (eg. 0.1) and 
    // an overflow change (eg. 2pi - 0.1). Techinally other values like 1 thru 
    // 6 would work, but they'd be slightly worse.

    // Correct by removing the overflow component
    // This pi does matter since it's the actual overflow ammount
    if (w_diff > 0) w_diff -= 2 * M_PI; 
    else w_diff += 2 * M_PI;
  }

  current_w += w_diff;
  prev_w = latest_position.yaw;

  double z_cmd = 0, pitch_cmd = 0, roll_cmd = 0, yaw_cmd = latest_controller.yaw;

  z_cmd = pid_z.tick(latest_depth, desired_z, dt_s, false);
  // z_cmd = desired_z;

  pitch_cmd = pid_pitch.tick(latest_position.pitch, desired_p, dt_s, false);
  roll_cmd = pid_roll.tick(latest_position.roll, desired_r, dt_s, false);

  yaw_cmd = pid_yaw.tick(current_w, desired_w, dt_s, false);

  // ROS_INFO("Current Z: %6.6f desired Z:  %6.6f, Z cmd: %6.6f", latest_depth, desired_z, z_cmd);
  // ROS_INFO("Current R: %6.6f desired R:  %6.6f, R cmd: %6.6f\n", latest_position.pitch, desired_p, pitch_cmd);

  plant_output.z = z_cmd;
  plant_output.roll = roll_cmd;
  plant_output.pitch = pitch_cmd;
  plant_output.yaw = yaw_cmd;
  plant_output.x = latest_controller.x;
  plant_output.y = latest_controller.y;


  // Report the latest to the console
  printf("\033[3A"); // Move up 4 lines
  printf("DESIRED: X: %6.3f Y: %6.3f Z: %6.3f; R: %6.3f P: %6.3f W: %6.3f\n",
         0.0, 0.0, desired_z, desired_r, desired_p, desired_w);
  printf("ACTUAL:  X: %6.3f Y: %6.3f Z: %6.3f; R: %6.3f P: %6.3f W: %6.3f\n",
         latest_position.x, latest_position.y, latest_depth, latest_position.roll, latest_position.pitch, current_w);
  printf("COMMAND: X: %6.3f Y: %6.3f Z: %6.3f; R: %6.3f P: %6.3f W: %6.3f\n",
         plant_output.x, plant_output.y, plant_output.z, plant_output.roll, plant_output.pitch, plant_output.yaw);
}

void init()
{
  // TODO: PID initialization goes here
  desired_z = 0;
  desired_r = 0;
  desired_p = 0;
  desired_w = -999;

  prev_w = 0;

  latest_depth = 0;

  latest_position.x = 0;
  latest_position.y = 0;
  latest_position.x = 0;
  latest_position.roll = 0;
  latest_position.pitch = 0;
  latest_position.yaw = 0;

  prev_time = ros::Time::now();

  printf("\n\n\n");
}

int main(int argc, char *argv[])
{
  ROS_INFO("Control Loop beginning");

  ros::init(argc, argv, "control_loop");

  // This object must live as long as the program is running
  ros::NodeHandle node;

  // These handle communications to other nodes (the input/output of the program)
  ros::Subscriber joy_sub = node.subscribe<custom_msgs::EulerMotion>("robot_motion/controller_input", 1, &controllerRosCallback);
  ros::Subscriber mpu_sub = node.subscribe<custom_msgs::EulerMotion>("hardware/motion", 1, &accelGyroRosCallback);
  ros::Subscriber depth_sub = node.subscribe<std_msgs::Float32>("hardware/depth", 1, &depthRosCallback);
  ros::Publisher output_vector_pub = node.advertise<custom_msgs::EulerMotion>("robot_motion/control_loop_output", 1);
  ros::ServiceServer lockout_service = node.advertiseService("lockout", &lockoutCallback);

  // Run one-time initialization
  init();

  ROS_DEBUG("Control loop initialized, entering  runtime.\n");

  while (ros::ok())
  {
    // Look for new data on the subscribers
    ros::spinOnce();

    if (controller_data_ready_flag && accel_data_ready_flag && depth_data_ready_flag)
    {
      loop();
      output_vector_pub.publish(plant_output);
      controller_data_ready_flag = 0;
      accel_data_ready_flag = 0;
      depth_data_ready_flag = 0;
    }
  }
  return 0;
}
