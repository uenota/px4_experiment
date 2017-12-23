/**
* @file 2d_movement_test.cpp
* @brief Move 1m forward, backward, left and right
* @author Takaki Ueno
*/

// C/C++ libraries
#include <cmath>
#include <string>

// roscpp
#include <ros/ros.h>
#include <ros/topic.h>

// laser assembler
//#include <laser_assembler/AssembleScans2.h>

// geometry msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// sensor msgs
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

// mavros msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>

//! Storage for vehicle state
mavros_msgs::State current_state;

/**
* @brief Callback function for state subscriber
* @param msg Incoming message
*/
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

//! Storage for local position
geometry_msgs::PoseStamped local_pos;

/**
* @brief Callback function for local position subscriber
* @param msg Incoming message
*/
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  local_pos = *msg;
}

//! Storage for current latitude
double curr_latitude;
//! Storage for current longitude
double curr_longitude;
//! Storage for current altitude
double curr_altitude;
/**
* @brief Callback function for global position subscriber
* @param msg Incoming message
*/
void curr_gpos_cb(const sensor_msgs::NavSatFix::ConstPtr& msgptr)
{
  sensor_msgs::NavSatFix msg = *msgptr;
  curr_latitude = msg.latitude;
  curr_longitude = msg.longitude;
  curr_altitude = msg.altitude;
}

//! Storage for current velocity of vehicle
geometry_msgs::TwistStamped fcu_vel;
/**
* @brief Callback function for velocity position subscriber
* @param msg Incoming message
*/
void fcu_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  fcu_vel = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_sweep_demo");
  ros::NodeHandle nh;

  // Subscriber
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100, local_pos_cb);
  ros::Subscriber curr_gpos_sub =
      nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, curr_gpos_cb);
  ros::Subscriber fcu_vel_sub =
      nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 100, fcu_vel_cb);

  // Publisher
  ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

  // Service Client
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_hp_client = nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");
  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
//  ros::ServiceClient pc_gen_client = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

  // wait for laser assembler
  //ros::service::waitForService("assemble_scans");
  //laser_assembler::AssembleScans2 pc_gen_cmd;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while (ros::ok() and current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // wait for message arriving from /mavros/global_position/global
  ROS_INFO("Waiting for message from /mavros/global_position/global");
  const std::string gpos_topic = "mavros/global_position/global";
  sensor_msgs::NavSatFix init_gpos = *ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpos_topic);
  double init_latitude = init_gpos.latitude;
  double init_longitude = init_gpos.longitude;
  double init_altitude = init_gpos.altitude;
  ROS_INFO("Initial Lat: %f  Lon: %f  Alt: %f", init_latitude, init_longitude, init_altitude);

  // set home position as current position
  mavros_msgs::CommandHome set_hp_cmd;
  set_hp_cmd.request.current_gps = true;
  while (not(set_hp_client.call(set_hp_cmd)) and set_hp_cmd.response.success)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("HP set.");

  // set mode as offboard
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  while (not(set_mode_client.call(offb_set_mode)))
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Offboard enabled.");

  // arm vehicle
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  while (not(arming_client.call(arm_cmd)))
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Vehicle armed.");

  // takeoff
  double climb_height = 1.5;
  mavros_msgs::CommandTOL takeoff_cmd;
  takeoff_cmd.request.altitude = init_altitude + climb_height;
  takeoff_cmd.request.longitude = init_longitude;
  takeoff_cmd.request.latitude = init_latitude;

  ROS_DEBUG("set lat: %f, lon: %f, alt: %f", init_latitude, init_longitude, init_altitude);

  while (not(takeoff_client.call(takeoff_cmd)) and takeoff_cmd.response.success)
  {
    ros::spinOnce();
    rate.sleep();
  }
  while (local_pos.pose.position.z < climb_height - 0.1)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_DEBUG("Current lat: %f, lon: %f, alt: %f", curr_latitude, curr_longitude, curr_altitude);
  }
  ROS_INFO("Vehicle tookoff.");

  // pc_gen_cmd.request.begin = ros::Time::now();

  // start moving
  geometry_msgs::TwistStamped target_vel_msg;
  // 0.1 meter per second
  target_vel_msg.twist.linear.x = 0.1;

  ROS_INFO("Vehicle moving forward...");

  ros::Time movement_start = ros::Time::now();
  // Move forward until 10 seconds elapse
  // 10 [s] * 0.1 [m/s] = 1.0 [m]
  while (ros::Time::now() - movement_start < ros::Duration(10.0))
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
    // when offboard disabled, set back the mode to offboard
    if (current_state.mode != "OFFBOARD")
    {
      while (not(set_mode_client.call(offb_set_mode)))
      {
        ros::spinOnce();
        rate.sleep();
      }
      ROS_INFO("Offboard enabled.");
    }
  }

  // make drone keep its current position
  target_vel_msg.twist.linear.x = 0.0;
  for (int i = 0; i < 20; ++i)
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
  }

  // -0.1 meter per second
  target_vel_msg.twist.linear.x = -0.1;

  ROS_INFO("Vehicle moving backward...");

  movement_start = ros::Time::now();
  // Move forward until 10 seconds elapse
  // 10 [s] * -0.1 [m/s] = -1.0 [m]
  while (ros::Time::now() - movement_start < ros::Duration(10.0))
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
    // when offboard disabled, set back the mode to offboard
    if (current_state.mode != "OFFBOARD")
    {
      while (not(set_mode_client.call(offb_set_mode)))
      {
        ros::spinOnce();
        rate.sleep();
      }
      ROS_INFO("Offboard enabled.");
    }
  }

  // make drone keep its current position
  target_vel_msg.twist.linear.x = 0.0;
  for (int i = 0; i < 20; ++i)
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
  }

  // 0.1 meter per second
  target_vel_msg.twist.linear.y = 0.1;

  ROS_INFO("Vehicle moving left...");

  movement_start = ros::Time::now();
  // Move forward until 10 seconds elapse
  // 10 [s] * 0.1 [m/s] = 1.0 [m]
  while (ros::Time::now() - movement_start < ros::Duration(10.0))
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
    // when offboard disabled, set back the mode to offboard
    if (current_state.mode != "OFFBOARD")
    {
      while (not(set_mode_client.call(offb_set_mode)))
      {
        ros::spinOnce();
        rate.sleep();
      }
      ROS_INFO("Offboard enabled.");
    }
  }

  // make drone keep its current position
  target_vel_msg.twist.linear.y = 0.0;
  for (int i = 0; i < 20; ++i)
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
  }

  // -0.1 meter per second
  target_vel_msg.twist.linear.y = -0.1;

  ROS_INFO("Vehicle moving right...");

  movement_start = ros::Time::now();
  // Move forward until 10 seconds elapse
  // 10 [s] * -0.1 [m/s] = -1.0 [m]
  while (ros::Time::now() - movement_start < ros::Duration(10.0))
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
    // when offboard disabled, set back the mode to offboard
    if (current_state.mode != "OFFBOARD")
    {
      while (not(set_mode_client.call(offb_set_mode)))
      {
        ros::spinOnce();
        rate.sleep();
      }
      ROS_INFO("Offboard enabled.");
    }
  }

  // make drone keep its current position
  target_vel_msg.twist.linear.y = 0.0;
  for (int i = 0; i < 20; ++i)
  {
    cmd_vel_pub.publish(target_vel_msg);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Vehicle arrived destination.");

  // Specify the end of recording pointcloud
  //pc_gen_cmd.request.end = ros::Time::now();
  //ROS_INFO("End buffering laser scan data.");

  // land
  mavros_msgs::CommandTOL landing_cmd;
  landing_cmd.request.altitude = curr_altitude - climb_height + 0.3;
  landing_cmd.request.longitude = curr_longitude;
  landing_cmd.request.latitude = curr_latitude;

  // send landing request
  while (not(landing_client.call(landing_cmd)) and landing_cmd.response.success)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Vehicle landing...");

  // landing
  while (local_pos.pose.position.z > 0.1)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Vehicle landed.");

  // disram
  arm_cmd.request.value = false;
  while (!arming_client.call(arm_cmd) && arm_cmd.response.success)
  {
  }
  ROS_INFO("Vehicle disarmed");

/*
  // generate pointcloud
  if (pc_gen_client.call(pc_gen_cmd))
  {
    ROS_INFO("Got cloud with %lu points.", pc_gen_cmd.response.cloud.data.size());
    for (int i = 0; i < 10; ++i)
    {
      point_cloud_pub.publish(pc_gen_cmd.response.cloud);
      ros::spinOnce();
      rate.sleep();
    }
  }
  else
  {
    ROS_INFO("Service \"assemble_scan\" call failed.");
  }
*/
  return 0;
}
