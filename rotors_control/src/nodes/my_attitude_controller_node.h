/*******************************************************************************
* @Copylift (c) 2021, Suvian, Inc.
* @
* @ pluginTemplate : [description]
* @
* @ filename : my_attitude_controller_node.h
* @ author   : Suvian(suvian@qq.com)
* @ create   : 2021/04/22 	 10:04:47
/******************************************************************************/

#ifndef __MY_ATTITUDE_CONTROLLER_NODE_H__
#define __MY_ATTITUDE_CONTROLLER_NODE_H__

////////////////////////////////////////////////////////////////////////////////
// Headers
//
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/crazyflie_complementary_filter.h"
#include "rotors_control/my_attitude_controller.h"
////////////////////////////////////////////////////////////////////////////////
// Typedefs & Constants
//

////////////////////////////////////////////////////////////////////////////////
// Classes
//
namespace rotors_control {

class AttitudeControllerNode {
public:
    AttitudeControllerNode();
    ~AttitudeControllerNode();

    void InitializeParams();
    void Publish();

private:
    bool waypointHasBeenPublished_  = false;
    bool enable_state_estimator_    = false;
    bool enable_minifly_controller_ = false;

    AttitudeController attitude_controller_;
    sensorData_t       sensors_;
    ros::Time          imu_msg_head_stamp_;

    std::string namespace_;

    ros::NodeHandle n_;
    ros::Timer      timer_Attitude_;
    ros::Timer      timer_highLevelControl;
    ros::Timer      timer_IMUUpdate;

    // Callback functions to compute the errors among axis and angles
    void CallbackAttitudeEstimation(const ros::TimerEvent& event);
    void CallbackHightLevelControl(const ros::TimerEvent& event);
    void CallbackIMUUpdate(const ros::TimerEvent& event);

    // subscribers
    ros::Subscriber cmd_get_target_attitude_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber imu_sub_;

    // publisher
    ros::Publisher motor_velocity_reference_pub_;

    // mav_msgs::EigenTrajectoryPointDeque commands_;
    // std::deque<ros::Duration>           command_waiting_times_;
    // ros::Timer                          command_timer_;

    void TargetAttitudeCallback(const mav_msgs::RollPitchYawrateThrust& msg);

    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

    void IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg);
};
}
////////////////////////////////////////////////////////////////////////////////
// Functions
//

#endif //__MY_ATTITUDE_CONTROLLER_NODE_H__
       ////////////////////////////////// EOF /////////////////////////////////////////