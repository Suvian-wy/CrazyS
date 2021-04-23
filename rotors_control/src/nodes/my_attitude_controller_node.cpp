/*******************************************************************************
* @Copylift (c) 2021, Suvian, Inc.
* @
* @ pluginTemplate : [description]
* @
* @ filename : my_attitude_controller_node.cpp
* @ author   : Suvian(suvian@qq.com)
* @ create   : 2021/04/22 	 CURRENT_HOUR:CURRENT_MINUTE:CURRENT_SECOND
/******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
// Headers
//
#include "my_attitude_controller_node.h"

#include <chrono>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <time.h>

#include "rotors_control/crazyflie_complementary_filter.h"
#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
////////////////////////////////////////////////////////////////////////////////
// Inner Scope Objects
//
#define ATTITUDE_UPDATE_DT 0.004 /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002     /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME 0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

////////////////////////////////////////////////////////////////////////////////
// Functions
//
namespace rotors_control {
AttitudeControllerNode::AttitudeControllerNode()
{

    ROS_INFO_ONCE("Started attitude controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    // This command diplays which controller is enabled
    if (pnh_node.getParam("enable_minifly_controller", enable_minifly_controller_)) {
        ROS_INFO("Got param 'enable_minifly_controller': %d", enable_minifly_controller_);
    }

    InitializeParams();

    // Topics subscribe
    if (enable_minifly_controller_) {
        cmd_get_target_attitude_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
            &AttitudeControllerNode::TargetAttitudeCallback, this);

        odometry_sub_
            = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &AttitudeControllerNode::OdometryCallback, this);
    }

    // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // The subscription to the IMU topic is made only if it is available, when simulating the Crazyflie dynamics
    // considering the also IMU values
    if (pnh_node.getParam("enable_state_estimator", enable_state_estimator_)) {
        ROS_INFO("Got param 'enable_state_estimator': %d", enable_state_estimator_);
    }

    if (enable_state_estimator_) {
        imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1, &AttitudeControllerNode::IMUCallback, this);

        // Timers allow to set up the working frequency of the control system
        timer_Attitude_ = n_.createTimer(
            ros::Duration(ATTITUDE_UPDATE_DT), &AttitudeControllerNode::CallbackAttitudeEstimation, this, false, true);

        timer_highLevelControl = n_.createTimer(
            ros::Duration(SAMPLING_TIME), &AttitudeControllerNode::CallbackHightLevelControl, this, false, true);

        timer_IMUUpdate = n_.createTimer(
            ros::Duration(RATE_UPDATE_DT), &AttitudeControllerNode::CallbackIMUUpdate, this, false, true);
    }
}

AttitudeControllerNode::~AttitudeControllerNode() { }

void AttitudeControllerNode::TargetAttitudeCallback(const mav_msgs::EigenRollPitchYawrateThrust& msg)
{
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    const size_t n_commands = msg->points.size();

    if (n_commands < 1) {
        ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
        return;
    }

    mav_msgs::EigenRollPitchYawrateThrust eigen_reference;
    mav_msgs::eigenRollPitchYawrateThrustFromMsg(msg->points.front(), &eigen_reference);
    commands_.push_front(eigen_reference);

    // We can trigger the first command immediately.
    attitude_controller_.SetTargetAttitude(eigen_reference);
    commands_.pop_front();

    if (n_commands >= 1) {
        waypointHasBeenPublished_ = true;
        ROS_INFO("AttitudeController got first MultiDOFJointTrajectory message.");
    }
}

void AttitudeControllerNode::InitializeParams()
{
    ros::NodeHandle pnh("~");
    if (enable_minifly_controller_) {
        // TODO:GET PARAMETERS AND SET THEM

        GetRosParameter(pnh, "angle_gain_kp/pitch", PidGain_AnglePitch_.kp, &PidGain_AnglePitch_.kp);
        GetRosParameter(pnh, "angle_gain_ki/pitch", PidGain_AnglePitch_.ki, &PidGain_AnglePitch_.ki);
        GetRosParameter(pnh, "angle_gain_kd/pitch", PidGain_AnglePitch_.kd, &PidGain_AnglePitch_.kd);

        GetRosParameter(pnh, "angle_gain_kp/roll", PidGain_AngleRoll_.kp, &PidGain_AngleRoll_.kp);
        GetRosParameter(pnh, "angle_gain_ki/roll", PidGain_AngleRoll_.ki, &PidGain_AngleRoll_.ki);
        GetRosParameter(pnh, "angle_gain_kd/roll", PidGain_AngleRoll_.kd, &PidGain_AngleRoll_.kd);

        GetRosParameter(pnh, "angle_gain_kp/yaw", PidGain_AngleYaw_.kp, &PidGain_AngleYaw_.kp);
        GetRosParameter(pnh, "angle_gain_ki/yaw", PidGain_AngleYaw_.ki, &PidGain_AngleYaw_.ki);
        GetRosParameter(pnh, "angle_gain_kd/yaw", PidGain_AngleYaw_.kd, &PidGain_AngleYaw_.kd);

        GetRosParameter(pnh, "rate_gain_kp/pitch", PidGain_RatePitch_.kp, &PidGain_RatePitch_.kp);
        GetRosParameter(pnh, "rate_gain_ki/pitch", PidGain_RatePitch_.ki, &PidGain_RatePitch_.ki);
        GetRosParameter(pnh, "rate_gain_kd/pitch", PidGain_RatePitch_.kd, &PidGain_RatePitch_.kd);

        GetRosParameter(pnh, "rate_gain_kp/roll", PidGain_RateRoll_.kp, &PidGain_RateRoll_.kp);
        GetRosParameter(pnh, "rate_gain_ki/roll", PidGain_RateRoll_.ki, &PidGain_RateRoll_.ki);
        GetRosParameter(pnh, "rate_gain_kd/roll", PidGain_RateRoll_.kd, &PidGain_RateRoll_.kd);

        GetRosParameter(pnh, "rate_gain_kp/yaw", PidGain_RateYaw_.kp, &PidGain_RateYaw_.kp);
        GetRosParameter(pnh, "rate_gain_ki/yaw", PidGain_RateYaw_.ki, &PidGain_RateYaw_.ki);
        GetRosParameter(pnh, "rate_gain_kd/yaw", PidGain_RateYaw_.kd, &PidGain_RateYaw_.kd);

        GetRosParameter(pnh, "pos_z_gain_kp/pitch", PidGain_PosZ_.kp, &PidGain_PosZ_.kp);
        GetRosParameter(pnh, "pos_z_gain_ki/pitch", PidGain_PosZ_.ki, &PidGain_PosZ_.ki);
        GetRosParameter(pnh, "pos_z_gain_kd/pitch", PidGain_PosZ_.kd, &PidGain_PosZ_.kd);

        GetRosParameter(pnh, "pos_vz_gain_kp/pitch", PidGain_VelZ_.kp, &PidGain_VelZ_.kp);
        GetRosParameter(pnh, "pos_vz_gain_ki/pitch", PidGain_VelZ_.ki, &PidGain_VelZ_.ki);
        GetRosParameter(pnh, "pos_vz_gain_kd/pitch", PidGain_VelZ_.kd, &PidGain_VelZ_.kd);

        ROS_INFO_ONCE("[Position Controller] Set controller gains and vehicle parameters");

        // Reading the parameters come from the launch file
        bool        dataStoringActive;
        double      dataStoringTime;
        std::string user;

        if (pnh.getParam("user_account", user)) {
            ROS_INFO("Got param 'user_account': %s", user.c_str());
            attitude_controller_.user_ = user;
        } else
            ROS_ERROR("Failed to get param 'user'");

        if (pnh.getParam("csvFilesStoring", dataStoringActive)) {
            ROS_INFO("Got param 'csvFilesStoring': %d", dataStoringActive);
            attitude_controller_.dataStoring_active_ = dataStoringActive;
        } else
            ROS_ERROR("Failed to get param 'csvFilesStoring'");

        if (pnh.getParam("csvFilesStoringTime", dataStoringTime)) {
            ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
            attitude_controller_.dataStoringTime_ = dataStoringTime;
        } else
            ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

        attitude_controller_.SetLaunchFileParameters();
    }

    // if (enable_state_estimator_)
    //     attitude_controller_.crazyflie_onboard_controller_.SetControllerGains(
    //         attitude_controller_.controller_parameters_);
}

void AttitudeControllerNode::Publish() { }

void AttitudeControllerNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{

    ROS_INFO_ONCE("attitudeController got first imu message.");

    // Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;

    // Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp_ = imu_msg->header.stamp;
}

void AttitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{

    ROS_INFO_ONCE("AttitudeController got first odometry message.");

    if (waypointHasBeenPublished_) {
        EigenOdometry odometry;
        eigenOdometryFromMsg(odometry_msg, &odometry);
        if (enable_state_estimator_) {
            attitude_controller_.SetOdometryWithStateEstimator(odometry);
        } else {
            attitude_controller_.SetOdometryWithoutStateEstimator(odometry);
        }

        Eigen::Vector4d ref_rotor_velocities;
        attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

        // creating a new mav message. actuator_msg is used to send the velocities of the propellers.
        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

        // we use clear because we later want to be sure that we used the previously calculated velocity.
        actuator_msg->angular_velocities.clear();
        // for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
            actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        actuator_msg->header.stamp = odometry_msg->header.stamp;

        motor_velocity_reference_pub_.publish(actuator_msg);
    }
}
void AttitudeControllerNode::CallbackAttitudeEstimation(const ros::TimerEvent& event)
{

    if (waypointHasBeenPublished_)
        attitude_controller_.CallbackAttitudeEstimation();
}

// The high level control is run only if the waypoint has been published
void AttitudeControllerNode::CallbackHightLevelControl(const ros::TimerEvent& event)
{

    if (waypointHasBeenPublished_)
        attitude_controller_.CallbackHightLevelControl();
}

void AttitudeControllerNode::CallbackIMUUpdate(const ros::TimerEvent& event)
{

    attitude_controller_.SetSensorData(sensors_);

    ROS_INFO_ONCE("IMU Message sent to attitude controller");

    if (waypointHasBeenPublished_) {

        Eigen::Vector4d ref_rotor_velocities;
        attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

        // A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

        // The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular
        // velocities).
        actuator_msg->angular_velocities.clear();
        // for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
            actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        actuator_msg->header.stamp = imu_msg_head_stamp_;

        motor_velocity_reference_pub_.publish(actuator_msg);
    }
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "attitude_controller_node_with_stateEstimator");

    ros::NodeHandle nh2;

    rotors_control::AttitudeControllerNode attitude_controller_node;

    ros::spin();

    return 0;
}

////////////////////////////////// EOF /////////////////////////////////////////