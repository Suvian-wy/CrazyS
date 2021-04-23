/*******************************************************************************
* @Copylift (c) 2021, Suvian, Inc.
* @
* @ pluginTemplate : [description]
* @
* @ filename : my_attitude_controller.cpp
* @ author   : Suvian(suvian@qq.com)
* @ create   : 2021/04/22 	 CURRENT_HOUR:CURRENT_MINUTE:CURRENT_SECOND
/******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
// Headers
//
#include "rotors_control/my_attitude_controller.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/sensfusion6.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/transform_datatypes.h"

#include <chrono>
#include <math.h>
#include <ros/ros.h>
#include <time.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
////////////////////////////////////////////////////////////////////////////////
// Inner Scope Objects
//
#define M_PI 3.14159265358979323846          /* pi [rad]*/
#define OMEGA_OFFSET 6874                    /* OMEGA OFFSET [PWM]*/
#define ANGULAR_MOTOR_COEFFICIENT 0.2685     /* ANGULAR_MOTOR_COEFFICIENT */
#define MOTORS_INTERCEPT 426.24              /* MOTORS_INTERCEPT [rad/s]*/
#define MAX_PROPELLERS_ANGULAR_VELOCITY 2618 /* MAX PROPELLERS ANGULAR VELOCITY [rad/s]*/
#define MAX_R_DESIDERED 3.4907               /* MAX R DESIDERED VALUE [rad/s]*/
#define MAX_THETA_COMMAND 0.5236             /* MAX THETA COMMMAND [rad]*/
#define MAX_PHI_COMMAND 0.5236               /* MAX PHI COMMAND [rad]*/
#define MAX_POS_DELTA_OMEGA 1289             /* MAX POSITIVE DELTA OMEGA [PWM]*/
#define MAX_NEG_DELTA_OMEGA -1718            /* MAX NEGATIVE DELTA OMEGA [PWM]*/
#define SAMPLING_TIME 0.01                   /* SAMPLING TIME [s] */
// NOTE:minifly default parameters

static const PidParameter pidAnglePitchDefault(0, 0, 0, 0, 8, 0, 0, 30, 0, 0.004);
static const PidParameter pidAngleRollDefault(0, 0, 0, 0, 8, 0, 0, 30, 0, 0.004);
static const PidParameter pidAngleYawDefault(0, 0, 0, 0, 20, 0, 1.5, 180, 0, 0.004);
static const PidParameter pidRatePitchDefault(0, 0, 0, 0, 300, 0, 6.5, 500, 0, 0.002);
static const PidParameter pidRateRollDefault(0, 0, 0, 0, 300, 0, 6.5, 500, 0, 0.002);
static const PidParameter pidRateYawDefault(0, 0, 0, 0, 200, 18.5, 0.0, 500, 0, 0.002);
static const PidParameter pidPosZDefault(0, 0, 0, 0, 6, 0, 4.5, 0, 120, 0.004);
static const PidParameter pidVelZDefault(0, 0, 0, 0, 100, 150, 10, 0, 40000, 0.004);

static double PidUpdate(PidParameter* pid, const double error)
{
    double output;

    pid->error = error;

    pid->integ += pid->error * pid->dt;

    //积分限幅
    if (pid->iLimit != 0) {
        if (pid->integ > pid->iLimit) {
            pid->integ = pid->iLimit;
        } else if (pid->integ < -pid->iLimit) {
            pid->integ = -pid->iLimit;
        }
    }

    pid->deriv = (pid->error - pid->prevError) / pid->dt;

    double outP = pid->kp * pid->error;
    double outI = pid->ki * pid->integ;
    double outD = pid->kd * pid->deriv;

    output = outP + outI + outD;

    if (pid->outputLimit != 0) {
        if (output > pid->outputLimit)
            output = pid->outputLimit;
        else if (output < -pid->outputLimit)
            output = -pid->outputLimit;
    }

    pid->prevError = pid->error;
    return output;
}
////////////////////////////////////////////////////////////////////////////////
// Functions
//
namespace rotors_control {

AttitudeController::AttitudeController()
    : controller_active_(false)
    , state_estimator_active_(false)
    , dataStoring_active_(false)
    , angle_control_active_(true)
    , p_command(0)
    , q_command(0)
    , r_command(0)
{

    // The control variables are initialized to zero
    control_t_.roll    = 0;
    control_t_.pitch   = 0;
    control_t_.yawRate = 0;
    control_t_.thrust  = 0;

    state_.angularAcc.x = 0; // Angular Acceleration x
    state_.angularAcc.y = 0; // Angular Acceleration y
    state_.angularAcc.z = 0; // Angular Acceleration z

    state_.attitude.roll  = 0; // Roll
    state_.attitude.pitch = 0; // Pitch
    state_.attitude.yaw   = 0; // Yaw

    state_.position.x = 0; // Position.x
    state_.position.y = 0; // Position.y
    state_.position.z = 0; // Position.z

    state_.angularVelocity.x = 0; // Angular velocity x
    state_.angularVelocity.y = 0; // Angular velocity y
    state_.angularVelocity.z = 0; // Angular velocity z

    state_.linearVelocity.x = 0; // Linear velocity x
    state_.linearVelocity.y = 0; // Linear velocity y
    state_.linearVelocity.z = 0; // Linear velocity z

    state_.attitudeQuaternion.x = 0; // Quaternion x
    state_.attitudeQuaternion.y = 0; // Quaternion y
    state_.attitudeQuaternion.z = 0; // Quaternion z
    state_.attitudeQuaternion.w = 0; // Quaternion w

    PidGain_AnglePitch_ = pidAnglePitchDefault;
    PidGain_AngleRoll_  = pidAngleRollDefault;
    PidGain_AngleYaw_   = pidAngleYawDefault;
    PidGain_RatePitch_  = pidRatePitchDefault;
    PidGain_RateRoll_   = pidRateRollDefault;
    PidGain_RateYaw_    = pidRateYawDefault;
    PidGain_PosZ_       = pidPosZDefault;
    PidGain_VelZ_       = pidVelZDefault;
}

AttitudeController::~AttitudeController() { }

void AttitudeController::CallbackSaveData(const ros::TimerEvent& event)
{

    if (!dataStoring_active_) {
        return;
    }

    ofstream filePropellersVelocity;
    ofstream fileDroneAttiude;
    ofstream filePWM;
    ofstream filePWMComponents;
    ofstream fileCommandAttiude;
    ofstream fileRCommand;
    ofstream fileOmegaCommand;
    ofstream fileXeYe;
    ofstream fileDeltaCommands;
    ofstream filePQCommands;
    ofstream fileDronePosition;

    ROS_INFO("CallbackSavaData function is working. Time: %f seconds, %f nanoseconds", odometry_.timeStampSec,
        odometry_.timeStampNsec);

    filePropellersVelocity.open("/home/" + user_ + "/PropellersVelocity.csv", std::ios_base::app);
    fileDroneAttiude.open("/home/" + user_ + "/DroneAttiude.csv", std::ios_base::app);
    filePWM.open("/home/" + user_ + "/PWM.csv", std::ios_base::app);
    filePWMComponents.open("/home/" + user_ + "/PWMComponents.csv", std::ios_base::app);
    fileCommandAttiude.open("/home/" + user_ + "/CommandAttitude.csv", std::ios_base::app);
    fileRCommand.open("/home/" + user_ + "/RCommand.csv", std::ios_base::app);
    fileOmegaCommand.open("/home/" + user_ + "/OmegaCommand.csv", std::ios_base::app);
    fileXeYe.open("/home/" + user_ + "/XeYe.csv", std::ios_base::app);
    fileDeltaCommands.open("/home/" + user_ + "/DeltaCommands.csv", std::ios_base::app);
    filePQCommands.open("/home/" + user_ + "/PQCommands.csv", std::ios_base::app);
    fileDronePosition.open("/home/" + user_ + "/DronePosition.csv", std::ios_base::app);

    // Saving control signals in a file
    for (unsigned n = 0; n < listPropellersVelocity_.size(); ++n) {
        filePropellersVelocity << listPropellersVelocity_.at(n);
    }

    for (unsigned n = 0; n < listDroneAttiude_.size(); ++n) {
        fileDroneAttiude << listDroneAttiude_.at(n);
    }

    for (unsigned n = 0; n < listPWM_.size(); ++n) {
        filePWM << listPWM_.at(n);
    }

    for (unsigned n = 0; n < listPWMComponents_.size(); ++n) {
        filePWMComponents << listPWMComponents_.at(n);
    }

    for (unsigned n = 0; n < listCommandAttiude_.size(); ++n) {
        fileCommandAttiude << listCommandAttiude_.at(n);
    }

    for (unsigned n = 0; n < listRCommand_.size(); ++n) {
        fileRCommand << listRCommand_.at(n);
    }

    for (unsigned n = 0; n < listOmegaCommand_.size(); ++n) {
        fileOmegaCommand << listOmegaCommand_.at(n);
    }

    for (unsigned n = 0; n < listXeYe_.size(); ++n) {
        fileXeYe << listXeYe_.at(n);
    }

    for (unsigned n = 0; n < listDeltaCommands_.size(); ++n) {
        fileDeltaCommands << listDeltaCommands_.at(n);
    }

    for (unsigned n = 0; n < listPQCommands_.size(); ++n) {
        filePQCommands << listPQCommands_.at(n);
    }

    for (unsigned n = 0; n < listDronePosition_.size(); ++n) {
        fileDronePosition << listDronePosition_.at(n);
    }

    // Closing all opened files
    filePropellersVelocity.close();
    fileDroneAttiude.close();
    filePWM.close();
    filePWMComponents.close();
    fileCommandAttiude.close();
    fileRCommand.close();
    fileOmegaCommand.close();
    fileXeYe.close();
    fileDeltaCommands.close();
    filePQCommands.close();
    fileDronePosition.close();

    // To have a one shot storing
    dataStoring_active_ = false;
}

void AttitudeController::SetLaunchFileParameters()
{

    // The boolean variable is used to inactive the logging if it is not useful
    if (dataStoring_active_) {

        // Time after which the data storing function is turned on
        timer_
            = n_.createTimer(ros::Duration(dataStoringTime_), &AttitudeController::CallbackSaveData, this, false, true);

        // Cleaning the string vector contents
        listPropellersVelocity_.clear();
        listDroneAttiude_.clear();
        listPWM_.clear();
        listPWMComponents_.clear();
        listCommandAttiude_.clear();
        listRCommand_.clear();
        listOmegaCommand_.clear();
        listXeYe_.clear();
        listDeltaCommands_.clear();
        listPQCommands_.clear();
        listDronePosition_.clear();
    }
}

// void AttitudeController::SetControllerGains() { }

void AttitudeController::SetTargetAttitude(const mav_msgs::EigenRollPitchYawrateThrust& target)
{
    target_attitude_   = target;
    controller_active_ = true;
}

void AttitudeController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const
{
    assert(roll);
    assert(pitch);
    assert(yaw);

    // The estimated quaternion values
    double x, y, z, w;
    x = state_.attitudeQuaternion.x;
    y = state_.attitudeQuaternion.y;
    z = state_.attitudeQuaternion.z;
    w = state_.attitudeQuaternion.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3  m(q);
    m.getRPY(*roll, *pitch, *yaw);

    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);
}

void AttitudeController::SetOdometryWithoutStateEstimator(const EigenOdometry& odometry)
{

    odometry_ = odometry;

    // Such function is invoked when the ideal odometry sensor is employed
    SetSensorData();

    if (dataStoring_active_) {

        // Saving drone attitude in a file
        std::stringstream tempDronePosition;
        tempDronePosition << odometry_.position[0] << "," << odometry_.position[1] << "," << odometry_.position[2]
                          << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

        listDronePosition_.push_back(tempDronePosition.str());
    }
}

void AttitudeController::SetSensorData()
{

    // Only the position sensor is ideal, any virtual sensor or systems is available to get it
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];

    state_.attitudeQuaternion.x = odometry_.orientation.x();
    state_.attitudeQuaternion.y = odometry_.orientation.y();
    state_.attitudeQuaternion.z = odometry_.orientation.z();
    state_.attitudeQuaternion.w = odometry_.orientation.w();

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];
}

void AttitudeController::SetOdometryWithStateEstimator(const EigenOdometry& odometry)
{

    odometry_ = odometry;
}

void AttitudeController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities)
{
    assert(rotor_velocities);

    // This is to disable the controller if we do not receive a trajectory
    if (!controller_active_) {
        *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
        return;
    }

    double PWM_1, PWM_2, PWM_3, PWM_4;
    ControlMixer(&PWM_1, &PWM_2, &PWM_3, &PWM_4);

    double omega_1, omega_2, omega_3, omega_4;
    omega_1 = ((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_2 = ((PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_3 = ((PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_4 = ((PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);

    // The omega values are saturated considering physical constraints of the system
    if (!(omega_1 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_1 > 0))
        if (omega_1 > MAX_PROPELLERS_ANGULAR_VELOCITY)
            omega_1 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
            omega_1 = 0;

    if (!(omega_2 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_2 > 0))
        if (omega_2 > MAX_PROPELLERS_ANGULAR_VELOCITY)
            omega_2 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
            omega_2 = 0;

    if (!(omega_3 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_3 > 0))
        if (omega_3 > MAX_PROPELLERS_ANGULAR_VELOCITY)
            omega_3 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
            omega_3 = 0;

    if (!(omega_4 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_4 > 0))
        if (omega_4 > MAX_PROPELLERS_ANGULAR_VELOCITY)
            omega_4 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
            omega_4 = 0;

    if (dataStoring_active_) {
        // Saving drone attitude in a file
        std::stringstream tempPropellersVelocity;
        tempPropellersVelocity << omega_1 << "," << omega_2 << "," << omega_3 << "," << omega_4 << ","
                               << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

        listPropellersVelocity_.push_back(tempPropellersVelocity.str());
    }
}

void AttitudeController::ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4)
{
    assert(PWM_1);
    assert(PWM_2);
    assert(PWM_3);
    assert(PWM_4);

    // if (!state_estimator_active_)
    //     // When the state estimator is disable, the delta_omega_ value is computed as soon as the new odometry
    //     // message is available.
    //     // The timing is managed by the publication of the odometry topic
    HoveringController(&control_t_.thrust);

    // Control signals are sent to the on board control architecture if the state estimator is active
    double delta_phi, delta_theta, delta_psi;

    RateController(&delta_phi, &delta_theta, &delta_psi);

    *PWM_1 = control_t_.thrust - (delta_theta / 2) - (delta_phi / 2) - delta_psi;
    *PWM_2 = control_t_.thrust + (delta_theta / 2) - (delta_phi / 2) + delta_psi;
    *PWM_3 = control_t_.thrust + (delta_theta / 2) + (delta_phi / 2) - delta_psi;
    *PWM_4 = control_t_.thrust - (delta_theta / 2) + (delta_phi / 2) + delta_psi;

    if (dataStoring_active_) {
        // Saving drone attitude in a file
        std::stringstream tempPWM;
        tempPWM << *PWM_1 << "," << *PWM_2 << "," << *PWM_3 << "," << *PWM_4 << "," << odometry_.timeStampSec << ","
                << odometry_.timeStampNsec << "\n";

        listPWM_.push_back(tempPWM.str());

        // Saving drone attitude in a file
        std::stringstream tempPWMComponents;
        tempPWMComponents << control_t_.thrust << "," << delta_theta << "," << delta_phi << "," << delta_psi << ","
                          << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

        listPWMComponents_.push_back(tempPWMComponents.str());
    }

    ROS_DEBUG("Omega: %f, Delta_theta: %f, Delta_phi: %f, delta_psi: %f", control_t_.thrust, delta_theta, delta_phi,
        delta_psi);
    ROS_DEBUG("PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2, *PWM_3, *PWM_4);
}

void AttitudeController::HoveringController(double* omega)
{
    assert(omega);

    double z_error, z_reference, dot_zeta;
    z_reference = target_attitude_.thrust[2]; //使用z_thrust代替hight
    z_error     = z_reference - state_.position.z;

    // Velocity along z-axis from body to inertial frame
    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    // Needed because both angular and linear velocities are expressed in the aircraft body frame
    dot_zeta = -sin(pitch) * state_.linearVelocity.x + sin(roll) * cos(pitch) * state_.linearVelocity.y
               + cos(roll) * cos(pitch) * state_.linearVelocity.z;

    double inner_input = PidUpdate(PidGain_PosZ_, z_error);
    double delta_omega = PidUpdate(PidGain_VelZ_, inner_input - dot_zeta);

    // Delta omega value is saturated considering the aircraft physical constraints
    if (delta_omega > MAX_POS_DELTA_OMEGA || delta_omega < MAX_NEG_DELTA_OMEGA)
        if (delta_omega > MAX_POS_DELTA_OMEGA)
            delta_omega = MAX_POS_DELTA_OMEGA;
        else
            delta_omega = -MAX_NEG_DELTA_OMEGA;

    *omega = OMEGA_OFFSET + delta_omega;

    if (dataStoring_active_) {
        // Saving drone attitude in a file
        std::stringstream tempOmegaCommand;
        tempOmegaCommand << *omega << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

        listOmegaCommand_.push_back(tempOmegaCommand.str());

        // Saving drone attitude in a file
        std::stringstream tempDroneAttitude;
        tempDroneAttitude << roll << "," << pitch << "," << yaw << "," << odometry_.timeStampSec << ","
                          << odometry_.timeStampNsec << "\n";

        listDroneAttiude_.push_back(tempDroneAttitude.str());
    }

}

void AttitudeController::RateController(double* delta_phi, double* delta_theta, double* delta_psi)
{
    assert(delta_phi);
    assert(delta_theta);
    assert(delta_psi);

    double p, q, r;
    p = state_.angularVelocity.x;
    q = state_.angularVelocity.y;
    r = state_.angularVelocity.z;

    if (angle_control_active_) {
        angle_control_active_ = false;
        AngleController(&p_command, &q_command, &r_command);
    } else {
        angle_control_active_ = true;
    }

    // double r_command;
    // YawPositionController(&r_command);

    double p_error, q_error, r_error;
    p_error = p_command - p;
    q_error = q_command - q;
    r_error = r_command - r;

    *delta_phi   = PidUpdate(PidGain_RateRoll_, p_error);
    *delta_theta = PidUpdate(PidGain_RatePitch_, q_error);
    *delta_psi   = PidUpdate(PidGain_RateYaw, r_error);

    if (dataStoring_active_) {
        // Saving drone attitude in a file
        std::stringstream tempDeltaCommands;
        tempDeltaCommands << *delta_phi << "," << *delta_theta << "," << *delta_psi << "," << odometry_.timeStampSec
                          << "," << odometry_.timeStampNsec << "\n";

        listDeltaCommands_.push_back(tempDeltaCommands.str());
    }
}

void AttitudeController::AngleController(double* p_command, double* q_command, double, r_command)
{
    assert(p_command);
    assert(q_command);

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    // double theta_command, phi_command;
    // XYController(&theta_command, &phi_command);

    double roll_error, pitch_error, yaw_error;
    roll_error  = target_attitude.roll - roll;
    pitch_error = target_attitude.pitch - pitch;
    yaw_error   = target_attitude.yaw_rate - yaw; //使用yaw_rate代替desire_yaw

    *p_command = PidUpdate(PidGain_AngleRoll, roll_error);
    *q_command = PidUpdate(PidGain_AnglePitch, pitch_error);
    *r_command = PidUpdate(PidGain_AngleYaw, yaw_error);


    if (dataStoring_active_) {
        // Saving drone attitude in a file
        std::stringstream tempPQCommands;
        tempPQCommands << *p_command << "," << *q_command << "," << odometry_.timeStampSec << ","
                       << odometry_.timeStampNsec << "\n";

        listPQCommands_.push_back(tempPQCommands.str());
    }

    ROS_DEBUG("Phi_c: %f, Phi_e: %f, Theta_c: %f, Theta_e: %f", target_attitude.roll, roll_error, target_attitude.pitch,
        pitch_error);
}

void AttitudeController::CallbackAttitudeEstimation()
{

    // Angular velocities updating
    complementary_filter_crazyflie_.EstimateAttitude(&state_, &sensors_);

    ROS_DEBUG("Attitude Callback");
}

// The high level control runs with a frequency of 100Hz
void AttitudeController::CallbackHightLevelControl()
{

    // Thrust value
    HoveringController(&control_t_.thrust);

    // Phi and theta command signals. The Error Body Controller is invoked every 0.01 seconds. It uses XYController's
    // outputs
    // XYController(&control_t_.pitch, &control_t_.roll);

    // Yaw rate command signals
    // YawPositionController(&control_t_.yawRate);

    ROS_DEBUG(
        "Position_x: %f, Position_y: %f, Position_z: %f", state_.position.x, state_.position.y, state_.position.z);

    ROS_DEBUG("Angular_velocity_x: %f, Angular_velocity_y: %f, Angular_velocity_z: %f", state_.angularVelocity.x,
        state_.angularVelocity.y, state_.angularVelocity.z);

    ROS_DEBUG("Linear_velocity_x: %f, Linear_velocity_y: %f, Linear_velocity_z: %f", state_.linearVelocity.x,
        state_.linearVelocity.y, state_.linearVelocity.z);
}

void PositionController::SetSensorData(const sensorData_t& sensors)
{

    // The functions runs at 500Hz, the same frequency with which the IMU topic publishes new values (with a frequency
    // of 500Hz)
    sensors_ = sensors;
    complementary_filter_crazyflie_.EstimateRate(&state_, &sensors_);

    // if (!state_estimator_active_)
    //     state_estimator_active_ = true;

    // Only the position sensor is ideal, any virtual sensor or systems is available to get these data
    // Every 0.002 seconds the odometry message values are copied in the state_ structure, but they change only 0.01
    // seconds
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];
}

}
////////////////////////////////// EOF /////////////////////////////////////////