/*******************************************************************************
* @Copylift (c) 2021, Suvian, Inc.
* @
* @ pluginTemplate : [description]
* @
* @ filename : my_attitude_controller.h
* @ author   : Suvian(suvian@qq.com)
* @ create   : 2021/04/22 	 15:01:57
/******************************************************************************/

#ifndef __MY_ATTITUDE_CONTROLLER_H__
#define __MY_ATTITUDE_CONTROLLER_H__

////////////////////////////////////////////////////////////////////////////////
// Headers
//
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <string>

#include <ros/time.h>

#include "common.h"
#include "controller_parameters.h"
#include "crazyflie_complementary_filter.h"
#include "parameters.h"
#include "sensfusion6.h"
#include "stabilizer_types.h"

#include <time.h>

////////////////////////////////////////////////////////////////////////////////
// Typedefs & Constants
//
using namespace std;

typedef struct {
    double error;       //< error
    double prevError;   //< previous error
    double integ;       //< integral
    double deriv;       //< derivative
    double kp;          //< proportional gain
    double ki;          //< integral gain
    double kd;          //< derivative gain
    double iLimit;      //< integral limit
    double outputLimit; //< total PID output limit, absolute value. '0' means no limit.
    double dt;

    PidParameter(double err, double pError, double integral, double der, double p, double i, double d, double iL,
        double oL, double t)
        : error(err)
        , prevError(pError)
        , integ(integral)
        , deriv(der)
        , kp(p)
        , ki(i)
        , kd(d)
        , iLimit(iL)
        , outputLimit(oL)
        , dt(t)
    {
    }

} PidParameter;
////////////////////////////////////////////////////////////////////////////////
// Classes
//
namespace rotors_control {

class AttitudeController {
public:
    AttitudeController();
    ~AttitudeController();
    void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

    void SetOdometryWithStateEstimator(const EigenOdometry& odometry);
    void SetOdometryWithoutStateEstimator(const EigenOdometry& odometry);
    void SetSensorData(const sensorData_t& sensors);
    void SetTargetAttitude(const mav_msgs::EigenRollPitchYawrateThrust& target);
    // void SetControllerGains();
    void CallbackAttitudeEstimation();
    void CallbackHightLevelControl();
    void SetLaunchFileParameters();

    // Lunch file parameters
    std::string user_;
    double      dataStoringTime_;
    bool        dataStoring_active_;

    PidParameter PidGain_AnglePitch_;
    PidParameter PidGain_AngleRoll_;
    PidParameter PidGain_AngleYaw_;
    PidParameter PidGain_RatePitch_;
    PidParameter PidGain_RateRoll_;
    PidParameter PidGain_RateYaw_;
    PidParameter PidGain_PosZ_;
    PidParameter PidGain_VelZ_;

    // AttitudeControllerParameters  controller_parameters_;
    ComplementaryFilterCrazyflie2 complementary_filter_crazyflie_;

private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool   controller_active_;
    bool   state_estimator_active_;
    bool   angle_control_active_;
    double p_command;
    double q_command;
    double r_command;

    control_s control_t_;

    // Lists for data saving
    std::vector<string> listPropellersVelocity_;
    std::vector<string> listDroneAttiude_;
    std::vector<string> listPWM_;
    std::vector<string> listPWMComponents_;
    std::vector<string> listCommandAttiude_;
    std::vector<string> listRCommand_;
    std::vector<string> listOmegaCommand_;
    std::vector<string> listXeYe_;
    std::vector<string> listDeltaCommands_;
    std::vector<string> listPQCommands_;
    std::vector<string> listDronePosition_;

    // Callbacks
    ros::NodeHandle n_;
    ros::Timer      timer_;
    void            CallbackSaveData(const ros::TimerEvent& event);

    // pid parameters

    void SetSensorData();

    mav_msgs::EigenRollPitchYawrateThrust target_attitude_;
    EigenOdometry                         odometry_;
    sensorData_t                          sensors_;
    state_t                               state_;

    void RateController(double* delta_phi, double* delta_theta, double* delta_psi);
    void AngleController(double* p_command, double* q_command);
    void HoveringController(double* delta_omega);
    void ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4);
    void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
};

}
////////////////////////////////////////////////////////////////////////////////
// Functions
//

#endif //__MY_ATTITUDE_CONTROLLER_H__
       ////////////////////////////////// EOF /////////////////////////////////////////