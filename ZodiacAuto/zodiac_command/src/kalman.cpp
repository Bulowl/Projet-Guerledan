// ----------------------------------------------------------------------------------
// ROS node "kalman" 
// Subscribes to "fix", "vel", "ekf_euler" and "imu" topics.
// Publishes "boat_headng", "currents" topics.
//
// Written by Calvin Lacher
// ----------------------------------------------------------------------------------

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <zodiac_command/mathUtility.h>
#include "geometry_msgs/TwistStamped.h"
#include <sbg_driver/SbgEkfEuler.h>
#include <sbg_driver/SbgImuData.h>
#include <zodiac_command/Kalman.h>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64.h"

using namespace std;
using namespace Eigen;

#define DATA_OUT_OF_RANGE -2000

#define NB_MESURES 2
#define NB_STATES 3
#define NB_COMMAND 2

// Publishers
ros::Publisher currents_pub;
ros::Publisher boatHeading_pub;

// Messages
std_msgs::Float64 currents_msg;
std_msgs::Float64 boatHeading_msg;

// ----------------------------------------------------------------------------------
// Variables:
// ----------------------------------------------------------------------------------

// GPS
double boatVelLon = DATA_OUT_OF_RANGE;      // m/s [vx]
double boatVelLat = DATA_OUT_OF_RANGE;      // m/s [vy]
double boatVelLon_old = boatVelLon;
double boatVelLat_old = boatVelLat;

// IMU
double boatHeading_ned = DATA_OUT_OF_RANGE; // rad [theta] (north-east-down: geographical coordinate system)
double boatHeading_enu = DATA_OUT_OF_RANGE; // rad [theta] (east-north-up: mathematical coordinate system)
double boatAccX = DATA_OUT_OF_RANGE;        // m/s² [ax]
double boatAccY = DATA_OUT_OF_RANGE;        // m/s² [ay]
double magneticDeclination;

// Node parameters
double loopRate;                            // Hz [f]
bool test = true;

// ----------------------------------------------------------------------------------
// Kalman filter parameters:
// ----------------------------------------------------------------------------------

// System Disturbances
double sigmaSpeedOverWater;
double sigmaCurrents;

// Measurement Disturbances
double sigmaGpsPos;
double sigmaGpsVel;

// Covariances
double initialCovariance;

// ----------------------------------------------------------------------------------
// Callbacks:
// ----------------------------------------------------------------------------------


void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)
{
    boatVelLon = vel_msg->twist.linear.x;
    boatVelLat = vel_msg->twist.linear.y;
}      

void ekf_callback(const sbg_driver::SbgEkfEuler::ConstPtr& msg)
{
    boatHeading_ned = mathUtility::sawtoothFunction(msg->angle.z - mathUtility::degreeToRadian(magneticDeclination));
    boatHeading_enu = mathUtility::sawtoothFunction(-boatHeading_ned + M_PI/2);

    boatHeading_msg.data = mathUtility::radianToDegree(boatHeading_ned);
    boatHeading_pub.publish(boatHeading_msg);
}

void imu_callback(const sbg_driver::SbgImuData::ConstPtr& msg)
{
    boatAccX = msg->accel.x;
    boatAccY = msg->accel.y;
}

// ----------------------------------------------------------------------------------
// Kalman filter:
// ----------------------------------------------------------------------------------

void kalman_predict(const Matrix<double,NB_STATES, 1> &xup,
                    const Matrix<double,NB_STATES, NB_STATES> &Gup,
                    const Matrix<double,NB_COMMAND, 1> &u,
                    const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                    const Matrix<double,NB_STATES, NB_STATES> &Ak,
                    const Matrix<double,NB_STATES, NB_COMMAND> &Bk,
                    Matrix<double,NB_STATES, 1> &xnew,
                    Matrix<double,NB_STATES, NB_STATES> &gamma){
  gamma = Ak*Gup*Ak.transpose()+gamma_alpha;
  xnew = Ak*xup+Bk*u;
}

void kalman_correc(const Matrix<double,NB_STATES, 1> &x0,
                   const Matrix<double,NB_STATES,NB_STATES> &gamma_0,
                   const Matrix<double,NB_MESURES, 1> &y,
                   const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
                   const Matrix<double,NB_MESURES, NB_STATES> &Ck,
                   Matrix<double,NB_STATES, 1> &xup,
                   Matrix<double,NB_STATES,NB_STATES> &Gup){
  Matrix<double,NB_MESURES,NB_MESURES> S = Ck * gamma_0 * Ck.transpose() + gamma_beta;
  Matrix<double,NB_STATES, NB_MESURES> K = gamma_0 * Ck.transpose() * S.inverse();
  Matrix<double,NB_MESURES, 1> ytilde = y - Ck*x0;
  Gup = (Matrix<double,NB_STATES,NB_STATES>::Identity()-K*Ck)*gamma_0;
  xup = x0 + K*ytilde;
}

void kalman(Matrix<double,NB_STATES, 1> &x,
            Matrix<double,NB_STATES,NB_STATES> &gamma,
            const Matrix<double,NB_COMMAND, 1> &u,
            const Matrix<double,NB_MESURES, 1> &y,
            const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
            const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
            const Matrix<double,NB_STATES, NB_STATES> &Ak,
            const Matrix<double,NB_STATES, NB_COMMAND> &Bk,
            const Matrix<double,NB_MESURES, NB_STATES> &Ck){
  Matrix<double, NB_STATES, 1> xup;
  Matrix<double, NB_STATES, NB_STATES> Gup;
  kalman_correc(x, gamma, y, gamma_beta, Ck, xup, Gup);
  kalman_predict(xup, Gup, u, gamma_alpha, Ak, Bk, x, gamma);
}

// ----------------------------------------------------------------------------------
// Main:
// ----------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // ------------------------------------------------------------------------------
    
    ros::Subscriber vel_sub = nh.subscribe("vel", 1, vel_callback);
    ros::Subscriber ekf_sub = nh.subscribe("ekf_euler", 1, ekf_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu_data", 1, imu_callback);
    
    // ------------------------------------------------------------------------------
    
    boatHeading_pub = nh.advertise<std_msgs::Float64>("boat_heading", 1);
    currents_pub = nh.advertise<zodiac_command::Kalman>("currents", 1);
    
    zodiac_command::Kalman msg;
    
    // ------------------------------------------------------------------------------
    
    nhp.param<double>("kalman/sigma_speed_over_water", sigmaSpeedOverWater, 1);
    nhp.param<double>("kalman/sigma_currents", sigmaCurrents, 1);
    nhp.param<double>("kalman/sigma_gps_vel", sigmaGpsVel, 1);
    nhp.param<double>("kalman/initial_covariance", initialCovariance, 1);
    nhp.param<double>("ekf/magnetic_declination", magneticDeclination, 1);
    nhp.param<double>("kalman/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);
    
    // ------------------------------------------------------------------------------
    // Kalman filter initialization:
    // ------------------------------------------------------------------------------
    
    bool update;
    
    Matrix<double, NB_STATES,  NB_STATES>  Ak = Matrix<double, NB_STATES, NB_STATES>::Identity();
    Matrix<double, NB_STATES,  NB_COMMAND> Bk;
    Matrix<double, NB_MESURES, NB_STATES>  Ck;

    Matrix<double, NB_STATES,  NB_STATES>  gamma_alpha;
    Matrix<double, NB_MESURES, NB_MESURES> gamma_beta;

    Matrix<double, NB_STATES,  1>          xhat;
    Matrix<double, NB_STATES,  NB_STATES>  gamma;

    Matrix<double, NB_MESURES, 1>          gps; // [y]
    Matrix<double, NB_COMMAND, 1>          imu; // [u]

    // Time:
    double dt;
    ros::Time t_last, t;
    t_last = ros::Time::now();

    // Estimation Vector
    xhat << 2.,
            0.0001,
            0.0001;

    // Co-Variance Matrix of Estimation error
    gamma << initialCovariance, 0, 0,
             0, initialCovariance, 0,
             0, 0, initialCovariance;

    // Co-Variance Matrix of discrete disturbances
    gamma_alpha << sigmaSpeedOverWater, 0, 0,
                   0, sigmaCurrents, 0,
                   0, 0, sigmaCurrents;

    // Matrix of Sensor error
    gamma_beta << sigmaGpsVel, 0,
                  0, sigmaGpsVel;
    
    // ------------------------------------------------------------------------------
    // Ros loop:
    // ------------------------------------------------------------------------------
    
    while (ros::ok())
    {
        t = ros::Time::now();
        dt = (t-t_last).toSec();
        t_last = t;
        
        update = false;

        // ----------------------------------------------------------------------
        // Kalman filter iteration:
        // ----------------------------------------------------------------------

        // IMU update
        imu << 0,
               0;
        
        // Inputs matrix upddate
        Bk << dt, 0,
              0, dt,
              0, 0;   
        
        if(boatHeading_ned != DATA_OUT_OF_RANGE)
        {
            if((boatVelLat != boatVelLat_old) && (boatVelLon != boatVelLon_old))
            {            
                // GPS update
                gps << boatVelLon,
                       boatVelLat;
                      
                Ck << cos(boatHeading_enu), 1, 0,
                      sin(boatHeading_enu), 0, 1;
                
                // Filter call
                kalman(xhat, gamma, imu, gps, gamma_alpha, gamma_beta, Ak, Bk, Ck);
                
                boatVelLon_old = boatVelLon;
                boatVelLat_old = boatVelLat;
                
                update = true;
            }
            else 
            {
                // Only call Prediction without new measurement
                kalman_predict(xhat, gamma, imu, gamma_alpha, Ak, Bk, xhat, gamma);
                
                update = true;
            }
        }
        
        // --------------------------------------------------------------------------
        // Publish:
        // --------------------------------------------------------------------------
        
        if(update)
        {
            msg.speed_over_water  = xhat(0);
            msg.current_speed_lon = xhat(1);
            msg.current_speed_lat = xhat(2);

            msg.covariance[0] = gamma(0,0);
            msg.covariance[1] = gamma(1,1);
            msg.covariance[2] = gamma(2,2);

            currents_pub.publish(msg);
        }
        
        // --------------------------------------------------------------------------
        // Terminal output:
        // --------------------------------------------------------------------------
        
        if(test && update)
        {        
            //cout << "gps: " << gps << endl;
            //std::cout << "boatAccX: " << boatVelLon << std::endl;
            //std::cout << "boatAccY: " << boatVelLat << std::endl;
            //std::cout << "Ak: " << std::endl << Ak_tmp << std::endl;
            //std::cout << "gamma: " << std::endl << gamma << std::endl;
            //std::cout << "gamma_alpha: " << std::endl << gamma_alpha_tmp << std::endl;
            //std::cout << "Ck: " << std::endl << Ck << std::endl;
            //std::cout << "gamma_beta: " << std::endl << gamma_beta << std::endl;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
