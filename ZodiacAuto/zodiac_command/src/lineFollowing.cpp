// ----------------------------------------------------------------------------------
// ROS node "lineFollowing" 
// Subscribes to "waypoint_line", "kalman" and "boat_heading" topics.
// Publishes "signed_distance", "line_angle", "desired_course" and "helm_cmd" topics.
//
// Developer Notes:
//    Algorithm inspired and modified from:
//    Luc Jaulin and Fabrice Le Bars "An Experimental Validation of a Robust
//    Controller with the VAIMOS Autonomous Sailboat"
//
// Written by Mael le Gallique
// Revised by Calvin Lacher
// ----------------------------------------------------------------------------------

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <zodiac_command/WaypointListMission.h>
#include <zodiac_command/mathUtility.h>
#include <zodiac_command/Kalman.h>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64.h"

using namespace std;
using namespace Eigen;

#define DATA_OUT_OF_RANGE -2000

// Publishers
ros::Publisher signedDistance_pub;
ros::Publisher lineAngle_pub;
ros::Publisher desiredCourse_pub;
ros::Publisher helmCmd_pub;

// Messages
std_msgs::Float64 signedDistance_msg;
std_msgs::Float64 lineAngle_msg;
std_msgs::Float64 desiredCourse_msg;
std_msgs::Float64 helmCmd_msg;

// ----------------------------------------------------------------------------------
// Variables:
// ----------------------------------------------------------------------------------

vector<zodiac_command::WaypointMission> waypointLine;

// Boat state estimation
double boatLongitude = DATA_OUT_OF_RANGE;      // [x]
double boatLatitude = DATA_OUT_OF_RANGE;       // [y]
//double boatVelLon = DATA_OUT_OF_RANGE;       // [vx]
//double boatVelLat = DATA_OUT_OF_RANGE;       // [vy]
double speedOverWater = DATA_OUT_OF_RANGE; // m/s [p1]
double currentsLon = DATA_OUT_OF_RANGE;    // m/s [p2]
double currentsLat = DATA_OUT_OF_RANGE;    // m/s [p3]

// Compass (IMU)
double boatHeading = DATA_OUT_OF_RANGE;    // rad [theta]

// Control parameters
double maxDist;                            // meters [r] 
double motorBias;                          // degrees
double gainP;                              // [P]
int regulatorType;

// Node parameters
double loopRate;                            // Hz [f]

// ----------------------------------------------------------------------------------
// Controllers:
// ----------------------------------------------------------------------------------

double regulatorTanh(const double heading, const double dist, const double phi)
{
    // Transform heading to waypoint line coordinate system
    double headingLine = mathUtility::sawtoothFunction(heading - phi);
    
    // Control
    double rudder = -headingLine - tanh(dist/maxDist) - sin(headingLine)/(1+pow((dist),2));
    
    return rudder;
}

double regulatorTanhCurrents(const double heading, const double dist, const double phi, const double p1, const double p2, const double p3)
{
    // Transform heading to waypoint line coordinate system
    double headingLine = mathUtility::sawtoothFunction(heading - phi);

    // Transform currents to waypoint line coordinate system
    Matrix<double, 3, 1> p;
    p << p1,
         p2,
         p3;
    Matrix<double, 3, 3> Rot;
    Rot << 1,         0,        0,
           0,  cos(phi), sin(phi),
           0, -sin(phi), cos(phi);
    Matrix<double, 3, 1> pLine = Rot*p;
    
    // Control
    double a = pLine(0)*cos(headingLine) + pLine(1);
    double dista = -pow(pLine(0),2)*sin(headingLine);
    double b = pLine(0)*sin(headingLine) + pLine(2);
    double distb = pow(pLine(0),2)*cos(headingLine);
    double w = (a*distb-b*dista) / (pow(a,2)+pow(b,2));
    double y = atan2( pLine(0)*sin(headingLine)+pLine(2), pLine(0)*cos(headingLine)+pLine(1) ) + tanh(dist/maxDist);
    double rudder = ( -y - b*(pow((1/cosh(dist)),2)) ) / w;

    return rudder;
}

// ----------------------------------------------------------------------------------
// Controller switch:
// ----------------------------------------------------------------------------------
double calculateTargetCourse(const double heading, const double dist, const double phi, const double p1, const double p2, const double p3)
{
    double helmCmd;

    switch(regulatorType)
    {
    case 1 : // Arctan regulator
        helmCmd = regulatorTanh(heading, dist, phi);
        break;
    case 2 : // Arctan + Kalman regulator
        helmCmd = regulatorTanhCurrents(heading, dist, phi, p1, p2, p3);
        break;
    }

    return helmCmd;
}

// ----------------------------------------------------------------------------------
// Callbacks:
// ----------------------------------------------------------------------------------

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
{
    if (fix_msg->status.status >= fix_msg->status.STATUS_FIX)
    {
        boatLatitude = fix_msg->latitude;
        boatLongitude = fix_msg->longitude;

    }
    else
    {
        ROS_WARN_THROTTLE(5, "No gps fix");
    }
}

void waypointLine_callback(const zodiac_command::WaypointListMission::ConstPtr& msg)
{
    waypointLine = msg->waypoints;
}

void kalman_callback(const zodiac_command::Kalman::ConstPtr& kalman_msg)
{
    speedOverWater = kalman_msg->speed_over_water; // [p1]
    currentsLon = kalman_msg->current_speed_lon;   // [p2]
    currentsLat = kalman_msg->current_speed_lat;   // [p3]
}

void boatHeading_callback(const std_msgs::Float64::ConstPtr& msg)
{
    boatHeading = mathUtility::degreeToRadian(mathUtility::limitAngleRange180( (-1)*(msg->data-90) )); // east-north-up
}

// ----------------------------------------------------------------------------------
// Main:
// ----------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lineFollowing");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    
    // ------------------------------------------------------------------------------
    
    ros::Subscriber fix_sub = nh.subscribe("fix", 1, fix_callback);
    ros::Subscriber waypointLine_sub = nh.subscribe("waypoint_line", 1, waypointLine_callback);
    ros::Subscriber kalman_sub = nh.subscribe("currents", 1, kalman_callback);
    ros::Subscriber boatHeading_sub = nh.subscribe("boat_heading", 1, boatHeading_callback);
    
    // ------------------------------------------------------------------------------

    signedDistance_pub = nh.advertise<std_msgs::Float64>("signed_distance", 1);
    lineAngle_pub = nh.advertise<std_msgs::Float64>("line_angle", 1);
    desiredCourse_pub = nh.advertise<std_msgs::Float64>("desired_course", 1);
    helmCmd_pub = nh.advertise<std_msgs::Float64>("helm_angle_cmd", 1);
    
    // ------------------------------------------------------------------------------

    nhp.param<double>("lineFollowing/max_distance_from_line", maxDist, 5);
    nhp.param<double>("lineFollowing/motor_angle_bias", motorBias, 1);
    nhp.param<double>("lineFollowing/proportional_gain", gainP, 1);
    nhp.param<double>("lineFollowing/loop_rate", loopRate, 1);
    nhp.param<int>("lineFollowing/regulator_type", regulatorType, 1);

    // ------------------------------------------------------------------------------
    // Ros loop:
    // ------------------------------------------------------------------------------
    
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        if((boatLatitude != DATA_OUT_OF_RANGE) && (boatLongitude != DATA_OUT_OF_RANGE) && (waypointLine.size()>0))
        {
            // ----------------------------------------------------------------------
            // Control:
            // ----------------------------------------------------------------------
            
            // Calculate signed distance to the line            [dist]
            double signedDistance = mathUtility::calculateSignedDistanceToLine(waypointLine.at(1).longitude, waypointLine.at(1).latitude, 
                                    waypointLine.at(0).longitude, waypointLine.at(0).latitude, boatLongitude, boatLatitude);
            
            // Calculate the angle of the line to be followed   [phi -> north-east-down]
            double lineAngle_ned = mathUtility::calculateAngleOfDesiredTrajectory(waypointLine.at(1).longitude, waypointLine.at(1).latitude, 
                                    waypointLine.at(0).longitude, waypointLine.at(0).latitude, boatLongitude, boatLatitude);
            // Transform to control coordinate frame            [phi -> east-north-up]
            double lineAngle_enu = mathUtility::sawtoothFunction(-lineAngle_ned + M_PI/2);
                       
            // Calculate controller output                      [u]
            double helmCmd = calculateTargetCourse(boatHeading, signedDistance, lineAngle_enu, speedOverWater, currentsLon, currentsLat);
            
            // ----------------------------------------------------------------------
            // Additional parameters:
            // ----------------------------------------------------------------------
                        
            // Compute desired course (not used in controller)  [north-east-down]
            double desiredCourse = mathUtility::sawtoothFunction(lineAngle_ned + tanh(signedDistance/maxDist));
            
            // ----------------------------------------------------------------------
            // Publish:
            // ----------------------------------------------------------------------
            
            signedDistance_msg.data = signedDistance;
            signedDistance_pub.publish(signedDistance_msg);
            
            lineAngle_msg.data = lineAngle_ned*(180/M_PI);
            lineAngle_pub.publish(lineAngle_msg);
            
            desiredCourse_msg.data = desiredCourse*(180/M_PI);
            desiredCourse_pub.publish(desiredCourse_msg);
            
            helmCmd_msg.data = ( helmCmd*(180/M_PI) + motorBias )*gainP;
            helmCmd_pub.publish(helmCmd_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
