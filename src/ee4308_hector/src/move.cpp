#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <signal.h>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

ros::ServiceClient en_mtrs;
void disable_motors(int sig)
{
    ROS_INFO(" HMOVE : Disabling motors...");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv); 
}

double target_x = NaN, target_y = NaN, target_z = NaN;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_x = msg->point.x;
    target_y = msg->point.y;
    target_z = msg->point.z;
}

double x = NaN, y = NaN, z = NaN, a = NaN;
void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}

bool rotate = false;
void cbRotate(const std_msgs::Bool::ConstPtr &msg)
{
    rotate = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" HMOVE : Param enable_move not found, set to true");
    if (!enable_move)
        return 0;
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" HMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" HMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double Kp_z;
    if (!nh.param("Kp_z", Kp_z, 1.0))
        ROS_WARN(" HMOVE : Param Kp_z not found, set to 1.0");
    double Ki_z;
    if (!nh.param("Ki_z", Ki_z, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_z;
    if (!nh.param("Kd_z", Kd_z, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double yaw_rate;
    if (!nh.param("yaw_rate", yaw_rate, 0.5))
        ROS_WARN(" HMOVE : Param yaw_rate not found, set to 0.5");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 2.0))
        ROS_WARN(" HMOVE : Param max_lin_vel not found, set to 2");
    double max_z_vel;
    if (!nh.param("max_z_vel", max_z_vel, 0.5))
        ROS_WARN(" HMOVE : Param max_z_vel not found, set to 0.5");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" HMOVE : Param move_iter_rate not found, set to 25");
    
    // --------- Enable Motors ----------
    ROS_INFO(" HMOVE : Enabling motors...");
    en_mtrs = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = true;
    if (en_mtrs.call(en_mtrs_srv))
        ROS_INFO(" HMOVE : Motors enabled!");
    else
        ROS_WARN(" HMOVE : Cannot enable motors!");
    signal(SIGINT, disable_motors);

    // --------- Subscribers ----------
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);
    ros::Subscriber sub_rotate = nh.subscribe("rotate", 1, &cbRotate);

    // --------- Publishers ----------
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // --------- Wait for Topics ----------
    ROS_INFO(" HMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && (std::isnan(target_x) || std::isnan(x))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce(); // update the topics

    // --------- Begin Controller ----------
    ROS_INFO(" HMOVE : ===== BEGIN =====");
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic
    double cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z, cmd_lin_vel_a;
    double dt;
    double prev_time = ros::Time::now().toSec();

    // -------PID VARIABLES------- //
    double pid_x_p_term = 0;
    double pid_x_i_accum = 0;
    double pid_x_i_term = 0;
    double pid_x_d_term = 0;

    double pid_y_p_term = 0;
    double pid_y_i_accum = 0;
    double pid_y_i_term = 0;
    double pid_y_d_term = 0;

    double pid_z_p_term = 0;
    double pid_z_i_accum = 0;
    double pid_z_i_term = 0;
    double pid_z_d_term = 0;

    double pid_a_p_term = 0;
    double pid_a_i_accum = 0;
    double pid_a_i_term = 0;
    double pid_a_d_term = 0;

    // intermediate outputs
    double pid_x_output, pid_y_output, pid_z_output;

    // saturated pid outputs
    double pid_x_output_sat;
    double pid_x_output_sat_prev = 0;

    double pid_y_output_sat;
    double pid_y_output_sat_prev = 0;

    double pid_z_output_sat;
    double pid_z_output_sat_prev = 0;

    // acceleration variables for pid saturation
    double acc_x_est;
    double acc_y_est;
    double acc_z_est;

    // positional errors
    double pos_x_err;
    double pos_x_err_prev = 0;

    double pos_y_err;
    double pos_y_err_prev = 0;

    double pos_z_err;
    double pos_z_err_prev = 0;
    //-----------------------------//


    // main loop
    while (ros::ok() && nh.param("run", true))
    {
        // update all topics
        ros::spinOnce();

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;

        //-------PID CONTROLLER-------//
        pos_x_err = target_x - x;
        pos_y_err = target_y - y;
        pos_z_err = target_z - z;

        // positional pid terms
        pid_x_p_term = Kp_lin * pos_x_err;
        pid_y_p_term = Kp_lin * pos_y_err;
        pid_z_p_term = Kp_z * pos_z_err;

        pid_x_i_accum += pos_x_err * dt;
        pid_y_i_accum += pos_y_err * dt;
        pid_z_i_accum += pos_z_err * dt;

        pid_x_i_term = Ki_lin * pid_x_i_accum;
        pid_y_i_term = Ki_lin * pid_y_i_accum;
        pid_z_i_term = Ki_z * pid_z_i_accum;

        pid_x_d_term = Kd_lin * (pos_x_err - pos_x_err_prev);
        pid_y_d_term = Kd_lin * (pos_y_err - pos_y_err_prev);
        pid_z_d_term = Kd_z * (pos_z_err - pos_z_err_prev);

        // intermediate pid outputs
        pid_x_output = pid_x_p_term + pid_x_i_term + pid_x_d_term;
        pid_y_output = pid_y_p_term + pid_y_i_term + pid_y_d_term;
        pid_z_output = pid_z_p_term + pid_z_i_term + pid_z_d_term;

        
        // pid saturation calculation

        // z
        acc_z_est = (pid_z_output - pid_z_output_sat_prev) / dt;
        pid_z_output_sat = sat(pid_z_output_sat_prev + acc_z_est * dt, max_z_vel);


        /* ----- heading lock implementation ----- */

        double heading = a * 180.0f / M_PI;

        if (heading < 0 ) {
            heading = 360.0f + heading;
        }

        cmd_lin_vel_x = sat(pid_x_output * cos(heading * (M_PI / 180.0f)) + pid_y_output * cos((heading - 90.0f) * (M_PI / 180.0f)), sqrt(max_lin_vel));
        cmd_lin_vel_y = sat(pid_y_output * cos(heading * (M_PI / 180.0f)) + pid_x_output * cos((heading + 90.0f) * (M_PI / 180.0f)), sqrt(max_lin_vel));


        // set to variable to publish
        cmd_lin_vel_z = pid_z_output_sat;
        
        if (rotate) {
            cmd_lin_vel_a = yaw_rate;
        }
        else {
            cmd_lin_vel_a = 0;
        }



        // publish speeds
        msg_cmd.linear.x = cmd_lin_vel_x;
        msg_cmd.linear.y = cmd_lin_vel_y;
        msg_cmd.linear.z = cmd_lin_vel_z;
        msg_cmd.angular.z = cmd_lin_vel_a;
        pub_cmd.publish(msg_cmd);

        //// IMPLEMENT /////
        pos_x_err_prev = pos_x_err;
        pos_y_err_prev = pos_y_err;
        pos_z_err_prev = pos_z_err;

        //pid_x_output_sat_prev = pid_x_output_sat;
        //pid_y_output_sat_prev = pid_y_output_sat;
        pid_z_output_sat_prev = pid_z_output_sat;

        // verbose
        if (verbose)
        {
            ROS_INFO(" HMOVE : Current position (%6.3f, %6.3f, %6.3f)" , x, y, z);
            ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) FV(%6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", target_x, target_y, target_z, cmd_lin_vel_a, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
        }

        // wait for rate
        rate.sleep();
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.linear.y = 0;
    msg_cmd.linear.z = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    // disable motors
    ROS_INFO(" HMOVE : Motors Disabled");
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);

    ROS_INFO(" HMOVE : ===== END =====");

    return 0;
}