#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"
#include <string>
#include <fstream>

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

double smoothForZeroAngError(double pid_output_vel, double ang_err) {
    double temp;
    if (ang_err > (M_PI / 6.0f)) {
    	temp = 0;
    }
    else if (ang_err < (-1.0f * (M_PI / 6.0f))) {
    	temp = 0;
    }
    else if (ang_err <= (M_PI / 6.0f) && ang_err >= 0) {
    	temp = (M_PI - ang_err) / M_PI;
    }
    else if (ang_err >= (-1.0f * (M_PI / 6.0f)) && ang_err < 0) {
    	temp = (ang_err - (-1.0f * M_PI)) / M_PI;
    }
    
    return (pid_output_vel * temp);
}

double biDirectionalAngError(double ang_err, double *direction) {
    if (ang_err >= 0 + 1e-5 && ang_err < (M_PI / 2.0f) - 1e-5) {
        *direction = 1;
        return ang_err;
    }

    else if (ang_err <= M_PI - 1e-5 && ang_err >= (M_PI / 2) + 1e-5) {
        *direction = -1;
        return -1.0f * (M_PI - ang_err);
    }

    else if (ang_err < 0 - 1e-5 && ang_err > (-1.0f * M_PI / 2.0f) + 1e-5) {
        *direction = 1;
        return ang_err;
    }

    else {
        *direction = -1;
        return  (ang_err - (-1.0f * M_PI));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;
    std::ofstream data_file;
    
    double begin = ros::Time::now().toSec();

    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");

    //std::string data_filename = "/home/ducanh/team13/";
    //data_filename = data_filename + std::to_string(Kp_lin) + "_" + std::to_string(Kd_lin) + "_" + std::to_string(Kp_ang) + "_" + std::to_string(Kd_ang) + ".txt"; 
    //data_file.open(data_filename);

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    
    double pos_err; // positional error of robot
    double pos_err_prev = 0; // previous positional error of robot
    
    double ang_err; // angular error of robot
    double ang_err_prev = 0; // previous angular error of robot
    
    // variables for positional pid
    double pid_pos_p_term = 0;
    double pid_pos_i_accum = 0;
    double pid_pos_i_term = 0;
    double pid_pos_d_term = 0;
    
    // variables for angular pid
    double pid_ang_p_term = 0;
    double pid_ang_i_accum = 0;
    double pid_ang_i_term = 0;
    double pid_ang_d_term = 0;
    
    // intermediate pid outputs
    double pid_pos_output;
    double pid_ang_output;
    
    // saturated pid outputs
    double pid_pos_output_sat;
    double pid_pos_output_sat_prev = 0;
    
    double pid_ang_output_sat;
    double pid_ang_output_sat_prev = 0;
    
    // acceleration variables for pid saturation
    double acc_lin_est;
    double acc_lin_est_sat;
 
    double acc_ang_est;
    double acc_ang_est_sat;

    // direction of motion according to bi-directional pid
    double direction = 1;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            
            // calculate positional ang angular error
            pos_err = sqrt((target.y - pos_rbt.y) * (target.y - pos_rbt.y) + (target.x - pos_rbt.x) * (target.x - pos_rbt.x));
            ang_err = limit_angle(atan2((target.y - pos_rbt.y), (target.x - pos_rbt.x)) - ang_rbt);
            
            // put a function here to re-calculate angular error
            ang_err = biDirectionalAngError(ang_err, &direction);

            // calculate positional pid terms
            pid_pos_p_term = Kp_lin * pos_err;
            pid_pos_i_accum += pos_err * dt;
            pid_pos_i_term = Ki_lin * pid_pos_i_accum;
            pid_pos_d_term = Kd_lin * (pos_err - pos_err_prev);
            
            // calculate angular pid terms
            pid_ang_p_term = Kp_ang * ang_err;
            pid_ang_i_accum += ang_err * dt;
            pid_ang_i_term = Ki_ang * pid_ang_i_accum;
            pid_ang_d_term = Kd_ang * (ang_err - ang_err_prev);
            
            // intermediate pid outputs
            pid_pos_output = smoothForZeroAngError((pid_pos_p_term + pid_pos_i_term + pid_pos_d_term), ang_err) * direction;
            pid_ang_output = pid_ang_p_term + pid_ang_i_term + pid_ang_d_term;
            
            // positional pid saturation calculation
            acc_lin_est = (pid_pos_output - pid_pos_output_sat_prev) / dt;
            acc_lin_est_sat = sat(acc_lin_est, max_lin_acc);
            pid_pos_output_sat = sat(pid_pos_output_sat_prev + (acc_lin_est_sat * dt), max_lin_vel);
            
            // angular pid saturation calculation
            acc_ang_est = (pid_ang_output - pid_ang_output_sat_prev) / dt;
            acc_ang_est_sat = sat(acc_ang_est, max_ang_acc);
            pid_ang_output_sat = sat(pid_ang_output_sat_prev + (acc_ang_est_sat * dt), max_ang_vel);
            
            
            // set to cmd_lin anf cmd_ang variables
            cmd_lin_vel = pid_pos_output_sat;
            cmd_ang_vel = pid_ang_output_sat;

            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);
            
            // set previous error to current value
            pos_err_prev = pos_err;
            ang_err_prev = ang_err;
            
            pid_pos_output_sat_prev = pid_pos_output_sat;
            pid_ang_output_sat_prev = pid_ang_output_sat;


            //data_file << ros::Time::now().toSec() - begin << "\t" << pos_err << "\t" << ang_err << std::endl;
            // verbose
            if (verbose)
            {
                //ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
                ROS_INFO("Pos Err = (%6.3f)   Ang Err= (%6.3f)", pos_err, ang_err);
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);
    data_file.close();

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}
