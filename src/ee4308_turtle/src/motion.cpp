#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

double imu_ang_vel = -10, imu_lin_acc = 0; // unlikely to be spinning at -10 at the start
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ang_vel = msg->angular_velocity.z;
    imu_lin_acc = msg->linear_acceleration.x;
}

double wheel_l = 10, wheel_r = 10; // init as 10 bcos both are unlikely to be exactly 10 (both can start at non-zero if u reset the sim). whole doubles can also be exactly compared
void cbWheels(const sensor_msgs::JointState::ConstPtr &msg)
{
    wheel_l = msg->position[0]; 
    wheel_r = msg->position[1]; 
}

nav_msgs::Odometry msg_odom;
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_motion");
    ros::NodeHandle nh;

    // Parse ROS parameters
    bool use_internal_odom;
    if (!nh.param("use_internal_odom", use_internal_odom, true))
        ROS_WARN(" TMOVE : Param use_internal_odom not found, set to true");
    bool verbose;
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_motion not found, set to false");
    double initial_x, initial_y;
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" TMOVE : Param initial_x not found, set to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" TMOVE : Param initial_y not found, set to 0.0");

    // Publisher
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

    // Prepare published message
    geometry_msgs::PoseStamped pose_rbt;
    pose_rbt.header.frame_id = "world"; //for rviz

    if (use_internal_odom)
    { // subscribes to odom topic --> is the exact simulated position in gazebo; when used in real life, is derived from wheel encoders (no imu).
        // Subscriber
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);

        // initialise rate
        ros::Rate rate(25);

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && msg_odom.header.seq == 0) // dependent on odom
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // begin loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            // write to published message
            pose_rbt.pose = msg_odom.pose.pose;
            // uncomment the following when running on real hardware, bcos odom always starts at 0.
            // pose_rbt.pose.position.x += initial_x;
            // pose_rbt.pose.position.y += initial_y;


            // publish pose
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                // get ang_rbt from quaternion
                auto &q = pose_rbt.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)  FVel(%6.3f)  AVel(%6.3f)",
                         pose_rbt.pose.position.x, pose_rbt.pose.position.y, atan2(siny_cosp, cosy_cosp),
                         msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);
            }

            rate.sleep();
        }
    }
    else
    {
        // Parse additional ROS parameters
        Position pos_rbt(initial_x, initial_y);
        if (!nh.param("initial_x", pos_rbt.x, 0.0))
            ROS_WARN(" TMOVE : Param initial_x not found, set to 0.0");
        if (!nh.param("initial_y", pos_rbt.y, 0.0))
            ROS_WARN(" TMOVE : Param initial_y not found, set to 0.0");
        double wheel_radius;
        if (!nh.param("wheel_radius", wheel_radius, 0.033))
            ROS_WARN(" TMOVE : Param wheel_radius not found, set to 0.033");
        double axle_track;
        if (!nh.param("axle_track", axle_track, 0.16))
            ROS_WARN(" TMOVE : Param axle_track not found, set to 0.16");
        double weight_odom_v;
        if (!nh.param("weight_odom_v", weight_odom_v, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_v not found, set to 0.5");
        double weight_odom_w;
        if (!nh.param("weight_odom_w", weight_odom_w, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_w not found, set to 0.5");
        double weight_imu_v = 1 - weight_odom_v;
        double weight_imu_w = 1 - weight_odom_w;
        double straight_thresh;
        if (!nh.param("straight_thresh", straight_thresh, 0.05))
            ROS_WARN(" TMOVE : Param straight_thresh not found, set to 0.05");
        double motion_iter_rate;
        if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
            ROS_WARN(" TMOVE : Param motion_iter_rate not found, set to 50");

        // Subscribers
        ros::Subscriber sub_wheels = nh.subscribe("joint_states", 1, &cbWheels);
        ros::Subscriber sub_imu = nh.subscribe("imu", 1, &cbImu);

        // initialise rate
        ros::Rate rate(motion_iter_rate); // higher rate for better estimation

        // initialise message for publishing
        pose_rbt.pose.orientation.x = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF
        pose_rbt.pose.orientation.y = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && (wheel_l == 10 || wheel_r == 10 || imu_ang_vel == -10)) // dependent on imu and wheels
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // declare / initialise other variables
        double ang_rbt = 0; // robot always start at zero.
        double lin_vel = 0, ang_vel = 0;
        double prev_time = ros::Time::now().toSec();
        double dt = 0;
        ////////////////// DECLARE VARIABLES HERE //////////////////

        // loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION FILTER HERE //////////////////

            // publish the pose
            // inject position and calculate quaternion for pose message, and publish
            pose_rbt.pose.position.x = pos_rbt.x;
            pose_rbt.pose.position.y = pos_rbt.y;
            pose_rbt.pose.orientation.w = cos(ang_rbt / 2);
            pose_rbt.pose.orientation.z = sin(ang_rbt / 2);
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)",
                         pos_rbt.x, pos_rbt.y, ang_rbt);
            }

            // sleep until the end of the required frequency
            rate.sleep();
        }
    }

    ROS_INFO("TMOTION: ===== END =====");
    return 0;
}