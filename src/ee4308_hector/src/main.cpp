#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

enum HectorState
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL
};
std::string to_string(HectorState state)
{
    switch (state)
    {
    case TAKEOFF:
        return "TAKEOFF";
    case LAND:
        return "LAND";
    case TURTLE:
        return "TURTLE";
    case START:
        return "START";
    case GOAL:
        return "GOAL";
    default:
        return "??";
    }
}

bool verbose;
double initial_x, initial_y, initial_z;
double x = NaN, y = NaN, z = NaN, a = NaN;
Position pos_hec(0,0);
Position pos_rbt(0,0);

void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    pos_hec.x = x;
    pos_hec.y = y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}
double turtle_x = NaN, turtle_y = NaN;
void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_x = p.x;
    turtle_y = p.y;

    pos_rbt.x = turtle_x;
    pos_rbt.y = turtle_y;
}
double vx = NaN, vy = NaN, vz = NaN, va = NaN;
void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    va = msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    double height;
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    double look_ahead;
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    double average_speed;
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");
    // get the final goal position of turtle
    std::string goal_str;
    double goal_x = NaN, goal_y = NaN;
    if (nh.param("/turtle/goals", goal_str, std::to_string(initial_x) + "," + std::to_string(initial_y))) // set to initial hector positions
    {
        char goal_str_tok[goal_str.length() + 1];
        strcpy(goal_str_tok, goal_str.c_str()); // to tokenise --> convert to c string (char*) first
        char *tok = strtok(goal_str_tok, " ,");
        try
        {
            while (tok != nullptr)
            {
                goal_x = strtod(tok, nullptr);
                goal_y = strtod(strtok(nullptr, " ,"), nullptr);
                tok = strtok(nullptr, " ,");
            }
            ROS_INFO(" HMAIN : Last Turtle Goal is (%lf, %lf)", goal_x, goal_y);
        }
        catch (...)
        {
            ROS_ERROR(" HMAIN : Invalid Goals: %s", goal_str.c_str());
            ros::shutdown();
            return 1;
        }
    }
    else
        ROS_WARN(" HMAIN : Param goal not found, set to %s", goal_str.c_str());

    // --------- Subscribers ----------
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);

    // --------- Publishers ----------
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world";
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    std_msgs::Bool msg_rotate;
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";

    // --------- Wait for Topics ----------
    while (ros::ok() && nh.param("run", true) && (std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");
    HectorState state = TAKEOFF;
    ros::Rate rate(main_iter_rate);

    // My variables
    int t = 0; 
    std::vector<Position> trajectory;
    Position pos_target;
    Position pos_goal(goal_x, goal_y);
    Position pos_start(initial_x, initial_y);
    double target_dt = 0.05;
    bool next_state = false; 
    bool new_trajectory = false;
    int look_ahead_t_idx = 2 * (int) ((look_ahead / average_speed) / target_dt);


    while (ros::ok() && nh.param("run", true))
    {
        // get topics
        ros::spinOnce();
        

        //// IMPLEMENT ////
        if (state == TAKEOFF)
        {
            next_state = false;
            // Disable Rotate
            msg_rotate.data = false;
            pub_rotate.publish(msg_rotate);

            // Setting trajectory points to 2m at the same start
            msg_traj.poses.clear();
            msg_target.point.x = pos_start.x;
            msg_target.point.y = pos_start.y;
            msg_target.point.z = 2;
            pub_target.publish(msg_target);

            if (abs(z - 2) < 0.1) next_state = true;

            if (next_state) {
                state = TURTLE;
                new_trajectory = true;
            }
        }
        else if (state == TURTLE)
        {   
            next_state = false;
            pos_target = pos_rbt;
            //if (dist_euc(pos_hec, pos_target) < look_ahead) next_state = true;
            
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);
            if (next_state) {
                state = GOAL;
                new_trajectory = true;
            }
        }
        else if (state == START)
        {
            next_state = false;
            pos_target = pos_start;
            if (dist_euc(pos_hec, pos_target) < look_ahead) next_state = true;
            if (!nh.param("/turtle/run", false))
            { // when the turtle reaches the final goal
                state = LAND;
                new_trajectory = false;
            }
            else{
                if (next_state) {
                    state = TURTLE;
                    new_trajectory = true;
                }
            }
        }
        else if (state == GOAL)
        {
            next_state = false;
            new_trajectory = true;
            pos_target = pos_goal;
            if (dist_euc(pos_hec, pos_target) < look_ahead) next_state = true;

            if (next_state) {
                state = START;
                new_trajectory = true;
            }   
        }
        else if (state == LAND)
        {
            next_state = false;
            new_trajectory = false;

            msg_rotate.data = false;
            pub_rotate.publish(msg_rotate);

            // Landing trajectory
            msg_traj.poses.clear();
            msg_target.point.x = pos_start.x;
            msg_target.point.y = pos_start.y;
            msg_target.point.z = 0;
            pub_target.publish(msg_target);
        }

        if (verbose)
            ROS_INFO_STREAM(" HMAIN : " << to_string(state));

        if (new_trajectory){
            trajectory.clear();
            trajectory = generate_trajectory_cubic(pos_hec, pos_target, average_speed, target_dt, vx, vy, a);       

            if (trajectory.empty())
                ROS_WARN(" HMAIN : No trajectory found!!!!");
            else 
            { 
            // publish trajectroy to trajectory topic
                msg_traj.poses.clear();
                for (Position &pos : trajectory)
                {
                    msg_traj.poses.push_back(geometry_msgs::PoseStamped()); // insert a posestamped initialised to all 0
                    msg_traj.poses.back().pose.position.x = pos.x;
                    msg_traj.poses.back().pose.position.y = pos.y;
                }
                pub_traj.publish(msg_traj);
            
                // get new target
                t = 0; // first entry
                // pick the more distant target so turtlebot does not stop intermitently around very close targets when new path is generated
                if (trajectory.size() > look_ahead_t_idx)
                    t = look_ahead_t_idx; // this is the average_speed * 15 * target_dt away
                else { 
                    t = trajectory.size() - 1;
                    new_trajectory = false;
                }
                
                pos_target = trajectory[t];

                // publish to target topic

                msg_target.point.x = pos_target.x;
                msg_target.point.y = pos_target.y;
                msg_target.point.z = 2;
                pub_target.publish(msg_target);
                ROS_WARN("Publised trajectory point: x: %f, y: %f", pos_target.x, pos_target.y);
            }

        }

        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}
