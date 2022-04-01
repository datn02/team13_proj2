#include "common.hpp"
#include <cmath>

Index::Index() : i(0), j(0) {};
Index::Index(int i, int j) : i(i), j(j) {};
Position::Position() : x(0), y(0) {};
Position::Position(double x, double y) : x(x), y(y) {};
double sign(double value)
{
    if (value > 0)
        return 1;
    else if (value < 0)
        return -1;
    else
        return 0;
}

double sat(double input_value, double max_value) {
    if (input_value > max_value) {
    	return max_value;
    }
    else if (input_value < (-1 * max_value)) {
    	return (-1 * max_value);
    } 
    else {
    	return input_value;
    }
}

double dist_oct(Index src, Index tgt)
{
    return dist_oct(src.i, src.j, tgt.i, tgt.j);
}
double dist_oct(Position src, Position tgt)
{
    return dist_oct(src.x, src.y, tgt.x, tgt.y);
}
double dist_oct(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double abs_Dx = abs(tgt_x - src_x);
    double abs_Dy = abs(tgt_y - src_x);
    double ordinals, cardinals;
    if (abs_Dx > abs_Dy)
    {
        ordinals = abs_Dy; // ordinal (diagonal)
        cardinals = abs_Dx - abs_Dy; // cardinal (vertical or horizontal)
    }
    else
    {
        ordinals = abs_Dx;
        cardinals = abs_Dy - abs_Dx;
    }
    return ordinals * M_SQRT2 + cardinals; // M_SQRT2 is from cmath
}
double dist_euc(Index src, Index tgt)
{
    return dist_euc(src.i, src.j, tgt.i, tgt.j);
}
double dist_euc(Position src, Position tgt)
{
    return dist_euc(src.x, src.y, tgt.x, tgt.y);
}
double dist_euc(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double Dx = tgt_x - src_x;
    double Dy = tgt_y - src_y;
    return sqrt(Dx*Dx + Dy*Dy);
}
double heading(Position src, Position tgt)
{
    return heading(src.x, src.y, tgt.x, tgt.y);
}
double heading(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double Dx = tgt_x - src_x;
    double Dy = tgt_y - src_y;

    return atan2(Dy, Dx);
}
double limit_angle(double angle)
{
    // faster version, but -2PI <= angle < 2PI must be guaranteed (may not happen if any sensor readings surges temporarily due to noise and high speed, e.g. imu ang vel)
    /*
    if (angle >= M_PI)
        return angle - M_PI;
    else if (angle < -M_PI)
        return angle + M_PI;
    */

    // // slower version, but guaranteed to limit the angle in most cases
    // return fmod(angle + M_PI, M_PI*2) - M_PI;
    double result = fmod(angle + M_PI, M_PI*2); // fmod rounds remainders to zero. we want remainders to be +ve like mod() in matlab and % in python
    return result >= 0 ? result - M_PI : result + M_PI;
}



std::vector<Position> generate_trajectory_cubic(Position pos_begin, Position pos_end, double average_speed, double target_dt, double initial_vel_x, double initial_vel_y, double angle)
{

    double s_ang = limit_angle(heading(pos_begin, pos_end));
    // Tune this
    // double turn_vel = 0.1;

    double final_vel_x = average_speed * cos(s_ang);
    double final_vel_y = average_speed * sin(s_ang);

    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx * Dx + Dy * Dy) / average_speed;

    double a0 = pos_begin.x;
    double a1 = initial_vel_x;
    double a2 = ((-3 / pow(duration, 2)) * pos_begin.x) + ((-2 / duration) * initial_vel_x) + ((3 / pow(duration, 2)) * pos_end.x) + ((-1 / duration) * final_vel_x);
    double a3 = ((2 / pow(duration, 3)) * pos_begin.x) + ((1 / pow(duration, 2)) * initial_vel_x) + ((-2 / pow(duration, 3)) * pos_end.x) + ((1 / pow(duration, 2)) * final_vel_x);

    double b0 = pos_begin.y;
    double b1 = initial_vel_y;
    double b2 = ((-3 / pow(duration, 2)) * pos_begin.y) + ((-2 / duration) * initial_vel_y) + ((3 / pow(duration, 2)) * pos_end.y) + ((-1 / duration) * final_vel_y);
    double b3 = ((2 / pow(duration, 3)) * pos_begin.y) + ((1 / pow(duration, 2)) * initial_vel_y) + ((-2 / pow(duration, 3)) * pos_end.y) + ((1 / pow(duration, 2)) * final_vel_y);

    std::vector<Position> trajectory = {pos_begin};
    for (double time = target_dt; time < duration; time += target_dt)
    {
        trajectory.emplace_back(
            a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3) ,
            b0 + b1 * time + b2 * pow(time, 2) + b3 * pow(time, 3));
    }
    return trajectory;
}

std::vector<Position> generate_trajectory_quintic(Position pos_begin, Position pos_end, double average_speed, double target_dt, double initial_vel_x, double initial_vel_y, double angle)
{

    double s_ang = limit_angle(heading(pos_begin, pos_end));
    // Tune this
    // double turn_vel = 0.1;

    double final_vel_x = average_speed * cos(s_ang);
    double final_vel_y = average_speed * sin(s_ang);
    double acc_x = 0;
    double acc_y = 0;

    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx * Dx + Dy * Dy) / average_speed;

    double a0 = pos_begin.x;
    double a1 = initial_vel_x;
    double a2 = 0.5 * acc_x;
    double a3 = (-10 / pow(duration, 3) * pos_begin.x) - (6 / pow(duration, 2) * initial_vel_x) - ((3 / 2 * duration) * acc_x) + (10 / pow(duration, 3) * pos_end.x) - (4 / pow(duration, 2) * final_vel_x) + ((1 / 2 * duration) * acc_x);
    double a4 = (15 / pow(duration, 4) * pos_begin.x) + (8 / pow(duration, 3) * initial_vel_x) + (3 / 2 / pow(duration, 2) * acc_x) + (-15 / pow(duration, 4) * pos_end.x) + (7 / pow(duration, 3) * final_vel_x) + (-1 / pow(duration, 2) * acc_x);
    double a5 = (-6 / pow(duration, 5) * pos_begin.x) + (-3 / pow(duration, 4) * initial_vel_x) + (-1 / 2 / pow(duration, 3) * acc_x) + (6 / pow(duration, 5) * pos_end.x) + (-3 / pow(duration, 4) * final_vel_x) + (1 / 2 / pow(duration, 3) * acc_x);

    double b0 = pos_begin.y;
    double b1 = initial_vel_y;
    double b2 = 0.5 * acc_y;
    double b3 = (-10 / pow(duration, 3) * pos_begin.y) - (6 / pow(duration, 2) * initial_vel_y) - ((3 / 2 * duration) * acc_y) + (10 / pow(duration, 3) * pos_end.y) - (4 / pow(duration, 2) * final_vel_y) + ((1 / 2 * duration) * acc_y);
    double b4 = (15 / pow(duration, 4) * pos_begin.y) + (8 / pow(duration, 3) * initial_vel_y) + (3 / 2 / pow(duration, 2) * acc_y) + (-15 / pow(duration, 4) * pos_end.y) + (7 / pow(duration, 3) * final_vel_y) + (-1 / pow(duration, 2) * acc_y);
    double b5 = (-6 / pow(duration, 5) * pos_begin.y) + (-3 / pow(duration, 4) * initial_vel_y) + (-1 / 2 / pow(duration, 3) * acc_y) + (6 / pow(duration, 5) * pos_end.y) + (-3 / pow(duration, 4) * final_vel_y) + (1 / 2 / pow(duration, 3) * acc_y);

    std::vector<Position> trajectory = {pos_begin};
    for (double time = target_dt; time < duration; time += target_dt)
    {
        trajectory.emplace_back(
            a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3) + a4 * pow(time, 4) + a5 * pow(time, 5),
            b0 + b1 * time + b2 * pow(time, 2) + b3 * pow(time, 3) + b4 * pow(time, 4) + b5 * pow(time, 5));
    }
    return trajectory;
}

std::vector<Position> generate_trajectory_straight(Position pos_begin, Position pos_end, double average_speed, double target_dt)
{
    // (1) estimate total duration
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx*Dx + Dy*Dy) / average_speed;

    // (2) generate cubic / quintic trajectory
    // done by students

    // OR (2) generate targets for each target_dt
    std::vector<Position> trajectory = {pos_begin};
    for (double time = target_dt; time < duration; time += target_dt)
    {
        trajectory.emplace_back(
            pos_begin.x + Dx*time / duration,
            pos_begin.y + Dy*time / duration
        );
    }

    return trajectory; 
}