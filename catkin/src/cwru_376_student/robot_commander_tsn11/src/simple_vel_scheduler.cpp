
// try this, e.g. with roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
// or:  roslaunch cwru_376_launchers stdr_glennan_2.launch
// watch resulting velocity commands with: rqt_plot /robot0/cmd_vel/linear/x (or jinx/cmd_vel...)

//intent of this program: modulate the velocity command to comply with a speed limit, v_max,
// acceleration limits, +/-a_max, and come to a halt gracefully at the end of
// an intended line segment

// notes on quaternions:
/*
From:
http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/

qx = ax * sin(angle/2)
qy = ay * sin(angle/2)
qz = az * sin(angle/2)
qw = cos(angle/2)


so, quaternion in 2-D plane (x,y,theta):
ax=0, ay=0, az = 1.0

qx = 0;
qy = 0;
qz = sin(angle/2)
qw = cos(angle/2)

therefore, theta = 2*atan2(qz,qw)
*/

//Simple_velocity_scheduler

#include <simple_vel_scheduler.h>
#define PI 3.14159265

// set some dynamic limits
const double v_max = 0.6; //1m/sec is a slow walk
const double v_min = 0.1; // if command velocity too low, robot won't move
const double a_max = 0.3; //1m/sec^2 is 0.1 g's
//const double a_max_decel = 0.1; // TEST
const double omega_max = 0.6; //1 rad/sec-> about 6 seconds to rotate 1 full rev
const double alpha_max = 0.3; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
const double DT = 0.050; // choose an update rate of 20Hz; go faster with actual hardware
const double emergency_slow_down_fudge_factor = 1.2;
const double fudge_factor_allowable_distance = 0.05;

//Variables to store the motorsEnabled information
bool motorsEnabled;
bool motorsEnabled_ = true; //global variable to store motorsEnabled status

string check;

//Variables to store the lidar alarm information
bool lidar_alarm;
bool lidar_alarm_ = false; //global variable to store lidar status
string lidar_check;

//Soft_stop variables
bool soft_stop_ = false;  //in future add callback message


//arrays that hold the segment and turn information for plotting the course
double segments [] = {4.8, 0.0, 12.2, 0.0, 8, 0.0}; //variable to store movement segments
double turns [] = {0.0, PI / 2, 0.0, PI / 2, 0.0, PI / 2}; //variable to store turn segments
int segment_count = 6;
int counter = 1; //counts through segment and turn arrays


//starting at first values in array
double segment_length = segments[0];
double angle_rotation = turns[0];
double segment_length_done = 0;
double angle_turn_done = 0;

///////////////////////////////////////////////////////////////////////////
// Do some setup!
///////////////////////////////////////////////////////////////////////////
double T_accel = v_max / a_max; //...assumes start from rest
double T_decel = v_max / a_max; //(for same decel as accel); assumes brake to full halt
double dist_accel = 0.5 * a_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
double dist_decel = 0.5 * a_max * (T_decel * T_decel);; //same as ramp-up distance
double dist_const_v = 0;

//compute some properties of trapezoidal rotation profile plan
double T_accel_alpha = omega_max / alpha_max;
double T_decel_alpha = omega_max / alpha_max;
double rot_accel = 0.5 * alpha_max * (T_accel_alpha * T_accel_alpha);
double rot_decel = 0.5 * alpha_max * (T_decel_alpha * T_decel_alpha);
double rot_const_omega = 0;

//store motorsEnabled information in global variable
void motorsEnabledCallback(const std_msgs::Bool::ConstPtr &motorsEnabled)
{
    if (motorsEnabled->data == true)
    {
        check = "motorsEnabled_on";  // means motors are ENABLED
        motorsEnabled_ = true;
    }
    else if (motorsEnabled->data == false)
    {
        check = "motorsEnabled_off";  // means motors are DISABLED
        motorsEnabled_ = false;
    }


    ROS_INFO("%s", check.c_str());
}

//store lidar information in global variable
void lidarCallback(const std_msgs::Bool &lidar_alarm)
{
    if (lidar_alarm.data == true)
    {
        lidar_check = "lidar_alarm_on";
        lidar_alarm_ = true;
    }

    else if (lidar_alarm.data == false)
    {
        lidar_check = "lidar_alarm_off";
        lidar_alarm_ = false;
    }

    ROS_INFO("%s", lidar_check.c_str());
}

//cycle through the array of distance moves
int segmentCycle(double segment [], double turn [], int i)
{
    if (i >= segment_count)
    {
        segment_length = 0;
        angle_rotation = 0;
    }
    else
    {
        segment_length = segment [i];
        angle_rotation = turn [i];
        ROS_INFO("Segment length: %f, Rotation Angle: %f", segment_length, angle_rotation);
    }
    segment_length_done = 0;
    angle_turn_done = 0;
    makeProfile();
}

void makeProfile()
{
    segment_length -= segment_length_done;
    angle_rotation -= angle_turn_done;
    dist_accel = 0.5 * a_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
    dist_decel = 0.5 * a_max * (T_decel * T_decel);; //same as ramp-up distance

    rot_accel = 0.5 * alpha_max * (T_accel_alpha * T_accel_alpha);
    rot_decel = 0.5 * alpha_max * (T_decel_alpha * T_decel_alpha);

    // Check case of triangle velocity profile
    if (dist_accel >= (segment_length / 2))
    {
        dist_accel = segment_length / 2;
        dist_decel = dist_accel;
        dist_const_v = 0;
    }
    else
    {
        dist_const_v = segment_length - (2 * dist_accel);
    }

    if (rot_accel >= (angle_rotation / 2))
    {
        rot_accel = fabs(angle_rotation / 2);
        rot_decel = rot_accel;
        rot_const_omega = 0;
    }
    else
    {
        rot_const_omega = angle_rotation - (2 * rot_accel);
    }
}

void checkAlarms(geometry_msgs::Twist &cmd_vel, double &command_velocity, double &command_omega)
{
	//begin decel to stop if lidar alarm or soft stop is on
	if (lidar_alarm_ == true || soft_stop_ == true)
	{
        makeProfile();
	    ROS_WARN("LIDAR OR SOFT STOP");
        command_velocity -= emergency_slow_down_fudge_factor * a_max * DT;
        if (command_velocity < 0)
        {
            command_velocity = 0;
        }
        command_omega -= emergency_slow_down_fudge_factor * alpha_max * DT;
        if (command_omega < 0)
        {
            command_omega = 0;
        }
	}

	if (motorsEnabled_ == false)
	{
	    ROS_WARN("ESTOP ACTIVATED");
	    command_velocity = 0.0;
	    command_omega = 0.0;
        makeProfile();
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_scheduler"); // name of this node will be "minimal_publisher1"

    ros::NodeHandle nh; // get a ros nodehandle; standard yadda-yadda

    //subscribe to motors_enabled rosmsg
    ros::Subscriber submotorsEnabled = nh.subscribe("/motors_enabled", 1, motorsEnabledCallback);

    //subscribe to lidar_alarm rosmsg
    ros::Subscriber sublidar = nh.subscribe("/lidar_alarm", 1, lidarCallback);

    //create a publisher object that can talk to ROS and issue twist messages on named topic;
    // note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.
    ros::Publisher vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
    ros::Rate rtimer(1 / DT); // frequency corresponding to chosen sample period DT; the main loop will run this fast

    geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands

    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    double command_velocity = 0;
    double command_omega = 0;

    segment_length_done = 0;
    angle_turn_done = 0;
    makeProfile();

    while (ros::ok())
    {
        ros::spinOnce();
        // Track how far we've gone
        segment_length_done += command_velocity * DT;
        angle_turn_done += command_omega * DT;

        // we're accelerating
        if (segment_length_done < dist_accel)
        {
            command_velocity += a_max * DT;
        }

        // we're constant v
        else if (segment_length_done < (dist_accel + dist_const_v))
        {
        }

        //we're decelerating
        else if (segment_length_done < (dist_accel + dist_const_v + dist_decel - fudge_factor_allowable_distance))
        {
            command_velocity -= a_max * DT;
        }
        else
        {
            command_velocity = 0;
        }

        // We're accelerating to turn
        if (fabs(angle_turn_done) < fabs(rot_accel))
        {
            command_omega += alpha_max * DT;
        }

        // We're constant omega
        else if (fabs(angle_turn_done) < (fabs(rot_accel) + fabs(rot_const_omega)))
        {
        }

        // We're deccelerating omega
        else if (fabs(angle_turn_done) < (fabs(rot_accel) + fabs(rot_const_omega) + fabs(rot_decel) - fudge_factor_allowable_distance))
        {
            command_omega -= alpha_max * DT;
        }
        else
        {
            command_omega = 0;
        }

        checkAlarms(cmd_vel, command_velocity, command_omega);
        cmd_vel.linear.x = command_velocity;
        cmd_vel.angular.z = -command_omega;

        ROS_INFO("segment_length: %f, angle_rotation: %f", segment_length, angle_rotation);
        ROS_INFO("dist travelled: %f, angle turn done: %f", segment_length_done, angle_turn_done);

        ROS_INFO("cmd vel: %f", cmd_vel.linear.x); // debug output
        ROS_INFO("cmd omega: %f", cmd_vel.angular.z); // debug output

        vel_cmd_publisher.publish(cmd_vel); // publish the command to robot0/cmd_vel
        rtimer.sleep(); // sleep for remainder of timed iteration

        if (((segment_length_done + fudge_factor_allowable_distance) >= segment_length) && (fabs(angle_turn_done) + fudge_factor_allowable_distance >= angle_rotation))
        {
            segmentCycle(segments, turns, counter++);
        }
    }
}