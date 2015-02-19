//Must include this for all ROS cpp projects
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
//Including the Float32 class from std_msgs
#include <std_msgs/Float32.h>
// boolean message time
#include <std_msgs/Bool.h>
// we need to be able to math!
#include <cmath>


// set alarm if anything is within 0.5m of the front of robot
const double MIN_SAFE_DISTANCE = 2;
const double ALLOWABLE_CLOSE_PINGS = 2;

// these values to be set within the laser callback
// global var to hold length of a SINGLE LIDAR ping--in front
double ping_dist_in_front_ = 3.0;
// NOT real; callback will have to find this
int ping_index_ = -1;
int ping_low_limit_index_ = -1;
int ping_high_limit_index_ = -1;
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
double points_per_degree = -1;
bool laser_alarm_ = false;

double pi = 3.14159653;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;

double rad_to_deg(float rad)
{
    return rad / pi * 180;
}

double deg_to_rad(float deg)
{
	return deg / 180 * pi;
}

double index_to_deg_off_x(int index)
{
	return (index - ping_index_) / points_per_degree;
}

double index_to_rad_off_x(int index)
{
	return deg_to_rad(index_to_deg_off_x(index));
}

void laserCallback(const sensor_msgs::LaserScan &laser_scan)
{
    if (ping_index_ < 0)
    {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int)((angle_max_ - angle_min_) / angle_increment_) / 2;
        points_per_degree = ((ping_index_ * 2) / rad_to_deg(angle_max_ - angle_min_));
        int consider_pings_one_side = points_per_degree * 30;
        ping_low_limit_index_ = ping_index_ - consider_pings_one_side;
        ping_high_limit_index_ = ping_index_ + consider_pings_one_side;

        // Print out some debug info
        ROS_INFO("LIDAR setup: ping_index = %d", ping_index_);
        ROS_INFO("LIDAR setup: angle_min = %lfdeg", rad_to_deg(angle_min_));
        ROS_INFO("LIDAR setup: angle_max = %lfdeg", rad_to_deg(angle_max_));
        ROS_INFO("LIDAR setup: angle_inc = %lfdeg", rad_to_deg(angle_increment_));
        ROS_INFO("LIDAR setup: points per degree = %lf", points_per_degree);
        ROS_INFO("LIDAR setup: checking indicies %d through %d", ping_low_limit_index_, ping_high_limit_index_);
    }

    ping_dist_in_front_ = laser_scan.ranges[ping_index_];
    int too_close_count = 0;
    for (int i = ping_low_limit_index_; i < ping_high_limit_index_; i++)
    {
    	double ping_dist_in_front_ = laser_scan.ranges[i];
    	double cos_distance = ping_dist_in_front_ * std::cos(index_to_rad_off_x(i));
    	if (!std::isinf(ping_dist_in_front_))
    	{
		    //ROS_INFO("ping dist %lf at angle %lfdeg", ping_dist_in_front_, index_to_deg_off_x(i));
    	}
        if (cos_distance < MIN_SAFE_DISTANCE)
        {
            ROS_WARN("There appears to be something in front of us, %lfdeg off center, %lfm away in x", index_to_deg_off_x(i), cos_distance);
            if (++too_close_count > ALLOWABLE_CLOSE_PINGS)
            {
                ROS_WARN("DANGER, WILL ROBINSON!!");
                laser_alarm_ = true;
                break;
            }
        }
        else
        {
            laser_alarm_ = false;
        }
    }
    ping_dist_in_front_ = laser_scan.ranges[ping_index_];
    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
    std_msgs::Float32 lidar_dist_msg;
    lidar_dist_msg.data = ping_dist_in_front_;
    lidar_dist_publisher_.publish(lidar_dist_msg);
}

int main(int argc, char **argv)
{
    //name this node
    ros::init(argc, argv, "lidar_alarm");
    ros::NodeHandle nh;
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/lidar_alarm", 1);
    // let's make this global, so callback can use it
    lidar_alarm_publisher_ = pub;
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("/lidar_dist", 1);
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("/laser/scan", 1, laserCallback);
    //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    ros::spin();
    // should never get here, unless roscore dies
    return 0;
}

