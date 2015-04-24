/** delta_interactive_path_sender.h header file **/
#ifndef DELTA_INTERACTIVE_PATH_SENDER_H_
#define DELTA_INTERACTIVE_PATH_SENDER_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <cwru_srv/simple_bool_service_message.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <nav_msgs/Path.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>

class DeltaInteractivePathSender
{
public:
    DeltaInteractivePathSender(); //constructor
    DeltaInteractivePathSender(ros::NodeHandle* nodehandle);
    ros::Publisher  nav_path_pub_;
private:
    // put member data here
    ros::NodeHandle nh_; 
    ros::Subscriber nav_goal_callback_sub_;
    ros::Subscriber minimal_subscriber_;

    ros::ServiceServer nav_trigger_service_;

    ros::ServiceClient append_client_;
    ros::ServiceClient flush_client_;

    geometry_msgs::PoseStamped vertex;
    cwru_srv::path_service_message path_message;
    cwru_srv::simple_bool_service_message flush_message;
    
    // member methods as well:
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeServiceClients();

    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);

    void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr navPoseMsg);
    //void myCallback(const std_msgs::Float32& message_holder);
    //void myCallback(const std_msgs::Float32ConstPtr& message_holder);
    void myCallback(const std_msgs::Float32ConstPtr& message_holder);
    bool triggerCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
    //void gravityTorquesCB(const hku_msgs::vectorOfDoublesWHeaderConstPtr& message_holder);
};

#endif
