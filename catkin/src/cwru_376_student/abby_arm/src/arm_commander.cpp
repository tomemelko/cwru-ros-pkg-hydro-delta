// process_pcd_dev.cpp: wsn, April, 2015
// example code to acquire a pointcloud from disk, then perform various processing steps interactively.
// processing is invoked by point-cloud selections in rviz, as well as "mode" settings via a service
// e.g.:  rosservice call process_mode 0 induces processing in mode zero (plane fitting)
// adding more code for cylinder registration


#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>

#include <cwru_srv/simple_int_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // can use this for short-hand

using namespace std;

//define some processing modes; set these interactively via service
const int HOME = 0;
const int MID1 = 1;
const int MID2 = 2;
const int HOVER = 3;
const int GRAB_POSITION = 4;
const int GRAB = 5;
const int UP = 10;
const int DOWN = 11;

int g_process_mode = 0; // mode--set by service
bool g_trigger = false; // a trigger, to tell "main" to process points in the currently selected mode
bool g_can_found = false;

//more globals--to share info on planes and patches
geometry_msgs::PoseStamped goal_pose;
geometry_msgs::PoseStamped can_location;
tf::StampedTransform tf_from_can_to_arm;
tf::TransformListener* g_tfListener;

//use this service to set processing modes interactively
bool modeService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
    ROS_INFO("mode select service callback activated");
    response.resp = true; // boring, but valid response info
    g_process_mode = request.req;
    g_trigger = true; //signal that we received a request; trigger a response
    cout << "Mode set to: " << g_process_mode << endl;
    return true;
}

void canCB(const geometry_msgs::PoseStampedConstPtr& location) {
    g_can_found = true; // update our states to note that we have process a patch, and thus have valid plane info
    ROS_INFO("raw can_location: x: %lf y: %lf z: %lf", location->pose.position.x, location->pose.position.y, location->pose.position.z);
    g_tfListener->transformPose("/base_link", *location, can_location);
    ROS_INFO("can_location: x: %lf y: %lf z: %lf", can_location.pose.position.x, can_location.pose.position.y, can_location.pose.position.z);
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "arm_commander");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    // Subscribers
    ros::Subscriber getCanLocation = nh.subscribe<geometry_msgs::PoseStamped> ("/can_location", 1, canCB);

    // have rviz display both of these topics
    ros::Publisher pubGoalPose = nh.advertise<geometry_msgs::PoseStamped> ("/goal_pose", 1);

    // service used to interactively change processing modes
    ros::ServiceServer service = nh.advertiseService("arm_mode", modeService);

    g_tfListener = new tf::TransformListener;

    goal_pose.header.frame_id = "base_link";
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = .703516;
    goal_pose.pose.orientation.z = 0;
    goal_pose.pose.orientation.w = .710769;
    // Set a home position
    goal_pose.pose.position.x = 0.6;
    goal_pose.pose.position.y = -0.55;
    goal_pose.pose.position.z = 1.1;

    bool tferr = true;
    ROS_INFO("waiting for tf between camera_depth_optical_frame and base_link...");
    while (tferr)
    {
        tferr = false;
        try
        {
            //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame.
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            g_tfListener->lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), tf_from_can_to_arm);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;

            // sleep for half a second
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");

    while (ros::ok()) {
        if (!g_can_found) {
            ROS_INFO("Waiting on can location");
        }
        else if (g_trigger) {
            g_trigger = false; // reset the trigger

            switch (g_process_mode) { // what we do here depends on our mode; mode is settable via a service
            case HOME:
                goal_pose.pose.position.x = 0.6;
                goal_pose.pose.position.y = -0.55;
                goal_pose.pose.position.z = 1.1;
                break;
            case MID1:
                goal_pose.pose.position.x = 1;
                goal_pose.pose.position.y = -0.275;
                goal_pose.pose.position.z = 1.1;
                break;
            case MID2:
                goal_pose.pose.position.x = 1.25;
                goal_pose.pose.position.y = 0;
                goal_pose.pose.position.z = 1.2;
                break;
            case HOVER:
                goal_pose.pose.position.x = can_location.pose.position.x;
                goal_pose.pose.position.y = can_location.pose.position.y;
                goal_pose.pose.position.z = can_location.pose.position.z + 0.4;
                break;
            case GRAB_POSITION:
                goal_pose.pose.position.x = can_location.pose.position.x;
                goal_pose.pose.position.y = can_location.pose.position.y;
                goal_pose.pose.position.z = can_location.pose.position.z + 0.4 - .12;
                break;
            case GRAB:
                //grab here!
                break;
            case UP:
                goal_pose.pose.position.z += .01;
                break;
            case DOWN:
                goal_pose.pose.position.z -= .01;
                break;
            default:
                ROS_WARN("this mode is not implemented");

            }
        }

        pubGoalPose.publish(goal_pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}