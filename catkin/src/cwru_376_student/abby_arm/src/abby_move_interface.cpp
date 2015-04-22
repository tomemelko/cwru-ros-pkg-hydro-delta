// simple_marker_listener.cpp
// Wyatt Newman
// node that listens on topic "marker_listener" and prints pose received

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <irb120_kinematics.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

//callback to subscribe to marker state
Eigen::Vector3d goal_position;
Vectorq6x1 current_position;

//geometry_msgs::Quaternion g_quat; // global var for quaternion
Eigen::Quaterniond g_quat;
Eigen::Matrix3d goal_rotation;
Eigen::Affine3d goal_hand_pose;
bool g_trigger = false;

tf::TransformListener* g_tfListener;
tf::StampedTransform g_armlink1_wrt_baseLink;
geometry_msgs::PoseStamped g_marker_pose_in;
geometry_msgs::PoseStamped g_marker_pose_wrt_arm_base;

using namespace std;

void poseListenerCB(const geometry_msgs::PoseStamped &feedback) {
    ROS_INFO_STREAM("Goal pose is now at "
                    << feedback.pose.position.x << ", " << feedback.pose.position.y
                    << ", " << feedback.pose.position.z);
    ROS_INFO_STREAM("marker frame_id is " << feedback.header.frame_id);
    g_marker_pose_in.header = feedback.header;
    g_marker_pose_in.pose = feedback.pose;
    g_tfListener->transformPose("link1", g_marker_pose_in, g_marker_pose_wrt_arm_base);

    // For final project, create a Pose publisher, subscribe to that here, and use that information in this class to make our goal position for home/goal/beer first/etc.

    goal_position[0] = g_marker_pose_wrt_arm_base.pose.position.x;
    goal_position[1] = g_marker_pose_wrt_arm_base.pose.position.y;
    goal_position[2] = g_marker_pose_wrt_arm_base.pose.position.z;
    g_quat.x() = g_marker_pose_wrt_arm_base.pose.orientation.x;
    g_quat.y() = g_marker_pose_wrt_arm_base.pose.orientation.y;
    g_quat.z() = g_marker_pose_wrt_arm_base.pose.orientation.z;
    g_quat.w() = g_marker_pose_wrt_arm_base.pose.orientation.w;
    goal_rotation = g_quat.matrix();
}

//storing current position into current_position
void jointStateCB(const sensor_msgs::JointStatePtr &js_msg) {
    for (int i = 0; i < 6; i++)
    {
        current_position[i] = js_msg->position[i];
    }
}

bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    // boring, but valid response info
    response.resp = true;

    // grab the most recent IM data and repackage it as an Affine3 matrix to set a target hand pose;
    goal_hand_pose.translation() = goal_position;
    goal_hand_pose.linear() = goal_rotation;
    cout << "goal_position: " << goal_position.transpose() << endl;
    cout << "R: " << endl;
    cout << goal_rotation << endl;

    //inform "main" that we have a new goal!
    g_trigger = true;

    return true;
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void stuff_trajectory( Vectorq6x1 qvec, trajectory_msgs::JointTrajectory &new_trajectory) {

    //creating new variables, so three total trajectories
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2;

    //clear points in the new trajectory
    new_trajectory.points.clear();
    new_trajectory.joint_names.clear();

    //naming the joints
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");

    new_trajectory.header.stamp = ros::Time::now();

    //clear positions in these trajectories
    trajectory_point1.positions.clear();
    trajectory_point2.positions.clear();
    //fill in the points of the trajectory: initially, all home angles
    for (int ijnt = 0; ijnt < 6; ijnt++)
    {
        // stuff in position commands for 6 joints
        trajectory_point1.positions.push_back(current_position[ijnt]);
        //should also fill in trajectory_point.time_from_start
        // stuff in position commands for 6 joints
        trajectory_point2.positions.push_back(qvec[ijnt]);
    }

    //tell robot be at the final position at this time, but we want to give it multiple time durations as it is moving, creating a trapezoidal profile
    trajectory_point1.time_from_start = ros::Duration(0);
    trajectory_point2.time_from_start = ros::Duration(6.0);

    // add this single trajectory point to the trajectory vector
    new_trajectory.points.push_back(trajectory_point1);
    // append this point to trajectory
    new_trajectory.points.push_back(trajectory_point2);
}

int main(int argc, char** argv) {
    // this will be the node name;
    ros::init(argc, argv, "abby_move_interface");
    ros::NodeHandle nh;

    // Setup publisher
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);

    // Setup subscriber
    ROS_INFO("setting up subscribers ");
    ros::Subscriber sub_js = nh.subscribe("/joint_states", 1, jointStateCB);
    ros::Subscriber sub_im = nh.subscribe("/goal_pose", 1, poseListenerCB);

    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);

    Eigen::Vector3d p;
    Eigen::Vector3d n_des, t_des, b_des;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    // 10Hz update rate
    ros::Rate sleep_timer(10.0);

    //instantiate forward and IK solvers
    Irb120_fwd_solver irb120_fwd_solver;
    Irb120_IK_solver ik_solver;
    Eigen::Vector3d n_urdf_wrt_DH, t_urdf_wrt_DH, b_urdf_wrt_DH;
    // in home pose, R_urdf = I
    //DH-defined tool-flange axes point as:
    // z = 1,0,0
    // x = 0,0,-1
    // y = 0,1,0
    // but URDF frame is R = I
    // so, x_urdf_wrt_DH = z_DH = [0;0;1]
    // y_urdf_wrt_DH = y_DH = [0;1;0]
    // z_urdf_wrt_DH = -x_DH = [-1; 0; 0]
    // so, express R_urdf_wrt_DH as:
    n_urdf_wrt_DH << 0, 0, 1;
    t_urdf_wrt_DH << 0, 1, 0;
    b_urdf_wrt_DH << -1, 0, 0;
    Eigen::Matrix3d R_urdf_wrt_DH;
    R_urdf_wrt_DH.col(0) = n_urdf_wrt_DH;
    R_urdf_wrt_DH.col(1) = t_urdf_wrt_DH;
    R_urdf_wrt_DH.col(2) = b_urdf_wrt_DH;

    // an empty trajectory
    trajectory_msgs::JointTrajectory new_trajectory;

    //qvec << 0, 0, 0, 0, 0, 0;
    Eigen::Affine3d A_flange_des_DH;

    //create a transform listener
    g_tfListener = new tf::TransformListener;

    // wait to start receiving valid tf transforms between map and odom:
    bool tferr = true;
    ROS_INFO("waiting for tf between base_link and link1 of arm...");
    while (tferr)
    {
        tferr = false;
        try
        {
            //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame.
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            g_tfListener->lookupTransform("base_link", "link1", ros::Time(0), g_armlink1_wrt_baseLink);
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

    // from now on, tfListener will keep track of transforms

    int nsolns;

    while (ros::ok()) {
        ros::spinOnce();
        if (g_trigger) {
            double smallest_sum = 10000;
            double current_sum = 0;
            int index_of_smallest_sum = 0;

            // ooh!  excitement time!  got a new tool pose goal!

            // reset the trigger
            g_trigger = false;

            //is this point reachable?
            A_flange_des_DH = goal_hand_pose;
            A_flange_des_DH.linear() = goal_hand_pose.linear() * R_urdf_wrt_DH.transpose();
            cout << "R des DH: " << endl;
            cout << A_flange_des_DH.linear() << endl;
            nsolns = ik_solver.ik_solve(A_flange_des_DH);

            ROS_INFO("there are %d solutions", nsolns);

            if (nsolns == 1)
            //if only one solution, then use that solution
            {
                ik_solver.get_solns(q6dof_solns);
                qvec = q6dof_solns[0];
                stuff_trajectory(qvec, new_trajectory);

                pub.publish(new_trajectory);
            }
            else if (nsolns > 1)
            //pick solution with smallest angles abby should move
            {
                ik_solver.get_solns(q6dof_solns);

                for (int i = 0; i < nsolns; i++)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        current_sum += abs(q6dof_solns[i](j, 0)) * ((6 - j) * 20);
                    }
                    if (i == 0)
                    {
                        //go ahead and make the first solution the smallest soluion
                        smallest_sum = current_sum;
                    }
                    else
                    {
                        if (current_sum < smallest_sum)
                        {
                            //keep track of which index has the smallest sum
                            smallest_sum = current_sum;
                            index_of_smallest_sum = i;
                        }
                    }
                    current_sum = 0;
                }
                ROS_INFO("using %d index", index_of_smallest_sum);
                qvec = q6dof_solns[index_of_smallest_sum];

                stuff_trajectory(qvec, new_trajectory);

                pub.publish(new_trajectory);
            }
        }
        sleep_timer.sleep();
    }
    return 0;
}