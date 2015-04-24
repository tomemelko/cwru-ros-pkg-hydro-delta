//delta_interactive_path_sender.cpp:
// responds to 2D Nav Goal inputs from Rviz;
// add points to nav path message (and mark these in rviz)
// clear or transmit message upon command
// all pts in map frame

// this header incorporates all the necessary #include files and defines the class "DeltaInteractivePathSender"
#include "delta_interactive_path_sender.h"

//CONSTRUCTOR:
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
DeltaInteractivePathSender::DeltaInteractivePathSender(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of DeltaInteractivePathSender");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
    initializeServiceClients();
}

//member helper function to set up subscribers;
void DeltaInteractivePathSender::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    nav_goal_callback_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DeltaInteractivePathSender::navGoalCallback,this);  
    minimal_subscriber_ = nh_.subscribe("example_topic", 1, &DeltaInteractivePathSender::myCallback,this);  
    //minimal_subscriber_ = nh_.subscribe<std_msgs::Float32>("example_topic", 1, boost::bind(DeltaInteractivePathSender::myCallback,_1);      
    //ros::Subscriber sub = n.subscribe<atlas_msgs::AtlasState>("atlas/atlas_state", 1, boost::bind(getStateCB, _1, &solver, both_pub, left_pub, right_pub));    
}

//member helper function to set up publishers;
void DeltaInteractivePathSender::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    nav_path_pub_ = nh_.advertise<std_msgs::Bool>("pathMaker/test_pub", 1, true); 
}

//member helper function to set up services:
void DeltaInteractivePathSender::initializeServices()
{
    ROS_INFO("Initializing Services");
    nav_trigger_service_ = nh_.advertiseService("pathMaker/trigger",
                                                   &DeltaInteractivePathSender::triggerCallback,
                                                   this);   
}

// Initialize service clients
void DeltaInteractivePathSender::initializeServiceClients()
{
    ROS_INFO("Initializing Service Clients");
    append_client_ = nh_.serviceClient<cwru_srv::path_service_message>("appendPathService");
    flush_client_ = nh_.serviceClient<cwru_srv::simple_bool_service_message>("flushPathService");

}


//member function implementation for a subscriber callback function
void DeltaInteractivePathSender::navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr navPoseMsg)
{   
    // Flush previous messages from path
    flush_message.request.req = true;
    flush_client_.call(flush_message);

    // Add our new goal and header to the path
    ROS_INFO("navGoalCallback: received a navPoseMsg");
    vertex.header.frame_id =  navPoseMsg->header.frame_id.c_str();
    vertex.header.stamp = ros::Time::now();
    vertex.pose.position.x = navPoseMsg->pose.position.x;
    vertex.pose.position.y = navPoseMsg->pose.position.y;
    vertex.pose.orientation = navPoseMsg->pose.orientation;
    path_message.request.path.poses.push_back(vertex);
    append_client_.call(path_message);

}


void DeltaInteractivePathSender::myCallback(const std_msgs::Float32ConstPtr& message_holder) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "topic1"
    // Since this function is prompted by a message event, it does not consume CPU time polling for new data
    // the ROS_INFO() function is like a printf() function, except
    // it publishes its output to the default rosout topic, which prevents
    // slowing down this function for display calls, and it makes the
    // data available for viewing and logging purposes
    ROS_INFO("myCallback activated...");
   // ROS_INFO("received value is: %f", message_holder.data);
    //really could do something interesting here with the received data...but all we do is print it
}

//member function implementation for a service callback function
bool DeltaInteractivePathSender::triggerCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("path goal trigger callback activated");
    response.resp = true; // not really useful in this case--and not necessary
    return true;
}

double DeltaInteractivePathSender::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "DeltaInteractivePathSender");

    ros::NodeHandle nh; 
    
    std_msgs::Bool bool_msg;
    bool_msg.data = true;

    ROS_INFO("main: instantiating an object of type DeltaInteractivePathSender");
    DeltaInteractivePathSender pathMaker(&nh);  //instantiate a pathMaker object and pass in pointer to nodehandle
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    
    // Service that sends our nav_goals
    

    while(ros::ok()) {
        pathMaker.nav_path_pub_.publish(bool_msg); // this works (made nav_path_pub_ a public function)
        ros::spinOnce();
    }
    return 0;
} 

