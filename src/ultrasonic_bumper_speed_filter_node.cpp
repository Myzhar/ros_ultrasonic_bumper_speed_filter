#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>

#include <geometry_msgs/Twist.h>

#include <ros_ultrasonic_bumper/ultrasnd_bump_ranges.h>

using namespace std;
using namespace ros_ultrasonic_bumper;

// >>>>> Global functions
void loadParams();
void twist_msg_callback();
void bumper_msg_callback();
// <<<<< Global functions

// >>>>> Global variables
ros::NodeHandle* nh;
ros::NodeHandle* nhPriv;

double stop_range;
double secure_range;

bool new_twist;
ros::Publisher* twist_filt_pub_ptr;
// <<<<< Global variables

#define DEFAULT_STOP_RANGE      0.1
#define DEFAULT_SECURE_RANGE    1.0

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_bumper_node");

    ROS_INFO_STREAM("-----------------------------------------\r");
    ROS_INFO_STREAM("  Ultrasonic Bumper Board speed filter   \r");
    ROS_INFO_STREAM("-----------------------------------------\r");

    nh = new ros::NodeHandle(); // Node
    nhPriv = new ros::NodeHandle( "~" ); // Private node to load parameters

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    new_twist = false;

    // >>>>> Output message
    ros::Publisher twist_filt_pub_ptr = nh->advertise<ultrasnd_bump_ranges>( "twist_filtered", 1, false );
    // <<<<< Output message

    // Load parameters from param server
    loadParams();

    while( ros::ok() )
    {

        ros::spinOnce();
    }

    ROS_INFO_STREAM("... stopped!");

    return EXIT_SUCCESS;
}

void loadParams()
{
    ROS_INFO_STREAM( "Loading parameters from server" );

    if( !nhPriv->getParam( "stop_range", stop_range ) )
    {
        stop_range = DEFAULT_STOP_RANGE;
        nhPriv->setParam( "stop_range", stop_range );
        ROS_INFO_STREAM( "stop_range" << " not present. Default value set: " << stop_range );
    }
    else
        ROS_DEBUG_STREAM( "stop_range: " << stop_range );

    if( !nhPriv->getParam( "secure_range", secure_range ) )
    {
        secure_range = DEFAULT_SECURE_RANGE;
        nhPriv->setParam( "secure_range", secure_range );
        ROS_INFO_STREAM( "secure_range" << " not present. Default value set: " << secure_range );
    }
    else
        ROS_DEBUG_STREAM( "secure_range: " << secure_range );
}

void twist_msg_callback()
{
    new_twist = true;
}

void bumper_msg_callback()
{
    if( new_twist )
    {

        new_twist = false;
    }
}
