#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>

#include <geometry_msgs/Twist.h>

#include <ros_ultrasonic_bumper/ultrasnd_bump_ranges.h>

using namespace std;
using namespace ros_ultrasonic_bumper;

#define MAX_RANGES 4

// >>>>> Sensors position <-> index
#define SENSOR_FL   0
#define SENSOR_FR   1
#define SENSOR_RR   2
#define SENSOR_RL   3
// <<<<< Sensors position <-> index

// >>>>> Global functions
void loadParams();
void twist_msg_callback( const geometry_msgs::Twist::ConstPtr& cmd_vel );
void bumper_msg_callback(const ultrasnd_bump_ranges::ConstPtr& dist);
void do_filtering();
// <<<<< Global functions

// >>>>> Global variables
ros::NodeHandle* nh;
ros::NodeHandle* nhPriv;

float stop_range;
float secure_range;

bool new_twist;
bool new_ranges;

float fw_speed;
float rot_speed;

float dist[MAX_RANGES];

ros::Publisher twist_filt_pub;
// <<<<< Global variables

#define DEFAULT_STOP_RANGE      0.1
#define DEFAULT_SECURE_RANGE    1.0

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_bumper_speed_filter_node");

    ROS_INFO_STREAM("-----------------------------------------\r");
    ROS_INFO_STREAM("  Ultrasonic Bumper Board speed filter   \r");
    ROS_INFO_STREAM("-----------------------------------------\r");

    nh = new ros::NodeHandle(); // Node
    nhPriv = new ros::NodeHandle( "~" ); // Private node to load parameters

    /*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
        ros::console::notifyLoggerLevelsChanged();
    }*/

    new_twist = false;
    new_ranges = false;

    // >>>>> Output message
    twist_filt_pub = nh->advertise<geometry_msgs::Twist>( "cmd_vel_filtered", 1, false );
    // <<<<< Output message

    // >>>>> Input messages
    static ros::Subscriber sens_sub = nh->subscribe<ultrasnd_bump_ranges>( "ranges", 1, &bumper_msg_callback );
    static ros::Subscriber vel_sub = nh->subscribe<geometry_msgs::Twist>( "cmd_vel", 1, &twist_msg_callback );
    // <<<<< Input messages

    // Load parameters from param server
    loadParams();

    ros::Rate r(100); // 10 hz

    while( ros::ok() )
    {
        ros::spinOnce();        
        r.sleep();
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

void twist_msg_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    fw_speed = cmd_vel->linear.x;
    rot_speed = cmd_vel->angular.z;

    new_twist = true;

    do_filtering();
}

void bumper_msg_callback( const ros_ultrasonic_bumper::ultrasnd_bump_ranges::ConstPtr& ranges)
{
    new_ranges = true;

    dist[SENSOR_FL] = ranges->sensor_FL.range;
    dist[SENSOR_FR] = ranges->sensor_FR.range;
    dist[SENSOR_RR] = ranges->sensor_RR.range;
    dist[SENSOR_RL] = ranges->sensor_RL.range;

    do_filtering();
}

void do_filtering()
{
    if( new_twist && new_ranges )
    {
        float range_left;
        float range_right;

        float fw_filtered;
        float rot_filtered;

        // >>>>> Robot is going forward or backward?
        if( fw_speed >= 0 )
        {
            range_left = dist[SENSOR_FL];
            range_right = dist[SENSOR_FR];
        }
        else
        {
            // To avoid mismatching we consider the robot
            // as if it's going forward, with the rear as front part
            range_left = dist[SENSOR_RR];
            range_right = dist[SENSOR_RL];
        }
        // <<<<< Robot is going forward or backward?

        // >>>>> Filtering forward speed

        // We take the minimum range measured by the two front sensor
        float ref_range = std::min( range_left, range_right );

        if( ref_range > secure_range )
        {
            fw_filtered = fw_speed;
        }
        else if( ref_range < stop_range )
        {
            fw_filtered = 0.0;
        }
        else
        {
            float factor = (ref_range-stop_range)/(secure_range-stop_range);
            fw_filtered = factor*fw_speed;
        }
        // <<<<< Filtering forward speed

        // >>>>> Filtering rotation speed
        float rot_sign;

        if( ref_range < stop_range )
        {
            rot_filtered = 0;
        }
        else
            rot_filtered = rot_speed;
        // <<<<< Filtering rotation speed


        new_twist = false;
        new_ranges = false;

        // >>>>> Filtered speed publishing
        geometry_msgs::Twist cmd_vel_filtered;

        cmd_vel_filtered.linear.x = fw_filtered;
        cmd_vel_filtered.linear.y = 0.0;
        cmd_vel_filtered.linear.z = 0.0;
        cmd_vel_filtered.angular.x = 0.0;
        cmd_vel_filtered.angular.y = 0.0;
        cmd_vel_filtered.angular.z = rot_filtered;

        twist_filt_pub.publish( cmd_vel_filtered );
        // <<<<< Filtered speed publishing
    }
}
