#include "ros/ros.h"
#include "bayucaraka/poss.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <cmath>
#define PI 3.141592654

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
geometry_msgs::PoseStamped pose;
geometry_msgs::Twist msg;
mavros_msgs::State current_state;
double xnya, ynya, znya, inRad;
float x, y, vel_resultan = 0;
int stts;

tf2::Quaternion mengrotate;

double pitagoras(double a, double b){
    return sqrt(a*a + b*b);
}

void gerak(double xTujuan, double yTujuan){
    double jarakNow = pitagoras(xTujuan-xnya, yTujuan-ynya);
    // sudutnya akan selalu berubah
    double sudut = (yTujuan-ynya) / (jarakNow);
    double inDegree = asin(sudut)*180/PI;

    // sudutnya itu 0 sampe 180 lalu -180 sampe 0
    if(yTujuan > ynya && xTujuan < xnya) inDegree = 180-inDegree;
    if(yTujuan < ynya && xTujuan < xnya) inDegree = -180-inDegree;
    double inRad = inDegree * PI / 180;

    // dengan adanya informasi sudut dari tujuan, bisa mencari v_x dan v_y
    msg.linear.x = vel_resultan*cos(inRad);
    msg.linear.y = vel_resultan*sin(inRad);

    local_vel_pub.publish(msg);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void chatterCallback(const bayucaraka::poss::ConstPtr& msg)
{
    stts = msg->status;
    vel_resultan = msg->vel_res;
    x = msg->pos_x;
    y = msg->pos_y;
}

void callBack(const geometry_msgs::PoseStamped::ConstPtr& oke){
    xnya = oke->pose.position.x;
    ynya = oke->pose.position.y;
    znya = oke->pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mengDrone");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("finalBayucaraka", 1000, chatterCallback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 15, state_cb);
    ros::Subscriber inpo = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 15, callBack);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 15);
    local_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 15);        
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // jika udah diposisi nya break dari loopingan
        if(current_state.mode == "OFFBOARD" && current_state.armed && znya >= 0.5) break;
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    int rot = 0;
    while(ros::ok()){
        ros::Rate loop_rate(15);

        if(stts == 1){
            // drone naik
            pose.pose.position.z += 0.05;
        } else if(stts == 2){
            // drone turun
            pose.pose.position.z -= 0.05;
        } else if(stts == 4){
        //     // gerak melingkar
            rot+=1;
            if(rot > 180) rot -=360;
            gerak((10*cos(rot*PI/180)),(10*sin(rot*PI/180)));
        }else if(stts == 5){
            // gerak mengikuti tangan (khususnya jari tengah)
            gerak(x,y);
        }
        if(stts != 1 && stts != 2){
            pose.pose.position.z = znya;
        }

        // eular to quartenion
        // https://eater.net/quaternions
        // http://wiki.ros.org/tf2/Tutorials/Quaternions
        mengrotate.setRPY(0, 0 , inRad);
        mengrotate = mengrotate.normalize();
        pose.pose.position.x = xnya;
        pose.pose.position.y = ynya;
        pose.pose.orientation.x = mengrotate.getX();
        pose.pose.orientation.y = mengrotate.getY();
        pose.pose.orientation.z = mengrotate.getZ();
        pose.pose.orientation.w = mengrotate.getW();

        local_pos_pub.publish(pose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    
  return 0;
}