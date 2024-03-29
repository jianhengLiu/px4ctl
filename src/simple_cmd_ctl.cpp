#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include "TagDetector.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/Image.h"


using namespace std;

#define SIM 0

// Finite state machine
// VIO<-->TAG
enum FEEDBACK_STATE
{
    VIO,
    TAG
};

enum MISSION_STATE
{
    INIT,
    POSITION,  // TAKEOFF
    TAG_SERVO,
    LAND
};

FEEDBACK_STATE fb_state;

MISSION_STATE mission_state;

mavros_msgs::State px4_state, px4_state_prev;

#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0
#define TAKEOFF_ALTITUDE 0.32  //  动捕：0.35
#define UAV_HEIGHT 0.3
Eigen::Vector3f TAKEOFF_POS(0.0, 0.0, TAKEOFF_ALTITUDE);
Eigen::Vector3f TO_POINT_POS(1.0, 0.0, TAKEOFF_ALTITUDE);

double last_set_hover_pose_time;

ros::Publisher     target_pose_pub, tag_pose_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

Eigen::Affine3f T_W_B0, T_W_Bt;

cv::Mat         intrinsic;
cv::Mat         distort;
Eigen::Affine3f T_B_C;

std::vector<std::vector<cv::Point3f>> tag_coord;

// 0: 初始时刻； k：看到TAG时刻； t:实时
Eigen::Affine3f T_B0_DES;
Eigen::Affine3f T_W_DES;
Eigen::Affine3f T_W_TAGk, T_W_TAG;

Eigen::Affine3f T_Ck_TAG;

void odom_callback(const geometry_msgs::PoseStampedConstPtr &odom_msg)
{
    Eigen::Vector3f    p(odom_msg->pose.position.x, odom_msg->pose.position.y,
                         odom_msg->pose.position.z);
    Eigen::Quaternionf q(odom_msg->pose.orientation.w, odom_msg->pose.orientation.x,
                         odom_msg->pose.orientation.y, odom_msg->pose.orientation.z);

    T_W_Bt.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();
    T_W_Bt.matrix().block<3, 1>(0, 3) = p;
}

void rc_callback(const mavros_msgs::RCInConstPtr &rc_msg)
{
    double rc_ch[4];
    for (int i = 0; i < 4; i++)
    {
        // 归一化遥控器输入
        rc_ch[i] = ((double)rc_msg->channels[i] - 1500.0) / 500.0;
        if (rc_ch[i] > DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (rc_ch[i] < -DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            rc_ch[i] = 0.0;
    }

    if (rc_msg->channels[4] < 1250)
    {
        if (px4_state.mode != "STABILIZED")
        {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "STABILIZED";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Switch to STABILIZED!");
                px4_state_prev      = px4_state;
                px4_state_prev.mode = "STABILIZED";
            }
            else
            {
                ROS_WARN("Failed to enter STABILIZED!");
                return;
            }
        }
        mission_state = INIT;
    }
    else if (rc_msg->channels[4] > 1250 && rc_msg->channels[4] < 1750)  // heading to POSITION
    {
        if (mission_state == INIT)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0)
            {
                if (T_W_Bt.matrix()(0, 0) == 0)
                {
                    ROS_WARN("Switch to POSITION failed! Not receie Odom!");
                    return;
                }
                else
                {
                    T_W_B0                   = T_W_Bt;
                    mission_state            = POSITION;
                    last_set_hover_pose_time = ros::Time::now().toSec();
                    T_B0_DES                 = Eigen::Affine3f::Identity();
                    T_B0_DES.translation()   = TAKEOFF_POS;
                    T_W_TAG                  = Eigen::Affine3f::Identity();
                    ROS_INFO("Switch to POSITION succeed!");
                }
            }
            else
            {
                ROS_WARN("Switch to POSITION failed! Rockers are not in reset middle!");
                return;
            }
        }
        else if (mission_state == TAG_SERVO)
        {
            Eigen::Vector3f p_B0_DES   = (T_W_B0.inverse() * T_W_DES).translation();
            T_B0_DES.translation().x() = TAKEOFF_POS.x();
            T_B0_DES.translation().y() = TAKEOFF_POS.y();
            // T_B0_DES.translation().x() = p_B0_DES.x();
            // T_B0_DES.translation().y() = p_B0_DES.y();
            mission_state = POSITION;
            ROS_INFO("Swith to POSITION succeed! Heading to TAKEOFF_POS.");
        }
    }
    else if (rc_msg->channels[4] > 1750)
    {
        if (mission_state == POSITION)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0)
            {
                T_B0_DES.translation() = TO_POINT_POS;
                mission_state          = TAG_SERVO;
                ROS_INFO("Set setpoint to TO_POINT_POS succeed!");
            }
            else
            {
                ROS_WARN("Set setpoint to TO_POINT_POS failed! Rockers are not in reset middle!");
                return;
            }
        }
    }

    if (!SIM)
    {
        if (rc_msg->channels[5] > 1750)
        {
            if (mission_state == POSITION)
            {
                if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0 &&
                    !px4_state.armed)
                {
                    if (px4_state.mode != "OFFBOARD")
                    {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("Offboard enabled");
                            px4_state_prev      = px4_state;
                            px4_state_prev.mode = "OFFBOARD";
                        }
                        else
                        {
                            ROS_WARN("Failed to enter OFFBOARD!");
                            return;
                        }
                    }
                    else if (px4_state.mode == "OFFBOARD")
                    {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;

                        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        else
                        {
                            ROS_ERROR("Failed to armed");
                            return;
                        }
                    }
                }
                else if (!px4_state.armed)
                {
                    ROS_WARN("Arm denied! Rockers are not in reset middle!");
                    return;
                }
            }
        }
        else if (rc_msg->channels[5] > 1250 && rc_msg->channels[5] < 1750)
        {
            if (px4_state_prev.mode == "OFFBOARD")
            {
                mission_state = LAND;
            }
        }
        else if (rc_msg->channels[5] < 1250)
        {
            if (px4_state.armed)
            {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = false;

                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle disarmed");
                }
                else
                {
                    ROS_ERROR("Failed to disarmed");
                    return;
                }
                fb_state      = VIO;
                mission_state = INIT;
                ROS_INFO("Swith to INIT state!");
            }
        }
    }

    if (mission_state != INIT)
    {
        double now               = ros::Time::now().toSec();
        double delta_t           = now - last_set_hover_pose_time;
        last_set_hover_pose_time = now;

        // body frame: x-forward, y-left, z-up
        if (mission_state == POSITION)
        {
            T_B0_DES.translation().x() +=
                rc_ch[1] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_PITCH ? -1 : 1);
            T_B0_DES.translation().y() +=
                rc_ch[3] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_ROLL ? 1 : -1);
        }
        if (mission_state == LAND)
        {
            T_B0_DES.translation().z() -= 0.3 * delta_t;
        }
        else
        {
            T_B0_DES.translation().z() +=
                rc_ch[2] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_THROTTLE ? -1 : 1);
        }

        if (T_B0_DES.translation().z() < -0.3)
            T_B0_DES.translation().z() = -0.3;
        else if (T_B0_DES.translation().z() > 1.0)
            T_B0_DES.translation().z() = 1.0;
    }
}
void px4_state_callback(const mavros_msgs::StateConstPtr &state_msg)
{
    px4_state = *state_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_cmd_ctl");
    ros::NodeHandle nh("~");
    ros::Rate       rate(30);

    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 100, odom_callback, ros::VoidConstPtr(), ros::TransportHints());

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, px4_state_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, rc_callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    fb_state      = VIO;
    mission_state = INIT;
    while (ros::ok())
    {
        if (mission_state != INIT)
        {
            T_W_DES = T_W_B0 * T_B0_DES;
            if (mission_state == TAG_SERVO)
            {
                if (fb_state == TAG)
                {
                    T_W_DES.translation().x() = T_W_TAG.translation().x();
                    T_W_DES.translation().y() = T_W_TAG.translation().y();
                    T_W_DES.translation().z() =
                        T_W_TAG.translation().z() + UAV_HEIGHT + TAKEOFF_ALTITUDE;
                }
            }

            geometry_msgs::PoseStamped pose;
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = T_W_DES.translation().x();
            pose.pose.position.y = T_W_DES.translation().y();
            pose.pose.position.z = T_W_DES.translation().z();
            Eigen::Quaternionf q(T_W_DES.matrix().block<3, 3>(0, 0));
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            target_pose_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
