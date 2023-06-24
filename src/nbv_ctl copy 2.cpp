#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_toolbox.hpp"
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"
#include "servo_toolbox.hpp"
#include "std_msgs/Float32.h"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
using namespace std;

// SIM 0：实飞； 1：手持模拟飞
#define SIM 0
#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0

enum MISSION_STATE
{
    INIT,
    POSITION,  // TAKEOFF
    LAND
};
MISSION_STATE mission_state;

MavrosToolbox mavros_tb;
ServoToolbox  servo_tb;

ros::Publisher     target_pose_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

//  W: World;V: View; B: Body;
geometry_msgs::Pose T_A_Cs;
void                pose_set_callback(const geometry_msgs::PoseStampedConstPtr &pose_set_msg)
{
    if (mission_state != LAND)
    {
        T_A_Cs = pose_set_msg->pose;
    }
}

void pitch_set_callback(const std_msgs::Float32ConstPtr &pitch_set_msg)
{
    servo_tb.set_radian(pitch_set_msg->data);
}

void rc_callback(const mavros_msgs::RCInConstPtr &rc_msg)
{
    static double last_set_hover_pose_time;
    double        rc_ch[4];
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
        if (mavros_tb.state.mode != "STABILIZED")
        {
            if (!mavros_tb.set_mode("STABILIZED"))
                return;
        }
        mission_state = INIT;
    }
    else if (rc_msg->channels[4] > 1250 && rc_msg->channels[4] < 1750)  // heading to POSITION
    {
        if (mission_state == INIT)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0)
            {
                mission_state            = POSITION;
                last_set_hover_pose_time = ros::Time::now().toSec();
                // pose 33
                T_A_Cs.position.x    = -0.6;
                T_A_Cs.position.y    = 0.0;
                T_A_Cs.position.z    = 0.35;
                T_A_Cs.orientation.w = 1.0;
                T_A_Cs.orientation.x = 0.0;
                T_A_Cs.orientation.y = 0.0;
                T_A_Cs.orientation.z = 0.0;
                ROS_INFO("Switch to POSITION succeed!");
            }
            else
            {
                ROS_WARN("Switch to POSITION failed! Rockers are not in reset middle!");
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
                    !mavros_tb.state.armed)
                {
                    if (!mavros_tb.offboard_arm())
                        return;
                }
                else if (!mavros_tb.state.armed)
                {
                    ROS_WARN("Arm denied! Rockers are not in reset middle!");
                    return;
                }
            }
        }
        else if (rc_msg->channels[5] > 1250 && rc_msg->channels[5] < 1750)
        {
            if (mavros_tb.state_prev.mode == "OFFBOARD")
            {
                // goal_position = dynamixel_controller.convertRadian2Value(ID, INIT_RAD);
                mission_state = LAND;
            }
        }
        else if (rc_msg->channels[5] < 1250)
        {
            if (mavros_tb.state.armed)
            {
                if (!mavros_tb.disarm())
                    return;
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
            T_A_Cs.position.x +=
                rc_ch[1] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_PITCH ? -1 : 1);
            T_A_Cs.position.y +=
                rc_ch[3] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_ROLL ? 1 : -1);
        }
        if (mission_state == LAND)
        {
            T_A_Cs.position.z -= 0.3 * delta_t;
        }
        else
        {
            T_A_Cs.position.z +=
                rc_ch[2] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_THROTTLE ? -1 : 1);
        }

        if (T_A_Cs.position.z < -0.3)
            T_A_Cs.position.z = -0.3;
        else if (T_A_Cs.position.z > 2.0)
            T_A_Cs.position.z = 2.0;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nbv_ctl");
    ros::NodeHandle nh("~");
    ros::Rate       rate(30);

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, rc_callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber pose_set_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/ar/cam_pose_level_set", 100, pose_set_callback);
    ros::Subscriber pitch_set_sub =
        nh.subscribe<std_msgs::Float32>("/ar/cam_pitch_set", 100, pitch_set_callback);

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);

    mission_state = INIT;

    mavros_tb.init(nh);
    servo_tb.init();

    while (ros::ok())
    {
        // 获取当前舵机的位置
        cout << servo_tb.get_radian() << endl;

        // 持续发送setpoint保持飞控OFFBOARD模式
        if (mission_state != INIT)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose            = T_A_Cs;
            target_pose_pub.publish(pose);
        }
        // 下面写入callback里？
        // servo_tb.set_radian(0);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
