
#include <Eigen/Dense>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>

#include "mavros_toolbox.hpp"
#include "servo_toolbox.hpp"

using namespace std;

// SIM 0：实飞； 1：手持模拟飞
#define SIM 1
#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 1
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0
#define TAKEOFF_ALTITUDE 0.32  //  动捕：0.35
#define UAV_HEIGHT 0.3

Eigen::Vector3f TAKEOFF_POS(0.0, 0.0, TAKEOFF_ALTITUDE);
Eigen::Vector3f TO_POINT_POS(1.0, 0.0, TAKEOFF_ALTITUDE);

enum MISSION_STATE
{
    INIT,
    POSITION,  // TAKEOFF
    TASK,
    LAND
};
MISSION_STATE mission_state;

MavrosToolbox mavros_tb;
ServoToolbox  servo_tb;

float servo_set_pos;

ros::Publisher target_pose_pub;

// 0: 初始时刻； k：看到TAG时刻； t:实时; s(et):设置的目标位姿
//  W: World;V: View; B: Body; A: Apriltag
Eigen::Affine3f T_W_B0, T_W_Bt;
Eigen::Affine3f T_B0_Bs, T_W_Bs, T_A_Cs;
Eigen::Affine3f T_W_A, T_B_C;  // 外参

void pose_set_callback(const geometry_msgs::PoseStampedConstPtr &pose_set_msg)
{
    if (mission_state == TASK)
    {
        Eigen::Vector3f    p(pose_set_msg->pose.position.x, pose_set_msg->pose.position.y,
                             pose_set_msg->pose.position.z);
        Eigen::Quaternionf q(pose_set_msg->pose.orientation.w, pose_set_msg->pose.orientation.x,
                             pose_set_msg->pose.orientation.y, pose_set_msg->pose.orientation.z);
        T_A_Cs.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();
        T_A_Cs.matrix().block<3, 1>(0, 3) = p;
        T_W_Bs                            = T_W_A * T_A_Cs * T_B_C.inverse();
    }
}

void pitch_set_callback(const std_msgs::Float32ConstPtr &pitch_set_msg)
{
    servo_set_pos = -pitch_set_msg->data;
}

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
                    T_B0_Bs                  = Eigen::Affine3f::Identity();
                    T_B0_Bs.translation()    = TAKEOFF_POS;
                    ROS_INFO("Switch to POSITION succeed!");
                }
            }
            else
            {
                ROS_WARN("Switch to POSITION failed! Rockers are not in reset middle!");
                return;
            }
        }
        else if (mission_state == TASK)
        {
            Eigen::Vector3f p_B0_DES  = (T_W_B0.inverse() * T_W_Bs).translation();
            T_B0_Bs.translation().x() = p_B0_DES.x();
            T_B0_Bs.translation().y() = p_B0_DES.y();
            mission_state             = POSITION;
            ROS_INFO("Swith to POSITION succeed!");
        }
    }
    else if (rc_msg->channels[4] > 1750)
    {
        // TASK
        if (mission_state == POSITION)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0)
            {
                T_B0_Bs       = T_W_B0.inverse() * T_W_Bs;
                mission_state = TASK;
                ROS_INFO("Enter NBV mode!");
            }
            else
            {
                ROS_WARN("Enter NBV mode failed! Rockers are not in reset middle!");
                return;
            }
        }
    }

    // channel 6 control
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
            T_B0_Bs.translation().x() +=
                rc_ch[1] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_PITCH ? -1 : 1);
            T_B0_Bs.translation().y() +=
                rc_ch[0] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_ROLL ? 1 : -1);
        }
        if (mission_state == LAND)
        {
            T_B0_Bs.translation().z() -= 0.3 * delta_t;
        }
        else
        {
            T_B0_Bs.translation().z() +=
                rc_ch[2] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_THROTTLE ? -1 : 1);
        }

        if (T_B0_Bs.translation().z() < -0.3)
            T_B0_Bs.translation().z() = -0.3;
        else if (T_B0_Bs.translation().z() > 1.5)
            T_B0_Bs.translation().z() = 1.5;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nbv_ctl");
    ros::NodeHandle nh("~");
    ros::Rate       rate(30);

    ros::Subscriber odom_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100, odom_callback);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, rc_callback);

    ros::Subscriber pose_set_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/ar/cam_pose_level_set", 100, pose_set_callback);
    ros::Subscriber pitch_set_sub =
        nh.subscribe<std_msgs::Float32>("/ar/cam_pitch_set", 100, pitch_set_callback);

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);

    mission_state = INIT;

    mavros_tb.init(nh);
    servo_tb.init();

    // 控制舵机到初始位置，不确定位置先注释这段，避免机械损坏，使用主循环里的函数获取位置
    servo_set_pos = 0;
    servo_tb.set_radian(servo_set_pos);

    T_W_A.matrix() << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0., 0., 0., 1.;
    T_B_C.matrix() << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0., 0., 0., 1.;

    while (ros::ok())
    {
        // 获取当前舵机的位置
        // cout << servo_tb.get_radian() << endl;

        // 持续发送setpoint保持飞控OFFBOARD模式
        if (mission_state != INIT)
        {
            T_W_Bs = T_W_B0 * T_B0_Bs;

            geometry_msgs::PoseStamped pose;
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = T_W_Bs.translation().x();
            pose.pose.position.y = T_W_Bs.translation().y();
            pose.pose.position.z = T_W_Bs.translation().z();
            Eigen::Quaternionf q(T_W_Bs.matrix().block<3, 3>(0, 0));
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            target_pose_pub.publish(pose);
        }

        servo_tb.set_radian(servo_set_pos);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
