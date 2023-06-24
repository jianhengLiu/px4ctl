#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
using namespace std;

#define SIM 0

enum MISSION_STATE
{
    INIT,
    POSITION,  // TAKEOFF
    LAND
};

MISSION_STATE mission_state;

mavros_msgs::State px4_state, px4_state_prev;

#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0

double last_set_hover_pose_time;

ros::Publisher     target_pose_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

//  W: World;V: View; B: Body;
Eigen::Affine3f     T_W_Bt, T_V_Bt_set, T_W_V, T_B_C;
geometry_msgs::Pose T_W_B_set;
bool                receive_odom = false;
void                odom_callback(const geometry_msgs::PoseStampedConstPtr &odom_msg)
{
    T_W_Bt.matrix().block<3, 3>(0, 0) =
        Eigen::Quaternionf(odom_msg->pose.orientation.w, odom_msg->pose.orientation.x,
                           odom_msg->pose.orientation.y, odom_msg->pose.orientation.z)
            .toRotationMatrix();
    T_W_Bt.matrix().block<3, 1>(0, 3) = Eigen::Vector3f(
        odom_msg->pose.position.x, odom_msg->pose.position.y, odom_msg->pose.position.z);
    receive_odom = true;
}

bool receive_pose_set = false;
void pose_set_callback(const geometry_msgs::PoseStampedConstPtr &pose_set_msg)
{
    Eigen::Affine3f T_V_Ct_set;
    T_V_Ct_set.matrix().block<3, 3>(0, 0) =
        Eigen::Quaternionf(pose_set_msg->pose.orientation.w, pose_set_msg->pose.orientation.x,
                           pose_set_msg->pose.orientation.y, pose_set_msg->pose.orientation.z)
            .toRotationMatrix();
    T_V_Ct_set.matrix().block<3, 1>(0, 3) =
        Eigen::Vector3f(pose_set_msg->pose.position.x, pose_set_msg->pose.position.y,
                        pose_set_msg->pose.position.z);
    T_V_Bt_set = T_V_Ct_set * T_B_C.inverse();

    if (mission_state != LAND)
    {
        Eigen::Affine3f T_W_B_set_tmp = T_W_V * T_V_Bt_set;
        T_W_B_set.position.x          = T_W_B_set_tmp.translation().x();
        T_W_B_set.position.y          = T_W_B_set_tmp.translation().y();
        T_W_B_set.position.z          = T_W_B_set_tmp.translation().z();
        Eigen::Quaternionf q(T_W_B_set_tmp.matrix().block<3, 3>(0, 0));
        T_W_B_set.orientation.w = q.w();
        T_W_B_set.orientation.x = q.x();
        T_W_B_set.orientation.y = q.y();
        T_W_B_set.orientation.z = q.z();
    }
    receive_pose_set = true;
}

DynamixelWorkbench dynamixel_controller;  //声明对象

#define BAUDRATE 57600
#define ID 1
#define INIT_RAD -1.57233

int  goal_position;
void pitch_set_callback(const std_msgs::Float32ConstPtr &pitch_set_msg)
{
    float pitch_set = pitch_set_msg->data;
    float radian    = INIT_RAD + pitch_set;
    if (pitch_set < 0.01 && pitch_set > -1.58)
        goal_position = dynamixel_controller.convertRadian2Value(ID, radian);
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
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0 &&
                receive_odom && receive_pose_set)
            {
                mission_state            = POSITION;
                last_set_hover_pose_time = ros::Time::now().toSec();
                // pose 33
                T_W_B_set.position.x         = -0.6;
                T_W_B_set.position.y         = 0.0;
                T_W_B_set.position.z         = 0.35;
                T_W_B_set.orientation.w      = 1.0;
                T_W_B_set.orientation.x      = 0.0;
                T_W_B_set.orientation.y      = 0.0;
                T_W_B_set.orientation.z      = 0.0;
                Eigen::Affine3f T_W_B0       = T_W_Bt;
                Eigen::Affine3f T_V_B0_set   = T_V_Bt_set;
                T_V_B0_set.translation().z() = 0;
                T_W_V                        = T_W_B0 * T_V_B0_set.inverse();
                cout << "T_W_B0" << endl << T_W_B0.matrix() << endl;
                cout << "T_V_B0_set" << endl << T_V_B0_set.matrix() << endl;
                cout << "T_W_V" << endl << T_W_V.matrix() << endl;
                goal_position = dynamixel_controller.convertRadian2Value(ID, INIT_RAD);
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
                goal_position = dynamixel_controller.convertRadian2Value(ID, INIT_RAD);
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
            T_W_B_set.position.x +=
                rc_ch[1] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_PITCH ? -1 : 1);
            T_W_B_set.position.y +=
                rc_ch[3] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_ROLL ? 1 : -1);
        }
        if (mission_state == LAND)
        {
            T_W_B_set.position.z -= 0.3 * delta_t;
        }
        else
        {
            T_W_B_set.position.z +=
                rc_ch[2] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_THROTTLE ? -1 : 1);
        }

        if (T_W_B_set.position.z < -0.3)
            T_W_B_set.position.z = -0.3;
        else if (T_W_B_set.position.z > 1.0)
            T_W_B_set.position.z = 1.0;
    }
}
void px4_state_callback(const mavros_msgs::StateConstPtr &state_msg)
{
    px4_state = *state_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nbv_ctl");
    ros::NodeHandle nh("~");
    ros::Rate       rate(30);

    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 100, odom_callback, ros::VoidConstPtr(), ros::TransportHints());
    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, px4_state_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, rc_callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber pose_set_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/ar/cam_pose_level_set", 100, pose_set_callback);
    ros::Subscriber pitch_set_sub =
        nh.subscribe<std_msgs::Float32>("/ar/cam_pitch_set", 100, pitch_set_callback);

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mission_state = INIT;

    const char *log;

    while (!dynamixel_controller.begin("/dev/ttyUSB0", BAUDRATE, &log))  //打开串口
        cout << log << endl;
    uint16_t model_number = 0;
    bool     result       = dynamixel_controller.ping(ID, &model_number, &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
        ROS_ERROR("Can't find Dynamixel ID '%d'", ID);
        return result;
    }
    dynamixel_controller.torqueOff(ID, &log);
    cout << log << endl;
    dynamixel_controller.setPositionControlMode(ID, &log);  //设置位置控制模式
    cout << log << endl;
    dynamixel_controller.torqueOn(ID, &log);
    cout << log << endl;
    //上电，只有断电的情况下才能设置控制模式
    goal_position = dynamixel_controller.convertRadian2Value(ID, INIT_RAD);

    T_B_C.matrix() << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0., 0., 0., 1.;
    while (ros::ok())
    {
        // float radian = 0;
        // dynamixel_controller.getRadian(ID, &radian);
        if (mission_state != INIT)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose            = T_W_B_set;
            target_pose_pub.publish(pose);
        }
        result = dynamixel_controller.itemWrite(ID, "Goal_Position", goal_position,
                                                &log);  //写入目标的位置
        if (!result)
        {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[Goal_Position] to Dynamixel[ID : %d]",
                      goal_position, ID);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
