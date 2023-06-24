#pragma once
#include <ros/ros.h>
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"

class MavrosToolbox
{
public:
    MavrosToolbox() = default;
    MavrosToolbox(ros::NodeHandle &nh)
    {
        init(nh);
    }
    void init(ros::NodeHandle &nh)
    {
        state_sub       = nh.subscribe<mavros_msgs::State>("/mavros/state", 10,
                                                     &MavrosToolbox::px4_state_callback, this);
        arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    bool arm()
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Vehicle armed");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to armed");
            return false;
        }
    }

    bool disarm()
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = false;
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Vehicle disarmed");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to disarm");
            return false;
        }
    }

    bool set_mode(const std::string &_str_mode)
    {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = _str_mode;
        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
        {
            ROS_INFO("%s enabled", _str_mode.c_str());
            state_prev      = state;
            state_prev.mode = _str_mode;
            return true;
        }
        else
        {
            ROS_WARN("Failed to enter %s!", _str_mode.c_str());
            return false;
        }
    }

    bool offboard_arm()
    {
        if (!set_mode("OFFBOARD"))
            return false;
        if (!arm())
            return false;
        return true;
    }

    void px4_state_callback(const mavros_msgs::StateConstPtr &state_msg)
    {
        state = *state_msg;
    }

    mavros_msgs::State state, state_prev;

private:
    ros::Subscriber    state_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
};
