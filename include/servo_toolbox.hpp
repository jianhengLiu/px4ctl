#pragma once
#include <iostream>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

class ServoToolbox
{
public:
    ServoToolbox() = default;

    bool init()
    {
        // 舵机配置
        //打开串口
        while (!workbench.begin("/dev/ttyUSB0", BAUDRATE, &log))
            std::cout << log << std::endl;

        uint16_t model_number = 0;
        bool     result       = workbench.ping(ID, &model_number, &log);
        if (!result)
        {
            std::cerr << log << std::endl;
            std::cerr << "Can't find Dynamixel ID " << ID << std::endl;
            return result;
        }
        workbench.torqueOff(ID, &log);
        std::cout << log << std::endl;
        workbench.setPositionControlMode(ID, &log);  //设置位置控制模式
        std::cout << log << std::endl;
        // 在不确定位置的时候，可以先下电自行转动一下看输出
        workbench.torqueOn(ID, &log);
        std::cout << log << std::endl;
        //上电，只有断电的情况下才能设置控制模式
        return true;
    }

    float get_radian()
    {
        float radian = 0;
        workbench.getRadian(ID, &radian);
        return radian;
    }

    bool set_radian(float _radian_set)
    {
        float radian = INIT_RAD + _radian_set;
        goal_position = workbench.convertRadian2Value(ID, radian);
        auto result   = workbench.itemWrite(ID, "Goal_Position", goal_position,
                                            &log);  //写入目标的位置
        if (!result)
        {
            std::cerr << log << std::endl;
            std::cerr << "Failed to write value[" << goal_position
                      << "] on items[Goal_Position] to Dynamixel[ID : " << ID << "]" << std::endl;
            return false;
        }
        return true;
    }

    const int    BAUDRATE = 57600;
    const int    ID       = 1;
    const double INIT_RAD = -1.57;  // 水平角度，向上数值减小，向下数值增大

    DynamixelWorkbench workbench;  //声明对象
    const char        *log;
    int                goal_position;

private:
};