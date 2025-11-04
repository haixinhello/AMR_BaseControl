/*
 * Kinematics.hpp
 *
 *  Created on: Oct 19, 2025
 *      Author: Administrator
 */

#ifndef MYCODE_KINEMATICS_HPP_
#define MYCODE_KINEMATICS_HPP_

#define PI         3.1415926535897932384626433832795
#include <cstdint>

typedef struct
{
    uint8_t id;                // 电机编号
    uint16_t reducation_ratio; // 减速器减速比，轮子转一圈，电机需要转的圈数
    uint16_t pulse_ration;     // 脉冲比，电机转一圈所产生的脉冲数
    float wheel_diameter;      // 轮子的外直径，单位mm
    float per_pulse_distance;  // 无需配置，单个脉冲轮子前进的距离，单位mm，设置时自动计算
                               // 单个脉冲距离=轮子转一圈所行进的距离/轮子转一圈所产生的脉冲数
                               // per_pulse_distance= (wheel_diameter*3.1415926)/(pulse_ration*reducation_ratio)
    uint32_t speed_factor;     // 无需配置，计算速度时使用的速度因子，设置时自动计算，speed_factor计算方式如下
                               // 设 dt（单位us,1s=1000ms=10^6us）时间内的脉冲数为dtick
                               // 速度speed = per_pulse_distance*dtick/(dt/1000/1000)=(per_pulse_distance*1000*1000)*dtic/dt
                               // 记 speed_factor = (per_pulse_distance*1000*1000)
    int16_t motor_speed;       // 无需配置，当前电机速度mm/s，计算时使用
    int64_t last_encoder_tick; // 无需配置，上次电机的编码器读数
}motor_param_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct
{
    float x;                 // 坐标x
    float y;                 // 坐标y
    float yaw;               // yaw
    quaternion_t quaternion; // 姿态四元数
    float linear_speed;      // 线速度
    float angular_speed;     // 角速度
} odom_t;

class Kinematics
{
private:
    motor_param_t motor_param_[2];
    float K;//转向主销中心距
    float W;//小车轮距
    float L;//小车轴距
    odom_t odom_;
public:
    Kinematics() = default;
    ~Kinematics() = default;

    static void Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q);

    static void TransAngleInPI(float angle,float& out_angle);
    /**
     * @brief 设置电机相关参数
     *
     * @param id
     * @param reducation_ratio
     * @param pulse_ration
     * @param wheel_diameter
     */
    void set_motor_param(uint8_t id, uint16_t reducation_ratio, uint16_t pulse_ration, float wheel_diameter);
    /**
     * @brief 设置运动学相关参数
     *
     * @param K_:转向主销中心距
     * @param W_:小车轮距
     * @param L_:小车轴距
     */
    void set_kinematic_param(float K_,float W_,float L_);

    /**
     * @brief 运动学逆解，输入机器人当前线速度和角速度，输出左右轮子应该达到的目标速度
     *
     * @param line_speed
     * @param angle_speed
     * @param out_wheel1_speed
     * @param out_wheel2_speed
     */
    void kinematic_inverse(float line_speed, float angle_speed, float &out_wheel1_speed, float &out_wheel2_speed, float &alpha);


    /**
     * @brief 运动学正解，输入左右轮子速度，输出机器人当前线速度和角速度
     *
     * @param wheel1_speed
     * @param wheel2_speed
     * @param line_speed
     * @param angle_speed
     */
    void kinematic_forward(float wheel1_speed, float wheel2_speed, float &line_speed, float &angle_speed);

    /**
     * @brief 更新轮子的tick数据
     *
     * @param current_time
     * @param motor_tick1
     * @param motor_tick2
     */
    void update_motor_ticks(int32_t motor_tick1, int32_t motor_tick2);

    /**
     * @brief 获取轮子当前速度
     *
     * @param id
     * @return int16_t
     */
    int16_t motor_speed(uint8_t id);

    /**
     * @brief 更新机器人里程计信息
     *
     * @param dt 间隔时间dt
     */
    void update_bot_odom(uint32_t dt);

    /**
     * @brief 获取里程计函数
     *
     * @return odom_t&
     */
    odom_t &odom();
};

#endif /* MYCODE_KINEMATICS_HPP_ */
