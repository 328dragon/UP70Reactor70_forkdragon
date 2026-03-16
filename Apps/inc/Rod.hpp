#ifndef _ROD_HPP_
#define _ROD_HPP_

#pragma once
#include "System.hpp"
#include "stm32f4xx_hal.h"
#include "motor_dm.hpp"
#include "bsp_gpio.h"
#include "algorithm" 
#include "motor_dji.hpp"
#include "relay.hpp"


namespace mesgcode
{
    const uint8_t ToPC_Head = 0xFF;
    const uint8_t FromPC_Head = 0xFA;
    const uint8_t Take_Rod = 0xBB;

}


class RodType : public Application
{
    friend void MyUartRxcallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);

    SINGLETON(RodType) : Application{"Rod"} {};
    APPLICATION_OVERRIDE
    
    private:
 
    float speed[3]={1000,1000,2000}; //三个电机的速度限幅

    /**取杆结构目标位置**/
    float base_target_angle = 0;
    float top_target_angle = 0;
    float wheel_target_position = 0;
    bool clamped = false; 


    /**取杆结构实际位置**/
    float base_current_angle = 0;
    float top_current_angle = 0;
    float wheel_current_position = 0;

    /**取杆结构到达目标位置的标志**/
    bool top_arrived = false;
    bool base_arrived = false;
    bool wheel_arrived = false;
    bool clamp_action = false;

    bool rod_geted = false; //是否已经取到杆的标志

    /**参数属性**/
    bool _enabled = false; //取杆结构使能标志
    

   
    public:
/**取杆结构含有3个电机**/
    MotorDJI jaw_djmotor[3];
        /**       直接接口         **/
        void Enable();
        void Disable(); 

        /**       取杆结构控制函数         **/
        void ControlBaseRotate(float angle); //控制底座旋转,angle单位为度,正值为往上转
        void ControlTopRotate(float angle); //控制顶部旋转
        void ControlWheelRotate(float position); //控制轮子旋转
        void ControlClampRod(bool clamp); //控制夹爪夹紧或放松

        void Reset(bool reset); //重置电机位置到0

        Relay clamp_relay; // 夹爪控制继电器实例
        
        
};




































#endif