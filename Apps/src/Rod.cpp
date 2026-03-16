#include "Rod.hpp"
#include "bsp_uart.h"
#include "motor_dji.hpp"
#include "string.h"
#include "std_math.hpp"
#include "gpio.h"
#include "msg_coder.hpp"
#include <string.h>
#include "relay.hpp"
#include "Action.hpp"

float a = 0.0f; //测试用，实际应该由上位机发送目标角度
float b = 0.0f;
float c = 0.0f;

 float p_kp[3]={0.1f,0.5f,3.0f},p_kd[3]={0.0f,0.0f,0.0f},p_ki[3]={0.0f,0.0f,0.0f};
float s_kp=5.0f,s_kd=0.005f,s_ki=0.25f;

float current_base_target = 60.0f; 
float base_step = 0.1f;
 uint8_t i=0;

uint8_t receive[128];
uint8_t rx_length=0;
 BspUart_Instance uart6_inst;
 uint8_t sequence_step = 0;  // 记录当前执行到第几步
 bool start_sequence = false; // 设为 true 就会在 Update 里触发这套顺序动作
 bool reset_sequence = false; // 设为 true 就会重置电机位置
 bool clamed_sequence = false; // 设为 true 就会执行夹爪夹紧动作

void MyUartRxcallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);
void RodType::Start()
{
//    jaw_djmotor[0].SetSloperate(500000);
        for (int i = 0; i < 3; i++)
    {
       jaw_djmotor[i].Init(&hcan2, i+1,Pos_Control, false);
    //    jaw_djmotor[i].position_pid.SetParam(p_kp[i],p_ki[i],p_kd[i]);
    //    jaw_djmotor[i].speed_pid.SetParam(s_kp,s_ki,s_kd);
        jaw_djmotor[i].position_pid.Init(p_kp[i],p_ki[i],p_kd[i],0);
       jaw_djmotor[i].speed_pid.Init(s_kp,s_ki,s_kd,0);
//        jaw_djmotor[i].speed_pid.ForwardLize(PidGeneral::SpeedForward, 1.0f, 1.0f, 1.0f);  
       jaw_djmotor[i].position_pid.SetLimit(0,speed[i],0.9f);
    //    jaw_djmotor[i].speed_pid.SetLimit(0,5000,0);
       jaw_djmotor[i].Enable();
		
    }
        clamp_relay.Init(GPIOF, GPIO_PIN_0, Relay::HIGH_ON);
        _enabled = true; 
        ControlBaseRotate(0);
        ControlTopRotate(0);

        BspUart_InstRegist(&uart6_inst, &huart6, 64, BspUartType_DMA, BspUartType_DMA, MyUartRxcallback);

   
}
void MyUartRxcallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
    if(rxData[0] == mesgcode::FromPC_Head && RodType::GetInstance()._enabled)
    {
        if(rxData[1] == mesgcode::Take_Rod)
        {
            RodType::GetInstance().clamped = (rxData[2] != 0); // 非0为夹紧，0为放松
            memcpy(&RodType::GetInstance().base_target_angle, &rxData[3], sizeof(float));
            memcpy(&RodType::GetInstance().top_target_angle, &rxData[7], sizeof(float));
            memcpy(&RodType::GetInstance().wheel_target_position, &rxData[11], sizeof(float));
        }
    
    }
    
}

void RodType::Update()
{
   if(_enabled)
    {
//		ControlTopRotate(a);
//        ControlBaseRotate(b);
//        ControlWheelRotate(c);

//        ControlBaseRotate(60.0f);
//        Action.Wait(2000);

    //     Reset(reset_sequence);
        if (start_sequence)
    {
        switch (sequence_step)
        {
            case 0:
                // 第 1 步：底座电机转 60°
								while(i <= 500)
								{
                		ControlClampRod(true);
									i++;
									break;
								}
                ControlBaseRotate(60.0f);
                
                if (base_arrived)
                {
                    sequence_step = 1;          // 切换到下一步
                    top_arrived = false; // 【防弹操作】：强制清零下一步的标志位，防止误判
                }
                break;

            case 1:
                // 第 2 步：头部电机转 -90°
                ControlTopRotate(-90.0f);
                
                if (top_arrived)
                {
                    sequence_step = 2;           // 切换到下一步
                    base_arrived = false; // 【防弹操作】：强制清零下一步的标志位
                }
                break;
                
            case 2:
                // 第 3 步：底座电机转 180°
                ControlBaseRotate(180.0f);
                
                if (base_arrived)
                {
                    sequence_step = 3;           // 切换到结束状态
                }
                break;

            case 3:
                // 动作全部完成！
                start_sequence = false; // 关闭顺序执行使能，防止重复执行
                sequence_step = 0;      // 步骤归零，方便下次再次触发
                rod_geted = true;      
                break;
        }
    }
		
//       ControlBaseRotate(60.0f); // 
//       if (base_arrived)
//       {
//           ControlTopRotate(-100.0f); // 
//           if (top_arrived)
//           {
//               ControlBaseRotate(190.0f); 
//           }

//       }
//		 ControlTopRotate(30.0f); 
        
        // if (clamped)
        // {
        //     ControlClampRod(true);
        //     if(clamp_action)
        //     {
        //         ControlBaseRotate(base_target_angle);
        //         if (base_arrived)
        //         {
        //             ControlTopRotate(top_target_angle);
        //             if (top_arrived)
        //             {
        //                 ControlWheelRotate(wheel_target_position);
        //                 if (wheel_arrived)
        //                 {
        //                    rod_geted = true; // 认为取杆成功
        //                 }
        //             }
        //         }
        //     }
        // }
    }
}

void RodType::Enable()
{
    _enabled = true; //结构使能标志
}

void RodType::Disable()
{
    _enabled = false; //结构失能标志   
}


void RodType::ControlBaseRotate(float angle)
{
    base_target_angle = angle *1310.779f; 
    jaw_djmotor[0].SetPos(base_target_angle);
    base_current_angle = jaw_djmotor[0].measure.total_angle;
    if (abs(base_current_angle - base_target_angle) < 10000) // 误差小于10000认为到达
    {
        base_arrived = true;
    }
    else
    {
        base_arrived = false;
    }
}

void RodType::ControlTopRotate(float angle)
{
    top_target_angle = angle *2457.3f; 
    jaw_djmotor[1].SetPos(top_target_angle);
    top_current_angle = jaw_djmotor[1].measure.total_angle;
    if (abs(top_current_angle - top_target_angle) < 10000) // 误差小于10000认为到达
    {
        top_arrived = true;
    }
    else
    {
        top_arrived = false;
    }
}
void RodType::ControlWheelRotate(float position)
{
    wheel_target_position = position * 294876.0f; 
    jaw_djmotor[2].SetPos(wheel_target_position);
    wheel_current_position = jaw_djmotor[2].measure.total_angle;
    if (abs(wheel_current_position - wheel_target_position) < 10000) // 误差小于10000认为到达
    {
        wheel_arrived = true;
    }
    else
    {
        wheel_arrived = false;
    }
    
}
void RodType::ControlClampRod(bool clamp)
{
    if (clamp)
    {
        clamp_relay.On(); // 夹紧
        clamp_action = true;
    }
    else
    {
        clamp_relay.Off(); // 放松
        clamp_action = false;
    }
}

void RodType::Reset(bool reset)
{
    if (reset)
    {
        ControlBaseRotate(0);
        ControlTopRotate(0);
        base_arrived = top_arrived = wheel_arrived = false;
        rod_geted = false;
        reset_sequence = false;
    }
}
