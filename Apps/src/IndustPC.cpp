#include "IndustPC.hpp"
#include "Chassis.hpp"
#include "System.hpp"
 #include "stdio.h"
void IndustPC_Callback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);
ChassisType& chas_ = ChassisType::GetInstance();

extern  bool reset_sequence; // 设为 true 就会重置电机位置
extern bool clamed_sequence; // 设为 true 就会执行夹爪夹紧动作
extern bool start_sequence; // 设为 true 就会在 Update 里触发这套顺序动作
Vec3 chas_pos;
Vec3 slam_pos;
int vofa_flag=0;
		char send_vofa[10];
void IndustPC::Start()
{
    indupc_coder.Init(&huart2);//Monitor也使用了huart2，现在在System.cpp中被注释掉了
    indupc_coder.SetCallback(IndustPC_Callback);

    
}

void IndustPC::Update()
{
    // 持续上传里程计的数据给工控机（频率100Hz从200Hz分）
    static uint8_t send_presc_cnt = 0;
    if (send_presc_cnt++ >= 1&&vofa_flag==0)
    {
        send_presc_cnt = 0;
        static IndustPCMsg msg; 
        msg = EncodeMsg(IndustPCConst::Odo_Code, ChassisType::GetInstance().chas_odom.pos);
         indupc_coder.SendRawMsg(msg.data, IndustPCConst::MsgLength);
    }
		else if(send_presc_cnt++ >= 1&&vofa_flag==1)
		{
		send_presc_cnt = 0;

			int len = sprintf(send_vofa, ":%.2f\n", ChassisType::GetInstance().chas_odom.pos.x);
indupc_coder.SendRawMsg((uint8_t*)send_vofa, 10);
		}
}


void IndustPC_Callback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
    if(rxData[0] == IndustPCConst::FromPC_Head && IndustPC::GetInstance()._enabled)
    {
        // 解析数据
        switch (rxData[1])
        {
            // 解析并覆盖底盘的速度
            case IndustPCConst::ChasSpeed_Code:
            {
                Vec3 chas_spd;
                memcpy(&chas_spd, &rxData[2], sizeof(Vec3));

                if(chas_spd.Length() > 1.5f)    chas_spd = chas_spd.Norm() * 1.5f;

                chas_.Move(chas_spd);
                break;
            }
            // 解析并覆盖底盘的位置环
            case IndustPCConst::ChasPos_Code:
            {
                
                memcpy(&chas_pos, &rxData[2], sizeof(Vec3));
//                chas_.MoveAt(chas_pos.ToVec2());
//                chas_.RotateAt(chas_pos.z);
                break;
            }
            // 获取SLAM的坐标--
            case IndustPCConst::SlamPos_Code:
            {
     
                memcpy(&slam_pos, &rxData[2], sizeof(Vec3));
                IndustPC::GetInstance().slam_transform = slam_pos;
                break;
            }
						    case IndustPCConst::Take_Rod:
            {
                start_sequence = (rxData[2] != 0); // 非0为夹紧，0为放松
                break;
            }
						
						
        }
    }
}


