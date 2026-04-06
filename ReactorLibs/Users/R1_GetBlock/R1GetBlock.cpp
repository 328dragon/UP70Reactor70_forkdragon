#include "R1GetBlock.hpp"
#include <algorithm>
#include <cmath>
#include "farcon.hpp"
#include "bsp_hardware.hpp"

extern Farcon farcon;
extern SystemType& System;

// 遥控器按键 8 存储高度的计数器（1→存200块高度，2→400，3→600）
static uint8_t entertime = 0;

void GetBlock::Start()
{
    // GPIO 注册，考虑是否放到Config.cpp里统一注册，这样才不会影响框架的功能（在改变硬件时只需要改变config文件），在这里直接用注册后的实例的名字
    vacuum_pump_pin = BSP::GPIO::Inst({'E', 4});
    release_air_pin = BSP::GPIO::Inst({'H', 4});

    // ---- 达妙舌头电机 ----
    rolldmmotor.Init(Hardware::hcan_sub, 0x11, 0x10, DM_MODE_POSANDVEL);
    rolldmmotor.SetAutoEnable(1,500);
    rolldmmotor.Enable();

    // ---- 大疆抬升电机（M2006，减速比36，CAN2 ID:1，位置串级模式）----
    liftmotor.Init(Hardware::hcan_sub, 1,DJI_C610);
    liftmotor.ConfigPID()
        .AsPosC()
        .Pos_Coeff(4.0f, 0.0f, 0.0f)      // 位置环 kp/ki/kd（待整定）
        .Pos_Limit(300.0f, 200.0f)         // 位置环积分限幅、输出速度限幅（rad/s）
        .Spd_Coeff(0.15f, 0.005f, 0.0f)    // 速度环 kp/ki/kd（待整定）
        .Spd_Limit(2.0f, 10.0f)       // 速度环积分限幅、电流输出限幅（code）
        .CurLimit(10)
        .Apply();
    liftmotor.driver.Enable();

    // ---- 大疆滑台电机（M2006，减速比36，CAN2 ID:2，位置串级模式）----
    slidemotor.Init(Hardware::hcan_sub, 2,DJI_C610);
    slidemotor.ConfigPID()
        .AsPosC()
        .Pos_Coeff(4.0f, 0.0f, 0.0f)      // 位置环 kp/ki/kd（待整定）
        .Pos_Limit(300.0f, 200.0f)         // 位置环积分限幅、输出速度限幅（rad/s）
        .Spd_Coeff(0.15f, 0.005f, 0.0f)    // 速度环 kp/ki/kd（待整定）
        .Spd_Limit(2.0f, 10.0f)       // 速度环积分限幅、电流输出限幅（code）
        .CurLimit(10)
        .Apply();
    slidemotor.driver.Enable();

    appstate = STATE_INIT;
}

// ======================== Update ========================

void GetBlock::Update()
{
    GetTargetBlockInfo();
    // ---- 状态机 ----
    if (appstate == STATE_INIT)
    {
        SetTargetState(0.0f, 0.0f, 0.0f, 2.0f);
        release_air_pin.Write(true);
    }

    if (appstate == STATE_LIFTED)
    {
        if (liftmotor.driver.measure.total_angle < -700000.0f)
        {
            SetTargetState(-2.30383492f, -750000.0f, 0.0f, 2.0f);
        }
        appstate = STATE_IDLE;
    }

    if (appstate == STATE_IDLE)
    {
        release_air_pin.Write(true);
    }

    // ---- 遥控器按键边沿检测 ----
    for (int i = 0; i < 8; i++)
    {
        uint8_t cur = farcon.button_first_half[i];
        btn_enter[i]      = (cur == 1 && last_btn_state[i] == 0); // 上升沿
        last_btn_state[i] = cur;
    }

    // ---- 按键动作 ----

    // 按键 1/2/3：一级动作，抬升到对应高度并预伸出滑台
    if (farcon.button_first_half[0] == 1)
    {
        appstate = STATE_GET200BLOCK;
        SetTargetState(-2.30383492f, blockheight_2_liftmotortargetpos[0], 130000.0f, 2.0f);
        vacuum_pump_pin.Write(true);
    }
    if (farcon.button_first_half[1] == 1)
    {
        appstate = STATE_GET400BLOCK;
        SetTargetState(-2.30383492f, blockheight_2_liftmotortargetpos[1], 130000.0f, 2.0f);
        vacuum_pump_pin.Write(true);
    }
    if (farcon.button_first_half[2] == 1)
    {
        appstate = STATE_GET600BLOCK;
        SetTargetState(-2.30383492f, blockheight_2_liftmotortargetpos[2], 130000.0f, 2.0f);
        vacuum_pump_pin.Write(true);
    }

    // 按键 4：二级动作，滑台全伸出吸附
    if (farcon.button_first_half[3] == 1)
    {
        SetTargetState(-2.30383492f, target_state_pos[1], 430000.0f, 2.0f);
        vacuum_pump_pin.Write(true);
    }

    // 按键 5：放块，关泵放气
    if (farcon.button_first_half[4] == 1)
    {
        appstate = STATE_RELEASEBLOCK;
        SetTargetState(target_state_pos[0], target_state_pos[1], target_state_pos[2], 2.0f);
        vacuum_pump_pin.Write(false);
        release_air_pin.Write(false);
        appstate = STATE_IDLE;
    }

    // 按键 6/7：手动微调抬升高度（单次触发，每次约 5cm）
    if (farcon.button_first_half[5] == 1 && btn_enter[5])
    {
        target_state_pos[1] -= 166666.0f; // 上升
        SetTargetState(target_state_pos[0], target_state_pos[1], target_state_pos[2], 2.0f);
    }
    if (farcon.button_first_half[6] == 1 && btn_enter[6])
    {
        target_state_pos[1] += 166666.0f; // 下降
        SetTargetState(target_state_pos[0], target_state_pos[1], target_state_pos[2], 2.0f);
    }

    // 按键 8：依次存储 200/400/600 块对应的当前抬升位置
    if (farcon.button_first_half[7] == 1 && btn_enter[7])
    {
        blockheight_2_liftmotortargetpos[entertime] = target_state_pos[1];
        entertime = (entertime + 1) % 3;
        // TODO: 将 entertime 回传到遥控器屏幕
    }
}

// ======================== Enable / Stop ========================

void GetBlock::Enable()
{
    enabled = true;
}

void GetBlock::Stop()
{
    rolldmmotor.Disable();
    liftmotor.driver.Disable();
    slidemotor.driver.Disable();
    vacuum_pump_pin.Write(false);
    enabled = false;
}

// ======================== SetTargetState ========================

/**
 * @brief 设置三电机目标状态并立即下发
 * @note  midswing（达妙）：SetPosVel 即时发包
 *        lift/slide（大疆）：SetPos 写入目标值，由 ControlAllMotors 统一调度发包
 *        lift_speed / slide_speed 在大疆串级位置模式下无法直接设置运动限速，参数保留供后续扩展
 */
void GetBlock::SetTargetState(float midswing_pos, float lift_pos, float slide_pos,
                               float midswing_speed, float lift_speed, float slide_speed)
{
    // 写入目标数组
    target_state_pos[0]   = midswing_pos;
    target_state_pos[1]   = lift_pos;
    target_state_pos[2]   = slide_pos;
    target_state_speed[0] = midswing_speed;
    target_state_speed[1] = lift_speed;
    target_state_speed[2] = slide_speed;

    //这个截断达妙电机层已实现了,这个逻辑现在有点乱，功能重复了
    for (int i = 0; i < 3; i++)
    {
        target_state_pos[i] = std::clamp(target_state_pos[i], pos_limit[i][0], pos_limit[i][1]);
    }

    // 下发给各电机
    rolldmmotor.SetPosVel(target_state_pos[0], target_state_speed[0]); 
    liftmotor.SetPos(target_state_pos[1]);                              
    slidemotor.SetPos(target_state_pos[2]);
}

// ======================== SetPosLimit ========================

void GetBlock::SetPosLimit(float midswing_min, float midswing_max,
                            float lift_min,     float lift_max,
                            float slide_min,    float slide_max)
{
    // 达妙舌头电机：通过 SetPosLimit 接口设置驱动层软限位
    rolldmmotor.SetPosLimit(midswing_max, midswing_min); // 注意：达妙接口为 (max, min)

    // 大疆电机：框架本身无软限位成员，限位由上层pos_limit 数组 + clamp 保证
    pos_limit[0][0] = midswing_min; pos_limit[0][1] = midswing_max;
    pos_limit[1][0] = lift_min;     pos_limit[1][1] = lift_max;
    pos_limit[2][0] = slide_min;    pos_limit[2][1] = slide_max;
}

// ======================== 取块动作 ========================

void GetBlock::Get_200Block()
{
    appstate = STATE_GET200BLOCK;
    SetTargetState(-2.30383492f, blockheight_2_liftmotortargetpos[0], 130000.0f, 2.0f);
    vacuum_pump_pin.Write(true);
}

void GetBlock::Get_400Block()
{
    appstate = STATE_GET400BLOCK;
    SetTargetState(-2.30383492f, blockheight_2_liftmotortargetpos[1], 130000.0f, 2.0f);
    vacuum_pump_pin.Write(true);
}

void GetBlock::Get_600Block()
{
    appstate = STATE_GET600BLOCK;
    SetTargetState(-2.30383492f, blockheight_2_liftmotortargetpos[2], 130000.0f, 2.0f);
    vacuum_pump_pin.Write(true);
}

void GetBlock::ReleaseBlock()
{
    appstate = STATE_RELEASEBLOCK;
    SetTargetState(-2.30383492f, -800000.0f, 0.0f, 2.0f);
    vacuum_pump_pin.Write(false);
}

void GetBlock::Action_LiftToHeight(float height)
{
    // TODO: 根据 height（mm）换算 liftmotor total_angle 目标值并下发
    (void)height;
}

// ======================== GetTargetBlockInfo ========================

/**
 * @brief 从遥控器解析当前帧可取 KFS 的坐标和高度，存入 target_block_pos
 */
void GetBlock::GetTargetBlockInfo()
{
    target_count = 0; // 每帧重新统计，避免累计

    for (int i = 0; i < 12; i++)
    {
        if (farcon.KFS_values[i] != 1) continue;

        if (System.GetCamp() == Systems::Camp_Red)
        {
            if (target_count >= 3) break;
            target_block_pos[target_count] = RED_ALL_BLOCKS_DATA[i];
            target_count++;
        }
        else if (System.GetCamp() == Systems::Camp_Blue)
        {
            // TODO: 蓝方块坐标数据
        }
    }
}