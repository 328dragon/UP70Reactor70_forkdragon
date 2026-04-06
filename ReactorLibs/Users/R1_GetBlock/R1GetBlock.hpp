#pragma once
#include "System.hpp"
#include "motor_dm.hpp"
#include "motor_dji.hpp"
#include "bsp_gpio.hpp"

// 块中心在场地坐标系的位置，xy 单位为米，height 单位为毫米
struct BlockInfo
{
    float x;
    float y;
    float height;
};

// 12 个物块的标准坐标（红方）
// TODO: 梅林位置不固定，后续优化遍历方式
const BlockInfo RED_ALL_BLOCKS_DATA[12] =
{
    {1.8f+2*1.2f, 3.8f+3*1.2f, 200.0f}, {1.8f+1*1.2f, 3.8f+3*1.2f, 400.0f}, {1.8f+0*1.2f, 3.8f+3*1.2f, 200.0f}, // 1-3
    {1.8f+2*1.2f, 3.8f+2*1.2f, 400.0f}, {1.8f+1*1.2f, 3.8f+2*1.2f, 600.0f}, {1.8f+0*1.2f, 3.8f+2*1.2f, 400.0f}, // 4-6
    {1.8f+2*1.2f, 3.8f+1*1.2f, 600.0f}, {1.8f+1*1.2f, 3.8f+1*1.2f, 400.0f}, {1.8f+0*1.2f, 3.8f+1*1.2f, 200.0f}, // 7-9
    {1.8f+2*1.2f, 3.8f+0*1.2f, 400.0f}, {1.8f+1*1.2f, 3.8f+0*1.2f, 200.0f}, {1.8f+0*1.2f, 3.8f+0*1.2f, 400.0f}  // 10-12
};

class GetBlock : public Application
{
    SINGLETON(GetBlock):Application("GetBlock"){};
    APPLICATION_OVERRIDE

public:
    MotorDM  rolldmmotor; // 舌头电机（达妙）
    MotorDJI liftmotor;     // 抬升电机（大疆 M2006，CAN2 ID:1）
    MotorDJI slidemotor;    // 滑台电机（大疆 M2006，CAN2 ID:2）
    BSP::GPIO::Inst vacuum_pump_pin;
    BSP::GPIO::Inst release_air_pin;

private:
    bool enabled = false;

    enum BlockState
    {
        STATE_IDLE = 0,
        STATE_INIT,
        STATE_LIFTED,
        STATE_GETTINGROD,  // 取杆中，舌头折叠
        STATE_HOLDROD,     // 取杆完成，舌头展开
        STATE_GET200BLOCK,
        STATE_GET400BLOCK,
        STATE_GET600BLOCK,
        STATE_RELEASEBLOCK,
        STATE_EMERGENCY
    };
    BlockState appstate = STATE_IDLE;

public:
    // 数组顺序：rolldmmotor->0, liftmotor->1, slidemotor->2
    float target_state_pos[3]   = {0.0f}; // 三个电机的目标位置
    float target_state_speed[3] = {0.0f}; // 三个电机的目标速度

    // 软限位：[电机][0]=min, [1]=max
    // midswing 单位 rad，lift/slide 单位 code（total_angle）
    float pos_limit[3][2] = {{0.0f, 0.0f},
                              {0.0f, 0.0f},
                              {0.0f, 0.0f}};

    // 取 200/400/600 块时抬升电机对应的 total_angle 目标值
    float blockheight_2_liftmotortargetpos[3] = {-150000.0f, -750000.0f, -1400000.0f};

    // 遥控器按键边沿检测
    uint8_t last_btn_state[8] = {0};
    bool    btn_enter[8]      = {false};

    // 当前帧目标 KFS 信息
    BlockInfo target_block_pos[3] = {{0}};
    uint8_t   target_count        = 0;

    void Enable();
    void Stop();

    /**
     * @brief 设置三个电机的目标状态并立即下发
     * @param midswing_pos   舌头目标位置（rad）
     * @param lift_pos       抬升目标位置（code，total_angle 语义）
     * @param slide_pos      滑台目标位置（code，total_angle 语义）
     * @param midswing_speed 舌头运动速度（rad/s，达妙 v_des）
     * @param lift_speed     抬升速度（大疆位置串级模式，此参数保留但当前忽略）
     * @param slide_speed    滑台速度（同上）
     */
    void SetTargetState(float midswing_pos   = 0.0f, float lift_pos   = 0.0f, float slide_pos   = 0.0f,
                        float midswing_speed = 2.0f, float lift_speed = 2.0f, float slide_speed = 2.0f);

    void SetPosLimit(float midswing_min, float midswing_max,
                     float lift_min,     float lift_max,
                     float slide_min,    float slide_max);

    void Get_200Block();
    void Get_400Block();
    void Get_600Block();
    void ReleaseBlock();

    void Action_LiftToHeight(float height); // TODO: 预留

    void GetTargetBlockInfo();
};
