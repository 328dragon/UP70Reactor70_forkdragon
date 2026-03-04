#pragma once
#include "System.hpp"
#include "relay.hpp"

// 按键绑定动作索引
enum KeyIndex
{

};

class KeyExecutor : public Application
{
    SINGLETON(KeyExecutor) : Application("KeyExecutor") {};
    APPLICATION_OVERRIDE

public:

    // void Start() override;
    // void Update() override;

private:
    uint8_t _last_state[16] = {0}; // 对应按键0 - 15的上次状态

    /* Key绑定动作需要的模块 */

    /* 按键响应的具体动作函数 */

};