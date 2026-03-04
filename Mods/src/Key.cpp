#include "Key.hpp"


extern Farcon farcon;

void KeyExecutor::Start()
{
    // 同步按键初始状态
    for (int i = 0; i < 16; i++)
    {
        _last_state[i] = (i < 8) ? farcon.button_first_half[i] : farcon.button_second_half[i - 8];
    }

    /* 初始化按键绑定动作需要的模块 */
    // 继电器初始化
    _platform_relay.Init(GPIOF, GPIO_PIN_0, Relay::HIGH_ON);
}

void KeyExecutor::Update()
{
    // 遍历所有按键，检测状态变化
    for (int i = 0; i < 16; i++)
    {
        uint8_t current_state = (i < 8) ? farcon.button_first_half[i] : farcon.button_second_half[i - 8];
        
        // 检测按键按下事件
        if (current_state == 1 && _last_state[i] == 0)
        {
            switch (i)
            {
                case 0:

                    break;
                
                default:
                    break;
            }
        }
        
        // 更新上次状态
        _last_state[i] = current_state;
    }
}

