#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "mpu.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "chassis.h"
/************************************************
 ALIENTEK 阿波罗STM32F746开发板 实验35
 MPU9250九轴传感器实验-HAL库函数版
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司
 作者：正点原子 @ALIENTEK
************************************************/
#define STATE_REMOTE 0
#define STATE_STOP 254
#define STATE_FREE 255
#define STATE_MAZE_STAGE1 1
#define STATE_MAZE_STAGE2 2
#define STATE_MAZE_STAGE3 3
#define STATE_MAZE_STAGE4 4
#define STATE_MAZE_STAGE5 5
#define STATE_MAZE_STAGE6 6
#define STATE_MAZE_STAGE7 7
// key是当前按键，len是字符串长度
extern float pitch, roll, yaw, yawDesire, yawDesire2,yawLineP; //欧拉角
float yawTemp = 0.0f;
u8 key, len;
int ReceivedTurnData, YawVel; //接受的角速度量
float YawControl = 0.0f;      //角速度控制量
int count = 0;

float stop_velocity = 0.3f;
float stop_velocity1 = 0.25f;
float stop_velocity2 = 0.18f;
float stop_velocity3 = 0.1f;
float line_velocity = 0.55f;     //巡线题线速度
float line_velocityOR = 0.7f;   //巡线题线速度 Base Speed Profile
float line_velocityOC = 0.85f;   //巡线题线速度 Overclock Profile 1
float line_velocityOC2 = 1.0f; //巡线题线速度 Overclock Profile 2
int state = STATE_REMOTE;       //状态机变量
//串口1发送1个字符
// c:要发送的字符
int abs(int input)
{
    if (input >= 0)
    {
        return input;
    }
    else
    {
        return -input;
    }
}

int main(void)
{
    line_velocity = line_velocityOR;
    Cache_Enable();                  //打开L1-Cache
    MPU_Memory_Protection();         //保护相关存储区域
    HAL_Init();                      //初始化HAL库
    Stm32_Clock_Init(432, 25, 2, 9); //设置时钟,216Mhz
    delay_init(216);                 //延时初始化
    uart_init(115200);               //串口初始化
    LED_Init();                      //初始化与LED连接的硬件接口
    KEY_Init();                      //初始化按键
    MPU9250_Init();                  //初始化MPU9250

    while (mpu_dmp_init())
    {
        delay_ms(100);
        LED0_Toggle; // DS0闪烁
    }
    LED0(0);
    while (1)
    {
        /* USER CODE END WHILE */
        ReceivedTurnData = 0;
        if (mpu_mpl_get_data(&pitch, &roll, &yawTemp) == 0)
        {
            yaw = yawTemp;
        }
        key = KEY_Scan(0);

        if (key == KEY1_PRES)
        {
            state = state + 1;
        }

        if (key == WKUP_PRES)
        {
            state = state + 1;
        }
        if (key == KEY2_PRES)
        {
            state = STATE_REMOTE;
        }
        if (key == KEY0_PRES)
        {
            if (line_velocity == line_velocityOR)
            {
                line_velocity = line_velocityOC;
							stop_velocity = stop_velocity2;
                LED1_Toggle;
            }
            else if (line_velocity == line_velocityOC)
            {
                line_velocity = line_velocityOC2;
							stop_velocity = stop_velocity3;
							yawLineP = 0.28f;
                LED1(0);
            }
            else
            {
                line_velocity = line_velocityOR;
							stop_velocity = stop_velocity1;
							yawLineP = 0.245f;
                LED1(1);
            }
        }
        if (state == STATE_MAZE_STAGE5)
        {
            state = STATE_REMOTE;
        }

        if (USART_RX_STA & 0x8000)
        {
            len = USART_RX_STA & 0x3fff;             //得到此次接收到的数据长度
            ReceivedTurnData = *(int *)USART_RX_BUF; // APPROX: -500~500 ,All Black:-65536,All white:-65535
            USART_RX_STA = 0;
            switch (state)
            {
            case STATE_MAZE_STAGE1:
            {
                /* code */
                count++;
                if (ReceivedTurnData == -65536)
                {
                    state = STATE_MAZE_STAGE2;
                }
                else if (ReceivedTurnData == -65535)
                {
                    if (count > 30)
                    {
                        state = STATE_MAZE_STAGE2;
                        count = 0;
                    }
                    if (YawControl > 0)
                    {
                        YawControl = 100.0f;
                    }
                    MazeVelocityControl(line_velocity, 0.0f, line_velocity * YawControl);
                    // MazeVelocityControl(line_velocity, 0.0f,0.0f);
                }
                else if (ReceivedTurnData <= 1000 && ReceivedTurnData >= -1000)
                {

                    YawControl = ((float)ReceivedTurnData) / -10.0f;

                    MazeVelocityControl(line_velocity, 0.0f, line_velocity * YawControl);
                    count = 0;
                    // printf("%d\r\n",ReceivedTurnData);
                }
                break;
            }
            case STATE_MAZE_STAGE2:
            {
                /* code */
                yawDesire2 = -yaw;
                VelocityClosedLoopControl(stop_velocity, 0.0f, 0.0f);
                state = STATE_MAZE_STAGE3;
                break;
            }
            case STATE_MAZE_STAGE3:
            {
                /* code */
                VelocityClosedLoopControl(stop_velocity, 0.0f, 0.0f);
                if (ReceivedTurnData == -65535)
                {
                    state = STATE_MAZE_STAGE4;
                    count = 0;
                }
                break;
            }
            case STATE_MAZE_STAGE4:
            {
                /* code */
                count += 1;
                VelocityClosedLoopControl(0.0f, 0.0f, 0.0f);
                if (count == 100)
                {
                    state = STATE_REMOTE;
                    count = 0;
                }

                break;
            }
            case STATE_REMOTE:
            {
                /* code */

                break;
            }

            default:
            {
                break;
            }
            }
        }
        /* USER CODE BEGIN 3 */
    }
}
