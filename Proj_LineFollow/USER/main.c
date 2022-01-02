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
 ALIENTEK ������STM32F746������ ʵ��35
 MPU9250���ᴫ����ʵ��-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾
 ���ߣ�����ԭ�� @ALIENTEK
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
// key�ǵ�ǰ������len���ַ�������
extern float pitch, roll, yaw, yawDesire, yawDesire2,yawLineP; //ŷ����
float yawTemp = 0.0f;
u8 key, len;
int ReceivedTurnData, YawVel; //���ܵĽ��ٶ���
float YawControl = 0.0f;      //���ٶȿ�����
int count = 0;

float stop_velocity = 0.3f;
float stop_velocity1 = 0.25f;
float stop_velocity2 = 0.18f;
float stop_velocity3 = 0.1f;
float line_velocity = 0.55f;     //Ѳ�������ٶ�
float line_velocityOR = 0.7f;   //Ѳ�������ٶ� Base Speed Profile
float line_velocityOC = 0.85f;   //Ѳ�������ٶ� Overclock Profile 1
float line_velocityOC2 = 1.0f; //Ѳ�������ٶ� Overclock Profile 2
int state = STATE_REMOTE;       //״̬������
//����1����1���ַ�
// c:Ҫ���͵��ַ�
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
    Cache_Enable();                  //��L1-Cache
    MPU_Memory_Protection();         //������ش洢����
    HAL_Init();                      //��ʼ��HAL��
    Stm32_Clock_Init(432, 25, 2, 9); //����ʱ��,216Mhz
    delay_init(216);                 //��ʱ��ʼ��
    uart_init(115200);               //���ڳ�ʼ��
    LED_Init();                      //��ʼ����LED���ӵ�Ӳ���ӿ�
    KEY_Init();                      //��ʼ������
    MPU9250_Init();                  //��ʼ��MPU9250

    while (mpu_dmp_init())
    {
        delay_ms(100);
        LED0_Toggle; // DS0��˸
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
            len = USART_RX_STA & 0x3fff;             //�õ��˴ν��յ������ݳ���
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
