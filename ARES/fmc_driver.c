#include "fmc_driver.h"

static int SDRAM_SendCommand(uint32_t CommandMode, uint32_t Bank, uint32_t RefreshNum, uint32_t RegVal)
{
    uint32_t CommandTarget;
    FMC_SDRAM_CommandTypeDef Command;
    
    if (Bank == 1) {
        CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    } else if (Bank == 2) {
        CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
    }
    
    Command.CommandMode = CommandMode;
    Command.CommandTarget = CommandTarget;
    Command.AutoRefreshNumber = RefreshNum;
    Command.ModeRegisterDefinition = RegVal;
    
    if (HAL_SDRAM_SendCommand(&hsdram1, &Command, 0x1000) != HAL_OK) {
        return -1;
    }
    
    return 0;
}

void SDRAM_Init(void)
{
    uint32_t temp;
    
    /* 1. ʱ��ʹ������ */
    SDRAM_SendCommand(FMC_SDRAM_CMD_CLK_ENABLE, 1, 1, 0);
    
    /* 2. ��ʱ������100us */
    HAL_Delay(1);
    
    /* 3. SDRAMȫ��Ԥ������� */
    SDRAM_SendCommand(FMC_SDRAM_CMD_PALL, 1, 1, 0);
    
    /* 4. �Զ�ˢ������ */
    SDRAM_SendCommand(FMC_SDRAM_CMD_AUTOREFRESH_MODE, 1, 8, 0);
    
    /* 5. ����SDRAMģʽ�Ĵ��� */   
    temp = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1            |          //����ͻ�����ȣ�1
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL     |          //����ͻ�����ͣ�����
                     SDRAM_MODEREG_CAS_LATENCY_3             |          //����CLֵ��3
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD   |          //���ò���ģʽ����׼
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;              //����ͻ��дģʽ���������  
    SDRAM_SendCommand(FMC_SDRAM_CMD_LOAD_MODE, 1, 1, temp);
    
    /* 6. ������ˢ��Ƶ�� */
    /*
        SDRAM refresh period / Number of rows��*SDRAMʱ���ٶ� �C 20
      = 64000(64 ms) / 4096 *108MHz - 20
      = 1667.5 ȡֵ1668
    */
    HAL_SDRAM_ProgramRefreshRate(&hsdram1, 1668);
}