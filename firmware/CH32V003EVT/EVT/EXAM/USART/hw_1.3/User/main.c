/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : MakerM0
 * Repo               : https://github.com/MakerM0/LuckyGift
 * Version            : V1.0.0
 * Date               : 2023/12/25
 * Description        : Main program body.
 *******************************************************************************/

#include "debug.h"
#include <string.h>
#include <stdlib.h>
//#include "ws2812.h"

#define AUTHOR "M a k e r M 0"
#define VER_SW "1.0.0"

// #define LOWPOWER

#define VER_1_3 // 需要设置下芯片型号,CH32V003J4M6

void ADC_Function_Init(void);
void ADC_Function_Deinit(void);
// ADC channel 2 (PC4)
u16 Get_ADC_Val(u8 ch);

uint32_t SYS_GetSysTickCnt(void);
void systick_init();
void systick_int_disable(void);

void EXTI0_INT_INIT(void);
void shock_INIT(void);

void ch7800_handle(uint8_t rxdata);
void ch7800_sendCMD(uint8_t *cmd);
#define LENGTH 32

uint8_t aTxBuffer_play[7] = {0x7e, 0x13, 0x00, 0x02, 0x00, 0x01, 0xef};
uint8_t aRxBuffer[LENGTH] = {};

uint8_t aTxBuffer_files[7] = {0x7e, 0x4a, 0x00, 0x02, 0x00, 0x00, 0xef};
uint8_t aTxBuffer_reset[7] = {0x7e, 0x0c, 0x00, 0x02, 0x00, 0x00, 0xef};
uint8_t aTxBuffer_baud[7] = {0x7e, 0x0b, 0x00, 0x02, 0x00, 0x07, 0xef};
uint8_t aTxBuffer_stop[7] = {0x7e, 0x26, 0x00, 0x02, 0x00, 0x00, 0xef};
uint8_t aTxBuffer_factory[7] = {0x7e, 0xd1, 0x00, 0x02, 0x00, 0x00, 0xef};
#define CH_WAITING 50 // ms
#define MAX_NUM 999

#define CH_HEAD 0x7E
#define CH_TAIL 0xEF

#define CMD_SONG_NEXT 0x11         // 下一曲;7E 11 00 02 00 00 EF
#define CMD_SONG_PREV 0x12         // 上一曲;7E 12 00 02 00 00 EF
#define CMD_SONG_SET 0x13          // 指定曲目(NUM) -- 播放第8段;7E 13 00 02 00 08 EF
#define CMD_VOL_HIGH 0x14          // 音量+;7E 14 00 02 00 00 EF
#define CMD_VOL_LOW 0x15           // 音量-;7E 15 00 02 00 00 EF
#define CMD_VOL_SET 0x16           // 指定音量 -- 取值范围[0--30] --指定为10级;7E 16 00 02 00 0A EF
#define CMD_SONG_LOOP 0x18         // 7E 18 00 02 00 01 EF单曲循环指定曲目播放 -- 循环播放第1段
#define CMD_BAUD_SET 0x0b          // 7E 0B 00 02 00 07 EF 指定波特率--115200
#define CMD_RESET 0x0c             // 7E 0C 00 02 00 00 EF 芯片复位
#define CMD_PLAY 0x1d              // 7E 1D 00 02 00 00 EF 播放
#define CMD_PAUSE 0x1e             // 7E 1E 00 02 00 00 EF 暂停
#define CMD_LOOP 0x21              // 7E 21 00 02 00 00 EF 指定当前的设备全部循环播放
#define CMD_STOP 0x26              // 7E 26 00 02 00 00 EF 停止
#define CMD_RANDOM 0x28            // 7E 28 00 02 00 01 EF 指定当前的设备全部随机播放
#define CMD_SONG_CUREENT_LOOP 0x29 // 7E 29 00 02 00 00 EF 当前播放的曲目设置为循环播放，要在播放时发送才有效
#define CMD_SONG_GROUP 0x31        // 7E 31 00 05 01 03 02 05 04 EF 组合播放001/003/002/005/004根目录5个文件,比较方便比如报时功能
#define CMD_FACTORY 0xd1           // 7E D1 00 02 00 00 EF 恢复出厂设置

#define CMD_STATUS 0x42 // 7E 42 00 02 00 00 EF 查询当前状态
#define CMD_VOL 0x43    // 7E 43 00 02 00 00 EF 查询当前音量
#define CMD_FILES 0x4a  // 7E 4A 00 02 00 00 EF 查询总文件数

#define CMD_ACK 0x41 // 芯片主动返回，说明成功接收到数据

#define STEP_HEAD 0
#define STEP_CMD 1
#define STEP_LEN1 2
#define STEP_LEN2 3
#define STEP_DATA 4
#define STEP_TAIL 5

int step = STEP_HEAD;
typedef struct
{
    uint16_t num;
} info_t;

info_t info = {0};

/* Global define */
#define TxSize1 (size(TxBuffer1))
#define size(a) (sizeof(a) / sizeof(*(a)))

/* Global typedef */
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

/* Global Variable */
u8 TxBuffer1[] = "*Buffer1 Send from USART1 using Interrupt!";
u8 RxBuffer1[TxSize1] = {0};

volatile u8 TxCnt1 = 0, RxCnt1 = 0;

volatile u8 Rxfinish1 = 0;

TestStatus TransferStatus1 = FAILED;

/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_SetBits(GPIOD, GPIO_Pin_0);
}

/*********************************************************************
 * @fn      Buffercmp
 *
 * @brief   Compares two buffers
 *
 * @param   Buf1,Buf2 - buffers to be compared
 *          BufferLength - buffer's length
 *
 * @return  PASSED - Buf1 identical to Buf
 *          FAILED - Buf1 differs from Buf2
 */
TestStatus Buffercmp(uint8_t *Buf1, uint8_t *Buf2, uint16_t BufLength)
{
    while (BufLength--)
    {
        if (*Buf1 != *Buf2)
        {
            return FAILED;
        }
        Buf1++;
        Buf2++;
    }
    return PASSED;
}

/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
void USARTx_CFG(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1,
                           ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    ;

    USART_Cmd(USART1, ENABLE);
}

typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIOMode_TypeDef GPIO_Mode;

} io_cfg_t;

io_cfg_t iocfg[] = {
    {GPIOC, GPIO_Pin_0, GPIO_Mode_IN_FLOATING}, // busy
    {GPIOA, GPIO_Pin_2, GPIO_Mode_IN_FLOATING}, // led_pwr
    {GPIOD, GPIO_Pin_0, GPIO_Mode_IPU},         // SHOCK
    {GPIOC, GPIO_Pin_2, GPIO_Mode_IN_FLOATING}, // SCL
    {GPIOC, GPIO_Pin_1, GPIO_Mode_IN_FLOATING}, // SDA
    {GPIOC, GPIO_Pin_6, GPIO_Mode_IN_FLOATING}, // LED
    {GPIOC, GPIO_Pin_3, GPIO_Mode_IN_FLOATING}, // ACC_INT

};

void io_init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    //    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    for (uint8_t i = 0; i < 7; i++)
    {
        GPIO_InitStructure.GPIO_Pin = iocfg[i].GPIO_Pin;
        GPIO_InitStructure.GPIO_Mode = iocfg[i].GPIO_Mode;
        GPIO_Init(iocfg[i].GPIOx, &GPIO_InitStructure);
    }
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    io_init();
    ADC_Function_Init();
    srand(Get_ADC_Val(2));
    ADC_Function_Deinit();

    USARTx_CFG();
    ch7800_sendCMD(aTxBuffer_reset);
    Delay_Ms(1000);
    ch7800_sendCMD(aTxBuffer_files);
    Delay_Ms(50);
    ch7800_sendCMD(aTxBuffer_files);
    Delay_Ms(50);
    ch7800_sendCMD(aTxBuffer_files);
    Delay_Ms(50);
    ch7800_sendCMD(aTxBuffer_files);
    Delay_Ms(50);

    while (!info.num)
        ;
    uint16_t num;
    num = (rand() % info.num) + 1;
    //  num = 2;
    aTxBuffer_play[4] = (num >> 8) & 0xff;
    aTxBuffer_play[5] = num & 0xff;
    ch7800_sendCMD(aTxBuffer_play);
    Delay_Ms(100);
    ch7800_sendCMD(aTxBuffer_stop);

#ifdef LOWPOWER
    EXTI0_INT_INIT();
    __WFI();
#else
    shock_INIT();

#endif
    while (1)
    {
#ifndef VER_1_3
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) == Bit_RESET)
#else
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == Bit_RESET)
#endif
        {

            num = (rand() % info.num) + 1;
            //            num = 2;
            aTxBuffer_play[4] = (num >> 8) & 0xff;
            aTxBuffer_play[5] = num & 0xff;
            ch7800_sendCMD(aTxBuffer_stop);
            Delay_Ms(60);
            ch7800_sendCMD(aTxBuffer_play);
            Delay_Ms(100);
        }
#ifdef LOWPOWER
        __WFI();
#endif
    }
}

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USART1_IRQHandler
 *
 * @brief   This function handles USART3 global interrupt request.
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {

        ch7800_handle((uint8_t)USART_ReceiveData(USART1));
    }
}

volatile static uint32_t systick_millis = 0;
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    systick_millis++;
    SysTick->SR &= ~(1 << 0);
}

/*******************************************************************************
 * Function Name  : SYS_GetSysTickCnt
 * Description    : 获取当前系统(SYSTICK)计数值
 * Input          : None
 * Return         : 当前计数值
 *******************************************************************************/
uint32_t SYS_GetSysTickCnt(void)
{
    uint32_t val;

    val = SysTick->CNT;
    return (val);
}

void systick_init()
{
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->SR &= ~(1 << 0);
    SysTick->CMP = SystemCoreClock - 1;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xF;
}
void systick_int_disable(void)
{
    NVIC_DisableIRQ(SysTicK_IRQn);
}

uint8_t rxbuffer_x[1];
uint8_t id = 0;
uint8_t tmp;

static uint16_t len;
static uint8_t cmd;
static uint8_t data[2];
static uint8_t data_index;
void ch7800_handle(uint8_t rxdata)
{

    {
        aRxBuffer[id] = rxdata;

        id++;
        if (id == LENGTH)
        {
            id = 0;
            memset(aRxBuffer, 0, LENGTH);
        }
        tmp = rxdata;
        switch (step)
        {
        case STEP_HEAD:
            if (tmp == CH_HEAD)
            {
                data_index = 0;
                step = STEP_CMD;
            }
            break;

        case STEP_CMD:
            cmd = tmp;
            if (tmp == CMD_FILES)
            {
                step = STEP_LEN1;
            }
            else if (tmp == CMD_ACK)
            {
                step = STEP_TAIL;
            }
            else
            {
                step = STEP_HEAD;
            }
            break;

        case STEP_LEN1:
            if (tmp == 0)
            {
                step = STEP_LEN2;
            }
            else
            {
                step = STEP_HEAD;
            }
            break;

        case STEP_LEN2:
            if (tmp == 2)
            {
                step = STEP_DATA;
                len = 2;
                data_index = 0;
            }
            else
            {
                step = STEP_HEAD;
            }
            break;

        case STEP_DATA:
            if (data_index < len)
            {
                data[data_index] = tmp;
                data_index++;
            }
            else
            {
                if (cmd == CMD_FILES)
                {
                    uint16_t size = ((data[0] << 8) & 0xff00) | data[1];

                    if (size > 999)
                    {
                        //                        info.num = 1;
                        NVIC_SystemReset();
                    }
                    else
                    {
                        info.num = size;
                    }
                }
                cmd = 0;
                step = STEP_HEAD;
                data_index = 0;
            }
            break;

        case STEP_TAIL:

            if (cmd == CMD_FILES)
            {
                uint16_t size = ((data[0] << 8) & 0xff00) | data[1];

                if (size > 999)
                {
                    //                    info.num = 1;
                    NVIC_SystemReset();
                }
                else
                {
                    info.num = size;
                }
            }
            cmd = 0;
            data_index = 0;
            step = STEP_HEAD;

            break;

        default:
            break;
        }
    }
}

void ch7800_sendCMD(uint8_t *cmd)
{
    for (uint8_t i = 0; i < 7; i++)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) /* waiting for sending finish */
        {
        }
        USART_SendData(USART1, cmd[i]);
    }
}

/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void ADC_Function_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
        ;
}

void ADC_Function_Deinit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ADC_DeInit(ADC1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      Get_ADC_Val
 *
 * @brief   Returns ADCx conversion result data.
 *
 * @param   ch - ADC channel.
 *            ADC_Channel_0 - ADC Channel0 selected.
 *            ADC_Channel_1 - ADC Channel1 selected.
 *            ADC_Channel_2 - ADC Channel2 selected.
 *            ADC_Channel_3 - ADC Channel3 selected.
 *            ADC_Channel_4 - ADC Channel4 selected.
 *            ADC_Channel_5 - ADC Channel5 selected.
 *            ADC_Channel_6 - ADC Channel6 selected.
 *            ADC_Channel_7 - ADC Channel7 selected.
 *            ADC_Channel_8 - ADC Channel8 selected.
 *            ADC_Channel_9 - ADC Channel9 selected.
 *
 * @return  none
 */

// ADC channel 2 (PC4)
u16 Get_ADC_Val(u8 ch)
{
    u16 val;

    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_241Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
        ;
    val = ADC_GetConversionValue(ADC1);

    return val;
}

void shock_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
#ifndef VER_1_3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
#else

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#endif
}
/*********************************************************************
 * @fn      EXTI0_INT_INIT
 *
 * @brief   Initializes EXTI0 collection.
 *
 * @return  none
 */

void EXTI0_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* GPIOA.0 ----> EXTI_Line0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      EXTI0_IRQHandler
 *
 * @brief   This function handles EXTI0 Handler.
 *
 * @return  none
 */
void EXTI7_0_IRQHandler(void)
{

    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        //        printf("EXTI0 Wake_up\r\n");
        EXTI_ClearITPendingBit(EXTI_Line0); /* Clear Flag */
    }
}

void led_loop()
{

}
