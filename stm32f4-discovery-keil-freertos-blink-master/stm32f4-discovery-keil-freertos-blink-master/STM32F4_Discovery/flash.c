#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "flash.h"
#include <string.h>
#include "uprintf.h"
#include "io.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "portmacro.h"
#include "timers.h"

xTimerHandle xADETimer;
uint8_t ui8_flag_ade = 0;
// DEVICE1
#define ADE7758_DEV1_MOSI_PIN           GPIO_Pin_1
#define ADE7758_DEV1_MISO_PIN           GPIO_Pin_12
#define ADE7758_DEV1_SCK_PIN            GPIO_Pin_0
#define ADE7758_DEV1_NSS_PIN            GPIO_Pin_10
#define ADE7758_DEV1_MOSI_GPIO          GPIOD
#define ADE7758_DEV1_MISO_GPIO          GPIOC
#define ADE7758_DEV1_SCK_GPIO           GPIOD
#define ADE7758_DEV1_NSS_GPIO           GPIOC
#define ADE7758_DEV1_GPIO_CLK           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD
//

// DEVICE2
#define ADE7758_DEV2_MOSI_PIN           GPIO_Pin_8
#define ADE7758_DEV2_MISO_PIN           GPIO_Pin_14
#define ADE7758_DEV2_SCK_PIN            GPIO_Pin_15
#define ADE7758_DEV2_NSS_PIN            GPIO_Pin_12
#define ADE7758_DEV2_MOSI_GPIO          GPIOC
#define ADE7758_DEV2_MISO_GPIO          GPIOD
#define ADE7758_DEV2_SCK_GPIO           GPIOD
#define ADE7758_DEV2_NSS_GPIO           GPIOD

#define ADE7758_DEV2_GPIO_CLK           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD
//

// DEVICE3
#define ADE7758_DEV3_MOSI_PIN           GPIO_Pin_15
#define ADE7758_DEV3_MISO_PIN           GPIO_Pin_13
#define ADE7758_DEV3_SCK_PIN            GPIO_Pin_14
#define ADE7758_DEV3_NSS_PIN            GPIO_Pin_15
#define ADE7758_DEV3_MOSI_GPIO          GPIOB
#define ADE7758_DEV3_MISO_GPIO          GPIOB
#define ADE7758_DEV3_SCK_GPIO           GPIOB
#define ADE7758_DEV3_NSS_GPIO           GPIOE
#define ADE7758_DEV3_GPIO_CLK           RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE
//



#define SPI_ADE7758                 				SPI1

#define ADUM5401_ENABLE1_PORT        GPIOD            
#define ADUM5401_ENABLE1_PIN         GPIO_Pin_2

#define ADUM5401_ENABLE2_PORT        GPIOC            
#define ADUM5401_ENABLE2_PIN         GPIO_Pin_9

#define ADUM5401_ENABLE3_PORT        GPIOD           
#define ADUM5401_ENABLE3_PIN         GPIO_Pin_8

#define ADDR_MASK           0x7F
#define ADE_SPI_HARDWARE    0

#define ADE7758_DEV1_SPI_SCK(x)   			((x) ? (GPIO_SetBits(ADE7758_DEV1_SCK_GPIO, ADE7758_DEV1_SCK_PIN)) : (GPIO_ResetBits(ADE7758_DEV1_SCK_GPIO, ADE7758_DEV1_SCK_PIN)))
#define ADE7758_DEV1_SPI_MOSI(x)   			((x) ? (GPIO_SetBits(ADE7758_DEV1_MOSI_GPIO, ADE7758_DEV1_MOSI_PIN)) : (GPIO_ResetBits(ADE7758_DEV1_MOSI_GPIO, ADE7758_DEV1_MOSI_PIN)))
#define ADE7758_DEV1_CS(x)   				((x) ? (GPIO_SetBits(ADE7758_DEV1_MISO_GPIO, ADE7758_DEV1_MISO_PIN)) : (GPIO_ResetBits(ADE7758_DEV1_MISO_GPIO, ADE7758_DEV1_MISO_PIN)))


#define ADE7758_DEV2_SPI_SCK(x)   			((x) ? (GPIO_SetBits(ADE7758_DEV2_SCK_GPIO, ADE7758_DEV2_SCK_PIN)) : (GPIO_ResetBits(ADE7758_DEV2_SCK_GPIO, ADE7758_DEV2_SCK_PIN)))
#define ADE7758_DEV2_SPI_MOSI(x)   			((x) ? (GPIO_SetBits(ADE7758_DEV2_MOSI_GPIO, ADE7758_DEV2_MOSI_PIN)) : (GPIO_ResetBits(ADE7758_DEV2_MOSI_GPIO, ADE7758_DEV2_MOSI_PIN)))
#define ADE7758_DEV2_CS(x)   				((x) ? (GPIO_SetBits(ADE7758_DEV2_MISO_GPIO, ADE7758_DEV2_MISO_PIN)) : (GPIO_ResetBits(ADE7758_DEV2_MISO_GPIO, ADE7758_DEV2_MISO_PIN)))

#define ADE7758_DEV3_SPI_SCK(x)   			((x) ? (GPIO_SetBits(ADE7758_DEV3_SCK_GPIO, ADE7758_DEV3_SCK_PIN)) : (GPIO_ResetBits(ADE7758_DEV3_SCK_GPIO, ADE7758_DEV3_SCK_PIN)))
#define ADE7758_DEV3_SPI_MOSI(x)   			((x) ? (GPIO_SetBits(ADE7758_DEV3_MOSI_GPIO, ADE7758_DEV3_MOSI_PIN)) : (GPIO_ResetBits(ADE7758_DEV3_MOSI_GPIO, ADE7758_DEV3_MOSI_PIN)))
#define ADE7758_DEV3_CS(x)   				((x) ? (GPIO_SetBits(ADE7758_DEV3_MISO_GPIO, ADE7758_DEV3_MISO_PIN)) : (GPIO_ResetBits(ADE7758_DEV3_MISO_GPIO, ADE7758_DEV3_MISO_PIN)))

//extern uint32_t AVRMSCalib;

//extern uint32_t AIRMSCalib;
extern char flag;
uint32_t ui32_ade_timer = 0;


uint32_t ui32_temp = 0;


void ADE7758_init(void)
{
    GPIO_InitTypeDef 	GPIO_InitStructure;
    SPI_InitTypeDef   SPI_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHB1PeriphClockCmd(ADE7758_DEV1_GPIO_CLK | ADE7758_DEV2_GPIO_CLK | ADE7758_DEV3_GPIO_CLK, ENABLE);
    
    if (ADE_SPI_HARDWARE)
    {
        GPIO_InitStructure.GPIO_Pin = ADE7758_DEV1_NSS_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; ////GPIO_Mode_Out_PP
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(ADE7758_DEV1_NSS_GPIO, &GPIO_InitStructure);
        
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
        GPIO_Init(ADE7758_DEV1_MOSI_GPIO, &GPIO_InitStructure);
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);    
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
        
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
        
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
        
//        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
//        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;
        SPI_Init(SPI_ADE7758, &SPI_InitStructure);
        SPI_Cmd(SPI_ADE7758, ENABLE);
        
    }
    else
    {
        GPIO_InitStructure.GPIO_Pin = ADUM5401_ENABLE1_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT; //GPIO_Mode_Out_PP
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(ADUM5401_ENABLE1_PORT, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = ADUM5401_ENABLE2_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT; //GPIO_Mode_Out_PP
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(ADUM5401_ENABLE2_PORT, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = ADUM5401_ENABLE3_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT; //GPIO_Mode_Out_PP
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(ADUM5401_ENABLE3_PORT, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = ADE7758_DEV1_SCK_PIN | ADE7758_DEV1_MOSI_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT; //GPIO_Mode_Out_PP
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(ADE7758_DEV1_MOSI_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = ADE7758_DEV1_MISO_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN; //GPIO_Mode_IPU
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(ADE7758_DEV1_MISO_GPIO, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = ADE7758_DEV1_NSS_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; ////GPIO_Mode_Out_PP
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(ADE7758_DEV1_NSS_GPIO, &GPIO_InitStructure);
        
//        GPIO_InitStructure.GPIO_Pin = ADE7758_NSS2 | ADE7758_NSS3 | ADE7758_NSS4;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //GPIO_Mode_Out_PP
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        GPIO_Init(ADE7758_NSS2_GPIO, &GPIO_InitStructure);
    }
    ADE7758_off(1); 
//    ADE7758_off(2); 
//    ADE7758_off(3); 
//    ADE7758_off(4); 
}

void ADUM5401_on(char adum)
{
    if(adum == 1)
    {
        GPIO_ResetBits(ADUM5401_ENABLE1_PORT, ADUM5401_ENABLE1_PIN);
    }
    else
    if(adum == 2)
    {
        GPIO_ResetBits(ADUM5401_ENABLE2_PORT, ADUM5401_ENABLE2_PIN);
    }
    else
    if(adum == 3)
    {
        GPIO_ResetBits(ADUM5401_ENABLE3_PORT, ADUM5401_ENABLE3_PIN);
    }
}
void ADUM5401_off(char adum)
{
    if(adum == 1)
    {
        GPIO_SetBits(ADUM5401_ENABLE1_PORT, ADUM5401_ENABLE1_PIN);
    }
    else
    if(adum == 2)
    {
        GPIO_SetBits(ADUM5401_ENABLE2_PORT, ADUM5401_ENABLE2_PIN);
    }
    else
    if(adum == 3)
    {
        GPIO_SetBits(ADUM5401_ENABLE3_PORT, ADUM5401_ENABLE3_PIN);
    }
}

void ADE7758_on(char ade)
{
    if(ade == 1){
    
        GPIO_ResetBits(ADE7758_DEV1_NSS_GPIO, ADE7758_DEV1_NSS_PIN);
    }
    else if(ade == 2)
    {
        GPIO_ResetBits(ADE7758_DEV2_NSS_GPIO, ADE7758_DEV2_NSS_PIN);
    }
    else if(ade == 3)
        GPIO_ResetBits(ADE7758_DEV3_NSS_GPIO, ADE7758_DEV3_NSS_PIN);
}
void ADE7758_off(char ade)
{
    if(ade == 1)
    {
        GPIO_SetBits(ADE7758_DEV1_NSS_GPIO, ADE7758_DEV1_NSS_PIN);
    }
    else if(ade == 2)
    {
        GPIO_SetBits(ADE7758_DEV2_NSS_GPIO, ADE7758_DEV2_NSS_PIN);
    }
    else if(ade == 3)
        GPIO_SetBits(ADE7758_DEV3_NSS_GPIO, ADE7758_DEV3_NSS_PIN);
}

void vADE7758_timer_cb(void)
{
    ui32_ade_timer++;
    uprintf("tm\r\n");
}

void ADE7758_timer_init(void)
{ 
    xADETimer = xTimerCreate( "Timer",1/portTICK_RATE_MS, pdTRUE, ( void * ) 0,(tmrTIMER_CALLBACK)vADE7758_timer_cb); 
    xTimerStart( xADETimer, 0 );
}


void ADE7758_delay(void)
{
//    xTimerStart( xADETimer, 0 );
//    ui32_ade_timer = 0;
////    uprintf("delay\r\n");
//    while (1)
//    {
//        if(ui32_ade_timer < 5)
//        {
//            
//        }
//        else
//        {
//            uprintf("ss\r\n");
//            xTimerStop(xADETimer, 0);
//            break;
//        }
//    }
//    xTimerStop(xADETimer, 0);
    static uint32_t ui32_time = 0;
    for (ui32_time = 0; ui32_time < 5000; ui32_time++) //5000
    {
////        __nop();
    }

}

void ADE7758_data_init(void)
{
    flag_ade_connect data_init;
    data_init.flag_chanel123 = 0;
    data_init.flag_chanel456 = 0;
    data_init.flag_chanel789 = 0;
}
uint8_t ADE7758_Check(uint8_t chanel)
{
    uint8_t u8_temp = 0;
    u8_temp = (uint8_t)ADE7758Read8Op(chanel, 0x13);
    if (u8_temp == 0x04)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint16_t ADE7758_LineFreq(char phase)
{
    uint8_t mode;
    mode = ADE7758Read8Op(1, MMODE);
    ADE7758Write8Op(1, MMODE, ((mode & 0xFC) | 0));
    //Delay(10);
    return ADE7758Read16Op(1, FREQ);
}
float ADE7758_GetTime(void)
{
    uint16_t frequency, LineCYC;
    float LineFrequency ;
    frequency = ADE7758_LineFreq(PHASEA);
    LineCYC = ADE7758Read16Op(1, LINECYC);
    ADE7758Write8Op(1, LCYCMODE, 0xBF);
    LineFrequency = 1/(frequency*0.0000096);
    return LineCYC/(2*LineFrequency*3);
}
void ADE7758_GetEnery(uint8_t phase, uint16_t *AWattHr, uint16_t *BWattHr, uint16_t *CWattHr)
{
    ADE7758_LineFreq(PHASEA);
    ADE7758Write8Op(1, LCYCMODE, 0xBF);
    ADE7758Write16Op(1, LINECYC, 0x800);
    ADE7758Read16Op(1, MASK);
    ADE7758Write24Op(1, MASK, 1 << LENERGY);
    //Delay(10);
    ADE7758Read24Op(1, RSTATUS);
    while(flag == 1){};
        flag = 0;
    *AWattHr = ADE7758Read16Op(1, AWATTHR);
    *BWattHr = ADE7758Read16Op(1, BWATTHR);
    *CWattHr = ADE7758Read16Op(1, CWATTHR);
}

void ADE7758_GetEnery1(uint8_t phase, uint8_t samplingPeriod, uint16_t *AWattHr, uint16_t *BWattHr, uint16_t *CWattHr)
{
    float period = 0;
    long AWattHrSum = 0;
    long BWattHrSum = 0;    
    long CWattHrSum = 0;
    uint16_t AWattHrValue, BWattHrValue, CWattHrValue;
    while (period < samplingPeriod*60.0) {
        period += ADE7758_GetTime();
        ADE7758_GetEnery(phase, &AWattHrValue, &BWattHrValue, &CWattHrValue);
        AWattHrSum += AWattHrValue;
        BWattHrSum += BWattHrValue;
        CWattHrSum += CWattHrValue;

    }
    *AWattHr = AWattHrSum * 1;
    *BWattHr = BWattHrSum * 1;
    *CWattHr = CWattHrSum * 1;
}
void ADE7758_Calib(void)
{
    ADE7758Write16Op(1, AWG, 0x0000);
    ADE7758Write16Op(1, BWG, 0x0000);
    ADE7758Write16Op(1, CWG, 0x0000);
    ADE7758Write16Op(1, AVARG, 0x0000);
    ADE7758Write16Op(1, BVARG, 0x0000);
    ADE7758Write16Op(1, CVARG, 0x0000);
    ADE7758Write16Op(1, AVAG, 0x0000);
    ADE7758Write16Op(1, BVAG, 0x0000);
    ADE7758Write16Op(1, CVAG, 0x0000);
}
int ADE7758_ReadWrite(uint8_t chanel, unsigned char writedat)
{
    if (ADE_SPI_HARDWARE)
    {
        /* Loop while DR register in not emplty */
        while(SPI_I2S_GetFlagStatus(SPI_ADE7758, SPI_I2S_FLAG_TXE) == RESET);
         
        /* Send byte through the SPI1 peripheral */
        SPI_I2S_SendData(SPI_ADE7758, writedat);
         
        /* Wait to receive a byte */
        while(SPI_I2S_GetFlagStatus(SPI_ADE7758, SPI_I2S_FLAG_RXNE) == RESET);
         
        /* Return the byte read from the SPI bus */
        return SPI_I2S_ReceiveData(SPI_ADE7758);
    }
    else
    {
        unsigned char i = 0,data = 0;
        for (i=0;i<8;i++)
        { 
            if(chanel == 1)
                ADE7758_DEV1_SPI_SCK(1);
            else if(chanel == 2)
                ADE7758_DEV2_SPI_SCK(1);
            else if(chanel == 2)
                ADE7758_DEV3_SPI_SCK(1);
//            Delay(1);
            
            ADE7758_delay();
            if(writedat&0x80)
            {
//                ADE7758_DEV1_SPI_MOSI(1);
                if(chanel == 1)
                    ADE7758_DEV1_SPI_MOSI(1);
                else if(chanel == 2)
                    ADE7758_DEV2_SPI_MOSI(1);
                else if(chanel == 2)
                    ADE7758_DEV3_SPI_MOSI(1);
            }
            else
            {
//                ADE7758_DEV1_SPI_MOSI(0);
                if(chanel == 1)
                    ADE7758_DEV1_SPI_MOSI(0);
                else if(chanel == 2)
                    ADE7758_DEV2_SPI_MOSI(0);
                else if(chanel == 2)
                    ADE7758_DEV3_SPI_MOSI(0);
            }
//            Delay(1);
            ADE7758_delay();
            writedat=writedat<<1;
            if (chanel == 1)
                data=GPIO_ReadInputDataBit(ADE7758_DEV1_MISO_GPIO, ADE7758_DEV1_MISO_PIN);
            else if (chanel == 2)
                data=GPIO_ReadInputDataBit(ADE7758_DEV2_MISO_GPIO, ADE7758_DEV2_MISO_PIN);
            else if (chanel == 3)
                data=GPIO_ReadInputDataBit(ADE7758_DEV3_MISO_GPIO, ADE7758_DEV3_MISO_PIN);
            writedat=writedat|data;
//            ADE7758_DEV1_SPI_SCK(0);
            if(chanel == 1)
                ADE7758_DEV1_SPI_SCK(0);
            else if(chanel == 2)
                ADE7758_DEV2_SPI_SCK(0);
            else if(chanel == 2)
                ADE7758_DEV3_SPI_SCK(0);
            ADE7758_delay();
//            Delay(1);
        }
        return writedat;
    }
}
void ADE7758_setpara(uint8_t chanel)
{
    uprintf("setpara\r\n");
    ADE7758Write8Op(chanel, GAIN, 0x00);
    //Delay(10);
    ADE7758Write8Op(chanel, WDIV, 0x00);
    //Delay(10);
    ADE7758Write16Op(chanel, AWG, 0x000);
    //Delay(10);
    ADE7758Write16Op(chanel, AWAITOS, 0x000);
    //Delay(10);
    ADE7758Write8Op(chanel, VADIV, 0x00);
    //Delay(10);
    ADE7758Write16Op(chanel, AVAG, 0x00);
    //Delay(10);
    ADE7758Write16Op(chanel, AVAROS, 0x00);
    //Delay(10); //AIGAIN
    ADE7758Write16Op(chanel, AIGAIN, 0x00);
    //Delay(10);
    ADE7758Write8Op(chanel, OPMODE, 0x04);
    //Delay(10);
    ADE7758Write8Op(chanel, COMPMODE, 0x04);
    //Delay(10);
    
}

float ADE7758_readVRMS(uint8_t chanel, uint8_t phase)
{
    float RRMS = 0;

    ADE7758Write8Op(chanel, MMODE, 0x04);
    if (phase == 1)
        RRMS = (float)(((ADE7758Read24Op(chanel, AVRMS) * 500) / 0x193504) / 1.4145);  //1651972
    else if (phase == 2)
        RRMS = (float)(((ADE7758Read24Op(chanel, BVRMS) * 500) / 0x193504) / 1.4145);
    else if (phase == 3)
        RRMS = (float)(((ADE7758Read24Op(chanel, CVRMS) * 500) / 0x193504) / 1.4145);
    return RRMS;
}

float ADE7758_readIRMS(uint8_t chanel, uint8_t phase)
{
    float IRMS = 0;
    ADE7758Write8Op(chanel, OPMODE, 0x04);
    if (phase == 1)
        IRMS = (float)(((ADE7758Read24Op(chanel, AIRMS)) * 1.6 / 0x1D51C1) / 1.4145); //1921473
    else if (phase == 2)
        IRMS = (float)(((ADE7758Read24Op(chanel, BIRMS)) * 1.6 / 0x1D51C1) / 1.4145);
    else if (phase == 3)
        IRMS = (float)(((ADE7758Read24Op(chanel, CIRMS)) / 0x1D51C1) / 1.4145);
    return IRMS;
    
}
void ADE7758_readLENGRY(void)
{
    ADE7758Write8Op(1, LCYCMODE, 0x0D);
    //Delay(20);
    ADE7758Write16Op(1, LINECYC, 0x100);
    //Delay(20);
    ADE7758Read24Op(1, RSTATUS);
    while( !((ADE7758Read24Op(1, STATUS)) & 0x1000)); 
    
//    AVRMSCalib = (uint32_t)ADE7758Read16Op(1, 0x01);
    
//    AIRMSCalib = (uint32_t)ADE7758Read16Op(1, 0x07);
    
    ADE7758Read24Op(1, RSTATUS);
    
}
uint8_t ADE7758Read8Op(uint8_t chanel, unsigned char address)
{
    unsigned char dat = 0;
    
    ADE7758_on(chanel);
    dat = 0x00 | (address & ADDR_MASK);
    ADE7758_ReadWrite(chanel, dat);
    dat = ADE7758_ReadWrite(chanel, 0x00);
    ADE7758_off(chanel);
    return dat;
}

uint16_t ADE7758Read16Op(uint8_t chanel, unsigned char address)
{
    unsigned char dat = 0, dat1 = 0;
     
    ADE7758_on(chanel);
//    //Delay(1);
    dat = 0x00 | (address & ADDR_MASK);
    ADE7758_ReadWrite(chanel, dat);
//    //Delay(1);
    dat = ADE7758_ReadWrite(chanel, 0x00);
//    //Delay(1);
    dat1 = ADE7758_ReadWrite(chanel, 0x00);
    // release CS
    ADE7758_off(chanel);
    
    return (dat << 8) | dat1;
}

uint32_t ADE7758Read24Op(uint8_t chanel, unsigned char address)
{
    unsigned char dat = 0, dat1 = 0, dat2 = 0;
     
    ADE7758_on(chanel);
//    //Delay(1);
    dat = 0x00 | (address & ADDR_MASK);
    ADE7758_ReadWrite(chanel, dat);
//    //Delay(1);
    dat = ADE7758_ReadWrite(chanel, 0x00);
//    //Delay(1);
    dat1 = ADE7758_ReadWrite(chanel, 0x00);
//    //Delay(1);
    dat2 = ADE7758_ReadWrite(chanel, 0x00);
    // release CS
    ADE7758_off(chanel);
    
    return (dat << 16) | (dat1 << 8) | dat2;
}
 
void ADE7758Write8Op(uint8_t chanel, unsigned char address, unsigned char data)
{
    unsigned char dat = 0;
         
    ADE7758_on(chanel);
    //issue write command
//    //Delay(1);
    dat = 0x80 | (address & ADDR_MASK);
//    //Delay(1);
    ADE7758_ReadWrite(chanel, dat);
    //write data
//    //Delay(1);
    dat = data;
    ADE7758_ReadWrite(chanel, dat);
    ADE7758_off(chanel);
}

void ADE7758Write16Op(uint8_t chanel, unsigned char address, uint16_t data)
{
    unsigned char dat = 0;
        
    ADE7758_on(chanel);
    //issue write command
//    //Delay(10);
    dat = 0x80 | (address & ADDR_MASK);
//    //Delay(1);
    ADE7758_ReadWrite(chanel, dat);
    //write data
    dat = (data >> 8) & 0xFF; 
//    //Delay(1);
    ADE7758_ReadWrite(chanel, dat);
    
    dat = data & 0xFF; 
//    //Delay(1);
    ADE7758_ReadWrite(chanel, dat);
    ADE7758_off(chanel);
}

void ADE7758Write24Op(uint8_t chanel, unsigned char address, uint32_t data)
{
    unsigned char dat = 0;
         
    ADE7758_on(chanel);
    //issue write command
//    //Delay(1);
    dat = 0x80 | (address & ADDR_MASK);
//    //Delay(10);
    ADE7758_ReadWrite(chanel, dat);
    //write data
    dat = (data >> 16) & 0xFF;
//    //Delay(1);
    ADE7758_ReadWrite(chanel, dat);
        //write data
    dat = (data >> 8) & 0xFF;
//    //Delay(1);
    ADE7758_ReadWrite(chanel, dat);
        //write data
    dat = data & 0xFF;
//    //Delay(1);
    ADE7758_ReadWrite(chanel, dat);
    ADE7758_off(chanel);
}
uint32_t ADE7758getVRMS(char phase)
{
    char i=0;
    uint32_t volts=0;
    ADE7758Read24Op(1, AVRMS);
    for(i=0;i<10;++i){
        volts+=ADE7758Read24Op(1, AVRMS);
        //Delay(1);
    }
    //average
    return volts/10;
}
uint32_t ADE7758_test(char numSamples)
{
//    uint32_t AVRMS=0;
    uint8_t i = 0;
    uint32_t AVRMSBuffer = 0;
    uint32_t AIRMSBuffer = 0;
    ADE7758Write8Op(1, LCYCMODE, 0x38);
    ADE7758Write24Op(1, MASK, 0xE00);

    for (i=0; i < numSamples; i++) {
        ADE7758Read24Op(1, RSTATUS);
//        while ( _ADEINT != 0 ) { } // wait until INT go low
        
        while(flag == 0){}

            AVRMSBuffer += ADE7758getVRMS(0);
            AIRMSBuffer += ADE7758getVRMS(0);
            


    }
    flag = 1;
//    AVRMSCalib = AVRMSBuffer/numSamples;

//    AIRMSCalib = AIRMSBuffer/numSamples;
    return 0;

}
//void ADE7758_test(void)
//{
//    unsigned int a = 0, b = 0, c = 0;
//    GPIO_WriteBit(GPIOC, ADE7758_NSS , (BitAction)(0));

//    SPI_ADE7758_SendByte(0xD7);	
//    a = SPI_ADE7758_ReadByte(0xFF);

//    GPIO_WriteBit(GPIOC, ADE7758_NSS , (BitAction)(1));
//    printf("ADE7758: %d, %d, %d\r\n",a,b,c);
//}

//unsigned char DF_Read_status_Register(void)
//{
//	unsigned char rData=0;
//	GPIO_WriteBit(GPIOC, ADE7758_NSS , (BitAction)(0));

//	SPI_ADE7758_SendByte(0xD7);			//write opcode

//	///rData = SPI_ADE7758_ReadByte(0x00);					//read device's status 
//	rData = SPI_ADE7758_ReadByte(0xFF);			//read device's status

//	GPIO_WriteBit(GPIOC, ADE7758_NSS , (BitAction)(1));

//	return rData;
//}

////uint8_t SPI_ADE7758_SendByte(uint8_t byte)
////{	
////	/* Loop while DR register in not emplty */
////	while (SPI_I2S_GetFlagStatus(SPI_ADE7758, SPI_I2S_FLAG_TXE) == RESET);
////	//while (SPI_I2S_GetFlagStatus(SPI_ADE7758, SPI_I2S_FLAG_BSY) == SET);	
////	/* Send byte through the SPI1 peripheral */
////	SPI_I2S_SendData(SPI_ADE7758, byte);

////	/* Wait to receive a byte */
////	while (SPI_I2S_GetFlagStatus(SPI_ADE7758, SPI_I2S_FLAG_RXNE) == RESET);

////	/* Return the byte read from the SPI bus */
////	return SPI_I2S_ReceiveData(SPI_ADE7758);				
////}

////uint8_t SPI_ADE7758_ReadByte(uint8_t data)
////{
////	return (SPI_ADE7758_SendByte(data));
////}
