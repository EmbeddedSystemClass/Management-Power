#include "main.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "uprintf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "portmacro.h"

#include "io.h"
#include "flash.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>
char flag = 0;
//SemaphoreHandle_t
//xSemaphoreHandle xSemaphore = NULL;
xSemaphoreHandle semaphoreLcd = NULL;
display_button button_state_lcd = DISPLAY_BT_START;
#define LED_DEBUG1_PIN              GPIO_Pin_3
#define LED_DEBUG1_GPIO             GPIOB
#define LED_DEBUG1_GPIO_CLK         RCC_AHB1Periph_GPIOB

#define LED_DEBUG2_PIN              GPIO_Pin_7
#define LED_DEBUG2_GPIO             GPIOD
#define LED_DEBUG2_GPIO_CLK         RCC_AHB1Periph_GPIOD

#define LED_DEBUG3_PIN              GPIO_Pin_6
#define LED_DEBUG3_GPIO             GPIOD
#define LED_DEBUG3_GPIO_CLK         RCC_AHB1Periph_GPIOD

#define BUTTON_LCD_PIN                  GPIO_Pin_13
#define BUTTON_LCD_GPIO                 GPIOE
#define BUTTON_LCD_GPIO_CLK             RCC_AHB1Periph_GPIOE

#define BUTTON_LCD_EXTI_LINE                        EXTI_Line13
#define BUTTON_LCD_EXTI_PORT_SOURCE                 EXTI_PortSourceGPIOE
#define BUTTON_LCD_EXTI_PIN_SOURCE                  EXTI_PinSource13
#define BUTTON_LCD_EXTI_IRQn                        EXTI15_10_IRQn 
#define BUTTON_LCD_IRQ                              EXTI15_10_IRQHandler

#define DEVICE1_PIN                 GPIO_Pin_3
#define DEVICE2_PIN                 GPIO_Pin_4
#define DEVICE3_PIN                 GPIO_Pin_5

#define DEVICE4_PIN                 GPIO_Pin_8
#define DEVICE5_PIN                 GPIO_Pin_9
#define DEVICE6_PIN                 GPIO_Pin_15

#define DEVICE7_PIN                 GPIO_Pin_9
#define DEVICE8_PIN                 GPIO_Pin_10
#define DEVICE9_PIN                 GPIO_Pin_11

#define DEVICE1_GPIO                GPIOD
#define DEVICE2_GPIO                GPIOD
#define DEVICE3_GPIO                GPIOD

#define DEVICE4_GPIO                GPIOA
#define DEVICE5_GPIO                GPIOA
#define DEVICE6_GPIO                GPIOA

#define DEVICE7_GPIO                GPIOD
#define DEVICE8_GPIO                GPIOD
#define DEVICE9_GPIO                GPIOD

#define DEV123_GPIO                 GPIOD
#define DEV456_GPIO                 GPIOA
#define DEV789_GPIO                 GPIOD
#define DEVICE_GPIO_CLK             RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD

#define DEVICE1_MODE                GPIO_Mode_Out_PP
#define DEVICE2_MODE                GPIO_Mode_Out_PP
#define DEVICE3_MODE                GPIO_Mode_Out_PP
#define DEVICE4_MODE                GPIO_Mode_Out_PP
#define DEVICE5_MODE                GPIO_Mode_Out_PP
#define DEVICE6_MODE                GPIO_Mode_Out_PP

float f_ade_vrms[10];
float f_ade_irms[10];
float f_ade_pwr[10];

uint8_t ui8_button_state;

void EXTI15_10_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(EXTI_GetITStatus(BUTTON_LCD_EXTI_LINE) != RESET)
    {
//        uprintf("aa\r\n");
        xSemaphoreGiveFromISR(semaphoreLcd, &xHigherPriorityTaskWoken);
        EXTI_ClearITPendingBit(BUTTON_LCD_EXTI_LINE);
    }
}

void io_button_proc(void)
{
    semaphoreLcd = xSemaphoreCreateMutex();
    if(semaphoreLcd == NULL)
    {
		uprintf("create semaphore failed\n\r");
	}
    xSemaphoreTake(semaphoreLcd, 0);
    xTaskCreate( xButtonProc, (const signed char*)"LCD_Proc\r\n", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY + LED_TASK_PRIORITY, NULL );
    
}

void xButtonProc(void * pvParameters )
{
    VRMS_ADE proc_vrms;
    IRMS_ADE proc_irms;
    flag_ade_connect flag_ade_proc;
    static char buf[21];
    static char buf1[7];
    static char buf2[21];
    static uint8_t ui8_button_count = 0;
    static uint8_t ui8_display_timeout = 0;
    for(;;)
    {
        uprintf("BT proc\r\n");
        lcd_Clear();
        if(xSemaphoreTake(semaphoreLcd, 0)== pdTRUE)
        {
            ui8_button_count++;
            if (ui8_button_count >= 2)
            {
                ui8_button_count = 0;
                ui8_display_timeout = 0;
                ui8_button_state++;
                if (ui8_button_state == (DISPLAY_BT_DEVICE9 + 1))
                    ui8_button_state = DISPLAY_BT_MAIN;
                uprintf("interrupt: %d\r\n", ui8_button_state);
            }
        }
        
        if (ui8_button_state == DISPLAY_BT_START)
            button_state_lcd = DISPLAY_BT_START;
        if (ui8_button_state == DISPLAY_BT_MAIN_TIME)
            button_state_lcd = DISPLAY_BT_MAIN_TIME;
        if (ui8_button_state == DISPLAY_BT_MAIN)
            button_state_lcd = DISPLAY_BT_MAIN;
        if (ui8_button_state == DISPLAY_BT_DEVICE1)
            button_state_lcd = DISPLAY_BT_DEVICE1;
        if (ui8_button_state == DISPLAY_BT_DEVICE2)
            button_state_lcd = DISPLAY_BT_DEVICE2;
        if (ui8_button_state == DISPLAY_BT_DEVICE3)
            button_state_lcd = DISPLAY_BT_DEVICE3;
        if (ui8_button_state == DISPLAY_BT_DEVICE4)
            button_state_lcd = DISPLAY_BT_DEVICE4;
        if (ui8_button_state == DISPLAY_BT_DEVICE5)
            button_state_lcd = DISPLAY_BT_DEVICE5;
        if (ui8_button_state == DISPLAY_BT_DEVICE6)
            button_state_lcd = DISPLAY_BT_DEVICE6;
        if (ui8_button_state == DISPLAY_BT_DEVICE7)
            button_state_lcd = DISPLAY_BT_DEVICE7;   
        if (ui8_button_state == DISPLAY_BT_DEVICE8)
            button_state_lcd = DISPLAY_BT_DEVICE8;     
        if (ui8_button_state == DISPLAY_BT_DEVICE9)
            button_state_lcd = DISPLAY_BT_DEVICE9;
        switch(button_state_lcd)
            {
                case DISPLAY_BT_START:
                    uprintf("AGRITECH\r\n");
                    lcd_Goto(0,0);
                    lcd_Print_Data("AGRITECH");
                    break;
                case DISPLAY_BT_MAIN_TIME:
                    uprintf("DISPLAY_BT_MAIN_TIME\r\n");
                    break;
                case DISPLAY_BT_MAIN:
                    uprintf("DISPLAY_BT_MAIN\r\n");
                    lcd_Goto(0,0);
                    sprintf(buf,"DV1:XX DV2:XX DV3: XX");
                    lcd_Print_Data(buf);
                    if (flag_ade_proc.flag_chanel123 == 1)
                    {
                        lcd_Goto(0,3);
                        lcd_Print_Data("OK");
                        
                    }
                    if (flag_ade_proc.flag_chanel456 == 1)
                    {
                        lcd_Goto(0,10);
                        lcd_Print_Data("OK");
                    }
                    if (flag_ade_proc.flag_chanel789 == 1)
                    {
                        lcd_Goto(0,17);
                        lcd_Print_Data("OK");
                    }
                    
                    if((flag_ade_proc.flag_chanel123 == 1) && (flag_ade_proc.flag_chanel456 == 1) && (flag_ade_proc.flag_chanel789 == 1))
                    {
                        lcd_Goto(1,0);
                        sprintf(buf2, " AVG_VRMS: %.2f (V) ", proc_vrms.f_AVG_VRMS);
                        lcd_Print_Data(buf2);
                        lcd_Goto(2,0);
                        sprintf(buf2, " SUM_IRMS: %.2f (V) ", proc_irms.f_SUM_IRMS);
                        lcd_Print_Data(buf2);
                        lcd_Goto(3,0);
                        sprintf(buf2, " SUM_PWR : %.2f (V) ", (proc_irms.f_SUM_IRMS) * (proc_vrms.f_AVG_VRMS));
                        lcd_Print_Data(buf2);
                        
                    }
                    break;
                case DISPLAY_BT_DEVICE1:
                    
                    if(flag_ade_proc.flag_chanel123 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE1\r\n");
                        io_lcd_printf(DEVICE1);
                    }   
                    //New
                    else  
                        if(flag_ade_proc.flag_chanel456 == 1)
                            ui8_button_state = DISPLAY_BT_DEVICE4;
                    else
                        if(flag_ade_proc.flag_chanel789 == 1)
                            ui8_button_state = DISPLAY_BT_DEVICE7;
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;
                    //end New
                    break;
                    
                case DISPLAY_BT_DEVICE2:
                    
                    if(flag_ade_proc.flag_chanel123 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE2\r\n");
                        io_lcd_printf(DEVICE2);
                    }   
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;                    
                    break;    

                case DISPLAY_BT_DEVICE3:
                    
                    if(flag_ade_proc.flag_chanel123 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE3\r\n");
                        io_lcd_printf(DEVICE3);
                    }  
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;                    
                    break;      
                case DISPLAY_BT_DEVICE4:
                    
                    if(flag_ade_proc.flag_chanel456 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE4\r\n");
                        io_lcd_printf(DEVICE4);
                    }   
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;                    
                    break;    
                case DISPLAY_BT_DEVICE5:
                    
                    if(flag_ade_proc.flag_chanel456 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE5\r\n");
                        io_lcd_printf(DEVICE5);
                    }  
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;                    
                    break;    
                case DISPLAY_BT_DEVICE6:
                    
                    if(flag_ade_proc.flag_chanel456 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE6\r\n");
                        io_lcd_printf(DEVICE6);
                    }   
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;                    
                    break;    
                case DISPLAY_BT_DEVICE7:
                    
                    if(flag_ade_proc.flag_chanel789 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE7\r\n");
                        io_lcd_printf(DEVICE7);
                    }  
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;
                    break;    
                case DISPLAY_BT_DEVICE8:
                    
                    if(flag_ade_proc.flag_chanel789 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE8\r\n");
                        io_lcd_printf(DEVICE8);
                    }   
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;                    
                    break;    
                case DISPLAY_BT_DEVICE9:
                    
                    if(flag_ade_proc.flag_chanel789 == 1)
                    {
                        uprintf("DISPLAY_BT_DEVICE9\r\n");
                        io_lcd_printf(DEVICE9);
                    }  
                    else
                        ui8_button_state = DISPLAY_BT_MAIN;                    
                    break;                        
                
                default: break;
                    
            }
            ui8_display_timeout++;
            uprintf("timeout: %d %d %d\r\n", ui8_button_state, ui8_display_timeout, button_state_lcd);
            if(ui8_display_timeout == 5)
            {
                
                if(ui8_button_state == DISPLAY_BT_MAIN)
                {
                    ui8_button_state = DISPLAY_BT_MAIN_TIME;
                    uprintf("main_time\r\n");
                }
//                ui8_display_timeout = 0;
            }
            else
            if(ui8_display_timeout == 10)
            {
                ui8_display_timeout = 0;
                if (ui8_button_state != DISPLAY_BT_MAIN_TIME)
                    ui8_button_state = DISPLAY_BT_MAIN;
            }
        vTaskDelay(500/portTICK_RATE_MS);
    }
}

void io_lcd_printf(uint8_t device)
{
    VRMS_ADE proc_vrms_data;
    IRMS_ADE proc_irms_data;
    static char buf2[21];
    uint8_t ui8_t = 0;

    
    f_ade_vrms[1] = proc_vrms_data.f_VRMS_DEV1;
    f_ade_vrms[2] = proc_vrms_data.f_VRMS_DEV2;
    f_ade_vrms[3] = proc_vrms_data.f_VRMS_DEV3;
    f_ade_vrms[4] = proc_vrms_data.f_VRMS_DEV4;
    f_ade_vrms[5] = proc_vrms_data.f_VRMS_DEV5;
    f_ade_vrms[6] = proc_vrms_data.f_VRMS_DEV6;
    f_ade_vrms[7] = proc_vrms_data.f_VRMS_DEV7;
    f_ade_vrms[8] = proc_vrms_data.f_VRMS_DEV8;
    f_ade_vrms[9] = proc_vrms_data.f_VRMS_DEV9;
    
    f_ade_irms[1] = proc_irms_data.f_IRMS_DEV1;
    f_ade_irms[2] = proc_irms_data.f_IRMS_DEV2;
    f_ade_irms[3] = proc_irms_data.f_IRMS_DEV3;
    f_ade_irms[4] = proc_irms_data.f_IRMS_DEV4;
    f_ade_irms[5] = proc_irms_data.f_IRMS_DEV5;
    f_ade_irms[6] = proc_irms_data.f_IRMS_DEV6;
    f_ade_irms[7] = proc_irms_data.f_IRMS_DEV7;
    f_ade_irms[8] = proc_irms_data.f_IRMS_DEV8;
    f_ade_irms[9] = proc_irms_data.f_IRMS_DEV9;
    lcd_Goto(1,0);
    sprintf(buf2, " VRMS_1:6: %.2f (V) ", f_ade_vrms[device]);
    lcd_Print_Data(buf2);
    lcd_Goto(2,0);
    sprintf(buf2, " IRMS_1:6: %.2f (V) ", f_ade_irms[device]);
    lcd_Print_Data(buf2);
    f_ade_pwr[device] = f_ade_vrms[device] * f_ade_irms[device];
    lcd_Goto(3,0);
    sprintf(buf2, " PWR1:6 : %.2f (V) ", f_ade_pwr[device]);
    lcd_Print_Data(buf2);  
}
void io_button_init(void)
{
    GPIO_InitTypeDef 	GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(BUTTON_LCD_GPIO_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = BUTTON_LCD_PIN;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN;            
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_LCD_GPIO, &GPIO_InitStructure);
    
    SYSCFG_EXTILineConfig(BUTTON_LCD_EXTI_PORT_SOURCE, BUTTON_LCD_EXTI_PIN_SOURCE);
    
    EXTI_InitStructure.EXTI_Line = BUTTON_LCD_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_LCD_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY - 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_Init(&NVIC_InitStructure);
    
}
void io_led_debug_init(void)
{
    GPIO_InitTypeDef 	GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(LED_DEBUG1_GPIO_CLK | LED_DEBUG2_GPIO_CLK | LED_DEBUG3_GPIO_CLK, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = LED_DEBUG1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;            //GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_DEBUG1_GPIO, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LED_DEBUG2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;            //GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_DEBUG2_GPIO, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LED_DEBUG3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;            //GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_DEBUG3_GPIO, &GPIO_InitStructure);
    
}
void io_init(void)
{

    GPIO_InitTypeDef 	GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(DEVICE_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = DEVICE1_PIN | DEVICE2_PIN | DEVICE3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;            
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(DEV123_GPIO, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = DEVICE4_PIN | DEVICE5_PIN | DEVICE6_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;            
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(DEV456_GPIO, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = DEVICE7_PIN | DEVICE8_PIN | DEVICE9_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;            
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(DEV789_GPIO, &GPIO_InitStructure);  
}

void io_irq_init(void)
{
    //PD1
//    GPIO_InitTypeDef 	GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource1);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
        
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void io_on(uint8_t device)
{
    switch(device)
    {
        case DEVICE1: GPIO_SetBits(DEV123_GPIO, DEVICE1_PIN); break;
        case DEVICE2: GPIO_SetBits(DEV123_GPIO, DEVICE2_PIN); break;
        case DEVICE3: GPIO_SetBits(DEV123_GPIO, DEVICE3_PIN); break;
        
        case DEVICE4: GPIO_SetBits(DEV456_GPIO, DEVICE4_PIN); break;
        case DEVICE5: GPIO_SetBits(DEV456_GPIO, DEVICE5_PIN); break;
        case DEVICE6: GPIO_SetBits(DEV456_GPIO, DEVICE6_PIN); break;
        
        case DEVICE7: GPIO_SetBits(DEV789_GPIO, DEVICE7_PIN); break;
        case DEVICE8: GPIO_SetBits(DEV789_GPIO, DEVICE8_PIN); break;
        case DEVICE9: GPIO_SetBits(DEV789_GPIO, DEVICE9_PIN); break;
        default: break;
    }
    
}

void io_off(uint8_t device)
{
    switch(device)
    {
        case DEVICE1: GPIO_ResetBits(DEV123_GPIO, DEVICE1_PIN); break;
        case DEVICE2: GPIO_ResetBits(DEV123_GPIO, DEVICE2_PIN); break;
        case DEVICE3: GPIO_ResetBits(DEV123_GPIO, DEVICE3_PIN); break;
        
        case DEVICE4: GPIO_ResetBits(DEV456_GPIO, DEVICE4_PIN); break;
        case DEVICE5: GPIO_ResetBits(DEV456_GPIO, DEVICE5_PIN); break;
        case DEVICE6: GPIO_ResetBits(DEV456_GPIO, DEVICE6_PIN); break;
        
        case DEVICE7: GPIO_ResetBits(DEV789_GPIO, DEVICE7_PIN); break;
        case DEVICE8: GPIO_ResetBits(DEV789_GPIO, DEVICE8_PIN); break;
        case DEVICE9: GPIO_ResetBits(DEV789_GPIO, DEVICE9_PIN); break;
        default: break;
    }
}
void io_test(void)
{
    /* Toggle pins */ 
    
    GPIO_WriteBit(GPIOC, GPIO_Pin_2, Bit_SET);
//    GPIO_WriteBit(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 , (BitAction)(1));
    /* Insert delay */
//    Delay(1000);
//    GPIO_WriteBit(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 , (BitAction)(0));
    GPIO_WriteBit(GPIOC, GPIO_Pin_2, Bit_RESET);
//    Delay(1000);    
}

void timer_init(void)
{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//      /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 499;
//    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 8000000) - 1;
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
////    TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
//    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
//    /* TIM2 enable counter */
//    TIM_Cmd(TIM2, ENABLE);
//    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
}

//void adc_init(void)
//{
//    
//    GPIO_InitTypeDef GPIO_InitStructure;
//    ADC_InitTypeDef  ADC_InitStructure;
//    
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
//    
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    
//     /* ADC1 configuration ------------------------------------------------------*/
//    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//    ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Single Channel
//    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Scan on Demand
//    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//    ADC_InitStructure.ADC_NbrOfChannel = 1;
//    ADC_Init(ADC1, &ADC_InitStructure);
//    
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);

//    ADC_Cmd(ADC1, ENABLE);
//      	ADC_ResetCalibration(ADC1);
//  	/* Check the end of ADC1 reset calibration register */
//  	while(ADC_GetResetCalibrationStatus(ADC1));

//  	/* Start ADC1 calibaration */
//  	ADC_StartCalibration(ADC1);
//  	/* Check the end of ADC1 calibration */
//  	while(ADC_GetCalibrationStatus(ADC1));
//     
//  	/* Start ADC1 Software Conversion */ 
//  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//     ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

//    
//}

//int read_adc(void)
//{
//    int adc = 0;
//    GPIO_WriteBit(GPIOC, GPIO_Pin_2, Bit_RESET);
//    adc = ADC_GetConversionValue(ADC1);
//}


//void EXTI1_IRQHandler(void)
//{	
//	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
//	{
//		EXTI_ClearITPendingBit(EXTI_Line1);
////		printf("\n\rEXTI0 go on");
//        flag = 0;

//	}
//}

//void TIM2_IRQHandler(void)
//{
//    static uint32_t time=0, adc = 0, time1 = 0;
//    static float f_adc = 0;
//    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//    {
//         if (++time == 1000)
//         {
////             printf("Interrup\r\n");
//             adc = adc + read_adc();
//             time = 0;
//             time1++;
//             if (time1 == 20)
//             {
//                f_adc = (adc * 3.3) / (4096 * time1);
//                 printf("%f\r\n",f_adc);
//                 adc = 0;
//                 time1 = 0;
//            }
//                 
//         }
//         TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
//    }
//    
//    
//}
