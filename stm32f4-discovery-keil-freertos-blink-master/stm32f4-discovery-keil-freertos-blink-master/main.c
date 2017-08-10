//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
#include "flash.h"
#include "io.h"
#include "lcd.h"

#include "rtc.h"
#include "bq32000.h"
#include "gsm.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "portmacro.h"
//******************************************************************************
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "uprintf.h"

uint8_t ui8_check123 = 0, ui8_check456 = 0, ui8_check789 = 0, ui8_connect123 = 0, ui8_connect456 = 0, ui8_connect789 = 0;

uint8_t ui8_ADE_STATE = 0;
uint8_t ut_a = 0, ut_b = 0, ut_c = 0, ut_d = 0;
volatile uint8_t M[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
uint32_t timeout = 1000;
void vLedBlinkBlue(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vLedBlinkGreen(void *pvParameters);
void vLedBlinkOrange(void *pvParameters);
void vUartPrintf(void *pvParameters);
void v7segment_led1(void *pvParameters);
void v7segment_led2(void *pvParameters);
void v7segment_led3(void *pvParameters);
void v7segment_led4(void *pvParameters);
void vADC(void *pvParameters);
void vADRReadRMS(void *pvParameters);
xSemaphoreHandle xMutex;



//******************************************************************************
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */
	
	/*!< Most systems default to the wanted configuration, with the noticeable 
		exception of the STM32 driver library. If you are using an STM32 with 
		the STM32 driver library then ensure all the priority bits are assigned 
		to be preempt priority bits by calling 
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
	*/
    char buff[100], data_second = 0;
    
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	debug_init();
    debug_irq();
    //ADE init
    io_init();
    ADE7758_init();
//    ADE7758_data_init();

    io_off(DEVICE1);
    io_off(DEVICE2);
    io_off(DEVICE3);
    //end of ADE init

    uprintf("test\r\n");
    //GSM init
    gsm_init();
//    gsm_irq_init();
    debug_send_char('a');
    //
    //LCD init
    lcd_Init();		
    lcd_Clear();
    lcd_Goto(0,0);
    lcd_Print_Data(" LCD Test");
    lcd_Goto(1,0);
    lcd_Print_Data(" 1234567");
    //
    io_led_debug_init();
    io_button_init();
    
    //rtc init
    init_I2C1();
    bq32000_init_rtc(0);
//    while(1)
//    {
//        io_off(DEVICE1);
//        io_on(DEVICE2);
//        io_off(DEVICE3);
//    }
//    data_time.minutes = 1;
//    data_time.hour = 9;
//    data_time.date = 5;
//    data_time.month = 8;
//    data_time.year = 17;
//    bq32000_set_rtc_datetime(&data_time);

//    debug_send_char('a');
//    sprintf(buff,"test: %d\r\n", 1234);
//    debug_printf(buff);
//    uprintf("test\r\n");
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	xTaskCreate( vLedBlinkBlue, (const signed char*)"Blink Task Blue\r\n", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY + LED_TASK_PRIORITY, NULL );
    xTaskCreate( xBQ32000_Read, (const signed char*)"Read Time\r\n", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY + RTC_TASK_PRIORITY, NULL );
    
    xTaskCreate( vADRReadRMS, (const signed char*)"Read ADE\r\n", 
		STACK_SIZE_MIN + 512, NULL, tskIDLE_PRIORITY + ADE_TASK_PRIORITY, NULL );
    io_button_proc();
//    ADE7758_timer_init();
//    xTaskCreate( vGSM_TEST, (const signed char*)"GSM TEST\r\n", 
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY + GSM_INIT_TASK_PRIORITY, NULL );
//    if (alloc_queue() == 0)
//    {
//        uprintf("alloc ss\r\n");
//        xTaskCreate(vGSM_init, (const signed char*)"GSM_init\r\n", 
//            STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY + GSM_INIT_TASK_PRIORITY, NULL );
//        xTaskCreate(vGSM_PROC, (const signed char*)"GSM_proc\r\n", 
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY + GSM_PROC_TASK_PRIORITY, NULL );
//    }
//        

    vTaskStartScheduler();    
//    if(vPCinitCreat())
//    {
//        debug_irq();
//        xTaskCreate(vPCinit, (const signed char*)"PCinit\r\n", 
//            STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY + PC_TASK_PRIORITY, NULL );
//        
//    }
    while(1)
    {}
}
//******************************************************************************


void vADRReadRMS(void *pvParameters)
{
    flag_ade_connect flag_ade;

    static uint8_t u8_timeout = 0;
    static char buf[21];
    static char buf1[7];
    static char buf2[16];
    static char test[128];
    static uint8_t u8_checkade123 = 0;
    static uint8_t u8_checkade456 = 0;
    static uint8_t u8_checkade789 = 0;
    static float VRMSA = 0;
    VRMS_ADE read_vrms;
    IRMS_ADE read_irms;
    PWR_ADE read_pwr;
    uint8_t u8_temp = 0;
    flag_ade.flag_chanel123 = 0;

    read_vrms.f_VRMS_DEV1 = 0;
    read_vrms.f_VRMS_DEV2 = 0;
    read_vrms.f_VRMS_DEV3 = 0;
    read_vrms.f_VRMS_DEV4 = 0;
    read_vrms.f_VRMS_DEV5 = 0;
    read_vrms.f_VRMS_DEV6 = 0;
    read_vrms.f_VRMS_DEV7 = 0;
    read_vrms.f_VRMS_DEV8 = 0;
    read_vrms.f_VRMS_DEV9 = 0;
    uprintf("........%f %f\r\n", read_vrms.f_VRMS_DEV1, VRMSA);
    for(;;)
    {
        switch(ui8_ADE_STATE)
        {
            case 0:
                debug_printf("case 0\r\n");
                ADUM5401_on(ADE7758_DEV1);
                ADUM5401_on(ADE7758_DEV2);
                ADUM5401_on(ADE7758_DEV3);
                u8_checkade123 = ADE7758_Check(ADE7758_DEV1);
                u8_checkade456 = ADE7758_Check(ADE7758_DEV2);
                u8_checkade789 = ADE7758_Check(ADE7758_DEV3);
                uprintf("u8_check %d\r\n",u8_checkade123);
                if (u8_checkade123 == 1)
                {
                    if(flag_ade.flag_chanel123 == 0)
                    {
                        uprintf("flag_ade = 0\r\n");
                        flag_ade.flag_chanel123 = 1;
                        ADE7758_setpara(ADE7758_DEV1);
                        
                        io_on(DEVICE1);
                        io_on(DEVICE2);
                        io_on(DEVICE3);
                        
                        uprintf("ss\r\n");
                    }
                }
                else
                {
                    uprintf("u8_checkade123 = 0\r\n");
                    flag_ade.flag_chanel123 = 0;
                    io_off(DEVICE1);
                    io_off(DEVICE2);
                    io_off(DEVICE3);
                    u8_timeout = 0;
                    debug_printf("ADE1_ERROR\r\n");
                }
                
                //ADE2
                if (u8_checkade456 == 1)
                {
                    if(flag_ade.flag_chanel456 == 0)
                    {
                        flag_ade.flag_chanel456 = 1;
                        ADE7758_setpara(ADE7758_DEV2);
                        io_on(DEVICE4);
                        io_on(DEVICE5);
                        io_on(DEVICE6);
                    }
//                    ui8_ADE_STATE = 1;
                }
                else
                {
                    flag_ade.flag_chanel456 = 0;
                    io_off(DEVICE4);
                    io_off(DEVICE5);
                    io_off(DEVICE6);
//                    ui8_ADE_STATE = 3;
                    u8_timeout = 0;
                    debug_printf("ADE2_ERROR\r\n");
                }
                //ADE3
                if (u8_checkade789 == 1)
                {
                    if(flag_ade.flag_chanel789 == 0)
                    {
                        flag_ade.flag_chanel789 = 1;
                        ADE7758_setpara(ADE7758_DEV3);
                        io_on(DEVICE7);
                        io_on(DEVICE8);
                        io_on(DEVICE9);
                        ui8_ADE_STATE = 1;
                    }
                }
                else
                {
                    flag_ade.flag_chanel789 = 0;
                    io_off(DEVICE7);
                    io_off(DEVICE8);
                    io_off(DEVICE9);
//                    ui8_ADE_STATE = 3;
                    u8_timeout = 0;
                    debug_printf("ADE3_ERROR\r\n");
                }
                uprintf("done\r\n");
                ui8_ADE_STATE = 3;
                break;
            case 1:
                debug_printf("case 1\r\n");
                if (flag_ade.flag_chanel123 == 1)
                {
                    read_vrms.f_VRMS_DEV1 = (ADE7758_readVRMS(1, PHASEA) * 0.98);
                    read_vrms.f_VRMS_DEV2 = (ADE7758_readVRMS(1, PHASEB) * 0.98);
                    read_vrms.f_VRMS_DEV3 = (ADE7758_readVRMS(1, PHASEC) * 0.98);
                }
                
                if (flag_ade.flag_chanel456 == 1)
                {
                    read_vrms.f_VRMS_DEV4 = (float)(ADE7758_readVRMS(ADE7758_DEV2, PHASEA) * 0.98);
                    read_vrms.f_VRMS_DEV5 = (float)(ADE7758_readVRMS(ADE7758_DEV2, PHASEB) * 0.98);
                    read_vrms.f_VRMS_DEV6 = (float)(ADE7758_readVRMS(ADE7758_DEV2, PHASEC) * 0.98);
                }
                
                if (flag_ade.flag_chanel789 == 1)
                {
                    read_vrms.f_VRMS_DEV7 = (float)(ADE7758_readVRMS(ADE7758_DEV3, PHASEA) * 0.98);
                    read_vrms.f_VRMS_DEV8 = (float)(ADE7758_readVRMS(ADE7758_DEV3, PHASEB) * 0.98);
                    read_vrms.f_VRMS_DEV9 = (float)(ADE7758_readVRMS(ADE7758_DEV3, PHASEC) * 0.98);
                }
                sprintf(test, " SUM_VRMS: %.2f %.2f %.2f(I) ", read_vrms.f_VRMS_DEV1, read_vrms.f_VRMS_DEV2, read_vrms.f_VRMS_DEV3);
                uprintf(test);
                read_vrms.f_SUM_VRMS = read_vrms.f_VRMS_DEV1 + read_vrms.f_VRMS_DEV2 + read_vrms.f_VRMS_DEV3 + 
                                        read_vrms.f_VRMS_DEV4 + read_vrms.f_VRMS_DEV5 + read_vrms.f_VRMS_DEV6 + 
                                        read_vrms.f_VRMS_DEV7 + read_vrms.f_VRMS_DEV8 + read_vrms.f_VRMS_DEV9;
                read_vrms.f_AVG_VRMS = read_vrms.f_SUM_VRMS / 9;
                if ((flag_ade.flag_chanel123 == 0) && (flag_ade.flag_chanel456 == 0) && (flag_ade.flag_chanel789 == 0))
                {
                    read_vrms.f_AVG_VRMS = 0;
                } 
//                else
//                if (read_vrms.f_SUM_VRMS < 4400)
//                    read_vrms.f_TB_VRMS = 0;
//                lcd_Goto(1, 0);
//                sprintf(buf2, " MED_VRMS: %.2f (V) ", read_vrms.f_VRMS_DEV2);
//                lcd_Print_Data(buf2);
                ui8_ADE_STATE = 2;
                break;
            case 2:
                uprintf("case 2\r\n");
                if (flag_ade.flag_chanel123 == 1)
                {
                    read_irms.f_IRMS_DEV1 = (float)(ADE7758_readIRMS(1, PHASEA) * 10 * 1.23);
                    read_irms.f_IRMS_DEV2 = (float)(ADE7758_readIRMS(1, PHASEB) * 10 * 1.23);
                    read_irms.f_IRMS_DEV3 = (float)(ADE7758_readIRMS(1, PHASEC) * 10 * 1.23);
                }
                
                if (flag_ade.flag_chanel456 == 1)
                {
                    read_irms.f_IRMS_DEV4 = (float)(ADE7758_readIRMS(1, PHASEA) * 10 * 1.23);
                    read_irms.f_IRMS_DEV5 = (float)(ADE7758_readIRMS(1, PHASEB) * 10 * 1.23);
                    read_irms.f_IRMS_DEV6 = (float)(ADE7758_readIRMS(1, PHASEC) * 10 * 1.23);
                }
                
                if (flag_ade.flag_chanel789 == 1)
                {
                    read_irms.f_IRMS_DEV7 = (float)(ADE7758_readIRMS(1, PHASEA) * 10 * 1.23);
                    read_irms.f_IRMS_DEV8 = (float)(ADE7758_readIRMS(1, PHASEB) * 10 * 1.23);
                    read_irms.f_IRMS_DEV9 = (float)(ADE7758_readIRMS(1, PHASEC) * 10 * 1.23);
                }

                read_irms.f_SUM_IRMS = read_irms.f_IRMS_DEV1 + read_irms.f_IRMS_DEV2 + read_irms.f_IRMS_DEV3 + 
                                        read_irms.f_IRMS_DEV4 + read_irms.f_IRMS_DEV5 + read_irms.f_IRMS_DEV6 +
                                        read_irms.f_IRMS_DEV7 + read_irms.f_IRMS_DEV8 + read_irms.f_IRMS_DEV9;
                
                if ((flag_ade.flag_chanel123 == 0) && (flag_ade.flag_chanel456 == 0) && (flag_ade.flag_chanel789 == 0))
                {
                    read_irms.f_SUM_IRMS = 0;
                } else
                if (read_irms.f_SUM_IRMS < 0.27)
                    read_irms.f_SUM_IRMS = 0;
//                lcd_Goto(2, 0);
//                sprintf(buf2, " SUM_IRMS: %.2f (I) ", read_irms.f_SUM_IRMS);
//                lcd_Print_Data(buf2);
                read_pwr.f_SUM_PWR = read_vrms.f_AVG_VRMS * read_irms.f_SUM_IRMS;
//                lcd_Goto(3, 0);
//                sprintf(buf2, " SUM_PWR : %.2f (W) ", read_pwr.f_SUM_PWR);
//                lcd_Print_Data(buf2);
                ui8_ADE_STATE = 3;
                break;
            case 3:
                uprintf("case3: %d\r\n ",flag_ade.flag_chanel123);
//                lcd_Clear();
//                if (flag_ade.flag_chanel123 == 1)
//                {
//                    sprintf(buf,"TB1:OK ");
//                }
//                else
//                {
//                    sprintf(buf,"TB1:XX ");
//                }
//                if (flag_ade.flag_chanel456 == 1)
//                {
//                    sprintf(buf1,"TB2:OK ");
//                }
//                else
//                {
//                    sprintf(buf1,"TB2:XX ");
//                }
//                strcat(buf,buf1);
//                if (flag_ade.flag_chanel789 == 1)
//                {
//                    sprintf(buf1,"TB3:OK ");
//                }
//                else
//                {
//                    sprintf(buf1,"TB3:XX ");
//                }
//                strcat(buf,buf1);
//                lcd_Goto(0,0);
//                lcd_Print_Data(buf);
                ui8_ADE_STATE = 1;
                u8_timeout ++;
//                uprintf("TO: %d\r\n", u8_timeout);
                if (u8_timeout == 4)
                {
//                    ui8_ADE_STATE = 0;
                    u8_timeout = 0;
                }
                break;
            case 4:
                break;
            case 5: 
                break;
                
            default: break;
                
        }
        vTaskDelay( 1000 / portTICK_RATE_MS );
    }
}

void v7segment_led1(void *pvParameters)
{
     
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
    ut_a = 1, ut_b = 2, ut_c = 3, ut_d = 4;
    GPIO_WriteBit(GPIOD, LED2, Bit_SET);
    GPIO_WriteBit(GPIOD, LED3, Bit_SET);
    GPIO_WriteBit(GPIOD, LED4, Bit_SET);  
    GPIO_WriteBit(GPIOD, LED1, Bit_SET);    
	for(;;)
	{
        xSemaphoreTake( xMutex, portMAX_DELAY );

        debug_printf("LED1\r\n");
//        GPIO_WriteBit(GPIOD, LED1, Bit_RESET);
        GPIO_ResetBits(GPIOD, LED1);
        quet_led(M[ut_a]);
        vTaskDelay( 500 / portTICK_RATE_MS );
//        GPIO_WriteBit(GPIOD, LED1, Bit_SET);
        GPIO_SetBits(GPIOD, LED1);
        
        debug_printf("LED2\r\n");
//        GPIO_WriteBit(GPIOD, LED2, Bit_RESET);
        GPIO_ResetBits(GPIOD, LED2);
        quet_led(M[ut_b]); 
        vTaskDelay( 500 / portTICK_RATE_MS );
//        GPIO_WriteBit(GPIOD, LED2, Bit_SET);  
        GPIO_SetBits(GPIOD, LED2);
        
        debug_printf("LED3\r\n");
//        GPIO_WriteBit(GPIOD, LED3, Bit_RESET);
        GPIO_ResetBits(GPIOD, LED3);
        quet_led(M[ut_c]); 
        vTaskDelay( 500 / portTICK_RATE_MS );
//        GPIO_WriteBit(GPIOD, LED3, Bit_SET);   
        GPIO_SetBits(GPIOD, LED3);
        
        debug_printf("LED4\r\n");
//        GPIO_WriteBit(GPIOD, LED4, Bit_RESET);
        GPIO_ResetBits(GPIOD, LED4);
        quet_led(M[ut_d]);
        vTaskDelay( 500 / portTICK_RATE_MS );
//        GPIO_WriteBit(GPIOD, LED4, Bit_SET); 
        GPIO_SetBits(GPIOD, LED4);
        xSemaphoreGive( xMutex );
	}
}

void v7segment_led2(void *pvParameters)
{
     
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
    ut_b = 2;
	for(;;)
	{

        xSemaphoreTake( xMutex, portMAX_DELAY );
        GPIO_WriteBit(GPIOD, LED1, Bit_SET);
        GPIO_WriteBit(GPIOD, LED3, Bit_SET);
        GPIO_WriteBit(GPIOD, LED4, Bit_SET);
        
        GPIO_WriteBit(GPIOD, LED2, Bit_RESET);
        quet_led(M[ut_b]);    
        while(timeout)
            timeout --;  
//        vTaskDelayUntil(&xLastWakeTime, ( 1 / portTICK_RATE_MS) );
//        vTaskDelay( 1 / portTICK_RATE_MS );
        GPIO_WriteBit(GPIOD, LED2, Bit_SET); 
        xSemaphoreGive( xMutex ); 
//        vTaskDelay( 2 / portTICK_RATE_MS );        
          
	}
}

void v7segment_led3(void *pvParameters)
{
   
// 
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
      ut_c = 3;
	for(;;)
	{

        xSemaphoreTake( xMutex, portMAX_DELAY );
        GPIO_WriteBit(GPIOD, LED1, Bit_SET);
        GPIO_WriteBit(GPIOD, LED2, Bit_SET);
        GPIO_WriteBit(GPIOD, LED4, Bit_SET);
        GPIO_WriteBit(GPIOD, LED3, Bit_RESET);
        quet_led(M[ut_c]);
        while(timeout)
            timeout --;
//        vTaskDelay( 1 / portTICK_RATE_MS );
//        vTaskDelayUntil(&xLastWakeTime, ( 1 / portTICK_RATE_MS));
        GPIO_WriteBit(GPIOD, LED3, Bit_SET);
        xSemaphoreGive( xMutex ); 
//        vTaskDelay( 3 / portTICK_RATE_MS );
	}
}

void v7segment_led4(void *pvParameters)
{
    

//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
     ut_d = 4; 
	for(;;)
	{
        xSemaphoreTake( xMutex, portMAX_DELAY );
        GPIO_WriteBit(GPIOD, LED1, Bit_SET);
        GPIO_WriteBit(GPIOD, LED3, Bit_SET);
        GPIO_WriteBit(GPIOD, LED2, Bit_SET);
           
        GPIO_WriteBit(GPIOD, LED4, Bit_RESET);
        quet_led(M[ut_d]);
        while(timeout)
            timeout --;        
//            vTaskDelay( 1 / portTICK_RATE_MS );
//          vTaskDelayUntil(&xLastWakeTime, ( 1 / portTICK_RATE_MS) );
        GPIO_WriteBit(GPIOD, LED4, Bit_SET);
        xSemaphoreGive( xMutex ); 
//        vTaskDelay( 4 / portTICK_RATE_MS );
	}
}
void vADC(void *pvParameters)
{
    uint16_t tmp_adc = 0;
	for(;;)
	{
		ADC_SoftwareStartConv(ADC1);
//        xSemaphoreTake( xMutex, portMAX_DELAY );
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        tmp_adc = ADC_GetConversionValue(ADC1);
        if (tmp_adc > 2000);
//            debug_printf("ADC\r\n");
//        ut_a = tmp_adc / 1000;
//        ut_b = (tmp_adc - ut_a*1000) / 100;
//        ut_c = (tmp_adc - ut_a*1000 - ut_b*100) / 10;
//        ut_d = (tmp_adc - ut_a*1000 - ut_b*100 - ut_c*10);
        vTaskDelay( 1000 / portTICK_RATE_MS );
//        xSemaphoreGive( xMutex );
	}
}

void vLedBlinkBlue(void *pvParameters)
{

	for(;;)
	{   
        GPIOB->ODR ^= GPIO_Pin_3;
		STM_EVAL_LEDToggle(LED_BLUE);
        vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void vLedBlinkRed(void *pvParameters)
{
	for(;;)
	{
//        xSemaphoreTake( xMutex, portMAX_DELAY );
		STM_EVAL_LEDToggle(LED_RED);
        vTaskDelay( 100 / portTICK_RATE_MS );
//        STM_EVAL_LEDToggle(LED_BLUE);
//        xSemaphoreGive( xMutex );
//		vTaskDelay( 1 / portTICK_RATE_MS );
        
        
	}
}

void vLedBlinkGreen(void *pvParameters)
{
	for(;;)
	{
//        xSemaphoreTake( xMutex, portMAX_DELAY );
		STM_EVAL_LEDToggle(LED_GREEN);
        vTaskDelay( 500 / portTICK_RATE_MS );
//        STM_EVAL_LEDToggle(LED_BLUE);
//        xSemaphoreGive( xMutex );
//		vTaskDelay( 1 / portTICK_RATE_MS );
        
	}
}

void vLedBlinkOrange(void *pvParameters)
{
	for(;;)
	{
//        xSemaphoreTake( xMutex, portMAX_DELAY );
		STM_EVAL_LEDToggle(LED_ORANGE);
        vTaskDelay( 1000 / portTICK_RATE_MS );
//        STM_EVAL_LEDToggle(LED_BLUE);
//        xSemaphoreGive( xMutex );
//		vTaskDelay( 1 / portTICK_RATE_MS );
//        xSemaphoreGive( xMutex );
	}
}

void vUartPrintf(void *pvParameters)
{
    for (;;)
    {
        debug_printf("1234567890\r\n");
        vTaskDelay( 1000 / portTICK_RATE_MS );
    }

}


//******************************************************************************
