/**
  *					ENHANCED VERSION
  *
  *					jnosky
  *
  ******************************************************************************
  * @file    stm32f4_discovery.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   This file provides set of firmware functions to manage Leds and
  *          push-button available on STM32F4-Discovery Kit from STMicroelectronics.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "portmacro.h"
#include <stdio.h>
#include <string.h>
#include "gsm.h"
extern uint8_t ut_a, ut_b, ut_c, ut_d;

//#define RX_BUF_SIZE					128
//uint8_t gsm_rx_buf[RX_BUF_SIZE];
//uint32_t gsm_rx_buf_ptr;

xQueueHandle xQueue_Test;

//typedef struct{
//	uint8_t type;
//	uint8_t data;	/* command line accept maximum 391 characters */
//	uint16_t len;
//}at_command_t;

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */   
    
/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32F4-Discovery Kit from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Variables
  * @{
  */ 
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED_GREEN_GPIO_PORT, LED_BLUE_GPIO_PORT, LED_RED_GPIO_PORT,
                                 LED_ORANGE_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED_GREEN_PIN, LED_BLUE_PIN, LED_RED_PIN,
                                 LED_ORANGE_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED_GREEN_GPIO_CLK, LED_BLUE_GPIO_CLK, LED_RED_GPIO_CLK,
                                 LED_ORANGE_GPIO_CLK};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT }; 

const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN }; 

const uint32_t BUTTON_CLK[BUTTONn] = {USER_BUTTON_GPIO_CLK };

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE };

const uint8_t BUTTON_PORT_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PORT_SOURCE};
								 
const uint8_t BUTTON_PIN_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PIN_SOURCE }; 
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };

NVIC_InitTypeDef   NVIC_InitStructure;

/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Functions
  * @{
  */ 

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6
  * @retval None
  */
  
  
  
  void adc_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;   
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* ADC Channel 11 -> PC1 */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);   
}



 
/**************************************************************************************/
 
void adc_config(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* ADC Common Init */
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Manual
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular channel 11 configuration */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles); // PC1

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
//    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
}
void STM_EVAL_LEDInit(Led_TypeDef Led)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_AHB1PeriphClockCmd(GPIO_CLK[Led], ENABLE);
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  LED1 | LED2 | LED3 | LED4 | DT | SH | ST;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void quet_led(uint8_t ma)
{
    uint8_t i = 0, ut_tmp = 0;
    ut_tmp = ma;
    GPIO_SetBits(GPIOD, SH);
    GPIO_SetBits(GPIOD, ST);
    for (i = 0; i < 8; i++)
    {
        if ((ut_tmp & 0x80) == 0x80)
            GPIO_SetBits(GPIOD, DT);
        else
            GPIO_ResetBits(GPIOD, DT);
        ut_tmp = ut_tmp << 1;
        GPIO_ResetBits(GPIOD, SH);
        GPIO_SetBits(GPIOD, SH);
    }
    GPIO_ResetBits(GPIOD, ST);
    //Delay(1);
    GPIO_SetBits(GPIOD, ST);
    //Delay(1);
    
}

void pro_led(uint8_t u8_hours, uint8_t u8_minutes)
{
    ut_a = u8_hours / 10;
    ut_b = (u8_hours - ut_a*10);
    ut_c = (u8_minutes) / 10;
    ut_d = (u8_minutes -  ut_c*10);
}

void TIM_Init(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5, ENABLE);

    
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 2) / 1000000); 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  
    
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 
    TIM_Cmd(TIM4, ENABLE); 

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
}

void debug_init(void)
{
    GPIO_InitTypeDef 	GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(DEBUG_UART6_CLK, ENABLE); 
    RCC_AHB1PeriphClockCmd(DEBUG_UART6_TX_GPIO_CLK, ENABLE);

//    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 
    /* Configure USARTy Tx as alternate function open-drain */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = DEBUG_UART6_TX_PIN | DEBUG_UART6_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(DEBUG_UART6_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(DEBUG_UART6_PORT,DEBUG_UART6_TX_SOURCE,DEBUG_UART6_AF);
    GPIO_PinAFConfig(DEBUG_UART6_PORT,DEBUG_UART6_RX_SOURCE,DEBUG_UART6_AF);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 

    USART_Init(DEBUG_UART6, &USART_InitStructure);
    

    
    USART_Cmd(DEBUG_UART6, ENABLE);
}

void debug_irq(void)
{
    USART_ITConfig(DEBUG_UART6, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_UART6_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY - 2 ;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    USART_Cmd(DEBUG_UART6, ENABLE);
}

void USART6_IRQHandler(void)
{
//	portBASE_TYPE xTaskWokenByPost = pdFALSE;
    char t;
	if( USART_GetITStatus(DEBUG_UART6, USART_IT_RXNE) )
    {
		t = USART_ReceiveData(DEBUG_UART6);
        gsm_send_char(t);
        debug_send_char(t);
//        xQueueSendToBackFromISR(xQueue_Test, &t, &xTaskWokenByPost);
        USART_ClearFlag(DEBUG_UART6, USART_FLAG_RXNE);
	}
//    if( != pdPASS)
//        debug_printf("eror\r\n");
//    debug_printf("12");
    
//    portEND_SWITCHING_ISR(xTaskWokenByPost);
//    USART_ClearFlag(USART2, USART_FLAG_RXNE);
}

uint8_t vPCinitCreat(void)
{ 
    xQueue_Test = xQueueCreate(50, sizeof( char *));
    if(!xQueue_Test)
    {
        debug_printf("Queue init erorr\r\n");
        return 0;
    }
    else
        return 1;
}
void vPCinit(void *pvParameters)
{
//    at_command_t cmd;
//    char t;
    char buf[50];

//    for(;;)
//    {
//        if(xQueue_Test != 0)
//        {
//            if (xQueueReceive(xQueue_Test, &gsm_rx_buf[gsm_rx_buf_ptr], 50/portTICK_RATE_MS) == pdPASS)
//            {
//                gsm_rx_buf_ptr++;
//                if (xQueueReceive(xQueue_Test, &gsm_rx_buf[gsm_rx_buf_ptr], 50/portTICK_RATE_MS) == pdPASS)
//                {
//                    gsm_rx_buf_ptr++;
//                    if (strstr((char *) gsm_rx_buf, "abcdefgh")) 
//                    {
//                        sprintf(buf,":%s %d\r\n", gsm_rx_buf, gsm_rx_buf_ptr);
//                        debug_printf(buf);
//                        memset(gsm_rx_buf, 0xFF, RX_BUF_SIZE);
//                        gsm_rx_buf_ptr = 0;
//                    }
//                }
//            }
//            else
//                debug_printf("error\r\n");

//        }
//        vTaskDelay( 100 / portTICK_RATE_MS );
//        
//    }
}
void debug_send_char(char chr)
{
    /* Write one byte in the USARTz Transmit Data Register */
    while (USART_GetFlagStatus(DEBUG_UART6, USART_FLAG_TXE) == RESET);
    USART_SendData(DEBUG_UART6, (uint16_t)chr );
}

void debug_printf(char *ss)
{
    while(*ss)
    {
        debug_send_char(*ss);
        *(ss ++);
    }
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6  
  * @retval None
  */
void STM_EVAL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRL = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6 
  * @retval None
  */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRH = GPIO_PIN[Led];  
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6  
  * @retval None
  */
void STM_EVAL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  * @retval None
  */
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_AHB1PeriphClockCmd(BUTTON_CLK[Button], ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);

  if (Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER  
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */   

/**
  * @}
  */ 
//=============================================================================
//=============================================================================
//				PORTS FROM OLDER EVAL TO DISCOVERY BOARD
//=============================================================================
//=============================================================================

//COMM PORT
USART_TypeDef* COM_USART[COMn] = {EVAL_COM1};
GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT};
const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK};
const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK};
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK};
const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN};
const uint16_t COM_TX_PIN_SOURCE[COMn] = {EVAL_COM1_TX_SOURCE};
const uint16_t COM_RX_PIN_SOURCE[COMn] = {EVAL_COM1_RX_SOURCE};
const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF};
const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF};
/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:
  *     @arg COM1
  *     @arg COM2
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);

  if (COM == COM1)
  {
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  }

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  /* USART configuration */
  //USART_OverSampling8Cmd(COM_USART[COM], ENABLE);

  USART_Init(COM_USART[COM], USART_InitStruct);
    
  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);
}
//END COMM PORT

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
