#include "stm32f4xx_gpio.h"
#include "rf.h"
#include "FreeRTOS.h"
void rf_init(void)
{
    GPIO_InitTypeDef 	GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB1PeriphClockCmd(RF_UART2_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(RF_UART2_PORT_CLK | RF_PWR_GPIO_CLK, ENABLE);    

    //PWR
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RF_PWR_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(RF_PWR_GPIO_PORT, &GPIO_InitStructure);
    
    //AUX
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RF_AUX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(RF_AUX_GPIO_PORT, &GPIO_InitStructure);
    
    //UART
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = RF_UART2_RX_PIN | RF_UART2_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(RF_UART2_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(RF_UART2_PORT,RF_UART2_TX_PIN_SOURCE,RF_UART2_AF);
    GPIO_PinAFConfig(RF_UART2_PORT,RF_UART2_RX_PIN_SOURCE,RF_UART2_AF);
    
    USART_InitStructure.USART_BaudRate = RF_BAUDRATE;//9600
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 

    USART_Init(RF_UART2, &USART_InitStructure);
    USART_Cmd(RF_UART2, ENABLE);
}

void rf_irq_init(void)
{
    EXTI_InitTypeDef  EXTI_InitStructure;
    //uart
    
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_ITConfig(RF_UART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = RF_UART2_EXTI_IRQChannel;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY - 1 ;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    USART_Cmd(RF_UART2, ENABLE);
}

void rf_on(void)
{
    GPIO_ResetBits(RF_PWR_GPIO_PORT, RF_PWR_PIN);
}

void rf_off(void)
{
    GPIO_SetBits(RF_PWR_GPIO_PORT, RF_PWR_PIN);
}