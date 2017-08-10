#ifndef RF_H__
#define RF_H__

#define RF_UART2_CLK                           RCC_APB1Periph_USART2
#define RF_UART2_AF                            GPIO_AF_USART2

#define RF_UART2_RX_PIN                        GPIO_Pin_3
#define RF_UART2_RX_PIN_SOURCE                 GPIO_PinSource3
#define RF_UART2_RX_GPIO_PORT                  GPIOA
#define RF_UART2_RX_GPIO_CLK                   RCC_AHB1Periph_GPIOA

  
#define RF_UART2_TX_PIN                        GPIO_Pin_2
#define RF_UART2_TX_PIN_SOURCE                 GPIO_PinSource2
#define RF_UART2_TX_GPIO_PORT                  GPIOA
#define RF_UART2_TX_GPIO_CLK                   RCC_AHB1Periph_GPIOA

#define RF_UART2                               USART2

#define RF_UART2_PORT                          GPIOA
#define RF_UART2_PORT_CLK                      RCC_AHB1Periph_GPIOA
#define RF_BAUDRATE                            9600



#define RF_UART2_EXTI_IRQChannel               USART2_IRQn                     
#define RF_UART2_EXTI_PREE_PRIO                configMAX_SYSCALL_INTERRUPT_PRIORITY - 1
#define RF_UART2_EXTI_SUB_PRIO                 0

#define RF_PWR_PIN                             GPIO_Pin_0
#define RF_PWR_GPIO_PORT                       GPIOC
#define RF_PWR_GPIO_CLK                        RCC_AHB1Periph_GPIOC

#define RF_AUX_PIN                             GPIO_Pin_1
#define RF_AUX_GPIO_PORT                       GPIOC
#define RF_AUX_GPIO_CLK                        RCC_AHB1Periph_GPIOC

#define RF_M0_PIN                             GPIO_Pin_3
#define RF_M0_GPIO_PORT                       GPIOC
#define RF_M0_GPIO_CLK                        RCC_AHB1Periph_GPIOC

#define RF_M1_PIN                             GPIO_Pin_2
#define RF_M1_GPIO_PORT                       GPIOC
#define RF_M1_GPIO_CLK                        RCC_AHB1Periph_GPIOC

#endif