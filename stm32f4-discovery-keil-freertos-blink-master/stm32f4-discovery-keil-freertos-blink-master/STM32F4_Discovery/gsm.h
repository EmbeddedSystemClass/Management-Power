#ifndef GSM_H__
#define GSM_H__

//SIM5320_UART3_TX	PB10	
//SIM5320_UART3_RX	PB11
//SIM5320_PWON		PB3
//SIM5320_ENABLE		PE1
//SIM5320_RST			PD7
//SIM5320_RI			PE5

#define SIM5320_UART3_CLK                           RCC_APB1Periph_USART3
#define SIM5320_UART3_AF                            GPIO_AF_USART3

#define SIM5320_UART3_RX_PIN                        GPIO_Pin_11
#define SIM5320_UART3_RX_PIN_SOURCE                 GPIO_PinSource11
#define SIM5320_UART3_RX_GPIO_PORT                  GPIOB
#define SIM5320_UART3_RX_GPIO_CLK                   RCC_AHB1Periph_GPIOB

  
#define SIM5320_UART3_TX_PIN                        GPIO_Pin_10
#define SIM5320_UART3_TX_PIN_SOURCE                 GPIO_PinSource10
#define SIM5320_UART3_TX_GPIO_PORT                  GPIOB
#define SIM5320_UART3_TX_GPIO_CLK                   RCC_AHB1Periph_GPIOB

#define SIM5320_UART3                               USART3

#define SIM5320_UART3_PORT                          GPIOB
#define SIM5320_BAUDRATE                            115200

#define SIM5320_RST_PIN                             GPIO_Pin_4
#define SIM5320_RST_GPIO_PORT                       GPIOE
#define SIM5320_RST_GPIO_CLK                        RCC_AHB1Periph_GPIOE

#define SIM5320_DTR_PIN                             GPIO_Pin_0
#define SIM5320_DTR_GPIO_PORT                       GPIOE
#define SIM5320_DTR_GPIO_CLK                        RCC_AHB1Periph_GPIOE

//
#define SIM5320_ENABLE_PIN                          GPIO_Pin_5
#define SIM5320_ENABLE_GPIO_PORT                    GPIOE
#define SIM5320_ENABLE_GPIO_CLK                     RCC_AHB1Periph_GPIOE

#define SIM5320_PWON_PIN                            GPIO_Pin_9
#define SIM5320_PWON_GPIO_PORT                      GPIOB
#define SIM5320_PWON_GPIO_CLK                       RCC_AHB1Periph_GPIOB
//
#define SIM5320_RI_PIN                              GPIO_Pin_5
#define SIM5320_RI_GPIO_PORT                        GPIOE
#define SIM5320_RI_GPIO_CLK                         RCC_AHB1Periph_GPIOE

#define SIM5320_RI_EXTI_LINE                        EXTI_Line5
#define SIM5320_RI_EXTI_PORT_SOURCE                 EXTI_PortSourceGPIOE
#define SIM5320_RI_EXTI_PIN_SOURCE                  EXTI_PinSource5
#define USER_RI_EXTI_IRQn                           EXTI9_5_IRQn 
#define USER_RI_IRQ                                 EXTI9_5_IRQHandler

#define SIM5320_UART3_EXTI_IRQChannel               USART3_IRQn                     
#define SIM5320_UART3_EXTI_PREE_PRIO                configMAX_SYSCALL_INTERRUPT_PRIORITY - 1
#define SIM5320_UART3_EXTI_SUB_PRIO                 0

#define NULL                                        0
//#define RX_BUF_SIZE					128
//uint8_t gsm_rx_buf[RX_BUF_SIZE];
//uint32_t gsm_rx_buf_ptr;

#define QUEUE_COUNT             128

typedef enum {
	GSM_STARTUP = 0,
	GSM_CHECK_SIM,
	GSM_CHECK_NETWORK,
	GSM_CHECK_GPRS,
	GSM_INIT_MQTT,
	GSM_WAIT_MQTT,
	GSM_MQTT_CONNECTED,
	GSM_WAITING_SAVE_FTP,
	REVERSE,
}gsm_state_t;

typedef enum {
	GSM_ALLOC_MEM_FAIL				= -1,
	GSM_NO_ERR						= 0,
	GSM_COMMAND_ERROR				= 1,
	GSM_COMMAND_FAIL				= 2,
	GSM_QUEUE_FULL					= 3,
	GSM_QUEUE_EMPTY					= 4,
	GSM_CONTINUE_SEARCH				= 5,
	GSM_NO_CARRIER					= 6,		/* no connection */
	TCP_SESSION_ERROR				= 7,		/* problem in session, ktcp_notif */
	TCP_SEND_FAIL					= 8,		/* can't send data */
	FTP_CLIENT_ERROR				= 9,
	CALL_CONNECTED					= 10,		/* dau day ben kia bat may */
	CALL_BUSY						= 11,		/* dau day ben kia tu choi khong nghe */
	CALL_NO_ANSWER					= 12,		/* het timeout nhung dau day ben kia chua bat may */
	CALL_NO_DIALTONE				= 13,		/* truong hop sim het tien */
	CALL_NO_CARRIER					= 14		/* dau day ben kia nghe roi nhung xong lai tat may */
}gsm_error_t;


/* define struct at response */
typedef struct{
	uint8_t *data;	/* raw data */
	uint8_t len;
}at_response_t;

/* define struct at command */
typedef struct{
	uint8_t type;
	uint8_t *data;	/* command line accept maximum 391 characters */
	uint16_t len;
}at_command_t;

#define GSM_TIMEOUT_STEP		100 //100ms

typedef enum 
{
	AT_COMMAND = 0,
	GET_FILE
}command_type_t;

/*define queue for transfer tx and rx data */
#define AT_COMMAND_QUEUE_SIZE		20
#define AT_COMMAND_TYPE_SIZE(x)		sizeof(x)
#define RX_QUEUE_SIZE				1024
#define RX_TYPE_SIZE(x)				sizeof(x)
#define AT_RESPONSE_QUEUE_SIZE		30
#define AT_RESPONSE_TYPE_SIZE(x)	sizeof(x)

#define RX_BUF_SIZE					1500/* buffer to copy data from queue(for AT command, not file) */



void gsm_enable(void);
void gsm_disable(void);
void gsm_poweron(void);
void gsm_poweroff(void);
void gsm_putchar(char *s);
void gsm_send_char(char chr);
void gsm_irq_init(void);
void gsm_init(void);
void gsm_put(char *s, uint8_t len);
gsm_error_t gsm_send_multi_command(uint8_t **command_group) ;
gsm_error_t gsm_get_response_from_queue(at_response_t *resp, uint32_t timeout);
gsm_error_t gsm_send_at_command_and_wait(uint8_t *cmd, uint32_t len,  gsm_error_t (*callback)(uint8_t *resp, void *obj), uint32_t timeout, void * object);
gsm_error_t gsm_send_at_command_to_queue(uint8_t *cmd, uint32_t len, command_type_t type,  uint32_t timeout);
gsm_error_t terminal_response_callback(uint8_t *resp, void *object);
int8_t alloc_queue(void);
uint8_t gsm_queue_init(void);
void vGSM_init(void *pvParameters);
void vGSM_PROC(void *pvParameters);
void vGSM_TEST(void *pvParameters);
void gsm_reset_init(void);
void gsm_reset_hw(void);
void gsm_setpin_init(void);
gsm_error_t gsm_send_response_to_queue(at_response_t *resp, uint32_t timeout);
#endif
