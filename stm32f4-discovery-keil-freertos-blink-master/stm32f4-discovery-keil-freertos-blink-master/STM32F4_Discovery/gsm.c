#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "gsm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "discoveryf4utils.h"
#include "uprintf.h"
#include <stdio.h>
#include <string.h>

const uint8_t* gsm_startup_cmd_without_sim[]=
{
	(uint8_t*)"\r\nAT\r\n",
//	(uint8_t*)"\r\nATZ\r\n",								//reset and restore configuration
	(uint8_t*)"\r\nATE0\r\n",								//CMD_SET_ECHO_OFF
//	(uint8_t*)"\r\nAT+KSLEEP=2\r\n",						//The module never goes in sleep mode
//	(uint8_t*)"\r\nAT&R1\r\n",								//CTS is always ON
//	(uint8_t*)"\r\nAT+CFUN=1\r\n",							//CMD_SET_PHONE_FUNC
//	(uint8_t*)"\r\nAT+KPATTERN =\"--EOF--Pattern--\"\r\n",	//CMD_SET_EOF_PATTERN do not change this
//	(uint8_t*)"AT+CREG=1\r",								//show URC network registered
//	(uint8_t*)"AT+CLVL=10\r",

	NULL
};

const uint8_t* gsm_startup_cmd_with_sim[]=
{
	(uint8_t*)"\r\nAT+CMGD=0,4\r\n",						//Delete all messages
	(uint8_t*)"\r\nAT+CMGF=1\r\n",							//CMD_SET_SMS_FORMAT
	(uint8_t*)"\r\nAT+CNMI=2,1,0,1,0\r\n",					//CMD_NEW_SMS_INDICATION
	(uint8_t*)"\r\nAT+CSMP=49,167,0,0\r\n",					//CMD_SET_SMS_TEXT_MODE_PARAM
	(uint8_t*)"\r\nAT+CSCS=\"IRA\"\r\n",					//CMD_SET_TE_CHAR
	(uint8_t *)"AT+COLP=1\r",
	(uint8_t *)"AT+KURCCFG=\"TCP\",1\r",
	(uint8_t*)"AT+CLIP=1\r",
	NULL
};
xQueueHandle xQueue_GSM;
xQueueHandle at_cmd_queue;
xQueueHandle at_resp_queue;
xQueueHandle rx_queue;

#define RX_BUF_SIZE					128
uint8_t gsm_rx_buf[RX_BUF_SIZE];
uint32_t gsm_rx_buf_ptr;

gsm_state_t gsm_state;
void USART3_IRQHandler(void)
{
//	portBASE_TYPE xTaskWokenByPost = pdFALSE;
    char t;
	if( USART_GetITStatus(SIM5320_UART3, USART_IT_RXNE) )
    {
//        uprintf("in\r\n");
		t = USART_ReceiveData(SIM5320_UART3);
        debug_send_char(t);
//        debug_send_char('a');
//        xQueueSendToBackFromISR(xQueue_GSM, &t, &xTaskWokenByPost);
        USART_ClearFlag(SIM5320_UART3, USART_FLAG_RXNE);
	}
//    if( != pdPASS)
//        debug_printf("eror\r\n");
//    debug_printf("12");
    
//    portEND_SWITCHING_ISR(xTaskWokenByPost);
    
}

void gsm_init(void)
{
    GPIO_InitTypeDef 	GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB1PeriphClockCmd(SIM5320_UART3_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(SIM5320_PWON_GPIO_CLK | SIM5320_ENABLE_GPIO_CLK | SIM5320_UART3_TX_GPIO_CLK, ENABLE);    
    // RST PIN, ENABLE, COMMON PORT
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = SIM5320_ENABLE_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(SIM5320_ENABLE_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = SIM5320_PWON_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(SIM5320_PWON_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = SIM5320_RST_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(SIM5320_RST_GPIO_PORT, &GPIO_InitStructure);
    
    // RI
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_Pin = SIM5320_RI_PIN;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_Init(SIM5320_RI_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SIM5320_UART3_RX_PIN | SIM5320_UART3_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(SIM5320_UART3_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(SIM5320_UART3_PORT,SIM5320_UART3_TX_PIN_SOURCE,SIM5320_UART3_AF);
    GPIO_PinAFConfig(SIM5320_UART3_PORT,SIM5320_UART3_RX_PIN_SOURCE,SIM5320_UART3_AF);
    
    USART_InitStructure.USART_BaudRate = 115200;//SIM5320_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 

    USART_Init(SIM5320_UART3, &USART_InitStructure);
    USART_Cmd(SIM5320_UART3, ENABLE);
}

void gsm_irq_init(void)
{
    EXTI_InitTypeDef  EXTI_InitStructure;
    //uart
    
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_ITConfig(SIM5320_UART3, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = SIM5320_UART3_EXTI_IRQChannel;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY - 1 ;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    USART_Cmd(SIM5320_UART3, ENABLE);
    
    //RI PIN
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//    SYSCFG_EXTILineConfig(SIM5320_RI_EXTI_PORT_SOURCE, SIM5320_RI_EXTI_PIN_SOURCE);
//     /* Configure EXTI Line0 */
//    EXTI_InitStructure.EXTI_Line = SIM5320_RI_EXTI_LINE;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//  
//    /* Enable and set EXTI Line0 Interrupt to the lowest priority */
//    NVIC_InitStructure.NVIC_IRQChannel = USER_RI_EXTI_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
}

void EXTI9_5_IRQHandler(void)
{
    
}

void gsm_send_char(char chr)
{
    while (USART_GetFlagStatus(SIM5320_UART3, USART_FLAG_TXE) == RESET);
    USART_SendData(SIM5320_UART3, (uint16_t)chr );
}

void gsm_putchar(char *s)
{
    while(*s)
    {
        gsm_send_char(*s);
        *(s ++);
    }
}
void gsm_put(char *s, uint8_t len)
{
    while(len--)
    {
        gsm_send_char(*s++);
    }
}
void gsm_poweron(void)
{
    GPIO_ResetBits(SIM5320_ENABLE_GPIO_PORT, SIM5320_ENABLE_PIN);
}

void gsm_poweroff(void)
{
    GPIO_SetBits(SIM5320_ENABLE_GPIO_PORT, SIM5320_ENABLE_PIN);
}

void gsm_enable(void)
{
    GPIO_SetBits(SIM5320_PWON_GPIO_PORT, SIM5320_PWON_PIN);
}

void gsm_reset_hw(void)
{
    gsm_setpin_init();
    GPIO_SetBits(SIM5320_RST_GPIO_PORT, SIM5320_RST_PIN);
    
    GPIO_ResetBits(SIM5320_RST_GPIO_PORT, SIM5320_RST_PIN);
    vTaskDelay( 500 / portTICK_RATE_MS );
    GPIO_SetBits(SIM5320_RST_GPIO_PORT, SIM5320_RST_PIN);
    vTaskDelay( 2000 / portTICK_RATE_MS );
}
void gsm_disable(void)
{
    GPIO_ResetBits(SIM5320_PWON_GPIO_PORT, SIM5320_PWON_PIN);
}

void gsm_setpin_init(void)
{
    GPIO_SetBits(SIM5320_RST_GPIO_PORT, SIM5320_RST_PIN);
    GPIO_ResetBits(SIM5320_DTR_GPIO_PORT, SIM5320_DTR_PIN);
}
void gsm_reset_init(void)
{
    gsm_disable();
    
    gsm_poweron();
    vTaskDelay( 1000 / portTICK_RATE_MS );
    gsm_enable();
    vTaskDelay( 200 / portTICK_RATE_MS );
    gsm_disable();
    vTaskDelay( 6000 / portTICK_RATE_MS );
}
int8_t alloc_queue(void) 
{
	at_cmd_queue = xQueueCreate(AT_COMMAND_QUEUE_SIZE, AT_COMMAND_TYPE_SIZE(at_command_t));
	if (!at_cmd_queue) {
		/* logging error here */
		uprintf("\r\n[gsm_uart]alloc at_cmd_queue fail\r\n");
		return -1;
	}
	at_resp_queue = xQueueCreate(AT_RESPONSE_QUEUE_SIZE, AT_RESPONSE_TYPE_SIZE(at_response_t));
	if (!at_resp_queue) {
		/* logging error here */
		uprintf("\r\n[gsm_uart]alloc at_resp_queue fail\r\n");
		return -1;
	}
	rx_queue = xQueueCreate(RX_QUEUE_SIZE, RX_TYPE_SIZE(uint8_t));
	if (!rx_queue) {
		/* logging error here */
		uprintf("\r\n[gsm_uart]alloc rx buffer fail\r\n");
		return -1;
	}
	return 0;
}

gsm_error_t terminal_response_callback(uint8_t *resp, void *object) {
	if (strstr((char *)resp, "OK")) {
		return GSM_NO_ERR;
	} else if (strstr((char *)resp,"ERROR")) {
		return GSM_COMMAND_ERROR;
	} else {
		return GSM_CONTINUE_SEARCH;
	}
}

gsm_error_t gsm_send_multi_command(uint8_t **command_group) 
{
	uint8_t cmd_index = 0;
	gsm_error_t error = GSM_NO_ERR;
//	gsm_process_urc(0, NULL);
	while (command_group[cmd_index] != NULL) {
		error = gsm_send_at_command_and_wait(command_group[cmd_index], strlen((char *)command_group[cmd_index]), terminal_response_callback, 2000, NULL);
		if (error != GSM_NO_ERR)
		{
            uprintf("GSM1_error\n\r");
			return error;
		}
		cmd_index++;
	}
    uprintf("gsm_send_multi_comma nd done123\n\r");
	return GSM_NO_ERR;
}

gsm_error_t gsm_get_response_from_queue(at_response_t *resp, uint32_t timeout) 
{
	if (xQueueReceive(at_resp_queue, resp, timeout) == pdPASS) 
    {
        uprintf("1\r\n");
		return GSM_NO_ERR;
	} else 
    {
		//todo: logging here
        uprintf("from queue error\r\n");
		return GSM_QUEUE_EMPTY;
	}
    
}

gsm_error_t gsm_send_at_command_and_wait(uint8_t *cmd, uint32_t len,  gsm_error_t (*callback)(uint8_t *resp, void *obj), uint32_t timeout, void * object) {
	static uint32_t timeout_count;
	gsm_error_t error = GSM_COMMAND_FAIL;
	at_response_t at_resp;
//    uprintf("gsm_send_at_command_and_wait\n\r");
	error = gsm_send_at_command_to_queue(cmd, len, AT_COMMAND, 0);
	if (error == GSM_QUEUE_FULL || error == GSM_ALLOC_MEM_FAIL) 
    {
        uprintf("GSM2_error\n\r");
		return error;
	}
	/* check if need wait callback */
	if (callback == NULL) 
    {
        uprintf("GSM3_error\n\r");
		return GSM_NO_ERR;
	}
	timeout_count = timeout / GSM_TIMEOUT_STEP;
	do {
        uprintf("start do\r\n");
		if (gsm_get_response_from_queue(&at_resp, GSM_TIMEOUT_STEP/portTICK_RATE_MS ) == GSM_NO_ERR) 
        {
			// hanv14
			error = callback(at_resp.data, object);
//            uprintf("call back error\n\r");
			vPortFree(at_resp.data);
			if (error != GSM_CONTINUE_SEARCH) 
            {
//                uprintf("GSM4_error\n\r");
				return error;
			}
             uprintf("if\r\n");
		}
        else
            uprintf("error get respon\r\n");
        uprintf("do while  %d\r\n", timeout_count);
	} while (timeout_count--);
    uprintf("done do while\r\n");
	return GSM_COMMAND_FAIL;
}

gsm_error_t gsm_send_at_command_to_queue(uint8_t *cmd, uint32_t len, command_type_t type,  uint32_t timeout) {
	at_command_t  at_cmd;
		at_cmd.len = len;
		at_cmd.data = pvPortMalloc(at_cmd.len);
		at_cmd.type = type;
        
		if (!at_cmd.data) {
			//todo: set error, logging here
			return GSM_ALLOC_MEM_FAIL;
		}
		memcpy(at_cmd.data, cmd, at_cmd.len);
        
	if (xQueueSend(at_cmd_queue, &at_cmd, timeout) == pdPASS) {
		//todo:logging here
        uprintf("done\r\n");
		return GSM_NO_ERR;
	} 
    else 
        {
		vPortFree(at_cmd.data);
		//todo:logging here
            uprintf("error\r\n");
		return GSM_QUEUE_FULL;
	}
}

uint8_t gsm_queue_init(void)
{
    xQueue_GSM = xQueueCreate(QUEUE_COUNT, sizeof( char *));
    if(!xQueue_GSM)
    {
        debug_printf("Queue init erorr\r\n");
        return 0;
    }
    else
        return 1;
}

gsm_error_t gsm_send_response_to_queue(at_response_t *resp, uint32_t timeout) {
	if (xQueueSend(at_resp_queue, resp, timeout) == pdPASS) {
		return GSM_NO_ERR;
	} else {
		vPortFree(resp->data);
		//todo: logging here
		return GSM_QUEUE_FULL;
	}
}

void vGSM_init(void *pvParameters)
{
    at_command_t cmd;
    char buf[50];
    gsm_state = GSM_STARTUP;
    for(;;)
    {
        switch(gsm_state)
        {
            case GSM_STARTUP:
                uprintf("GSM_STARTUP\n\r");
                if (gsm_send_multi_command((uint8_t **)gsm_startup_cmd_without_sim) == GSM_NO_ERR) 
                {
                    gsm_state = GSM_CHECK_SIM;
                    uprintf("GSM_CHECK_SIM\r\n");
                } else
                {
                    uprintf("GSM_STARTUP NOT OK\n\r");
                }
                
                break;
//            case GSM_CHECK_SIM:
//                uprintf("GSM_CHECK_SIM\n\r");
//                if (gsm_send_multi_command((uint8_t **)gsm_startup_cmd_with_sim) != GSM_NO_ERR) {
//                    break;
//                }
//                gsm_state = GSM_CHECK_NETWORK;
//                break;
            default: break;
        }
        
        vTaskDelay( 100 / portTICK_RATE_MS );
    }
}

void vGSM_PROC(void *pvParameters)
{
    at_command_t cmd;
    at_response_t resp;
    char buf[50];
    uprintf("GSM_PROC\r\n");
    if (xQueueReceive(at_cmd_queue, &cmd, 100/portTICK_RATE_MS) == pdPASS) 
    {
        if (cmd.type == GET_FILE) 
        {
            uprintf("recv_file->status = SENDING_COMMAND\n\r");
        }
//			DBG_NPRINTF(cmd.data, cmd.len);
        gsm_put((char *)cmd.data, cmd.len);
        vPortFree(cmd.data);
    }
    if (xQueueReceive(rx_queue, &gsm_rx_buf[gsm_rx_buf_ptr], 50/portTICK_RATE_MS) == pdPASS) 
    {
        gsm_rx_buf_ptr++;
        while (gsm_rx_buf_ptr < RX_BUF_SIZE) 
        {
            uprintf("GSM_PROC\r\n");
            if (xQueueReceive(rx_queue, &gsm_rx_buf[gsm_rx_buf_ptr], 50/portTICK_RATE_MS) == pdPASS) 
            {
                gsm_rx_buf_ptr++;
            }
            resp.len = gsm_rx_buf_ptr;
            resp.data = pvPortMalloc(gsm_rx_buf_ptr + 1);
            if (!resp.data) {
            //todo: logging here
            uprintf("[gsm_uart]alloc mem fail\r\n");
            break;
            }
            memcpy(resp.data, gsm_rx_buf, gsm_rx_buf_ptr);
            resp.data[gsm_rx_buf_ptr] = 0;
            //						DBG_PRINTF((char*)resp.data);
            gsm_rx_buf_ptr = 0;
            gsm_send_response_to_queue(&resp, 0);
        }
    }
    else
        uprintf("no data queue\r\n");
    vTaskDelay( 100 / portTICK_RATE_MS );
}

void vGSM_TEST(void *pvParameters)
{
    gsm_state = GSM_STARTUP;
    uprintf("gsm_test_init\r\n");
    gsm_reset_init();
    gsm_setpin_init();
    uprintf("init done\r\n");
//    gsm_reset_hw();
    for(;;)
    {
         switch(gsm_state)
        {
            
            case GSM_STARTUP:
                
                uprintf("GSM_TEST\r\n");
                gsm_put("AT\r\n", 4);
//                vTaskDelay( 1000 / portTICK_RATE_MS );
//                gsm_put("ATE0\r\n",6);
//                vTaskDelay( 1000 / portTICK_RATE_MS );
                gsm_state = GSM_CHECK_SIM;
                break;
            case GSM_CHECK_SIM:
                uprintf("GSM_CHECK_SIM\r\n");
                break;
            default: break;
        }
        vTaskDelay( 2000 / portTICK_RATE_MS );
    }
}
