#ifndef MAIN_H__
#define MAIN_H__

#define LED1                GPIO_Pin_0
#define LED2                GPIO_Pin_1
#define LED3                GPIO_Pin_2
#define LED4                GPIO_Pin_3

#define DT                  GPIO_Pin_4
#define SH                  GPIO_Pin_6
#define ST                  GPIO_Pin_7

#define LED_TASK_PRIORITY				1
#define PC_TASK_PRIORITY				1
#define RTC_TASK_PRIORITY				1
#define ADE_TASK_PRIORITY				2
#define GSM_INIT_TASK_PRIORITY			1
#define GSM_PROC_TASK_PRIORITY			3
//SemaphoreHandle_t xSemaphore;
#define STACK_SIZE_MIN	512	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#endif
