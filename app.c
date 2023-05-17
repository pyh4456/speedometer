/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : YS
*                 DC
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <includes.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  APP_TASK_EQ_0_ITERATION_NBR              16u
/*
*********************************************************************************************************
*                                            TYPES DEFINITIONS
*********************************************************************************************************
*/

typedef enum {
   TASK_Measure,
   TASK_Alert,
   TASK_USART,

   TASK_N
}task_e;


typedef struct
{
   CPU_CHAR* name;
   OS_TASK_PTR func;
   OS_PRIO prio;
   CPU_STK* pStack;
   OS_TCB* pTcb;
}task_t;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void  AppTaskStart          (void     *p_arg);
static  void  AppTaskCreate         (void);
static  void  AppObjCreate          (void);

static void AppTask_Measure(void *p_arg);		//task measure the speed of object
static void AppTask_Alert(void *p_arg);			//control LED and buzzer
static void AppTask_USART(void *p_arg);			//take speed limit from USART and print the speed of object

static void  sensor_Init();
void TIM2_us_Delay(uint32_t delay);
uint32_t read_ultrasonic1();
uint32_t read_ultrasonic2();
uint32_t calc_speed(uint32_t distance1, uint32_t distance2,  OS_TICK time);
uint32_t isqrt(uint32_t number);


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/* ----------------- APPLICATION GLOBALS -------------- */
static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB   Task_Measure_TCB;
static  CPU_STK  Task_Measure_Stack[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB   Task_Alert_TCB;
static  CPU_STK  Task_Alert_Stack[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB   Task_USART_TCB;
static  CPU_STK  Task_USART_Stack[APP_CFG_TASK_START_STK_SIZE];

task_t cyclic_tasks[TASK_N] = {
   {"Task_Measure" , AppTask_Measure,  0, &Task_Measure_Stack[0] , &Task_Measure_TCB},
   {"Task_Alert" , AppTask_Alert,  2, &Task_Alert_Stack[0] , &Task_Alert_TCB},
   {"Task_USART", AppTask_USART, 1, &Task_USART_Stack[0], &Task_USART_TCB},
};

typedef enum{
	NOT_PASSING,
	REGULATION_SPEED,
	OVER_SPEED
}measurement_status;

measurement_status measure = NOT_PASSING;

int waiting_input = 1;		// 1: waiting USART input, 0: don't wait USART input
int speed_limit, speed;

OS_SEM MySem;							// Semaphore



/* ------------ FLOATING POINT TEST TASK -------------- */
/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

int main(void)
{
    OS_ERR  err;

    /* Basic Init */
    RCC_DeInit();
//    SystemCoreClockUpdate();

    /* BSP Init */
    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    CPU_Init();                                                 /* Initialize the uC/CPU Services                       */
    Mem_Init();                                                 /* Initialize Memory Management Module                  */
    Math_Init();                                                /* Initialize Mathematical Module                       */

    /* OS Init */
    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    //create semaphore
    OSSemCreate(&MySem,
             "My Semaphore",
            1,
            &err);

    OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,              /* Create the start task                                */
                 (CPU_CHAR     *)"App Task Start",
                 (OS_TASK_PTR   )AppTaskStart,
                 (void         *)0u,
                 (OS_PRIO       )APP_CFG_TASK_START_PRIO,
                 (CPU_STK      *)&AppTaskStartStk[0u],
                 (CPU_STK_SIZE  )AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
                 (OS_MSG_QTY    )0u,
                 (OS_TICK       )0u,
                 (void         *)0u,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);

   OSStart(&err);   /* Start multitasking (i.e. give control to uC/OS-III). */

   (void)&err;

   return (0u);
}
/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
static  void  AppTaskStart (void *p_arg)
{
    OS_ERR  err;

   (void)p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();
    BSP_Tick_Init();                                            /* Initialize Tick Services.                            */
    sensor_Init();


#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

   APP_TRACE_DBG(("Creating Application Kernel Objects\n\r"));
   AppObjCreate();                                             /* Create Applicaiton kernel objects                    */

   APP_TRACE_DBG(("Creating Application Tasks\n\r"));
   AppTaskCreate();                                            /* Create Application tasks                             */
}



static void AppTask_Measure(void *p_arg){
    OS_ERR  err;
    CPU_TS ts;
    uint32_t data1,dist1,data2,dist2, measured_speed;
    OS_TICK time;

    int button = 0;

    while (DEF_TRUE) {

    	OSSemPend(&MySem, 0, OS_OPT_PEND_BLOCKING, &ts,   &err);

    	switch(err){
    	case OS_ERR_NONE:
    		data1 = read_ultrasonic1();
    		OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    		dist1 = (data1*24)/500 ; //distance between sensor and object

    		while(dist1 > 400){
    			OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    			data1 = read_ultrasonic1();
    			OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    			dist1 = (data1*24)/500 ; //distance between sensor and object
    		}
    		OSTimeSet(0, &err);

    		OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    		data2 = read_ultrasonic2();
    		dist2 = (data2*24)/500 ; //distance between sensor and object

    		while(dist2 > 400){
    			OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    			data2 = read_ultrasonic2();
    			OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    			dist2 = (data2*24)/500 ; //distance between sensor and object
    		}

    		time = OSTimeGet(&err);

    		measured_speed = calc_speed(dist1, dist2,  time);
    		speed = measured_speed;

    		if(measured_speed > speed_limit){
    			measure = OVER_SPEED;
    		}
    		else{
    			measure = REGULATION_SPEED;
    		}

    		OSSemPost(&MySem, OS_OPT_POST_1, &err);

    	case OS_ERR_PEND_ABORT:
    		break;
    	case OS_ERR_OBJ_DEL:
    		break;
    	}

    	button = 0;

    	OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}


static void AppTask_Alert(void *p_arg){

	CPU_TS ts;
	OS_ERR  err;

	measurement_status status = 0;

 	 while (DEF_TRUE) {
 		 OSSemPend(&MySem, 0, OS_OPT_PEND_BLOCKING, &ts, &err);

 		 switch(err){
 		 case OS_ERR_NONE:
 			 status = measure;
 			 measure = NOT_PASSING;
 			 OSSemPost(&MySem, OS_OPT_POST_1, &err);

 		 case OS_ERR_PEND_ABORT:
 			 break;
 		 case OS_ERR_OBJ_DEL:
 			 break;
 		 }

 		 switch(status){
 		 case NOT_PASSING:
 			 BSP_LED_On(1);
 			 BSP_LED_Off(2);
 			 BSP_LED_Off(3);
 			 GPIO_ResetBits(GPIOA, GPIO_Pin_3);
 			 break;
 		 case REGULATION_SPEED:
 			 BSP_LED_Off(1);
 			 BSP_LED_On(2);
 			 BSP_LED_Off(3);
 			 GPIO_ResetBits(GPIOA, GPIO_Pin_3);
 			 break;
 		 case OVER_SPEED:
 			 BSP_LED_Off(1);
 			 BSP_LED_Off(2);
 			 BSP_LED_On(3);
 			 GPIO_SetBits(GPIOA, GPIO_Pin_3);
 			 break;
 		 }

 		 OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);

 		 BSP_LED_On(1);
 		 BSP_LED_Off(2);
 		 BSP_LED_Off(3);
 		 GPIO_ResetBits(GPIOA, GPIO_Pin_3);
 	 }
}


static void AppTask_USART(void *p_arg){

	uint16_t c;
    OS_ERR  err;
    CPU_TS ts;
    int sl = 0;		//speed limit
    int num, i = 0;
    OS_TICK time;
    measurement_status status;
    int wait_USART_input = 0;
    char text[20];
	char buf[15] = "";

    while (DEF_TRUE) {

       OSSemPend(&MySem, 0, OS_OPT_PEND_BLOCKING, &ts, &err);

       switch(err){
              case OS_ERR_NONE:
                 status = measure;
                 wait_USART_input = waiting_input;
                 waiting_input = 0;
                 speed_limit = sl;
                 num = speed;

                 OSSemPost(&MySem, OS_OPT_POST_1, &err);

              case OS_ERR_PEND_ABORT:
                 break;
              case OS_ERR_OBJ_DEL:
                 break;
       }


       if(wait_USART_input){
    	   i = 0;
    	   buf[15] = "";
    	   send_string("Input speed limit : ");

    	   while(DEF_TRUE){
    		   while(USART_GetFlagStatus(Nucleo_COM1, USART_FLAG_RXNE) == RESET){
    			   //OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
    		   }
    		   c = USART_ReceiveData(Nucleo_COM1);
    		   if(c == '\r'){
    			   break;
    		   }
    		   else{
    			   USART_SendData(Nucleo_COM1, c);
    			   buf[i++] = c;
    		   }
    	   }
    	   send_string("\n\r");
    	   send_string("The speed limit is set to ");
    	   send_string(buf);
    	   send_string("km/h\n\r\n\r");
    	   sl = atoi(buf);
       }
       else{

    	   switch(status){
    	   case NOT_PASSING:
    		   //send_string("not passing\n\r");
    		   break;
    	   case OVER_SPEED:
    		   send_string("speed : ");
    		   sprintf(text, "%u", num);
    		   send_string(text);
    		   send_string("km/h");
    		   send_string("\n\r");

    		   send_string("the vehicle is speeding\n\r\n\r");
    		   break;
    	   case REGULATION_SPEED:
    		   send_string("speed : ");
    		   sprintf(text, "%u", num);
    		   send_string(text);
    		   send_string("km/h");
    		   send_string("\n\r");

    		   send_string("The vehicle obeys the speed limit\n\r\n\r");
    		   break;
    	   }

       }

       OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);

    }
}


/*
*********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create application tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
   OS_ERR  err;

   u8_t idx = 0;
   task_t* pTask_Cfg;
   for(idx = 0; idx < TASK_N; idx++)
   {
      pTask_Cfg = &cyclic_tasks[idx];

      OSTaskCreate(
            pTask_Cfg->pTcb,
            pTask_Cfg->name,
            pTask_Cfg->func,
            (void         *)0u,
            pTask_Cfg->prio,
            pTask_Cfg->pStack,
            pTask_Cfg->pStack[APP_CFG_TASK_START_STK_SIZE / 10u],
            APP_CFG_TASK_START_STK_SIZE,
            (OS_MSG_QTY    )0u,
            (OS_TICK       )0u,
            (void         *)0u,
            (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
            (OS_ERR       *)&err
      );
   }
}

/*
*********************************************************************************************************
*                                          AppObjCreate()
*
* Description : Create application kernel objects tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppObjCreate (void)
{

}


static void  sensor_Init()
{
   GPIO_InitTypeDef sensor_init = {0};


   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
   RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   sensor_init.GPIO_Mode   = GPIO_Mode_OUT;
   sensor_init.GPIO_PuPd   = GPIO_PuPd_DOWN;
   sensor_init.GPIO_Pin    = GPIO_Pin_3;
   sensor_init.GPIO_OType  = GPIO_OType_PP;
   sensor_init.GPIO_Speed  = GPIO_Speed_25MHz;
   GPIO_Init(GPIOA, &sensor_init);


   //echo (ultrasonic sensor1)
   sensor_init.GPIO_Mode   = GPIO_Mode_IN;
   sensor_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
   sensor_init.GPIO_Pin    = GPIO_Pin_0;

   GPIO_Init(GPIOC, &sensor_init);

   //trig (ultrasonic sensor1)
   sensor_init.GPIO_Mode   = GPIO_Mode_OUT;
   sensor_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
   sensor_init.GPIO_Pin    = GPIO_Pin_3;
   sensor_init.GPIO_OType  = GPIO_OType_PP;
   sensor_init.GPIO_Speed  = GPIO_Speed_25MHz;
   GPIO_Init(GPIOC, &sensor_init);


   //echo (ultrasonic sensor2)
   sensor_init.GPIO_Mode   = GPIO_Mode_IN;
   sensor_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
   sensor_init.GPIO_Pin    = GPIO_Pin_3;

   GPIO_Init(GPIOF, &sensor_init);

   //trig (ultrasonic sensor2)
   sensor_init.GPIO_Mode   = GPIO_Mode_OUT;
   sensor_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
   sensor_init.GPIO_Pin    = GPIO_Pin_5;
   sensor_init.GPIO_OType  = GPIO_OType_PP;
   sensor_init.GPIO_Speed  = GPIO_Speed_25MHz;
   GPIO_Init(GPIOF, &sensor_init);


}

uint32_t read_ultrasonic1(){
    uint32_t data;
    int echo = 0;

	GPIO_ResetBits(GPIOC, GPIO_Pin_3);
	TIM2_us_Delay(2);
	GPIO_SetBits(GPIOC, GPIO_Pin_3);
	TIM2_us_Delay(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_3);

	TIM2_us_Delay(100);

	while(!echo){				//wait the echoback detection
		echo = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
	}

	while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)){	//count time until the echoback ends
		data = data + 1;
	}

	return data;
}

uint32_t read_ultrasonic2(){
    uint32_t data;
    int echo = 0;

	GPIO_ResetBits(GPIOF, GPIO_Pin_5);
	TIM2_us_Delay(2);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	TIM2_us_Delay(10);
	GPIO_ResetBits(GPIOF, GPIO_Pin_5);

	TIM2_us_Delay(100);

	while(!echo){				//wait the echoback detection
		echo = GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_3);
	}

	while(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_3)){	//count time until the echoback ends
		data = data + 1;
	}

	return data;
}

void TIM2_us_Delay(uint32_t delay){	//function makes delay in microseconds
    RCC->APB1ENR |=1; //Start the clock for the timer peripheral
    TIM2->ARR = (int)(delay/0.0625); // Total period of the timer
    TIM2->CNT = 0;
    TIM2->CR1 |= 1; //Start the Timer
    while(!(TIM2->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
    TIM2->SR &= ~(0x0001); //Reset the update interrupt flag
}

uint32_t isqrt(uint32_t number)
{
	uint32_t n = 1;
	uint32_t n1 = ((n + (number / n)) >> 1);

	while (n1 - n > 1){
		n = n1;
		n1 = ((n + (number / n)) >> 1);
	}
	while (n1 * n1 > number) n1--;

	return n1;
}

uint32_t calc_speed(uint32_t distance1, uint32_t distance2,  OS_TICK time) {
	uint32_t distance_f, speed;
	double c, length, calc;

	if(distance1 > distance2){
		c = (double)(distance1 - distance2);
	}
	else{
		c = (double)(distance2 - distance1);
	}

	length = 1000; // mm unit

	calc = (unsigned int)(c * c + 2 * c * length * 0.259 + length * length);

	distance_f = isqrt(calc);

	speed = (distance_f*3.6)/(time); // km/h


    return speed;
}
