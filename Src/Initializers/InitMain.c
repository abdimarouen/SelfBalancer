//
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * This notice applies to any and all portions of this file
//  * that are not between comment pairs USER CODE BEGIN and
//  * USER CODE END. Other portions of this file, whether 
//  * inserted by the user or by software development tools
//  * are owned by their respective copyright owners.
//  *
//  * Copyright (c) 2019 STMicroelectronics International N.V. 
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without 
//  * modification, are permitted, provided that the following conditions are met:
//  *
//  * 1. Redistribution of source code must retain the above copyright notice, 
//  *    this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  * 3. Neither the name of STMicroelectronics nor the names of other 
//  *    contributors to this software may be used to endorse or promote products 
//  *    derived from this software without specific written permission.
//  * 4. This software, including modifications and/or derivative works of this 
//  *    software, must execute solely and exclusively on microcontroller or
//  *    microprocessor devices manufactured by or for STMicroelectronics.
//  * 5. Redistribution and use of this software other than as permitted under 
//  *    this license is void and will automatically terminate your rights under 
//  *    this license. 
//  *
//  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
//  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
//  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
//  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
//  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
//  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
//  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "stm32f3_discovery.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
#include "usbd_cdc_if.h"
#include "LLdrivers.h"
#include <math.h>
#include <stdbool.h>

//static double globAccelangle = 0;

//
//
///* USER CODE END Includes */
//
///* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;
//
//SPI_HandleTypeDef hspi1;
//
//
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;
//
//UART_HandleTypeDef huart4;
//
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
//
///* USER CODE BEGIN PV */
///* Private variables ---------------------------------------------------------*/
//
///* USER CODE END PV */
//

//
///* USER CODE BEGIN PFP */
///* Private function prototypes -----------------------------------------------*/
//
///* USER CODE END PFP */
//
///* USER CODE BEGIN 0 */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
//extern uint8_t Buf[50];
uint8_t DataBuf[20];
uint8_t RXBuf[20] ;

osThreadId PrintserialID;
osThreadId defaultTaskID;

volatile float Alpha ;
float GyroDrift = GYROSCOPE_DRIFT;
static float sum = 0;
static float Asum = 0;
float sum1,sum2,AOA;
E_SytemState e_CurrState = E_STATE_Calibrating;
E_SytemSubState e_CurrSubState;

    typedef struct {
      double D_Angle;
    } properties_t;


    osMailQDef (object_pool_qCMD, 2, ST_CommParam);  // Declare mail queue
    osMailQId  (object_pool_q_idCMD);                 // Mail queue ID
    
    volatile int16_t xval, yval ,zval= 0x00; // accel val
    
    float yaw = 0;
    float AVGYaw = 0;
    
    extern bool b_DebugEnabled ;
    extern bool b_Reeinitialise;
    volatile float Xval,Yval,Yval1,Zval = 0x00; //gyro val
    volatile float Xval1=0;
    
    ST_CommParam stCurrentState;
    float CMD_Angle=0;
    float Angle;
//    float Ref_ACCELAngle;

    bool enableMotors;
///* USER CODE END 0 */

////add Timer 4 and timer 15 for square PWM command for the drivers of Steppers

////Timer4  channel4 -> PD15 stepper
////Timer15 channel1 -> PB4  stepper
////Timer16 channel1 -> PF9  servo
////Timer17 channel1 -> PB5  servo

//extern TIM_HandleTypeDef htim4;

float Buffer[3];
//float imuVAL =0;
float CALL_angle;
char reeinitfilter=false;
void AngleCalcTask(void const * argument)
{
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    init_PWMTimers();
    
//this variable prevents the gyroscopic drift
    
    HAL_TIM_Base_Start_IT(&htim7);
    osDelay(10);//this is a time bomb! the queue pointer is null if this task starts first that's why we delay XD !
    
    
    //osThreadTerminate(PrintserialID);
    //osThreadTerminate(defaultTaskID);

    
    for(;;)
    {
        GanttDebug(3);
        
        yaw = getAccelAngle();
        
        Angle = Alpha* Angle +(1-Alpha)*yaw;

        CMD_Angle = Angle - stCurrentState.angleOffset;
        
        if ((e_CurrState == E_STATE_Balancing)||(e_CurrState == E_STATE_Fall))
        {
            calculatePID(&stCurrentState,CMD_Angle);
        }
        
        //this function is responsible for the handling of the balance state and the fail  state case
        stateManage(ABS(CMD_Angle), &stCurrentState);
        
        setMotorCmd(&stCurrentState);
        
        //osDelay(ACTIVE_DELAY_MS);
        osDelay(10);
        //HAL_Delay(9);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
        
        //end TASK2 //////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}
unsigned char bfr[10] ;
void printSerial(void const * argument)
{
    
    HAL_UART_Receive_IT(&huart3,bfr,sizeof(bfr));
    PrintserialID = osThreadGetId();
    
    for(;;)
    {
        GanttDebug(1);
#if (DEBUG_VAL == DEBUG_SPEED)
        debugPrint(CMD_Angle, stCurrentState.speed);
#elif (DEBUG_VAL == DEBUG_ACCEL)
        //debugPrint(CMD_Angle, yaw);
        debugPrint(CMD_Angle, stCurrentState.speed);
#endif
        
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
        //HAL_UART_Receive(&huart3,bfr,8,100);
        // printf("--> got :<%s>",bfr);
        
        osDelay(10);
        //setMotorCmd(&stCurrentState);
        
    }
}

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    defaultTaskID = osThreadGetId();
    
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    enableLeftMDriver(Enable);
    enableRightMDriver(Enable);
    
    setStepperMotorMode(Eigth_S);
    
    //initialize PID & State machine VAR
    initialiseParam(&stCurrentState);
    HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    
    /* Infinite loop */
    for(;;)
    {
        GanttDebug(2);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);//Orange
        LLDriverCliMenu(DataBuf,&stCurrentState);
        //setMotorCmd(&stCurrentState);
        osDelay(100);
    }
}

int i=0;
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
    //HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    i=strlen((const char *)RXBuf);
    if ((RXBuf[i-1]== 0x0D)||(RXBuf[i-1]== 0x0A))//carriage return or line feed
    {
        HAL_UART_RxCpltCallback(&huart1);
        HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    }
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
    //HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    //i=strlen((const char *)RXBuf);
    //if ((RXBuf[i-1]== 0x0D)||(RXBuf[i-1]== 0x0A))//carriage return or line feed
    {
        HAL_UART_RxCpltCallback(&huart3);
        HAL_UART_Receive_IT(&huart2,bfr,sizeof(bfr));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart1)
    {
        memcpy(DataBuf,RXBuf,sizeof(DataBuf));
        //printf("HAL_UART_RxCpltCallback\r\n");
        HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
        //printf("DB%s\r\n",DataBuf);
        memset(RXBuf,'\0',sizeof(RXBuf));
        huart->RxState = HAL_UART_STATE_READY;
    }
    else if (huart==&huart3)
    {
    
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart1)
    {
        //errors could be overflow or overrun
        __NOP();
        // printf("HAL_UART_ErrorCallback\r\n");
        // printf("DB:%s\r\n",DataBuf);
        //clear the data
        memset(RXBuf,'\0',sizeof(RXBuf));
        HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    }
    else if (huart==&huart3)
    {
        
    }
}

void TIM7_IRQHandler(void)
{
  static int i = 0,C=1;
  static float xvalnoise = 0;
  BSP_GYRO_GetXYZ(Buffer);
  Xval = Buffer[0];
  
  //if calibrating 
  if (E_STATE_Calibrating == e_CurrState)
  {
      AllLedSetState(GPIO_PIN_SET);
      
      sum+=Xval;
      i++;
      Asum = AVG(Xval);
      Alpha = ALPHA_CALIB;
      
        if (i >=1500)
        {
            e_CurrState = E_STATE_POSTCalibVERIF;
            //e_CurrState = E_STATE_Balancing;
            i=0;
            sum/=1500;
            Angle = yaw;
            AllLedSetState(GPIO_PIN_RESET);
            
            reeinitfilter = 1;
            
            //Alpha = ALPHA;
            //sum = 0;
        }
  }

  if (E_STATE_POSTCalibVERIF == e_CurrState)
  {
      static int j = 0;
      static int stabilitycounter = 0;
      static float referenceValue = 0;
      j++;
      //magic!!
      AllLedSetState(GPIO_PIN_SET);
      if (j==10)
      {
          j=0;
          AOA = AVGOAVG(sum,reeinitfilter);
          reeinitfilter = 0;
          
          if ((AOA<(referenceValue + ANGLE_VERIF_TSH))&&(AOA>(referenceValue - ANGLE_VERIF_TSH)))
          {
            stabilitycounter++;
          }
          else
          {
            stabilitycounter = 0;
            referenceValue = AOA;
          }
      }
      
      //duplicated from next state---------------
      sum = AVG(Xval);
      if (ABS(Xval-Asum)<xvalnoise)
      {
        Xval = 0;
      }
      Xval1=Xval-Asum;
      
      sum1= Asum;
      sum2=sum;
      
      Angle += Xval1*0.004;
      //----------------------------------------
      if (stabilitycounter>ANGLE_VERIF_MAX_SAMPL)
      {
        stabilitycounter = 0;
        Asum = AOA;
        e_CurrState = E_STATE_Fall;
        AllLedSetState(GPIO_PIN_RESET);
        
      }

  }
  
  else if (E_STATE_Balancing == e_CurrState)
  {
      sum = AVG(Xval);
      
      //if ((E_STATE_Balancing == e_CurrState)&&(stCurrentState.speed==0)&&(ABS(Asum)-ABS(sum)<0.7))
      //{
      //     Asum = Asum * 0.8 + sum*0.2;
      //}
      if (ABS(Xval-Asum)<xvalnoise)
      {
        Xval = 0;
      }
      //if (ABS(Xval)*1000<2500)
      //{
      //  Xval = 0;
      //}
      Xval1=Xval-Asum;//-(stCurrentState.angleOffset*0.1);
      //sum1= Asum-(stCurrentState.angleOffset*0.1);
      sum1= Asum;
      sum2=sum;
      //Asum-=0.1;
      Angle += Xval1*0.004;
      
      //angle = angle * 0.9996 + yaw * 0.0004;
  }

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
