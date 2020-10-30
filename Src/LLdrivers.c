#include "stm32f3xx_hal.h"
#include "LLdrivers.h"


#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <stdbool.h>

#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"

extern E_SytemState e_CurrState;
extern E_SytemSubState e_CurrSubState;

#define ABS(X)     ((X>=0)?X:-X)
extern float AVGYaw;
double xAcc , yAcc, zAcc = 0;
double nxAcc , nyAcc, nzAcc = 0;
//#define MOTOR(N)   (54 * N + 60000)
//#define TF(N)       (long)trunc(1/(N^2 *0.00002))

bool b_DebugEnabled ;
bool b_Reeinitialise ;

extern float Alpha;
extern float GyroDrift;

extern char enableMotors;
//timers declared
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim7;


double TF(double N)
{
    return (1/(N*N *0.00002));
    //return (1/(N*N *0.0016));
}

void init_PWMTimers()
{
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
}

//this function manages the abstract motor cmd
//ship enable
//rototion sense
//speed throuough PWM square timer

void setMotorCmd(ST_CommParam *receivedCMD)
{
    //in the case of implementing a transfert funcion it should not be here !!!!
    //uint16_t = TF(receivedCMD->speed);
    
    if(enableMotors==false) //overrite whaterver value the speed is by 0
    {
        receivedCMD->speed = 0;
    }
    
    if(receivedCMD)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim4, ARR(receivedCMD->speed));//D15
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,__HAL_TIM_GET_AUTORELOAD(&htim4)/2);
        TIM4->EGR|=TIM_EGR_UG;
        
        __HAL_TIM_SET_AUTORELOAD(&htim16,ARR(receivedCMD->speed));//b4
        __HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,__HAL_TIM_GET_AUTORELOAD(&htim16)/2);
        TIM16->EGR|=TIM_EGR_UG;
    }
}

// to do add the return typedef struct
void LLDriverCliMenu(uint8_t* Buf,ST_CommParam *stCommParam)
{
    if (Buf[0]=='T')
    {
        char TBuf[]="D19;P3800;I40;O0;S19;";
        memcpy(Buf,TBuf,sizeof(TBuf));
        printf("test sequence !!\n\r");
        osDelay(10);
    }
    
    //check if it is a frame input or a shor command input
    if (strlen((char*)Buf)<=3)
    {
       if (Buf[0]=='?')
        {
            //printf("\n\rO : Offset\n\rP : KP\n\rI : KI\n\rD : KD\n\rS : Speed\n\r");
            
            printf("\n\rO : Offset %f\n\rD : KD %f \n\rI : KI %f\n\rP : KP %f\n\rS : Speed %d\n\rAlpha: %f\r\n GyroDrift: %f\r\n",
              stCommParam->angleOffset,
              stCommParam->KD,
              stCommParam->KI,
              stCommParam->KP,
              stCommParam->speed,
              Alpha,
              GyroDrift
            );
            osDelay(10);
        }
        
        else if (Buf[0] == ENTER_ASCII)
        {
            printf("\n\rCMD> ");
            osDelay(10);
        }
        else if(Buf[0] == 'd')
        {
            printf("\n\rDebug ON !!! ");
            b_DebugEnabled = true;
            osDelay(10);
        }
        
        else if(Buf[0] == '-')
        {
            printf("\n\rDebug OFF !!! ");
            b_DebugEnabled = false;
            osDelay(10);
        }
        //else //could only be used in inteerrupt mode
        //{
        //    printf("%d\r\n",strlen((const char *)Buf));
        //}
    }
    //set command
    else
    {
        char Value[10]="";
        char* pBuf = (char*)Buf;
        char i;
        
        for (i=0;i<strlen((char*)Buf)-1;i++)
        {
            if (pBuf[i] == STR_DELIM)
            {
                memcpy(Value,pBuf,i);
                pBuf=&pBuf[i]+1;
                
                if (Value[0]=='P')
                {
                    stCommParam->KP = atof((const char *)Value+1);
                    printf("KP set OK %f\r\n",stCommParam->KP);
                    osDelay(10);
                }
                
                else if (Value[0]=='I')
                {
                    stCommParam->KI = atof((const char *)Value+1);
                    printf("KI set OK %f\r\n",stCommParam->KI);
                    osDelay(10);
                }
                
                else if (Value[0]=='D')
                {
                    stCommParam->KD = atof((const char *)Value+1);
                    printf("KD set OK %f\r\n",stCommParam->KD);
                    osDelay(10);
                }
                
                else if (Value[0]=='O')
                {
                    stCommParam->angleOffset = atof((const char *)Value+1);
                    printf("angleOffset set OK %f\r\n",stCommParam->angleOffset);
                    osDelay(10);
                }
                
                else if (Value[0]=='S')
                {
                    stCommParam->speed = atoi((const char *)Value+1);
                    //printf("speed set OK %d\r\n",stCommParam->speed);
                    osDelay(10);
                }
                else if (Value[0]=='A')
                {
                    Alpha = atof((const char *)Value+1);
                    printf("Alpha set OK %f\r\n", Alpha);
                    osDelay(10);
                }
                else if (Value[0]=='G')
                {
                    GyroDrift = atof((const char *)Value+1);
                    printf("Drift set OK %f\r\n", GyroDrift);
                    osDelay(10);
                }
                else if (Value[0]=='M')
                {
                    char tempvar;
                    tempvar = atoi((const char *)Value+1);
                    if (tempvar == 1)
                    {
                        enableMotors = true;
                        printf("Motors Enabled \r\n");
                    }
                    else if(tempvar == 0)
                    {
                        enableMotors = false;
                        printf("Motors Disabled \r\n");
                    }
                    else
                    {
                        printf("KO! \r\n");
                        osDelay(10);
                    }
                    //printf("speed set OK %d\r\n",stCommParam->speed);
                    osDelay(10);
                }
                else
                {
                    printf("KO! \r\n");
                    osDelay(10);
                }
                //printf("\n\rCMD>\n\r");
                i=0;
            }
            memset(Value,'\0',strlen(Value));
        }
    }

    //flush buffer
    memset(Buf,'\0',strlen((char*)Buf));
    osDelay(10);
    //*Buf='\0';

}

    float error =0;
    int PID_Output = 0;
    static volatile float balanceSetPoint = 0;
void calculatePID(ST_CommParam *receivedCMD ,float sensorValue) 
{
    //#define PID_SCALER 0.01
    #define PID_SCALER 1
    extern float Xval;
    extern float GyroDrift;
    float previousError = 0; //(used by derivative control)
    char  PID_setPoint = 0 ; //radians, set by the user
    float proportional, derivative, integral;
    
    if (receivedCMD)
    {
        if (e_CurrState==E_STATE_Balancing)
        {
            error = sensorValue - balanceSetPoint - PID_setPoint;
        
            proportional = error;
            integral += error * 0.001f; 
            derivative = (error - previousError)*0.01f;
            previousError = error;
            
            setStepperAngleDir(error);
            
            PID_Output =
            (proportional*receivedCMD->KP)+
            (integral * receivedCMD->KI)+
            (derivative *receivedCMD->KD);
            
            if (PID_setPoint==0)
            {
                if (PID_Output<0)
                {
                    balanceSetPoint=balanceSetPoint + 0.02 ;
                    //balanceSetPoint++;
                }
                if (PID_Output>0)
                {
                    balanceSetPoint=balanceSetPoint - 0.02 ;
                    //balanceSetPoint--;
                }
            }
        }
        else
        {
            integral = 0;
            balanceSetPoint = 0;
            PID_Output = 0;
        }

         receivedCMD->speed = PID_Output;
    }
}

void enableLeftMDriver(E_State b_State)
{
    if (b_State == Enable)
    {
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
    }
}

void enableRightMDriver(E_State b_State)
{
    if (b_State == Enable)
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);// check if GPIO is defined !!!!
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
    }
}

//MS1------MS2------MS3------MicrostepResolution
//Low------Low------Low------Full step
//High-----Low------Low------Half step
//Low------High-----Low------Quarter step
//High-----High-----Low------Eighth step
//High-----High-----High-----Sixteenth step

void setLeftStepperMode(E_StepperMode E_Mode)
{ 
    //ms1 --> PF1
    //ms2 --> PC15
    //ms3 --> PC14
    switch(E_Mode)
    {
        case(Full_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET); //--->
            break;
        
        case(Half_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET); //--->
            break;
            
        case(Quarter_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET); //--->
            break;

        case(Eigth_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET); //--->
            break;
        
        case(Sixteenth_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET); //--->
            break;
        
        default:
            break;
    }
    
}

void setRightStepperMode(E_StepperMode E_Mode)
{
    //ms1 --> PF2
    //ms2 --> PA1
    //ms3 --> PC0
    switch(E_Mode)
    {
        case(Full_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //--->
            break;
        
        case(Half_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //--->
            break;
            
        case(Quarter_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //--->
            break;
        
        case(Eigth_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //--->
            break;
            
        case(Sixteenth_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET); //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET); //--->
            break;
        
        default:
            break;
    }
}

void initialiseParam(ST_CommParam *stArgCommParam)
{
    if (stArgCommParam)
    {
        stArgCommParam->angleOffset=ANGLE_OFFSET; //this is a mechanical constraint XD !
        stArgCommParam->KD = KD_M;
        stArgCommParam->KI = KI_M;
        stArgCommParam->KP = KP_M ;
        stArgCommParam->speed = 0;
        b_DebugEnabled = true;
        b_Reeinitialise = true;
        enableMotors = true;
    }
    else
    {
        Error_Handler();
    }
}

//extern float Ref_ACCELAngle;
//extern float Angle;
float speed;

void stateManage(float arg_CMD_Angle, ST_CommParam *stArgCommParam)
{
    speed = stArgCommParam->speed;
    static unsigned int driftCounter = 0;
    extern float AVGYaw ;
    if (stArgCommParam)
    {
        switch (e_CurrState)
        {
         case E_STATE_Balancing:
            if(arg_CMD_Angle > DEAD_ANGLE)
                e_CurrState = E_STATE_Fall;
            
            if (arg_CMD_Angle < BALANCE_RANGE )
            {
                e_CurrSubState = E_SSTATE_EQ;
                
                //b_Reeinitialise = false; // start the balancing process only when put at equilibrium point
                //stArgCommParam->speed=0; //overriding the pid value
                //Alpha = ALPHA; WTF ????
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
                //driftCounter++;
            }
            else
            {
                e_CurrSubState = E_SSTATE_OUT;
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
            }
             break;
         
         case E_STATE_Fall:
            //b_Reeinitialise = true; // to wait until put back at the balancing point
            stArgCommParam->speed=0; //overriding the pid value
            Alpha = ALPHA_CALIB; //to avoid setpoint drifts read from accelerometer !!
            if(arg_CMD_Angle < BALANCE_RANGE)
            {
                Alpha = ALPHA;
                e_CurrState = E_STATE_Balancing;
            }
             break;
         
        // case E_STATE_Calibrating:
        //    
        //   break;
         
         default:
             break;
        }
        
            
        //outside the balance range
        //if (arg_CMD_Angle > DEAD_ANGLE) 
        //{
        //    //b_Reeinitialise = true; // to wait until put back at the balancing point
        //    //stArgCommParam->speed=0; //overriding the pid value
        //    ////to avoid setpoint drifts read from accelerometer !!
        //    //Alpha = ALPHA_CALIB;
        //}
        //
        //// at equilibrium point
        //if (arg_CMD_Angle < BALANCE_RANGE )
        ////if (AVGYaw - stArgCommParam->angleOffset < BALANCE_RANGE )
        //{
        //    b_Reeinitialise = false; // start the balancing process only when put at equilibrium point
        //    stArgCommParam->speed=0; //overriding the pid value
        //    Alpha = ALPHA;
        //    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
        //    driftCounter++;
        //    //if ((driftCounter > 500)&&(ABS(AVGYaw)<BALANCE_RANGE))
        //    //{
        //    //    //Alpha = 0.8; // all value is read from accelerometer !!!
        //    //    driftCounter=0;
        //    //}
        //}
        //else
        //{
        //    //led is not on when not at equilibrium point ! 
        //    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
        //    driftCounter++;
        //}
    }
    else
    {
        Error_Handler();
    }
    
}

#if (DEBUG_VAL == DEBUG_SPEED)
void debugPrint(float arg_CMD_Angle, int speed)
#elif (DEBUG_VAL == DEBUG_ACCEL)
 void debugPrint(float arg_CMD_Angle, float speed)
#endif
{
extern float yaw;
extern float Xval,Xval1,Angle,sum1,sum2,AOA;

    if (b_DebugEnabled==true)
        {
            //do not remove this line it is for debug purposes !!
            //printf("%06.2f;%06.2f;\n\r",arg_CMD_Angle,speed); //leading zeros for the sign serial print the output is on 8
#if (DEBUG_VAL == DEBUG_SPEED)
            printf("%06.2f;%d;\n\r",arg_CMD_Angle,speed); //leading zeros for the sign serial print the output is on 8
#elif (DEBUG_VAL == DEBUG_ACCEL)
            //printf("G%06.2f;%06.2f;\n\r",arg_CMD_Angle,arg_CMD_Angle-ANGLE_OFFSET,speed); //leading zeros for the sign serial print the output is on 8
            //printf("%06.2f;%06.2f;%06.2f;\n\r",arg_CMD_Angle,speed,yaw/*,strDebug*/);
            //printf("%06.2f;%06.2f;%06.2f;;%06.2f\n\r",Xval,sum1,Angle,yaw);
            //printf("%06.2f;%06.2f;%06.2f;%06.2f;%06.2f;\n\r",sum2*1000,sum1*1000,AOA*1000,Angle,yaw);//to debug the gyro offset
            printf("%06.2f;%06.2f;0.0;0.0;0.0;\n\r",arg_CMD_Angle,ABS(speed));
#endif      
        }   
}

void setStepperMotorMode(E_StepperMode E_Mode)
{
    setLeftStepperMode(E_Mode);
    setRightStepperMode(E_Mode);
}

void setLeftStepperDir(E_Direction E_Dir)
{
    if (E_Dir==Forward)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
    }
}

void setRightStepperDir(E_Direction E_Dir)
{
    if (E_Dir==Forward)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
    }
}


void setStepperAngleDir(float argCMD_Angle)
{
    if (argCMD_Angle>0)
    {
        setLeftStepperDir(Forward);
        setRightStepperDir(Forward);
    }
    else
    {
        setLeftStepperDir(Backwards);
        setRightStepperDir(Backwards);
    }
}

void GanttDebug(char idx)
{
    //B11/13/15
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
    
    if (idx == 1)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
    }
    else if (idx == 2)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    }
    else if (idx == 3)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
    }
}

void toggleAllLeds(char delay)
{
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_13);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
        HAL_Delay(delay);
}

void AllLedSetState(GPIO_PinState STATE)
{
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9 ,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8 ,STATE);
}


float fyaw = 0;
float getAccelAngle(void)
{
    int16_t buffer[3] = {0};

    
    BSP_ACCELERO_GetXYZ(buffer);
    
    xAcc=(double)buffer[0]/16384;
    yAcc=(double)buffer[1]/16384;
    zAcc=(double)buffer[2]/16384;
    
    if (e_CurrState==E_STATE_Balancing)
    {
        nxAcc=AccelAVGX(xAcc);
        nyAcc=AccelAVGY(yAcc);
        nzAcc=AccelAVGZ(zAcc);
    }
    else
    {
      nxAcc=xAcc;
      nyAcc=yAcc;
      nzAcc=zAcc;
    }
    
    //calibrate offset
    //xAcc-=0.04;
    //yAcc-=0.09;
    //zAcc-=0.04;
    
    //fyaw = ((atan(zAcc/sqrt(xAcc*xAcc+yAcc*yAcc))*57.32));
    //return ((atan(zAcc/sqrt(xAcc*xAcc+yAcc*yAcc))*57.32));
    return (atan(nzAcc/sqrt(nxAcc*nxAcc+nyAcc*nyAcc))*57.32);
    
}

#define AVGNBR     200
float AVG(float newVal)
{
    static int i;
    static float AVGTAB [AVGNBR] = {0};
    static float sum;
    
    sum-=AVGTAB[i];
    
    sum+=newVal;
    AVGTAB[i] = newVal;
    
    if (i == (AVGNBR-1))
    {
        i=0;
    }
    else
    {
        i++;   
    }
    return(sum/AVGNBR);
}

#define AVGNBRAOA     501
static char toto = 0;
float AVGOAVG(float newVal,char reeinit)
{
    static int i;
    static float AVGOTAB [AVGNBRAOA] = {0};
    static float sumAOA;
    

    if (reeinit == 1)
    {
        toto = 1;
    }
    else
    {
        toto=2;
    }
    
    if (toto == 1)
    {
        for (int j=0;j<AVGNBRAOA;j++)
        {
            AVGOTAB[j] = newVal;
            sumAOA = newVal*AVGNBRAOA;
        }
    }
    else if (toto == 2)
    {
        sumAOA-=AVGOTAB[i];
        
        sumAOA+=newVal;
        AVGOTAB[i] = newVal;
        
        if (i == (AVGNBRAOA-1))
        {
            i=0;
        }
        else
        {
            i++;   
        }
    }
    
    return(sumAOA/AVGNBRAOA);
}

#define AVGNBRAOA2     50
float AVGOAVGOAVG(float newVal)
{
    static int i;
    static float AVGOTAB2 [AVGNBRAOA2] = {0};
    static float sumAOA;
    
    sumAOA-=AVGOTAB2[i];
    
    sumAOA+=newVal;
    AVGOTAB2[i] = newVal;
    
    if (i == (AVGNBRAOA2-1))
    {
        i=0;
    }
    else
    {
        i++;   
    }
    return(sumAOA/AVGNBRAOA2);
}

#define AVGNBR_ACCEL     150
float AccelAVGX(float newVal)
{
    static int i;
    static float AVGTAB [AVGNBR_ACCEL] = {0};
    static float sum;
    
    sum-=AVGTAB[i];
    
    sum+=newVal;
    AVGTAB[i] = newVal;
    
    if (i == (AVGNBR_ACCEL-1))
    {
        i=0;
    }
    else
    {
        i++;   
    }
    return(sum/AVGNBR_ACCEL);
}

float AccelAVGY(float newVal)
{
    static int i;
    static float AVGTAB [AVGNBR_ACCEL] = {0};
    static float sum;
    
    sum-=AVGTAB[i];
    
    sum+=newVal;
    AVGTAB[i] = newVal;
    
    if (i == (AVGNBR_ACCEL-1))
    {
        i=0;
    }
    else
    {
        i++;   
    }
    return(sum/AVGNBR_ACCEL);
}

float AccelAVGZ(float newVal)
{
    static int i;
    static float AVGTAB [AVGNBR_ACCEL] = {0};
    static float sum;
    
    sum-=AVGTAB[i];
    
    sum+=newVal;
    AVGTAB[i] = newVal;
    
    if (i == (AVGNBR_ACCEL-1))
    {
        i=0;
    }
    else
    {
        i++;   
    }
    return(sum/AVGNBR_ACCEL);
}
