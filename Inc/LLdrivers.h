#ifndef LLDRIVERS_H
#define LLDRIVERS_H

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include <stdbool.h>

#define STR_DELIM   ';'
#define ENTER_ASCII 0x0D

#define STR_DEBUG_ON   '*'
#define STR_DEBUG_OFF  ' '

#define ANGLE_VERIF_TSH           10
#define ANGLE_VERIF_MAX_SAMPL     500

#define OMEGA_MAX   1
#define STEPPER_MODE 4
//#define LINEARISE(X)  100/(200 * 0.00002)
#define ARR(X)  100/(200 *STEPPER_MODE* 0.00002 * OMEGA_MAX * X)


#define ABS(X)     ((X>=0)?X:-X)


//when the object is static we notice that the raw values are 
//around -550 these results in significant Drift! this vlue is to compensate that 
//after testing this is the value we got to nullify gyroscopic drift
#define X_AXIS_CALIB            620 

//this is the limit of wich the robot cannot recover from it's tilt better fall and hope for the best XD !!!
//the motors sould never operate past this limit
#define DEAD_ANGLE              24

//TODO : these config flags should be moved to an entierly new file .h
#define DEBUG_SPEED             0

#define DEBUG_ACCEL             1

#define DEBUG_VAL               DEBUG_ACCEL

#define BALANCE_RANGE           (0.2)

#define ALPHA                   (0.999)

#define ALPHA_CALIB             (0)

#define INTEGRATE_DELAY(X)     (X*0.001)

#define ACTIVE_DELAY_MS            10 

#define ANGLE_OFFSET            (-2.5) // meaning at equilibrium point the sensor reading is 2.6

#define CALIBRATION_CYCLE        500

#define GYROSCOPE_DRIFT         0.02

#define KP_M                    (120)
#define KI_M                    (0)
#define KD_M                    (0)


typedef enum
{
E_STATE_Balancing,
E_STATE_Fall,
E_STATE_Calibrating,
E_STATE_POSTCalibVERIF
}E_SytemState;



typedef enum
{
E_SSTATE_EQ,
E_SSTATE_OUT,
}E_SytemSubState;


typedef enum
{
Enable,
Disable
}E_State;

typedef enum
{
Forward,
Backwards
}E_Direction;

typedef enum
{
Full_S,
Half_S,
Quarter_S,
Eigth_S,
Sixteenth_S
}E_StepperMode;


typedef struct 
{
    float angleOffset;
    float KP;
    float KI;
    float KD;
    int speed;
}ST_CommParam;



typedef struct{
    uint8_t * pu8_BufISR;
    //uint8_t  u16_SizeISR ;
}ST_UART1_ISR;

float AVG(float newVal);
float AccelAVGX(float newVal);
float AccelAVGY(float newVal);
float AccelAVGZ(float newVal);

float AVGOAVG(float newVal,char reeinit);
float AVGOAVGOAVG(float newVal);


void init_PWMTimers(void);

float calibrateIMU(void);

float getAccelAngle(void);

void LLDriverCliMenu(uint8_t* Buf,ST_CommParam *stCommParam);

void calculatePID(ST_CommParam *receivedCMD,float CMD_Angle);

void setMotorCmd(ST_CommParam *receivedCMD);

void toggleAllLeds(char delay);

void AllLedSetState(GPIO_PinState STATE);

#if (DEBUG_VAL == DEBUG_SPEED)
        void debugPrint(float arg_CMD_Angle, int speed);
#elif (DEBUG_VAL == DEBUG_ACCEL)
        void debugPrint(float arg_CMD_Angle, float ACCEL);
#endif





//left motor functions
void enableLeftMDriver(E_State b_State);
void setLeftStepperMode(E_StepperMode E_Mode);
void setLeftStepperDir(E_Direction E_Dir);

//right motor functions
void enableRightMDriver(E_State b_State);
void setRightStepperMode(E_StepperMode E_Mode);
void setRightStepperDir(E_Direction E_Dir);

void setStepperMotorMode(E_StepperMode E_Mode);
void setStepperAngleDir(float argCMD_Angle);

void initialiseParam(ST_CommParam *stArgCommParam);

void stateManage(float arg_CMD_Angle, ST_CommParam *stArgCommParam);

void GanttDebug(char idx);

//osMailQDef (object_pool_qCMD, 4*sizeof(ST_CommParam), ST_CommParam);  // Declare mail queue
//osMailQId  (object_pool_q_idCMD);                 // Mail queue ID

#endif //LLDRIVERS_H

