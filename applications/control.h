#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx_hal.h"
#include "usart/bsp_usartx.h"
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "General_GPIO/bsp_general_gpio.h"
#include "led/bsp_led.h"
#include "beep/bsp_beep.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "GYRO.h"


typedef struct 
{
   __IO int      Forward;                              
	 __IO int      Back;                                
	 __IO int      Left;                               
   __IO int      Right;                                 
   __IO int    	 Stop;                                                              
}ORIENTATION;

typedef struct 
{
   __IO float      SetPoint;                                 // 设定目标 Desired Value
	 __IO float      SumError;                                 // ????
	 __IO float      AdcSumError;
	 __IO float      Proportion;                               // 比例常数 Proportional Const
   __IO float      Integral;                                 // 积分常数 Integral Const
   __IO float    	 Derivative;                               // 微分常数 Derivative Const
   __IO float      LastError;                                // Error[-1]
   __IO float      PrevError;                                // Error[-2]
	 __IO float      LastAngleError;                           // Error[-1]
   __IO float      PrevAngleError;                           // Error[-2]
	 __IO float      LastAdcError;                             // Error[-1]
   __IO float      PrevAdcError;                             // Error[-2]
}PID;

typedef enum {START = 0, ONGOING = 1, END = 2} NAVISTATE;

// Marco for Timers

#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define MAIN_LOOP_RATE 			RATE_100_HZ
#define MAIN_LOOP_DT			  (uint32_t)(1000/MAIN_LOOP_RATE)	/* ??ms*/

#define POS_ESTIMATE_RATE 	RATE_200_HZ
#define POS_ESTIMATE_DT			(uint32_t)(1000/POS_ESTIMATE_RATE)	/* ??ms*/

#define METER_PER_HALL   6.83365823e-4f    //  2.6065163e-4f
#define DEGREE_TO_RADIAN 0.017453293f

#define MOTOR_MAX_SPEED  1300
#define MOTOR_MIN_SPEED  600

#define BACKWARD_SPEED   -400
#define ROTATE_SPEED     400

#define ACT_BACK_DIST 0.05f
#define ACT_ROTATE_REACHED 11.0f
#define ACT_STRAIGHT_SHORT_DIST 0.20f

/* 私有类型定义 --------------------------------------------------------------*/
#define ENCODER1_LINE          12             // 编码器线数
#define ENCODER1_SPEEDRATIO    66.5           // 电机减速比
#define ENCODER1_PPR           (ENCODER1_SPEEDRATIO*ENCODER1_LINE) // Pulse/r 每圈可捕获的脉冲数
#define ENCODER2_LINE          12             // 编码器线数
#define ENCODER2_SPEEDRATIO    66.5           // 电机减速比
#define ENCODER2_PPR           (ENCODER2_SPEEDRATIO*ENCODER2_LINE) // Pulse/r 每圈可捕获的脉冲数
#define WHEEL_DIST             0.356f
#define LINEAR_ACCURACY        1000000.0f
#define ANGULAR_ACCURACY       1000.0f
#define BASE_FEED_BACK_LEN     17
#define SENSOR_FEED_BACK_LEN   9

#define IR_1_LOW               2
#define IR_1_HIGH              3
#define IR_0_LOW               1
#define IR_0_HIGH							 2
#define IR_LEADER_LOW          6
#define IR_LEADER_HIGH         10

#define LEFT									 0
#define MIDDLE_LEFT						 1
#define CENTER								 2
#define MIDDLE_RIGHT					 3
#define RIGHT									 4
#define NONE_IR_DATA           5

#define CENTER_LINE_UNDETECTED               0
#define FROM_NONE_SIG_ZONE_TO_LEFT           1
#define FROM_LEFT_SIG_ZONE_TO_NONE           2
#define FROM_LEFT_SIG_ZONE_TO_MIDDLE_LEFT    3
#define FROM_MIDDLE_LEFT_SIG_ZONE_TO_LEFT    4
#define FROM_RIGHT_SIG_ZONE_TO_MIDDLE_RIGHT  5
#define FROM_MIDDLE_RIGHT_SIG_ZONE_TO_RIGHT  6
#define FROM_NONE_SIG_ZONE_TO_RIGHT          7
#define FROM_RIGHT_SIG_ZONE_TO_NONE          8
#define CENTER_LINE_DETECTED                 9

#define FOLLOW_TIMEOUT        60000     //   1min
#define STOP_DELAY            50        // 
#define SHORT_PATH_DELAY      1333      //   (1333 0.2 888 0.15 666 0.1)
#define BACK_DIST_DELAY       333       //   (1333 0.2 888 0.15 666 0.1)
/***************************************/
// 定义PID相关宏
// 这三个参数设定对电机运行影响非常大
// PID参数跟采样时间息息相关
/*************************************/
#define SPD_P_DATA      2.0f          // P(0.2)
#define SPD_I_DATA      0.05f         // I(0.0)
#define SPD_D_DATA      0.0f          // D(0.0)

#define SPD_A        SPD_P_DATA*(1 + MAIN_LOOP_DT / SPD_I_DATA + SPD_D_DATA / MAIN_LOOP_DT)
#define SPD_B        SPD_P_DATA*(1 + 2 * SPD_D_DATA / MAIN_LOOP_DT)
#define SPD_C				 SPD_P_DATA * SPD_D_DATA / MAIN_LOOP_DT

#define TARGET_SPEED    400           // 0.2m/s  0.962 r/s (1150 0.3m/s  1.45 r/s)
#define START_SPEED     100
#define TARGET_PWM      950

#define MAX_STRAIGHT_DIST      8.0f   // straight line length
#define ZIPPER_WALKING_CNT     15			// zigtag number

#define GYRO_RESET_TICK  180000       // 3 min 
#define GYRO_OFF_SET_PT  -0.175f      // 0.2 degree per time

#define SPD_DEAD_ZONE    3            // Threshold of Stable Angle(50)
#define HALL_PID_FEED_CLAMP    20     // Threshold of Stable Angle(20)

#define STABLE_ANGLE_THRESHOLD 0.03f  // Threshold of Stable Angle(0.03)

#define SPD_SUM_ERROR_CLAMP    75.0f     // Clamp Value of Speed Sum Error: 5.0f,6.0f for L298N 14kHz(3.0)
#define SPD_PID_FEED_CLAMP     100.0f    // Clamp Value of Speed PID Feed: 10.0f,12.0f for L298N 14kHz(3.0)

#define GYRO_P_DATA      13.5f        // P (F103 0.5f F407 14.0f 13.5f)(3.0)
#define GYRO_I_DATA      0.04f        // I (F103 0.8f F407 00.0f 0.05f)(0.05)
#define GYRO_D_DATA      0.02f        // D (F103 0.5f F407 00.0f 0.02f)(0.0)

#define ROTATE_LEAD_CORRECTION 37.0f  // Lead-Correction for Rotation: (F407 6.5f 8.8f 9.5f)(5.0)

#define ADC_P_DATA      20.0f         // P
#define ADC_I_DATA      0.2f          // I
#define ADC_D_DATA      0.5f          // D

#define FORWARD_DELAY   1200          // ms
#define BACKWAED_DELAY  700           // ms
#define SERIAL_LEN      65            // lenth
#define RTK_LEN         55            // lenth
#define ADC_VOTAGE_LEN  48

#define DUTY_ZERO                   0      // 0%占空比
#define DUTY_FULL                   999    // 100%占空比
#define DUTY_HALF                   499

#define DUTY_FORWARD                800
#define DUTY_BACKWARD               199

#define FORWARD_SLOW                775
#define BACKWARD_SLOW               224

#define DUTY_FORWARD_SLOW           450
#define DUTY_BACKWARD_SLOW          549

#define LONG_PATH_TIME              25000
#define SHORT_PATH_TIME							1500


extern int32_t Forward_PWM_Duty;
extern int32_t M1_Forward_PWM_Duty;
extern int32_t M2_Forward_PWM_Duty;
extern int32_t M1_Forward_PWM_Duty_H; // CHN 1 rightN
extern int32_t M2_Forward_PWM_Duty_H; // CHN 2 left
extern int32_t M1_Forward_PWM_Duty_L; // CHN 3 right
extern int32_t M2_Forward_PWM_Duty_L; // CHN 4 leftN
extern PID sPID;
static PID *sptr = &sPID;
extern uint32_t gyro_reset_time;
extern int16_t map_index;
extern float map_data[256][5];
extern float source_angle;
extern float yaw_angle, pitch_angle, roll_angle;
typedef struct 
{
  __IO int32_t  SetPoint;                                 // 设定目标 Desired Value
  __IO long     SumError;                                 // 误差累计
  __IO float    Proportion;                               // 比例常数 Proportional Const
  __IO float    Integral;                                 // 积分常数 Integral Const
  __IO float    Derivative;                               // 微分常数 Derivative Const
  __IO int      LastError;                                // Error[-1]
  __IO int      PrevError;                                // Error[-2]
}PID_TypeDef;

struct RTK_Pose_N_Int
{
	int8_t nIntPose[4];
};

struct RTK_Pose_N_Dec
{
	int8_t nDecPose[8];
};

struct RTK_Pose_E_Int
{
	int8_t eIntPose[5];
};

struct RTK_Pose_E_Dec
{
	int8_t eDecPose[8];
};
struct JSN_Dist
{
	int8_t  dist[2];
};

/* 私有宏定义 ----------------------------------------------------------------*/
#define WIN         5


//#define  P_DATA      5.75f                                // P参数
//#define  I_DATA      0.03f                                // I参数
//#define  D_DATA      0.0f                                 // D参数

#define  TARGET_PULSE  1000

#define SERIAL_LEN      65           //  lenth
#define JSN_LEN         10 
#define abs(x)    ((x)>0?(x):-(x))
/* 私有变量 ------------------------------------------------------------------*/
extern uint32_t IS_EnableMotor;  // 使能电机标志
extern uint32_t Time_CNT;

extern uint32_t RTK_CNT;
extern unsigned char RTK_DATA_BUFF[RTK_LEN];
extern unsigned char RTKCnt;
extern double jsn_dist;
extern int bumper_event;
extern int avoidance_event;


extern double rtk_pose_n, rtk_pose_e;
extern int32_t  flag;
extern int32_t  start_flag;
extern int32_t Avg_cnt;
extern float Avg_Speed[WIN];
extern float angle, initAngle, forwardAngle, shortPathLeftAngle, backwardAngle,shortPathRightAngle, angle_rate;
extern float last_angle;
extern int initPathAngleFlag, initAngleFlag;
extern __IO uint32_t PWM_ChangeFlag;    // 输入捕获数

extern unsigned char ucRxBuffer[SERIAL_LEN];
extern unsigned char ucRxCnt;
extern uint8_t aRxCmd;
extern uint8_t aRxCmd_5,aRxCmd_2;
extern int speed_dead_zone;
extern float off_set_val;
extern float pose_tolerance;
extern int obj_detected;

//////////////////////////////////////
float normalizeAngle(__IO float _angle);

float IncPIDCalc(float destAngle);
int32_t LocPIDCalc(int32_t NextPoint);
int32_t LocPIDCalcWithNewSet(int32_t cur_speed, int32_t target_speed);

void CopySerial2Data(unsigned char ucData);
void RTKData_Analysis(unsigned char ucData);
void JSNDist_detect(unsigned char ucData);

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
void Analyse_Data_Callback(void);
void IncPIDInit(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void Move_Straight(void);
void Turn_Left(void);
void Turn_Right(void);
void Stop(void);
void Start(void);
void rotateAngle(__IO float _angle);
void rotateToAngleControl(__IO float _angle);
void Move_Back(void);
void angle_test_func(void);
void print_map_data(void);
void R6093U_Reset(void);


#endif /* __CONTROL_H */
