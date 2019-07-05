/**
  ******************************************************************************
 * Software Closed Source
 *
 * Copyright (c) 2019, iRobSen, Inc.
 * All rights reserved.
 *
 * File : control.c
  ******************************************************************************
	**/
	
#include "control.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SRes     stcReserved;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;

struct RTK_Pose_N_Int rtkNPoseInt;
struct RTK_Pose_N_Dec rtkNPoseDec;
struct RTK_Pose_E_Int rtkEPoseInt;
struct RTK_Pose_E_Dec rtkEPoseDec;

struct JSN_Dist jDist;

PID sPID;

int32_t M1_Forward_PWM_Duty_H = 0; // CHN 1 rightN
int32_t M2_Forward_PWM_Duty_H = 0; // CHN 2 left
int32_t M1_Forward_PWM_Duty_L = 0; // CHN 3 right
int32_t M2_Forward_PWM_Duty_L = 0; // CHN 4 leftN
int32_t M1_Forward_PWM_Duty = 0;
int32_t M2_Forward_PWM_Duty = 0;

/* 私有变量 ------------------------------------------------------------------*/
uint32_t IS_EnableMotor = 0;  // 使能电机标志
uint32_t Time_CNT = 0;
uint32_t RTK_CNT = 0;
unsigned char RTK_DATA_BUFF[RTK_LEN];
unsigned char RTKCnt = 0;

unsigned char JSN_DATA_BUFF[JSN_LEN];
unsigned char JSNCnt = 0;
int obj_detected = 0;
int bumper_event = 0;
int avoidance_event = 0;
double jsn_dist = 0.0f;
double rtk_pose_n, rtk_pose_e;
int32_t  flag = 0;
int32_t  start_flag = 0;
int32_t Avg_cnt = 0;
float Avg_Speed[WIN]= {0};
float angle = 0.0f, initAngle = 0.0f, forwardAngle = 0.0f, shortPathLeftAngle = 0.0f, backwardAngle = 0.0f,shortPathRightAngle = 0.0f , angle_rate = 0.0f;
float source_angle = 0.0f;
float last_angle = 0.0f;
int initPathAngleFlag = 0, initAngleFlag = 0;
__IO uint32_t PWM_ChangeFlag = 0;
__IO int32_t CaptureNumber = 0;      // 输入捕获数

unsigned char ucRxBuffer[SERIAL_LEN];
unsigned char ucRxCnt = 0;
uint8_t aRxCmd;
uint8_t aRxCmd_5,aRxCmd_2;
int speed_dead_zone = 3;
float pose_tolerance = 0.05f;
uint32_t gyro_reset_time = 0;
float off_set_val = 0.0f;
int16_t map_index = 0;
float map_data[256][5];
float source_gyro_angle = 0.0f;
static const short src_head = 42662;
static short src_rate = 0, src_angle = 0;
static int8_t src_checksum = 0, src_index = 0;
float yaw_angle = 0.0f, pitch_angle = 0.0f, roll_angle = 0.0f;
//////////////////////////////////////
float normalizeAngle(__IO float _angle);

int32_t LocPIDCalc(int32_t NextPoint);
int32_t LocPIDCalcWithNewSet(int32_t NextPoint, int32_t SetPoint);

/**
  * 函数功能: 系统滴答定时器中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 每发生一次滴答定时器中断进入该回调函数一次
  */
int32_t PID_Result = 0; 
void HAL_SYSTICK_Callback()
{

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == Left_Motor_Hall_Feedback_GPIO_PIN)
	{
		Left_Hall_count++;
	}
	
	if(GPIO_Pin == Right_Motor_Hall_Feedback_GPIO_PIN)
	{
	  Right_Hall_count++;
	}
	
	if(GPIO_Pin == Bumper_GPIO_PIN)
	{
		bumper_event = 1;
	}
	
	if(GPIO_Pin == Avoidance_GPIO_PIN)
	{
		avoidance_event = 1;
	}
}

void HAL_GPIO_NMI_Callback()
{

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USARTx_5)
	{
		RTKData_Analysis((unsigned char)aRxCmd_5);
		HAL_UART_Transmit(&husartx,&aRxCmd_5,1,0);
		HAL_UART_Receive_IT(&husartx_5,&aRxCmd_5,1);	
	}else
	if(UartHandle->Instance == DEBUG_USARTx)
	{
		CopySerial2Data((unsigned char)aRxCmd);//
		//HAL_UART_Transmit(&husartx,&aRxCmd,1,0xffff);
		HAL_UART_Receive_IT(&husart_debug,&aRxCmd,1);
	}
	else
	if(UartHandle->Instance == USARTx_2)
	{
		JSNDist_detect((unsigned char)aRxCmd_2);
		//HAL_UART_Transmit(&husartx,&aRxCmd_2,1,0);
		HAL_UART_Receive_IT(&husartx_2,&aRxCmd_2,1);
	}
}

float normalizeAngle(__IO float _angle)
{
	if(_angle > 180.0f)
		_angle -= 360.0f;
	
	if(_angle < -180.0f)
		_angle += 360.0f;
	
	return _angle;
}

void CopySerial2Data(unsigned char ucData)
{
	ucRxBuffer[ucRxCnt++]=ucData;
	
	if (ucRxBuffer[0]!=0xA6) //
	{
		ucRxCnt=0;
		return;
	}
	
	if (ucRxCnt<26) {return;} //
	else if (ucRxBuffer[1] != 0xA6)
	{
		ucRxCnt=0;
		return;
	}
	else
	{
		memcpy(&src_index, &ucRxBuffer[2], 1);
		memcpy(&stcAngle, &ucRxBuffer[3], 6);
		memcpy(&stcGyro, &ucRxBuffer[9], 6);
		memcpy(&stcAcc, &ucRxBuffer[15], 6);
		memcpy(&stcReserved, &ucRxBuffer[21], 4);
		memcpy(&src_checksum, &ucRxBuffer[25], 1);
		
		roll_angle = (float)(-stcAngle.Angle[0]) * 0.01f;
		pitch_angle = (float)stcAngle.Angle[1] * 0.01f;
		yaw_angle = (float)(-stcAngle.Angle[2]) * 0.01f;

    angle = yaw_angle;
		last_angle = yaw_angle;
	}
	ucRxCnt=0;
}

void RTKData_Analysis(unsigned char ucData)
{
//		RTK_CNT++;
	RTK_DATA_BUFF[RTKCnt++]=ucData;
	if (RTK_DATA_BUFF[0]!=0x24)
	{
		RTKCnt=0;
		return;
	}
	if(RTKCnt<51){return;}
	else if(RTK_DATA_BUFF[50] != 0x34)
	{
		RTKCnt=0;
		return;	
	}
	else
	{
		double rtk_n_temp = 0.0f,rtk_e_temp = 0.0f;
		double rtk_n_temp_int = 0.0f,rtk_e_temp_int = 0.0f,rtk_n_temp_dec = 0.0f,rtk_e_temp_dec = 0.0f;
		int8_t rtk_data_temp;
//	  memcpy(&rtkNPoseInt, &RTK_DATA_BUFF[17], 4);
//		memcpy(&rtkNPoseDec, &RTK_DATA_BUFF[22], 8);
//		memcpy(&rtkEPoseInt, &RTK_DATA_BUFF[33], 5);
//		memcpy(&rtkEPoseDec, &RTK_DATA_BUFF[39], 8);
		for(int rtk_index = 0; rtk_index < 4; rtk_index++)
		{
			memcpy(&rtk_data_temp, &RTK_DATA_BUFF[17+rtk_index], 1);
			if(rtk_index == 0)
				rtk_n_temp_int += (double)(rtk_data_temp - 0x30) * 1.0e1f;
			else if(rtk_index == 1)
				rtk_n_temp_int += (double)(rtk_data_temp - 0x30) * 1.0f;
			else if(rtk_index == 2)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-1f;
			else if(rtk_index == 3)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-2f;
		}
		
		for(int rtk_index = 0; rtk_index < 8; rtk_index++)
		{
			memcpy(&rtk_data_temp, &RTK_DATA_BUFF[22+rtk_index], 1);
			if(rtk_index == 0)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-3f;
			else if(rtk_index == 1)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-4f;
			else if(rtk_index == 2)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-5f;
			else if(rtk_index == 3)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-6f;
			else if(rtk_index == 4)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-7f;
			else if(rtk_index == 5)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-8f;
			else if(rtk_index == 6)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-9f;
			else if(rtk_index == 7)
				rtk_n_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-10f;
		}
		
		for(int rtk_index = 0; rtk_index < 5; rtk_index++)
		{
			memcpy(&rtk_data_temp, &RTK_DATA_BUFF[33+rtk_index], 1);
			if(rtk_index == 0)
				rtk_e_temp_int += (double)(rtk_data_temp - 0x30) * 1.0e2f;
			else if(rtk_index == 1)
				rtk_e_temp_int += (double)(rtk_data_temp - 0x30) * 1.0e1f;
			else if(rtk_index == 2)
				rtk_e_temp_int += (double)(rtk_data_temp - 0x30) * 1.0f;
			else if(rtk_index == 3)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-1f;
			else if(rtk_index == 4)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-2f;
		}
		for(int rtk_index = 0; rtk_index < 8; rtk_index++)
		{
			memcpy(&rtk_data_temp, &RTK_DATA_BUFF[39+rtk_index], 1);
			if(rtk_index == 0)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-3f;
			else if(rtk_index == 1)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-4f;
			else if(rtk_index == 2)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-5f;
			else if(rtk_index == 3)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-6f;
			else if(rtk_index == 4)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-7f;
			else if(rtk_index == 5)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-8f;
			else if(rtk_index == 6)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x30) * 1.0e-9f;
			else if(rtk_index == 7)
				rtk_e_temp_dec += (double)(rtk_data_temp - 0x31) * 1.0e-10f;			
		}
		
		rtk_n_temp = rtk_n_temp_int + (double)(rtk_n_temp_dec * 100.0f / 60.0f);
		rtk_e_temp = rtk_e_temp_int + (double)(rtk_e_temp_dec * 100.0f / 60.0f);
		
		rtk_pose_n = rtk_n_temp;
		rtk_pose_e = rtk_e_temp;
//		rtk_pose_e = (float)(rtkEPoseInt.eIntPose[0] - 0x30) * 10.0f;
		/*
		rtk_pose_n = (double)(rtkNPoseInt.nIntPose[0] * 10000.0f + rtkNPoseInt.nIntPose[1]*100.0f + 
			rtkNPoseDec.nDecPose[0] + rtkNPoseDec.nDecPose[1] * 0.01f + rtkNPoseDec.nDecPose[2] * 0.0001f + rtkNPoseDec.nDecPose[3] * 0.000001f);
		rtk_pose_e = (double)(rtkEPoseInt.eIntPose[0] * 100000.0f + rtkEPoseInt.eIntPose[1] * 1000.0f + rtkEPoseInt.eInt * 100.0f +
			rtkEPoseDec.eDecPose[0] + rtkEPoseDec.eDecPose[1] * 0.01f + rtkEPoseDec.eDecPose[2] * 0.0001f + rtkEPoseDec.eDecPose[3] * 0.000001f);
		*/
	}
	RTKCnt=0;
}

void JSNDist_detect(unsigned char ucData)
{
	JSN_DATA_BUFF[JSNCnt++]=ucData;
	if (JSN_DATA_BUFF[0]!=0xFF)
	{
		JSNCnt=0;
		return;
	}
	if(JSNCnt<4){return;}
//	else if(JSN_DATA_BUFF[3]!=(int8_t)(JSN_DATA_BUFF[0]+JSN_DATA_BUFF[1]+JSN_DATA_BUFF[2]))
//	{
//		JSNCnt=0;
//		return;
//	}
	else
	{
		memcpy(&jDist, &JSN_DATA_BUFF[1], 2);
		jsn_dist = (jDist.dist[0] * 256.0f + jDist.dist[1]) * 0.001f;
		//jsn_dist = jDist.dist * 0.001f;
		if(jsn_dist <= 0.5f && jsn_dist > 0.0f)
		{
			obj_detected = 1;
		}
		else
		{
			obj_detected = 0;
		}
	}
	JSNCnt=0;
}

/******************** 位置闭环 PID 控制设计 ************************************/
/** 
  * 函数名称：位置闭环PID控制设计
  * 输入参数：当前控制量
  * 返 回 值：目标控制量
  * 说    明：无
  */
int32_t LocPIDCalc(int32_t NextPoint)
{
  int32_t iError,dError;
  iError = sPID.SetPoint - NextPoint; //偏差
  if((iError<10 )&& (iError>-10))
    iError = 0;
  /* 积分分离 */
  if((iError<200 )&& (iError>-200))
  { 
    sPID.SumError += iError; //积分 
    /* 积分上限 */
    if(sPID.SetPoint>0)
    {
      if(sPID.SumError >= 7 * sPID.SetPoint)
        sPID.SumError = 7 * sPID.SetPoint;
    }
    else if(sPID.SumError <= 7 * sPID.SetPoint)
         sPID.SumError = 7 * sPID.SetPoint;
  }
  dError = iError - sPID.LastError; //微分
  sPID.LastError = iError;
  
  return (int32_t)(sPID.Proportion * iError //比例项
  + sPID.Integral * sPID.SumError //积分项
  + sPID.Derivative * dError); //微分项
}

/** 
  * 函数名称：增量式PID控制设计
  * 输入参数：当前控制量
  * 返 回 值：目标控制量
  * 说    明：无
  */
float IncPIDCalc(float destAngle)
{
//	readAngle(ucRxBuffer);
//	if(fabs(normalizeAngle(last_angle - angle)) > 10.0f)
//	{
//		errorAngleCnt++;
//		if(errorAngleCnt<3)
//			return 0.0f;
//	}
//	
//	errorAngleCnt = 0;
//	last_angle = angle;
	
	float iError, dError;
	float iIncpid = 0.0f;                                                                              //当前误差
	//iError=sptr->SetPoint-NextPoint;                                                                 //增量计算
	iError = normalizeAngle(destAngle - yaw_angle);
//	if (iError < STABLE_ANGLE_THRESHOLD && iError > -STABLE_ANGLE_THRESHOLD)                         //误差在正负0.05以内，则不做处理
//		return iIncpid;
//	if (iError > 180.0f)
//		iError -= 360.0f;
//	if (iError < STABLE_ANGLE_THRESHOLD && iError > -STABLE_ANGLE_THRESHOLD)                         //误差在正负0.05以内，则不做处理
//		return iIncpid;
//	if (iError < -180.0f)
//		iError += 360.0f;
	if (iError < STABLE_ANGLE_THRESHOLD && iError > -STABLE_ANGLE_THRESHOLD)                         //误差在正负0.05以内，则不做处理
		return iIncpid;
	
	sptr->SumError += iError;            // I
	
	if(sptr->SumError > SPD_SUM_ERROR_CLAMP)
			sptr->SumError = SPD_SUM_ERROR_CLAMP;
	else if(sptr->SumError < -SPD_SUM_ERROR_CLAMP)
			sptr->SumError = -SPD_SUM_ERROR_CLAMP;
	
	dError = iError - sptr->LastAngleError;                 // D 
	
  // iIncpid=(sptr->Proportion * iError)                  // E[k]项
  //            -(sptr->Integral * sptr->LastError)     	// E[k-1]项
  //           +(sptr->Derivative * sptr->PrevError);  		// E[k-2]项
	
	iIncpid=(GYRO_P_DATA * iError +                // ???
					+ GYRO_I_DATA * (float)sptr->SumError // ???
					+ GYRO_D_DATA * dError);
					
	sptr->PrevAngleError = sptr->LastAngleError;				
	sptr->LastAngleError=iError;
					
	// error = iError;
  // return(iIncpid);                                    //返回增量值
	
	if(iIncpid > SPD_PID_FEED_CLAMP)
			iIncpid = SPD_PID_FEED_CLAMP;
	else if(iIncpid < -SPD_PID_FEED_CLAMP)
			iIncpid = -SPD_PID_FEED_CLAMP;
	return(iIncpid);
}

/**
  * 函数功能: PID参数初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void IncPIDInit(void) 
{
    sptr->LastError=0;               //Error[-1]
    sptr->PrevError=0;               //Error[-2]
		sptr->LastAngleError=0;          //Error[-1]
    sptr->PrevAngleError=0;          //Error[-2]
		sptr->LastAdcError=0;            //Error[-1]
    sptr->PrevAdcError=0;            //Error[-2]
		sptr->SumError = 0;
		sptr->AdcSumError = 0;
    sptr->Proportion=SPD_P_DATA;      //比例常数 Proportional Const
    sptr->Integral=SPD_I_DATA;        //积分常数  Integral Const
    sptr->Derivative=SPD_D_DATA;      //微分常数 Derivative Const
    sptr->SetPoint=TARGET_SPEED;           //设定目标Desired Value
}

int32_t LocPIDCalcWithNewSet(int32_t cur_speed, int32_t target_speed)
{
	sptr->SetPoint = target_speed;
	int32_t iError,iIncpid;
	iError = target_speed - cur_speed; //偏差
//	printf("iError %d \n", iError);
	/* 死区限定 */
	if(abs(iError)<=speed_dead_zone)
		iError = 0;
	
	//	return 0;
	
	iIncpid=(int32_t)( (SPD_A * (float)iError)       //E[k]项
					- (SPD_B * (float)sptr->LastError)       //E[k-1]项
					+ (SPD_C * (float)sptr->PrevError));     //E[k-2]项

	sptr->PrevError = sptr->LastError;                    //存储误差，用于下次计算
	sptr->LastError = iError;
	
	if(iIncpid > HALL_PID_FEED_CLAMP)
	{
		iIncpid = HALL_PID_FEED_CLAMP;
	} else if(iIncpid < -HALL_PID_FEED_CLAMP)
	{
		iIncpid = -HALL_PID_FEED_CLAMP;
	}
	
	return (iIncpid);                                    //返回增量值
}

int rotateToAngle(__IO float _angle)
{
//   		readAngle(ucRxBuffer);
//		if(fabs(normalizeAngle(last_angle - angle)) > 10.0f)
//		{
//			 printf("ERROR angle : %3.3f\n", angle);
//			 errorAngleCnt++;
//			 if(errorAngleCnt < 3)
//					return 0;;
//		}
//		errorAngleCnt = 0;
//		last_angle = angle;
		if (normalizeAngle(angle - _angle) < ROTATE_LEAD_CORRECTION && normalizeAngle(angle - _angle) > -ROTATE_LEAD_CORRECTION)
			return 1;
		return 0;
}

void Move_Straight(void)
{
	HAL_GPIO_WritePin(Left_Motor_Orientation_GPIO, Left_Motor_Orientation_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_Motor_Orientation_GPIO, Right_Motor_Orientation_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Left_Motor_Break_GPIO, Left_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_Motor_Break_GPIO, Right_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
}

void Move_Back(void)
{
	HAL_GPIO_WritePin(Left_Motor_Orientation_GPIO, Left_Motor_Orientation_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Right_Motor_Orientation_GPIO, Right_Motor_Orientation_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Left_Motor_Break_GPIO, Left_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_Motor_Break_GPIO, Right_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
}

void Turn_Left(void)
{
	HAL_GPIO_WritePin(Left_Motor_Orientation_GPIO, Left_Motor_Orientation_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Right_Motor_Orientation_GPIO, Right_Motor_Orientation_GPIO_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(Left_Motor_Break_GPIO, Left_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_Motor_Break_GPIO, Right_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
}

void Turn_Right(void)
{
	HAL_GPIO_WritePin(Left_Motor_Orientation_GPIO, Left_Motor_Orientation_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_Motor_Orientation_GPIO, Right_Motor_Orientation_GPIO_PIN, GPIO_PIN_SET);		
	HAL_GPIO_WritePin(Left_Motor_Break_GPIO, Left_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_Motor_Break_GPIO, Right_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
}

void Stop(void)
{
	HAL_GPIO_WritePin(Left_Motor_Break_GPIO, Left_Motor_Break_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Right_Motor_Break_GPIO, Right_Motor_Break_GPIO_PIN, GPIO_PIN_RESET);
}

void Start(void)
{
	HAL_GPIO_WritePin(Left_Motor_Break_GPIO, Left_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_Motor_Break_GPIO, Right_Motor_Break_GPIO_PIN, GPIO_PIN_SET);
}

void rotateAngle(__IO float _angle)
{
//	readAngle(ucRxBuffer);
	float dest_angle = _angle + angle;
	dest_angle = normalizeAngle(dest_angle);
	rotateToAngleControl(dest_angle);
}

void rotateToAngleControl(__IO float _angle)
{
		if(normalizeAngle(angle - _angle) < ROTATE_LEAD_CORRECTION && normalizeAngle(angle - _angle) > -ROTATE_LEAD_CORRECTION)
		{
			return;
		}
		else
		{
		  while(!rotateToAngle(_angle));
		}
}

void angle_test_func(void)
{
	float time_sample = 0.0f;
	uint32_t sampling_hz = 0;
	uint32_t gyro_time = HAL_GetTick();
	while(1)
	{
		sampling_hz = HAL_GetTick() - gyro_time;
		if(sampling_hz > 1000)
		{
			gyro_time = HAL_GetTick();
			time_sample += 1.0f;
			printf("%3.3f,%3.3f\n",angle, time_sample);
		}
	} // angle test func
}

void R6093U_Reset()
{
	HAL_GPIO_WritePin(R6093U_GPIO, R6093U_GPIO_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(R6093U_GPIO, R6093U_GPIO_PIN, GPIO_PIN_SET);
//	HAL_Delay(3000);
}
